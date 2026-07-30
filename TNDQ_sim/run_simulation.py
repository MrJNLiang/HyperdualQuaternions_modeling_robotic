"""
闭环仿真主程序 —— 论文第 5/6 节 + CoppeliaSim 对接实验
（场景篇《KUKALBR4p场景_定点与圆周扰动对比实验设计.md》的代码实现）。

双后端结构：

  --backend internal     内部被控对象（无需 CoppeliaSim）
      --plant accel      式 (5.1) 加速度级理想对象  q̈ = q̈_ref + w_dyn（原版）
      --plant torque     力矩级刚体对象  M q̈ + C q̇ + g = τ + τ_ext
                         （RNEA 正动力学，config/lbr4_dynamics.py；
                          控制端/对象端可用不同参数 -> E3 参数失配）
  --backend coppeliasim  CoppeliaSim 动力学引擎替代内部积分器：
                         传感 sim.getJointPosition/Velocity -> (q, q̇)
                         执行 sim.setJointTargetForce(τ)，同步 sim.step()
                         （interfaces/coppeliasim_interface.py，力矩模式）

控制栈对两个后端完全同构（论文 §6.2 流水线，接口契约不变）：

    [FK 层]    TNDQ 链连乘 (3.4) -> x, ξ, J, J̇q̇（免构造读出, 3.5）, 残差 (3.8)
    [误差层]   HDQ 误差元素 (4.1) -> e_ξ, e_z, A（定理 1/2）
    [控制层]   几何一致计算力矩律 (5.2) -> q̈_ref
    [力矩层]   τ = M̂ q̈_ref + Ĉ q̇ + ĝ（§2.4，Gaz [11] 名义模型）
    [监控层]   V、能量核算（定理 3 证书）、约束残差 (3.8)、CSV/npz 输出

实验场景（场景篇 §4/§5）：
    --scenario line / circle       原版基座系轨迹（回归基线）
    --scenario setpoint            S1 定点：杯口上方 0.10 m 预抓取位姿
    --scenario cup-circle          S2 圆周：绕杯口上方 0.10 m、R=0.06 m 水平圆

实验条件（总方案 §5.3 场景矩阵 E1–E7 + 场景篇 §6 扰动方案）：
    --condition none               E1 标称（定理 3(b) 指数型收敛）
    --condition l2                 E2 L2 有限能量扰动（定理 3(c) H∞ 增益核验）
    --condition bias               L∞ 偏差扰动（定理 3(d) ISS 极限球，原版保留）
    --condition mismatch           E3 参数失配（控制器名义惯性参数高估 20%）
    --condition large-error        E4 大姿态误差初始位形（unwinding/参数化退化）
    --condition highspeed          E5 高速域（圆周角速率提至 2.5 rad/s）
    --condition noise              E6 测量噪声（16 bit 编码器量级）
    --condition contact            E7 接触扰动（内部后端：关节力矩脉冲；
                                   CoppeliaSim：场景椅背擦碰 + 接触力监测）

用法示例（TNDQ_sim 目录下）：

    python3 run_simulation.py                                        # 原版标称
    python3 run_simulation.py --plant torque --scenario setpoint     # S1 力矩级
    python3 run_simulation.py --plant torque --scenario cup-circle --condition highspeed
    python3 run_simulation.py --backend coppeliasim --scenario setpoint
    python3 run_simulation.py --backend coppeliasim --scenario cup-circle --condition l2

输出：results/*.csv（关键指标列，见 output/data_logger.py::save_csv）、
results/*.npz（全部原始数组）与终端定理 3 证书摘要。无绘图。
"""

import argparse
import time
import traceback

import numpy as np

from config import params
from config.lbr4_dynamics import (
    LBR4NominalDynamics, clip_torque, check_joint_limits, LBR4_JOINT_LIMITS,
)
from core.kinematics import TNDQSerialChain
from core.dq_algebra import dq_translation, dq_rotation
from control.error_system import full_error_state
from control.control_law import geometric_computed_torque_law, damped_pinv
from control.performance import (
    PerformanceAccumulator, check_hinf_condition_merged, iss_ultimate_bound,
    ResidualDisturbanceEstimator,
)
from simdata.trajectory_generator import (
    LineTrajectoryTNDQ, CircleTrajectoryTNDQ,
    SetpointTrajectoryTNDQ, CupCircleTrajectoryTNDQ,
    goto_trajectory, CompositeTrajectoryTNDQ,
)
from simdata.input_simulation import (
    L2Disturbance, BiasDisturbance, ZeroDisturbance, MeasurementNoise,
)
from output.data_logger import DataLogger


# ===========================================================================
# 被控对象抽象：三种后端统一为 read / apply / 推进 的最小接口
# ===========================================================================

class InternalAccelPlant:
    """式 (5.1) 加速度级理想对象：q̈ = q̈_ref + w_dyn，半隐式欧拉积分。
    这是论文核心实验的原版对象（定理 3 的直接语境），保留作回归基线。"""

    def __init__(self, q0, dt):
        self.q = np.asarray(q0, dtype=float).copy()
        self.q_dot = np.zeros_like(self.q)
        self.dt = dt

    def read(self):
        return self.q.copy(), self.q_dot.copy()

    def apply_accel(self, qddot):
        # 半隐式欧拉：先更新速度再更新位置（能量行为优于显式欧拉）
        self.q_dot = self.q_dot + qddot * self.dt
        self.q = self.q + self.q_dot * self.dt


class InternalTorquePlant:
    """力矩级刚体对象：M q̈ + C q̇ + g = τ + τ_ext（RNEA 正动力学）。

    对象端参数 = 名义表（mismatch_scale=1，视为"真值"）；
    E3 时控制器端另建一套高估 20% 的名义模型 -> 失配折算为 w_dyn，
    由定理 3(c)/(d) 证书兜底（总方案 §5.1 的失配源设计）。"""

    def __init__(self, q0, dt, dh_table):
        self.q = np.asarray(q0, dtype=float).copy()
        self.q_dot = np.zeros_like(self.q)
        self.dt = dt
        self.dyn = LBR4NominalDynamics(dh_table, mismatch_scale=1.0)

    def read(self):
        return self.q.copy(), self.q_dot.copy()

    def apply_torque(self, tau, tau_ext=None):
        if tau_ext is not None:
            tau = tau + tau_ext
        qddot = self.dyn.forward_dynamics(self.q, self.q_dot, tau)
        self.q_dot = self.q_dot + qddot * self.dt
        self.q = self.q + self.q_dot * self.dt


# ===========================================================================
# 场景与条件装配（场景篇 §4/§5/§6）
# ===========================================================================

def build_trajectory(scenario, chain, cup_pos, omega, q_init, t_go):
    """期望轨迹的 TNDQ 表示（式 3.3a）；σ² 通道 ξ̇_d 馈入 (5.2) 前馈。

    S1/S2 均先经 t_go 秒平滑趋近段（goto_trajectory，参考整形）：
    力矩模式下避免大误差瞬时指令超力矩/限位预算（场景篇 §5.1：
    "先经 S1 流程到达起点再切入跟踪，避免初始瞬态污染跟踪统计"）；
    t_go=0 时为纯调节压力测试（E4 大姿态误差实验的默认模式）。"""
    x0 = chain.fkm(q_init)
    if scenario == "line":
        return LineTrajectoryTNDQ(
            chain.fkm(params.Q_INIT), delta_p=[0.15, 0.10, -0.10],
            duration=6.0, rot_axis=[0.0, 0.0, 1.0], rot_angle=0.5)
    if scenario == "circle":
        return CircleTrajectoryTNDQ(
            chain.fkm(params.Q_INIT), radius=0.08, period=4.0, ramp_time=2.0)
    if scenario == "setpoint":
        # S1：杯口上方 SETPOINT_HEIGHT 的预抓取位姿，末端 z 轴竖直向下
        p_target = cup_pos + np.array([0.0, 0.0, params.SETPOINT_HEIGHT])
        if t_go > 0.0:
            # 趋近段到达后 s≡1、各阶导数为 0，自动退化为定点保持
            return goto_trajectory(x0, p_target, params.R_TOOL_DOWN, t_go)
        return SetpointTrajectoryTNDQ(p_target, params.R_TOOL_DOWN)
    if scenario == "cup-circle":
        # S2：圆心 = 杯口上方 CIRCLE_HEIGHT，水平圆周，姿态保持竖直向下；
        # 先趋近到圆心（圆轨迹 t=0 时 a=0 位于圆心），再切入圆周段
        center = cup_pos + np.array([0.0, 0.0, params.CIRCLE_HEIGHT])
        circle = CupCircleTrajectoryTNDQ(
            center, params.CIRCLE_RADIUS, omega, params.R_TOOL_DOWN,
            ramp_time=params.CIRCLE_RAMP_TIME)
        if t_go > 0.0:
            goto = goto_trajectory(x0, center, params.R_TOOL_DOWN, t_go)
            return CompositeTrajectoryTNDQ([(goto, 0.0), (circle, t_go)])
        return circle
    raise ValueError(f"unknown scenario '{scenario}'")


def build_condition(condition, n):
    """
    条件 -> (加速度级扰动 w_dyn, 测量噪声, 控制器模型失配因子)。
    w_dyn 语义（式 5.1）：力矩模式下经 M̂ 折算注入 τ += M̂ w（场景篇 §6.1），
    保证两类后端的扰动能量口径一致。
    """
    w_dyn = ZeroDisturbance(n)
    noise = None
    mismatch = 1.0
    if condition == "l2":                       # E2 -> 定理 3(c)
        w_dyn = L2Disturbance(n, amplitude=1.0, decay=0.5, omega=3.0, t_on=1.0)
    elif condition == "bias":                   # ISS -> 定理 3(d)
        w_dyn = BiasDisturbance(n, bias=0.5, amplitude=0.2, omega=2.0, t_on=1.0)
    elif condition == "mismatch":               # E3
        mismatch = params.MISMATCH_SCALE
    elif condition == "noise":                  # E6
        noise = MeasurementNoise(n, sigma_q=params.NOISE_SIGMA_Q,
                                 sigma_qdot=params.NOISE_SIGMA_QDOT)
    # large-error / highspeed / contact 在主流程中特殊处理
    return w_dyn, noise, mismatch


def contact_torque(t, J):
    """E7（内部后端）：椅背擦碰的等效关节力矩 τ_ext = Jᵀ [0; F_ext]
    （场景篇 §6.3：0.1–0.3 s 有限支撑力脉冲）。
    接触力 F_ext 作用在末端（纯力，无接触力矩），经几何雅可比转置
    映射到关节——力臂自动决定各关节分担，物理一致；
    半正弦包络保证 C0 连续。vec6 顺序 [ω; v] ⇒ wrench = [0; F]。"""
    t0, T = params.CONTACT_T_ON, params.CONTACT_DURATION
    if t0 <= t <= t0 + T:
        F = params.CONTACT_FORCE * np.sin(np.pi * (t - t0) / T)
        return J.T @ np.concatenate([np.zeros(3), F])
    return None


def joint_safety_governor(q, q_dot, qddot_ref):
    """关节级安全治理器（场景篇 §6.3 安全机制）：逐关节修正 q̈_ref。

    1) 速度限幅：|q̇_i| 超 LWR4+ 手册额定速度（params.QDOT_MAX）且指令
       仍在同向加速时，改为按 A_BRAKE 制动（允许自然减速）；
    2) 限位预测制动：若以最大制动减速度停车仍将进入限位缓冲区
       （刹车距离 d_stop = q̇²/(2 A_BRAKE)），强制反向减速。

    治理修正量是有界指令偏差，经式 (5.1) 归入 w_dyn → d(t)，由定理 3(c)/(d)
    的 H∞/ISS 证书兜底（诚实条款：触发即计数并写入汇总）。
    返回 (修正后 q̈_ref, 是否触发)。"""
    qdd = qddot_ref.copy()
    governed = False
    n = q.shape[0]
    soft_lim = LBR4_JOINT_LIMITS[:n] - params.LIMIT_BUFFER
    for i in range(n):
        # 1) 额定速度限幅
        if abs(q_dot[i]) > params.QDOT_MAX[i] and qdd[i] * q_dot[i] > 0.0:
            qdd[i] = -params.A_BRAKE * np.sign(q_dot[i])
            governed = True
        # 2) 限位预测制动（只对朝限位方向运动的关节生效）
        if q_dot[i] > 0.0:
            if q[i] + q_dot[i] ** 2 / (2.0 * params.A_BRAKE) > soft_lim[i]:
                qdd[i] = min(qdd[i], -params.A_BRAKE)
                governed = True
        elif q_dot[i] < 0.0:
            if q[i] - q_dot[i] ** 2 / (2.0 * params.A_BRAKE) < -soft_lim[i]:
                qdd[i] = max(qdd[i], params.A_BRAKE)
                governed = True
    return qdd, governed


# ===========================================================================
# 主流程
# ===========================================================================

def main():
    ap = argparse.ArgumentParser(description="TNDQ closed-loop simulation "
                                             "(internal / CoppeliaSim)")
    ap.add_argument("--backend", choices=["internal", "coppeliasim"],
                    default="internal")
    ap.add_argument("--plant", choices=["accel", "torque"], default="accel",
                    help="internal 后端的被控对象层级（coppeliasim 恒为力矩级）")
    ap.add_argument("--scenario",
                    choices=["line", "circle", "setpoint", "cup-circle"],
                    default=None, help="默认: internal->line, coppeliasim->setpoint")
    ap.add_argument("--condition",
                    choices=["none", "l2", "bias", "mismatch", "large-error",
                             "highspeed", "noise", "contact"],
                    default="none", help="实验条件 E1–E7（总方案 §5.3）")
    ap.add_argument("--t-end", type=float, default=params.T_END)
    ap.add_argument("--t-go", type=float, default=3.0,
                    help="S1/S2 平滑趋近段时长 [s]（0 = 纯调节压力测试）")
    # 兼容原版命令行（--trajectory/--disturbance 映射到新参数）
    ap.add_argument("--trajectory", choices=["line", "circle"], default=None,
                    help="[兼容] 等价于 --scenario")
    ap.add_argument("--disturbance", choices=["none", "l2", "bias"], default=None,
                    help="[兼容] 等价于 --condition")
    args = ap.parse_args()

    scenario = args.scenario or args.trajectory or (
        "setpoint" if args.backend == "coppeliasim" else "line")
    condition = args.condition if args.condition != "none" else \
        (args.disturbance or "none")

    chain = TNDQSerialChain(params.KUKA_LBR4_DH)
    n = chain.n

    # ---- 增益证书前置核验（定理 3(c-1)，式 5.6a） ---------------------------
    ok, lam_min, level = check_hinf_condition_merged(
        params.K_D, params.KAPPA, params.GAMMA_A)
    print(f"Gain condition (5.6a): lambda_min(K_d)={lam_min:.3f} "
          f">= {level:.3f} required -> {'OK' if ok else 'NOT SATISFIED'}")

    # ---- 条件装配（E1–E7） ---------------------------------------------------
    w_dyn, noise, mismatch = build_condition(condition, n)
    if condition == "bias":
        w_sup = w_dyn.sup_norm()
        print(f"A-priori ISS budget (5.7) with ||w||_inf~{w_sup:.2f}: "
              f"|e_xi| ball radius {iss_ultimate_bound(params.K_D, w_sup):.3e}")

    # 初始位形选择（场景篇 §4.1）：
    #   E4 大姿态误差 -> Q_INIT_LARGE_ERROR（姿态误差 ≥150°，unwinding 检验）；
    #   S1/S2 杯子任务 -> Q_INIT_TASK（前伸+工具朝下舒适位形，距目标姿态 ~9°，
    #     避免从竖直向上位形做 180° 翻转导致伪逆路径撞关节限位）；
    #   line/circle 回归基线 -> 原版 Q_INIT + 小偏置激发收敛过程。
    if condition == "large-error":
        q_init = params.Q_INIT_LARGE_ERROR.copy()
    elif scenario in ("setpoint", "cup-circle"):
        q_init = params.Q_INIT_TASK + 0.02 * np.array([1, -1, 1, -1, 1, -1, 1],
                                                      dtype=float)
    else:
        q_init = params.Q_INIT + 0.05 * np.array([1, -1, 1, -1, 1, -1, 1],
                                                 dtype=float)

    # 零空间居中参考：取任务初始位形而非限位中点 Q_CENTER=0
    # （2026-07 诊断：Q_CENTER=0 对前伸任务位形是强自运动吸引子，
    # 把 joint2/4 拉向竖直收拢位形，漂移到病态区后 joint6 被伪逆
    # 路径冲过 -120° 限位触发 [abort]；初始位形由 IK + 零空间居中
    # 求解，限位余量≥ 23°，是天然的安全姿态参考）
    q_nullspace_center = q_init.copy()

    # E5 高速域：圆周角速率提至 2.5 rad/s（J̇q̇ / Cq̇ 不可忽略域，场景篇 §5.1）
    omega = params.CIRCLE_OMEGA_FAST if condition == "highspeed" \
        else params.CIRCLE_OMEGA

    # ---- 控制器端名义动力学模型（§2.4 力矩层；E3 时与对象端失配） -------------
    # 理论依据：τ = M̂ q̈_ref + Ĉ q̇ + ĝ，M̂/Ĉ/ĝ 取 Gaz–Flacco–De Luca
    # LWR4+ 辨识模型（总方案文献 [11]）的名义参数表
    # CoppeliaSim 后端：接口层已把引擎关节 armature 写为同一张
    # LBR4_MOTOR_INERTIA 表（MuJoCo，2026-07 诊断固化），控制器用
    # 默认名义表即与引擎 M_total = M_links + diag(B) 严格匹配
    dyn_ctrl = LBR4NominalDynamics(params.KUKA_LBR4_DH, mismatch_scale=mismatch)

    # ---- 后端初始化 -----------------------------------------------------------
    interface = None
    torque_level = True          # 是否存在力矩层（accel 理想对象无 τ）
    if args.backend == "coppeliasim":
        from interfaces.coppeliasim_interface import (
            CoppeliaSimLBR4Interface, CoppeliaSimError,
        )
        interface = CoppeliaSimLBR4Interface()
        try:
            interface.connect(torque_mode=True,
                              engine_dt=params.COPPELIA_DT_TARGET)
        except CoppeliaSimError as exc:
            print(f"[error] CoppeliaSim 连接失败：{exc}")
            print("        请启动 CoppeliaSim 并加载 TNDQ_sim/KUKALBR4+_sim.ttt，"
                  "或改用 --backend internal。")
            return 1
        dt = interface.sim_dt or params.DT
        # 场景实时杯子位置（场景篇 §2：杯子 = 定点目标 / 圆心参照）
        cup_pos = interface.cup_position(default=params.CUP_POS_DEFAULT)
        # 初始位形写入引擎（仅初始化阶段允许直接设位）
        interface.set_joint_positions(q_init)
        interface.start()
        # FK 对齐诊断（场景篇 §1.2 第 9 项）：TNDQ FK vs 引擎末端位姿
        try:
            x_model = chain.fkm(q_init)
            x_sim = interface.read_tip_pose_dq(relative_to_base=True)
            dp = np.linalg.norm(dq_translation(x_model) - dq_translation(x_sim))
            dr = 2.0 * np.arccos(min(1.0, abs(
                float(dq_rotation(x_model) @ dq_rotation(x_sim)))))
            print(f"[check] FK 对齐残差: |Δp|={dp * 1e3:.2f} mm, "
                  f"Δθ={np.rad2deg(dr):.3f} deg "
                  f"{'OK' if dp < 1e-3 and dr < np.deg2rad(0.1) else '<- 超限，需核对 DH/基座位姿（场景篇 §1.2 第 8/9 项）'}")
        except Exception as exc:
            print(f"[warn] FK 对齐诊断失败（不影响运行）: {exc}")
        plant = None
    else:
        dt = params.DT
        cup_pos = params.CUP_POS_DEFAULT
        if args.plant == "torque":
            plant = InternalTorquePlant(q_init, dt, params.KUKA_LBR4_DH)
        else:
            plant = InternalAccelPlant(q_init, dt)
            torque_level = False
            if condition in ("mismatch", "contact"):
                print(f"[warn] 条件 '{condition}' 需要力矩层，加速度级理想对象"
                      f"下无效——请加 --plant torque")

    trajectory = build_trajectory(scenario, chain, cup_pos, omega,
                                  q_init, args.t_go)

    logger = DataLogger()
    perf = PerformanceAccumulator(params.K_D, params.K_P,
                                  params.KAPPA, params.GAMMA_A)
    # 证书通道等效扰动反演器（§6.5(6)）：注入项 J w 仅覆盖 l2/bias 工况，
    # 其余定理 3 诚实条款承认的扰动源（ΔM/Δg、噪声、伪逆残差、限幅/
    # 治理器、离散化）靠反演 e_ξ 动态拿到；纯诊断量，不进控制律
    d_est = ResidualDisturbanceEstimator(params.K_D, params.K_P, dt)

    n_steps = int(round(args.t_end / dt))
    print(f"Running {n_steps} steps ({args.t_end}s, dt={dt}s), "
          f"backend={args.backend}, plant="
          f"{'engine' if interface else args.plant}, "
          f"scenario={scenario}, condition={condition} ...")

    saturated_steps = 0
    governed_steps = 0
    contact_impulse = 0.0
    aborted = False

    try:
        for k in range(n_steps):
            t = k * dt

            # ---- 传感层：读取 (q, q̇) --------------------------------------
            if interface:
                q_true, q_dot_true = interface.read_joint_states()
            else:
                q_true, q_dot_true = plant.read()

            # E6 测量噪声：控制器只见带噪状态；噪声经 FK 进入 d(t)
            # （定理 3 的扰动通道，总方案 §6.2 噪声敏感度机制）
            if noise is not None:
                q_meas, q_dot_meas = noise(q_true, q_dot_true)
            else:
                q_meas, q_dot_meas = q_true, q_dot_true

            # ---- 安全监控：关节限位（LWR4+ 手册值） -------------------------
            over = check_joint_limits(q_true)
            if over:
                print(f"[abort] t={t:.3f}s 关节 {over} 超限位（LWR4+ 手册值，"
                      f"config/lbr4_dynamics.py::LBR4_JOINT_LIMITS），"
                      f"安全终止，数据已保存至当前步")
                aborted = True
                break

            # ---- 控制器（计时段：FK + 误差 + 控制律 + 力矩装配） -------------
            tic = time.perf_counter()

            # [FK 层] 测量链（式 3.4）；q̈ 未知 -> q̈=0 链给出 ξ 与 J̇q̇（式 3.5）
            fk = chain.fk_outputs(q_meas, q_dot_meas, q_ddot=None,
                                  with_jacobian=True)

            # [期望] TNDQ 轨迹（式 3.3a）-> HDQ 截断（命题 2 无损）
            des = trajectory.evaluate(t)

            # [误差层] 定理 1/2：HDQ 误差元素 -> e_ξ, e_z, A
            err = full_error_state(fk["x_breve"], des["x_breve_d"])

            # 奇异监控：σ_min(J) 过小时提升阻尼（残差计入 d(t)，
            # 定理 3 诚实条款 (i)——阻尼伪逆偏差是显式承认的扰动源）
            damping = params.PINV_DAMPING
            sig_min = np.linalg.svd(fk["J"], compute_uv=False)[-1]
            if sig_min < params.SINGULARITY_TOL:
                damping = params.SINGULARITY_DAMPING
                if k % 100 == 0:
                    print(f"[warn] t={t:.3f}s 接近奇异 sigma_min(J)="
                          f"{sig_min:.2e}，阻尼提升至 {damping}")

            # [控制层] 式 (5.2) 几何一致计算力矩律 -> q̈_ref
            qddot_ref, _ = geometric_computed_torque_law(
                err, des["xi_d"], des["xi_dot_d"],
                fk["J"], fk["Jdot_qdot"],
                params.K_D, params.K_P, damping=damping)

            # 零空间冗余消解（7R 臂 n=7 > m=6）：任务一致投影
            # N = I - J⁺J 内的关节居中 + 阻尼，推离 LWR4+ 限位；
            # 零空间分量不改变 ξ = Jq̇ 任务动态，定理 1–3 误差体系不受影响
            Jp = damped_pinv(fk["J"], damping=damping)
            N_proj = np.eye(n) - Jp @ fk["J"]
            qddot_ref = qddot_ref + N_proj @ (
                params.NULLSPACE_K * (q_nullspace_center - q_meas)
                - params.NULLSPACE_D * q_dot_meas)

            # 指令限幅：保护力矩/限位预算（饱和残差计入 d(t)，
            # 定理 3 诚实条款；触发即计数，场景篇 §6.3 记录规则）
            qn = np.linalg.norm(qddot_ref)
            if qn > params.QDDOT_MAX:
                qddot_ref = qddot_ref * (params.QDDOT_MAX / qn)
                saturated_steps += 1

            # 关节级安全治理：速度限幅 + 限位预测制动（场景篇 §6.3；
            # E4 大姿态误差纯调节时防止伪逆路径高速冲过限位）
            qddot_ref, governed = joint_safety_governor(
                q_meas, q_dot_meas, qddot_ref)
            governed_steps += int(governed)

            # [力矩层] τ = M̂ q̈_ref + Ĉ q̇ + ĝ（§2.4，Gaz [11] 名义模型）
            tau = None
            if torque_level:
                tau = dyn_ctrl.computed_torque(q_meas, q_dot_meas, qddot_ref)

            runtime = time.perf_counter() - tic

            # ---- 扰动注入与指令下发 ------------------------------------------
            w = w_dyn(t)
            if interface:
                # CoppeliaSim：w 经 M̂ 折算注入力矩通道（场景篇 §6.1，
                # 保证与加速度级口径一致）；力矩限幅 = 安全上限（§6.3）
                if np.any(w):
                    tau = tau + dyn_ctrl.mass_matrix(q_meas) @ w
                tau, sat = clip_torque(tau)
                saturated_steps += int(sat)
                interface.send_joint_targets(tau, mode="torque")
                interface.step()
                # E7：接触力监测（场景椅背擦碰，sim.getContactInfo）
                if condition == "contact":
                    contact_impulse += interface.read_contact_force_norm() * dt
            elif torque_level:
                if np.any(w):
                    tau = tau + plant.dyn.mass_matrix(q_true) @ w
                tau, sat = clip_torque(tau)
                saturated_steps += int(sat)
                # E7（内部）：接触力经 Jᵀ 折算的等效力矩直接作用在对象端
                plant.apply_torque(tau, tau_ext=contact_torque(t, fk["J"])
                                   if condition == "contact" else None)
            else:
                # 原版加速度级对象（式 5.1）
                plant.apply_accel(qddot_ref + w)

            # ---- 监控层 --------------------------------------------------------
            # d̂ = 反演的证书通道等效扰动（全部源），d_inj = J w 仅注入分量；
            # (5.6)/(5.7) 的判据走 d̂，旧口径的 d_inj 并行保留供对照（E4）
            d_hat = d_est.update(err["e_xi"], err["e_z"], err["A"])
            d_inj = fk["J"] @ w        # 等效任务空间注入扰动 d = J w
            V = perf.update(err["e_xi"], err["e_z"], d_hat, dt,
                            d_injected=d_inj)

            if fk["c0"] > params.C0_TOL or fk["c1"] > params.C1_TOL \
                    or fk["c2"] > params.C2_TOL:
                print(f"[warn] t={t:.3f}: constraint residuals (3.8) above tol: "
                      f"c0={fk['c0']:.2e} c1={fk['c1']:.2e} c2={fk['c2']:.2e}")

            if k % params.LOG_EVERY == 0:
                logger.log(
                    t=t, e_z=err["e_z"], e_xi=err["e_xi"],
                    qddot_ref=qddot_ref, tau=tau,
                    x_d=des["x_d"], xi_d=des["xi_d"], xi_dot_d=des["xi_dot_d"],
                    x=fk["x"], xi=fk["xi"],
                    V=V, c0=fk["c0"], c1=fk["c1"], c2=fk["c2"],
                    runtime=runtime, d_hat=d_hat)

    except KeyboardInterrupt:
        print("\n[interrupt] 用户中断，保存已记录数据后安全退出 ...")
        aborted = True
    except Exception:
        print("[error] 仿真循环异常，安全断开后保留已有数据：")
        traceback.print_exc()
        aborted = True
    finally:
        # 异常恢复机制：任何退出路径都先把力矩清零再停仿真（场景篇 §8）
        if interface:
            interface.disconnect()

    # ---- 输出 ------------------------------------------------------------------
    if not logger.as_arrays()["t"].size:
        print("[error] 无有效数据（首步即失败），不生成输出文件。")
        return 1

    tag = f"{args.backend}_{scenario}_{condition}"
    npz_path = logger.save_npz(f"results/tndq_{tag}.npz",
                               extra={"K_d": params.K_D, "k_p": params.K_P,
                                      "dt": dt})
    csv_path = logger.save_csv(f"results/tndq_{tag}.csv")
    print(f"Saved raw data  : {npz_path}")
    print(f"Saved CSV       : {csv_path}")

    if saturated_steps:
        print(f"[note] 力矩饱和步数: {saturated_steps}/{n_steps} "
              f"（场景篇 §6.3：任何控制器触发饱和均须记录）")
    if governed_steps:
        print(f"[note] 安全治理器触发步数: {governed_steps}/{n_steps} "
              f"（速度限幅/限位预测制动，修正量计入 d(t)）")
    if condition == "contact" and interface:
        print(f"[note] 接触力冲量累计（扰动能量估计）: {contact_impulse:.3f} N·s")
    if aborted:
        print("[note] 本次运行提前终止，统计量基于已完成步数。")

    logger.print_summary(performance_summary=perf.summary())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
