"""公平对比指标组合实验：C1 (5.2) vs C2 / C2-abl / C3，同一力矩出口。

从任何目录运行：python TNDQ_sim/experiments/compare_metrics_portfolio.py

目的
----
把"合理公平对比指标"组成一个组合（portfolio），在内部力矩级对象
（RNEA 正动力学 + 摩擦 + 力矩限幅，无需 CoppeliaSim）上对四条控制律
统一计算，回答"在哪种指标下 C1 有优势"：

    C1     论文式 (5.2) 几何一致计算力矩律（GAIN_SETS["tuned"]）
    C2     忠实 [Ch20] resolved-acceleration 律（screw-log 位姿反馈）
    C2-abl 朴素 twist 差消融律（差分前馈，无 Ad 输运 / 无 A^T 整形）
    C3     一阶 DQ H-inf 运动学律 + 内环速度伺服（velocity_to_accel_ref）

公平性约束（总方案 5.2，与 run_grasp_circle.py 同构）：
  - 同一被控对象（RNEA 正动力学 + 摩擦）、同一积分器、同一 dt；
  - 同一力矩出口 tau = M qddot_ref + C qdot + g、同一名义模型 M/C/g；
  - 同一伪逆阻尼/奇异升阻尼逻辑、同一零空间投影、同一 qddot/tau 限幅
    与关节级安全治理器；
  - 四律增益取 params.py 的"逐通道线性化配平"组（同极点 {-4,-20}、
    同 DC 刚度 80、同阻尼），预算差异本身作为数据报告；
  - Pareto 扫描（lambda 乘全部反馈/位姿增益）给出"等性能比努力"的
    最严格口径，不依赖单点配平。

指标组合（合理性依据见 docs/metric_portfolio_20260925）：

  A. 执行器负担（"力矩从 0 到 T 积分"一族的严格化）
     E_tau_impulse  int sum(tau_i) dt  签名冲量——反例指标：被重力主导，
                                       与控制律几乎无关（表中可见退化）
     E_tau1         int ||tau||_1 dt   L1 努力（稀疏/燃料型）
     E_tau2         int ||tau||^2 dt   焦耳热 / RMS 力矩（标准控制努力）
     E_nongrav      int ||tau - g||^2  重力补偿之外的努力（剔除各律共享
                                       的持重基座项——公平性关键）
     W_pos/W_neg    int max(±tau·qdot,0) dt  正/制动机械功（能耗/再生）
     W_net          int tau·qdot dt    净机械功
     tau_peak       max |tau_i|        峰值力矩（驱动器余量）
     tau_slew_rms   RMS ||dtau/dt||    力矩抖振（减速箱磨损/声噪）
     E_qddot        int ||qddot_ref||^2 dt  共享接口入口处的指令能量
     sat_frac       力矩饱和时间占比；qdd_sat_frac 指令限幅占比
     gov_frac       安全治理器触发占比

  B. 跟踪/调节性能（用同样的努力买到了什么）
     p_rms/a_rms    全程与稳态窗 RMS；IAE = int ||e|| dt（全程与稳态窗）
     p_peak/a_peak  峰值误差；settle_s 进入 5 mm/2 deg 带并保持的时间
     p_end/a_end    终端误差

  C. 证书类（定理 3 的经验对应物）
     gamma_xi/gamma_ez  l2 工况 ||e_xi||_L2/||d||_L2 与 ||e_z|| 版本
     bias_ball          bias 工况末段误差球半径（ISS (5.7)）

  D. 结构敏感工况（把结构差异推到可观测域，S3 同思路）
     l2 / bias / noise / mismatch / friction2 / contact / coarse-dt
     coarse-dt：控制周期 x5（ZOH），差分前馈一拍滞后 x5，解析前馈
     （C1/C2）不受影响——"无二阶通道"缺点的放大镜。

输出：results/metrics_portfolio.csv、results/metrics_portfolio_pareto.csv
"""

import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import params
from config.lbr4_dynamics import (
    LBR4NominalDynamics, LBR4_VISCOUS_FRICTION, LBR4_COULOMB_FRICTION,
    LBR4_JOINT_LIMITS, clip_torque, check_joint_limits,
)
from core.kinematics import TNDQSerialChain
from core.dq_algebra import dq_translation, dq_rotation
from control.error_system import full_error_state
from control.control_law import (
    geometric_computed_torque_law, dq_chandra2020_law, dq_ctc_law,
    dq_hinf_kinematic_law, damped_pinv, velocity_to_accel_ref,
)
from simdata.trajectory_generator import (
    LineTrajectoryTNDQ, CupCircleTrajectoryTNDQ, SetpointTrajectoryTNDQ,
)
from simdata.input_simulation import (
    L2Disturbance, BiasDisturbance, MeasurementNoise,
)

DT = 1e-3
ROOT = os.path.join(os.path.dirname(__file__), "..", "results")

# ---------------------------------------------------------------------------
# 性能补丁（仅本进程，不改仓库文件）：RNEA 中的 np.cross 对 3 维向量调用
# 占单步耗时 ~77%（numpy 通用实现的 moveaxis/normalize 开销）。先在随机
# 向量上验证与原实现逐位一致，再进程内替换。仅影响本实验的运行速度。
# ---------------------------------------------------------------------------
_np_cross_orig = np.cross


def _fast_cross3(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return np.array([a[1] * b[2] - a[2] * b[1],
                     a[2] * b[0] - a[0] * b[2],
                     a[0] * b[1] - a[1] * b[0]])


def _enable_fast_cross():
    rng = np.random.default_rng(0)
    for _ in range(200):
        u = rng.standard_normal(3)
        v = rng.standard_normal(3)
        assert np.array_equal(_fast_cross3(u, v), _np_cross_orig(u, v))
    np.cross = _fast_cross3


_enable_fast_cross()

LAWS = ("C1", "C2", "C2abl", "C3")
LAW_TAGS = {"C1": "tndq(5.2)", "C2": "chandra20", "C2abl": "ctc-abl",
            "C3": "hinf-vel"}

# 指标方向：全部为 lower-is-better（能量/努力/误差类）
KEY_METRICS = (
    # A 组：执行器负担
    "E_nongrav", "E_tau2", "E_tau1", "W_pos", "tau_peak", "tau_slew_rms",
    "E_qddot", "sat_frac",
    # B 组：性能
    "IAE_p", "IAE_a_deg", "IAE_p_ss", "a_rms_ss_deg", "p_peak",
    "a_peak_deg", "settle_s",
    # C 组：证书类
    "gamma_xi", "bias_ball",
)


# ---------------------------------------------------------------------------
# 场景与工况装配
# ---------------------------------------------------------------------------

def build_scenario(scenario, chain):
    """返回 (q_init, trajectory, t_end, ss_window)。

    circle : 连续跟踪（周期激励，前馈通道可观测）
    line   : 小误差点到点 + 保持（回归基线）
    large  : E4 型大姿态误差纯调节（screw-log 映射大误差域的压力测试）
    """
    x_target = chain.fkm(params.Q_INIT)
    if scenario == "circle":
        # S2 杯口圆周（tuned 增益组的整定/验证场景，整圈 IK 余量已验证；
        # 原版竖直圆 in 近奇异区会使伪逆爆炸，不适用于二阶律对比）
        q_init = params.Q_INIT_TASK + 0.02 * np.array([1, -1, 1, -1, 1, -1, 1])
        center = params.CUP_POS_DEFAULT + np.array([0.0, 0.0, params.CIRCLE_HEIGHT])
        traj = CupCircleTrajectoryTNDQ(center, params.CIRCLE_RADIUS,
                                       params.CIRCLE_OMEGA, params.R_TOOL_DOWN,
                                       params.CIRCLE_RAMP_TIME)
        return q_init.copy(), traj, 10.0, (6.0, 10.0)
    if scenario == "line":
        q_init = params.Q_INIT + 0.05 * np.array([1, -1, 1, -1, 1, -1, 1])
        traj = LineTrajectoryTNDQ(x_target, [0.15, 0.10, -0.10], 3.5,
                                  [0., 0., 1.], 0.5)
        return q_init.copy(), traj, 8.0, (5.0, 8.0)
    if scenario == "large":
        q_init = params.Q_INIT_LARGE_ERROR.copy()
        traj = SetpointTrajectoryTNDQ(dq_translation(x_target),
                                      dq_rotation(x_target))
        return q_init, traj, 8.0, (5.0, 8.0)
    raise ValueError(scenario)


class _NullW:
    def __call__(self, t):
        return np.zeros(7)


def build_condition(condition):
    """工况 -> 扰动源/测量噪声/失配/摩擦/控制分频/接触开关。"""
    cfg = dict(w=_NullW(), noise=None, ctrl_mismatch=1.0, friction_scale=1.0,
               decim=1, contact=False)
    if condition == "l2":
        cfg["w"] = L2Disturbance(7, amplitude=1.0, decay=0.5, omega=3.0,
                                 t_on=1.0, seed=0)
    elif condition == "bias":
        cfg["w"] = BiasDisturbance(7, bias=0.5, amplitude=0.2, omega=2.0,
                                   t_on=1.0, seed=1)
    elif condition == "noise":
        cfg["noise"] = MeasurementNoise(7, sigma_q=params.NOISE_SIGMA_Q,
                                        sigma_qdot=params.NOISE_SIGMA_QDOT)
    elif condition == "mismatch":       # 控制器名义惯性高估 20%（E3）
        cfg["ctrl_mismatch"] = params.MISMATCH_SCALE
    elif condition == "friction2":      # 未建模摩擦 x2（敏感性档）
        cfg["friction_scale"] = 2.0
    elif condition == "contact":        # E7 接触脉冲（内部后端口径）
        cfg["contact"] = True
    elif condition == "coarse-dt":      # 控制周期 x5（S3 敏感条件）
        cfg["decim"] = 5
    else:
        assert condition == "none", condition
    return cfg


def contact_wrench(t):
    """E7 椅背擦碰等效接触力（基座系，N），半正弦包络。"""
    t0, T = params.CONTACT_T_ON, params.CONTACT_DURATION
    if t0 <= t <= t0 + T:
        return params.CONTACT_FORCE * np.sin(np.pi * (t - t0) / T)
    return None


def gain_pack(law, scale, c1_gain_set="tuned"):
    """各律的反馈/位姿增益（lambda 同乘全部通道；K_SERVO 不动）。

    ``c1_gain_set`` is explicit so budget-matched studies can use the
    original paper ``base`` gains and tune only against an observed fairness
    metric.  The default keeps the historical portfolio script unchanged.
    """
    if law == "C1":
        g = params.GAIN_SETS[c1_gain_set]
        return dict(kind="c1", K_d=g["K_d"] * scale, K_p=g["k_p"] * scale)
    if law == "C2":
        return dict(kind="c2", K_v=params.CH20_K_V * scale,
                    K_P=params.CH20_K_P * scale)
    if law == "C2abl":
        return dict(kind="c2abl", K_d=params.DQC_K_D * scale,
                    K_p=params.DQC_K_P * scale)
    if law == "C3":
        return dict(kind="c3",
                    gamma_O=params.DQH_GAMMA_O / scale,
                    gamma_T=params.DQH_GAMMA_T / scale)
    raise ValueError(law)


def governor(q, q_dot, qddot_ref):
    """关节级安全治理器（run_simulation.joint_safety_governor 同构）。"""
    qdd = qddot_ref.copy()
    governed = False
    soft_lim = LBR4_JOINT_LIMITS[:q.shape[0]] - params.LIMIT_BUFFER
    for i in range(q.shape[0]):
        if abs(q_dot[i]) > params.QDOT_MAX[i] and qdd[i] * q_dot[i] > 0.0:
            qdd[i] = -params.A_BRAKE * np.sign(q_dot[i])
            governed = True
        if q_dot[i] > 0.0 and \
                q[i] + q_dot[i] ** 2 / (2.0 * params.A_BRAKE) > soft_lim[i]:
            qdd[i] = min(qdd[i], -params.A_BRAKE)
            governed = True
        elif q_dot[i] < 0.0 and \
                q[i] - q_dot[i] ** 2 / (2.0 * params.A_BRAKE) < -soft_lim[i]:
            qdd[i] = max(qdd[i], params.A_BRAKE)
            governed = True
    return qdd, governed


# ---------------------------------------------------------------------------
# 单次闭环仿真 + 指标核算
# ---------------------------------------------------------------------------

def simulate(law, scenario, condition="none", scale=1.0, t_end=None,
             c1_gain_set="tuned", return_trace=False):
    cfg = build_condition(condition)
    chain = TNDQSerialChain(params.KUKA_LBR4_DH)
    n = chain.n
    q_init, trajectory, t_horizon, ss_win = build_scenario(scenario, chain)
    if t_end is not None:              # 预算标定用的短时程覆盖
        t_horizon = float(t_end)
        ss_win = (0.75 * t_horizon, t_horizon)
    gains = gain_pack(law, scale, c1_gain_set=c1_gain_set)

    # 对象端（"真值"）与控制器端名义模型（mismatch 条件下失配 20%）
    plant = LBR4NominalDynamics(
        params.KUKA_LBR4_DH, mismatch_scale=1.0, friction=True,
        viscous_friction=cfg["friction_scale"] * LBR4_VISCOUS_FRICTION,
        coulomb_friction=cfg["friction_scale"] * LBR4_COULOMB_FRICTION)
    dyn_ctrl = LBR4NominalDynamics(params.KUKA_LBR4_DH,
                                   mismatch_scale=cfg["ctrl_mismatch"])

    q = q_init.copy()
    qdot = np.zeros(n)
    q_center = q_init.copy()
    decim = cfg["decim"]
    dt_ctrl = DT * decim
    noise, w_dyn = cfg["noise"], cfg["w"]

    # 律状态
    qdot_cmd_prev = None       # C3 差分前馈状态
    xi_d_prev = J_prev = None  # C2abl 差分状态

    # 指标累加器
    acc = dict(tau_sq=0.0, tau_l1=0.0, tau_imp=0.0, ng_sq=0.0,
               w_pos=0.0, w_neg=0.0, w_net=0.0, qdd_sq=0.0,
               d_sq=0.0, d_inj_sq=0.0, exi_sq=0.0, ez_sq=0.0,
               slew_sq=0.0, n_slew=0)
    cnt = dict(sat=0, qdd_sat=0, gov=0, steps=0)
    tau_peak = np.zeros(n)
    err_log = []               # (t, |T|, theta_deg, in_ss)
    trace = None
    if return_trace:
        trace = dict(
            t=[], p_err=[], a_err_deg=[], tau_norm=[], tau_peak_inst=[],
            tau_slew_norm=[], qddot_norm=[], nongrav_norm=[],
            p_iae_cum=[], a_iae_cum=[], E_nongrav_cum=[], E_qddot_cum=[])
    trace_ng = 0.0
    trace_qdd = 0.0
    trace_p_iae = 0.0
    trace_a_iae = 0.0
    trace_prev_tau = None
    trace_last_p = np.nan
    trace_last_a = np.nan
    ever_inside = False        # settle 统计（调节场景）
    last_violation = None
    aborted = False
    prev_tau = None
    tau_zoh = np.zeros(n)
    qdd_zoh = np.zeros(n)
    grav_dyn = LBR4NominalDynamics(params.KUKA_LBR4_DH)  # 指标端真值 g(q)

    for k in range(int(round(t_horizon / DT))):
        t = k * DT

        # ---- 对象端外生信号（每步都作用，与控制分频无关） -----------------
        w = w_dyn(t)
        F = contact_wrench(t) if cfg["contact"] else None

        if k % decim == 0:
            # ---- 控制步：传感（noise 条件下只见带噪测量）+ 律 + 力矩出口 --
            if noise is not None:
                qm, vm = noise(q, qdot)
            else:
                qm, vm = q, qdot
            fk = chain.fk_outputs(qm, vm, q_ddot=None, with_jacobian=True)
            des = trajectory.evaluate(t)
            err = full_error_state(fk["x_breve"], des["x_breve_d"])

            damping = params.PINV_DAMPING
            if np.linalg.svd(fk["J"], compute_uv=False)[-1] \
                    < params.SINGULARITY_TOL:
                damping = params.SINGULARITY_DAMPING

            if law == "C1":
                qddot_ref, _ = geometric_computed_torque_law(
                    err, des["xi_d"], des["xi_dot_d"],
                    fk["J"], fk["Jdot_qdot"],
                    gains["K_d"], gains["K_p"], damping=damping)
            elif law == "C2":
                qddot_ref, _ = dq_chandra2020_law(
                    err, des["xi_d"], des["xi_dot_d"],
                    fk["J"], fk["Jdot_qdot"],
                    gains["K_v"], gains["K_P"], damping=damping)
            elif law == "C2abl":
                xi_dot_d_num = (np.zeros(6) if xi_d_prev is None
                                else (des["xi_d"] - xi_d_prev) / dt_ctrl)
                Jdot_qdot_num = (np.zeros(6) if J_prev is None
                                 else (fk["J"] - J_prev) / dt_ctrl @ vm)
                qddot_ref, _ = dq_ctc_law(
                    err, fk["xi"], des["xi_d"], xi_dot_d_num, Jdot_qdot_num,
                    fk["J"], gains["K_d"], gains["K_p"], damping=damping)
                xi_d_prev = np.asarray(des["xi_d"], float).copy()
                J_prev = fk["J"].copy()
            else:
                task_vel = dq_hinf_kinematic_law(
                    err, des["xi_d"], gains["gamma_O"], gains["gamma_T"])
                qdot_cmd = damped_pinv(
                    fk["J"], damping=max(damping, params.DQH_DAMPING)) @ task_vel
                qddot_ref = velocity_to_accel_ref(
                    qdot_cmd, qdot_cmd_prev, vm, dt_ctrl, params.DQH_K_SERVO)
                qdot_cmd_prev = qdot_cmd.copy()

            # 共享后处理：零空间投影 + 指令限幅 + 安全治理器
            Jp = damped_pinv(fk["J"], damping=damping)
            qddot_ref = qddot_ref + (np.eye(n) - Jp @ fk["J"]) @ (
                params.NULLSPACE_K * (q_center - qm) - params.NULLSPACE_D * vm)
            qn = np.linalg.norm(qddot_ref)
            if qn > params.QDDOT_MAX:
                qddot_ref *= params.QDDOT_MAX / qn
                cnt["qdd_sat"] += 1
            qddot_ref, govd = governor(q, qdot, qddot_ref)
            cnt["gov"] += int(govd)

            tau_cmd = dyn_ctrl.computed_torque(qm, vm, qddot_ref)
            tau_cmd, sat = clip_torque(tau_cmd)
            cnt["sat"] += int(sat)
            tau_zoh, qdd_zoh = tau_cmd, qddot_ref

            # ---- 控制步采样的指标（误差类 + 接口入口能量） ----------------
            p = float(np.linalg.norm(err["T"]))
            ang = float(np.rad2deg(2.0 * np.arctan2(
                np.linalg.norm(err["x_tilde"][1:4]),
                max(0.0, err["x_tilde"][0]))))
            err_log.append((t, p, ang, ss_win[0] <= t <= ss_win[1]))
            trace_last_p = p
            trace_last_a = ang
            if scenario != "circle":
                if p < 0.005 and ang < 2.0:
                    ever_inside = True
                else:
                    last_violation = t
            if np.any(w):
                d_inj = fk["J"] @ w
                acc["d_inj_sq"] += float(d_inj @ d_inj) * DT
            if t >= 1.0 and condition == "l2":
                acc["exi_sq"] += float(err["e_xi"] @ err["e_xi"]) * DT
                acc["ez_sq"] += float(err["e_z"] @ err["e_z"]) * DT
            if prev_tau is not None:
                slew = (tau_cmd - prev_tau) / dt_ctrl
                acc["slew_sq"] += float(slew @ slew)
                acc["n_slew"] += 1
            prev_tau = tau_cmd.copy()
        else:
            # ---- 非控制步：ZOH 重发上一拍力矩（扰动/接触仍逐步作用） ------
            tau_cmd = tau_zoh

        # ---- 对象端物理（每步）：扰动 + 接触 + 限幅 + RNEA 正动力学 ---------
        tau_ext = np.zeros(n)
        if np.any(w):
            tau_ext += plant.mass_matrix(q) @ w
        if F is not None:
            tau_ext += chain.jacobian(q).T @ np.concatenate([np.zeros(3), F])

        acc["tau_sq"] += float(tau_cmd @ tau_cmd) * DT
        acc["tau_l1"] += float(np.sum(np.abs(tau_cmd))) * DT
        acc["tau_imp"] += float(np.sum(tau_cmd)) * DT
        acc["qdd_sq"] += float(qdd_zoh @ qdd_zoh) * DT
        ng = tau_cmd - grav_dyn.gravity_vector(q)
        acc["ng_sq"] += float(ng @ ng) * DT
        power = float(tau_cmd @ qdot)
        acc["w_pos"] += max(0.0, power) * DT
        acc["w_neg"] += max(0.0, -power) * DT
        acc["w_net"] += power * DT
        tau_peak[:] = np.maximum(tau_peak, np.abs(tau_cmd))
        acc["d_sq"] += float(w @ w) * DT

        if return_trace:
            if trace_prev_tau is None:
                slew_trace = 0.0
            else:
                slew_trace = float(np.linalg.norm((tau_cmd - trace_prev_tau) / DT))
            ng_norm = float(np.linalg.norm(ng))
            qdd_norm = float(np.linalg.norm(qdd_zoh))
            trace_p_iae += (0.0 if not np.isfinite(trace_last_p)
                            else trace_last_p) * DT
            trace_a_iae += (0.0 if not np.isfinite(trace_last_a)
                            else trace_last_a) * DT
            trace_ng += float(ng @ ng) * DT
            trace_qdd += float(qdd_zoh @ qdd_zoh) * DT
            trace["t"].append(t)
            trace["p_err"].append(trace_last_p)
            trace["a_err_deg"].append(trace_last_a)
            trace["tau_norm"].append(float(np.linalg.norm(tau_cmd)))
            trace["tau_peak_inst"].append(float(np.max(np.abs(tau_cmd))))
            trace["tau_slew_norm"].append(slew_trace)
            trace["qddot_norm"].append(qdd_norm)
            trace["nongrav_norm"].append(ng_norm)
            trace["p_iae_cum"].append(trace_p_iae)
            trace["a_iae_cum"].append(trace_a_iae)
            trace["E_nongrav_cum"].append(trace_ng)
            trace["E_qddot_cum"].append(trace_qdd)
            trace_prev_tau = tau_cmd.copy()

        qdd = plant.forward_dynamics(q, qdot, tau_cmd + tau_ext)
        qdot = qdot + qdd * DT
        q = q + qdot * DT
        cnt["steps"] += 1

        if check_joint_limits(q):
            aborted = True
            break

    # ---- 汇总 --------------------------------------------------------------
    T_run = max(cnt["steps"], 1) * DT
    if err_log:
        t_arr = np.array([e[0] for e in err_log])
        p_arr = np.array([e[1] for e in err_log])
        a_arr = np.array([e[2] for e in err_log])
        ss = np.array([e[3] for e in err_log], bool)
        spacing = t_arr[1] - t_arr[0] if len(t_arr) > 1 else DT

        def _iae(vals, mask):
            return float(np.sum(vals[mask]) * spacing) if mask.any() else np.nan

        p_rms = float(np.sqrt(np.mean(p_arr ** 2)))
        a_rms = float(np.sqrt(np.mean(a_arr ** 2)))
        p_rms_ss = float(np.sqrt(np.mean(p_arr[ss] ** 2))) if ss.any() else np.nan
        a_rms_ss = float(np.sqrt(np.mean(a_arr[ss] ** 2))) if ss.any() else np.nan
        p_end, a_end = float(p_arr[-1]), float(a_arr[-1])
        p_peak, a_peak = float(np.max(p_arr)), float(np.max(a_arr))
        iae_p, iae_a = _iae(p_arr, ss | True), _iae(a_arr, ss | True)
        iae_p_ss, iae_a_ss = _iae(p_arr, ss), _iae(a_arr, ss)
        if ever_inside and p_arr[-1] < 0.005 and a_arr[-1] < 2.0:
            settle = 0.0 if last_violation is None else last_violation + spacing
        else:
            settle = np.nan
    else:
        p_rms = a_rms = p_rms_ss = a_rms_ss = np.nan
        p_end = a_end = p_peak = a_peak = np.nan
        iae_p = iae_a = iae_p_ss = iae_a_ss = np.nan
        settle = np.nan

    row = dict(scenario=scenario, condition=condition, law=law, scale=scale,
               law_tag=LAW_TAGS[law], aborted=int(aborted), steps=cnt["steps"])
    row.update(
        E_tau_impulse=acc["tau_imp"],
        E_tau1=acc["tau_l1"],
        E_tau2=acc["tau_sq"],
        E_nongrav=acc["ng_sq"],
        W_pos=acc["w_pos"], W_neg=acc["w_neg"], W_net=acc["w_net"],
        tau_peak=float(np.max(tau_peak)),
        tau_rms=float(np.sqrt(acc["tau_sq"] / T_run)),
        tau_slew_rms=float(np.sqrt(acc["slew_sq"] / max(acc["n_slew"], 1))),
        E_qddot=acc["qdd_sq"],
        sat_frac=cnt["sat"] / max(cnt["steps"], 1),
        qdd_sat_frac=cnt["qdd_sat"] / max(cnt["steps"], 1),
        gov_frac=cnt["gov"] / max(cnt["steps"], 1),
        p_rms=p_rms, a_rms_deg=a_rms,
        p_rms_ss=p_rms_ss, a_rms_ss_deg=a_rms_ss,
        IAE_p=iae_p, IAE_a_deg=iae_a, IAE_p_ss=iae_p_ss, IAE_a_ss_deg=iae_a_ss,
        p_peak=p_peak, a_peak_deg=a_peak, p_end=p_end, a_end_deg=a_end,
        settle_s=settle,
    )
    if condition == "l2" and acc["d_inj_sq"] > 1e-12:
        row["gamma_xi"] = float(np.sqrt(acc["exi_sq"] / acc["d_inj_sq"]))
        row["gamma_ez"] = float(np.sqrt(acc["ez_sq"] / acc["d_inj_sq"]))
    if condition == "bias" and err_log:
        tail = t_arr >= (t_horizon - 2.0)
        if tail.any():
            row["bias_ball"] = float(np.sqrt(np.mean(
                p_arr[tail] ** 2 + np.deg2rad(a_arr[tail]) ** 2)))
    if return_trace:
        return row, {key: np.asarray(value) for key, value in trace.items()}
    return row


def _run_job(job):
    """multiprocessing worker：job = (law, scenario, condition, scale)。"""
    return simulate(*job)


# ---------------------------------------------------------------------------
# 汇总打印
# ---------------------------------------------------------------------------

def _print_verdict(rows, pareto):
    print("\n" + "=" * 78)
    print("预算核对（circle/none, lambda=1：E_nongrav 相对 C1 的比值）")
    print("=" * 78)
    base = next(r for r in rows if r["scenario"] == "circle"
                and r["condition"] == "none" and r["law"] == "C1")
    for law in LAWS:
        r = next(r for r in rows if r["scenario"] == "circle"
                 and r["condition"] == "none" and r["law"] == law)
        print(f"  {law:<5} E_nongrav ratio = {r['E_nongrav'] / base['E_nongrav']:.4f}"
              f"   IAE_p_ss ratio = {r['IAE_p_ss'] / base['IAE_p_ss']:.4f}")

    print("\n" + "=" * 78)
    print("逐工况逐指标胜者（lower = better；* = C1 胜出）")
    print("=" * 78)
    wins = {law: 0 for law in LAWS}
    groups = {}
    for r in rows:
        groups.setdefault((r["scenario"], r["condition"]), []).append(r)
    for (scenario, condition), grp in sorted(groups.items()):
        line = []
        for m in KEY_METRICS:
            vals = [(r[m], r["law"]) for r in grp
                    if m in r and not np.isnan(r[m])]
            if not vals:
                continue
            best = min(vals)
            wins[best[1]] += 1
            if best[1] == "C1":
                line.append(m + "*")
        print(f"{scenario:>6}/{condition:<9}: C1 -> "
              f"{', '.join(line) if line else '(none)'}")
    print("\nwin tally:", wins)

    print("\n" + "=" * 78)
    print("Pareto 支配检查（circle: (E_nongrav, IAE_p_ss) 前沿，1% 容差）")
    print("=" * 78)
    for condition in ("none", "l2"):
        c1 = [(r["E_nongrav"], r["IAE_p_ss"])
              for r in pareto if r["law"] == "C1" and r["condition"] == condition]
        for law in ("C2", "C2abl", "C3"):
            dominated = total = 0
            for r in pareto:
                if r["law"] != law or r["condition"] != condition:
                    continue
                total += 1
                if any(e1 <= r["E_nongrav"] * 1.01 and i1 <= r["IAE_p_ss"] * 1.01
                       for e1, i1 in c1):
                    dominated += 1
            print(f"  {condition:>3} {law:<5}: C1 支配 {dominated}/{total} 个前沿点")


def main():
    matrix = {
        "circle": ["none", "l2", "noise", "mismatch", "friction2",
                   "contact", "coarse-dt"],
        "line": ["none", "bias", "l2"],
        "large": ["none"],
    }
    jobs = [(law, sc, cond, 1.0)
            for sc, conds in matrix.items() for cond in conds for law in LAWS]
    # Pareto 扫描：lambda ∈ {0.7, 1.5}（lambda=1 已在条件矩阵中，直接复用）
    for cond in ("none", "l2"):
        for law in LAWS:
            for scale in (0.7, 1.5):
                jobs.append((law, "circle", cond, scale))

    rows = []
    try:
        import multiprocessing as mp
        ctx = mp.get_context("fork")
        nproc = min(6, os.cpu_count() or 1)
        with ctx.Pool(nproc) as pool:
            for i, row in enumerate(pool.imap(_run_job, jobs)):
                rows.append(row)
                print(f"[{i + 1:>2}/{len(jobs)}] {row['scenario']:>6}/"
                      f"{row['condition']:<9} {row['law']:<5} λ={row['scale']:.1f}"
                      f"  IAE_p_ss={row['IAE_p_ss']:.4e}"
                      f"  E_nongrav={row['E_nongrav']:.4e}"
                      f"{'  ABORTED' if row['aborted'] else ''}",
                      flush=True)
    except Exception as exc:            # pragma: no cover - 回退串行
        print(f"[warn] multiprocessing 不可用（{exc}），回退串行")
        rows = []
        for i, job in enumerate(jobs):
            row = _run_job(job)
            rows.append(row)
            print(f"[{i + 1:>2}/{len(jobs)}] {row['scenario']:>6}/"
                  f"{row['condition']:<9} {row['law']:<5} λ={row['scale']:.1f}"
                  f"  IAE_p_ss={row['IAE_p_ss']:.4e}"
                  f"  E_nongrav={row['E_nongrav']:.4e}"
                  f"{'  ABORTED' if row['aborted'] else ''}", flush=True)

    os.makedirs(ROOT, exist_ok=True)
    cond_rows = [r for r in rows if r["scale"] == 1.0]
    pareto_rows = [r for r in rows if r["scenario"] == "circle"
                   and r["condition"] in ("none", "l2")]
    # 各工况行含条件特有键（gamma_*/bias_ball），fieldnames 取并集；
    # 先落 pickle 再写 CSV，重复运行/部分失败时不丢仿真结果
    import pickle
    with open(os.path.join(ROOT, "metrics_portfolio_rows.pkl"), "wb") as stream:
        pickle.dump({"rows": rows}, stream)

    def _write_csv(path, table):
        keys = []
        for r in table:
            for k in r:
                if k not in keys:
                    keys.append(k)
        with open(path, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=keys)
            writer.writeheader()
            writer.writerows(table)

    _write_csv(os.path.join(ROOT, "metrics_portfolio.csv"), cond_rows)
    _write_csv(os.path.join(ROOT, "metrics_portfolio_pareto.csv"), pareto_rows)
    print("saved", os.path.join(ROOT, "metrics_portfolio.csv"))
    print("saved", os.path.join(ROOT, "metrics_portfolio_pareto.csv"))
    _print_verdict(cond_rows, pareto_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
