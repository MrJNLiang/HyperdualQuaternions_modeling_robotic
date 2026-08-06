"""
S3 抓取-搬运实验：抓杯 -> 带载圆周 -> 空载/带载定量对比
（experiments/run_grasp_circle.py，依托 run_simulation.py 的同构控制栈）。

实验设计（用户需求 1–5 的代码化）：

  1. 抓取动作 —— 内撑式：RG2 两指内距 48 mm < 杯内径 ~80 mm，指尖从
     杯口开口垂直伸入 30 mm（全程走杯内空气，无穿模）；随后在静止保持
     段中点用力传感器把杯子刚性附着到末端 dynamic shape
     （interfaces.attach_cup_rigid：杯子进入机器人动力学树 = 真实动载荷，
     区别于纯层级挂接）。
  2. 带载圆周 —— 提杯、斜线过渡到椅子前方自由空域后，绕
     GRASP_CIRCLE_CENTER 做水平圆（R=0.06 m、ω=CIRCLE_OMEGA，整圈 IK
     限位余量 15.1°）。
  3. 误差监控 —— DataLogger 全通道：位置误差 |T|、姿态误差 |O|、
     twist 误差 e_ξ、Lyapunov V、力矩 τ、约束残差 (3.8)。
  4. 物理交互 —— 每记录步采样：附着点力旋量（readForceSensor =
     抓握力/负载力直接测量：静态≈杯重，圆周段叠加向心/切向惯性力）、
     杯子接触合力（getContactInfo）、机器人↔椅子/杯↔椅最小净距
     （checkDistance —— 碰撞响应被掩码屏蔽后的独立"无穿模"审计量）。
  5. 对比分析 —— 两个维度的交叉对比：
     ① 负载维：--mode noload / load 跑同一条轨迹（唯一差别：是否附着 +
        杯质量改写 CUP_LOAD_MASS）；
     ② 控制律维（总方案 §4/§5.2 同台对比）：--law tndq / dq-chandra /
        dq-hinf / dq-ctc 在完全相同的实验环境（同轨迹/同力矩出口/同安全
        预算/同监控）下切换：
          tndq      = C1 几何一致计算力矩律（式 5.2，本文新理论）
          dq-chandra= C2 忠实 [Ch20] resolved-acceleration 律（式 32–35：
                    Ad 搬运 twist 误差 ω_e=Ad ξ_d−ξ、screw-log 位姿反馈
                    K_P·vec6(2 ln x̃)、解析 ξ̇_d/J̇q̇；params CH20_*；与 C1
                    的唯一结构差异 = 位姿反馈形式，增益与 C1 tuned/C3
                    逐通道恒等 -> 同预算公平比较）
          dq-hinf   = C3 一阶 DQ H∞ 运动学律（hdq_hinf_coppeliasim 原实现，
                    “之前理论”；基线增益带宽对齐 -4 /s，params DQH_*）
          dq-ctc    = C2-abl 朴素 twist 差消融律（不对应任何已发表理论，
                    [Ch20]/[P2] 均含 Ad 搬运；仅消融 C1 结构：无 Ad 输运/
                    无 Aᵀ 整形/差分前馈；params DQC_*，增益同预算配平）
     ③ 增益维（§5.3 整定，仅 tndq）：--gains base / tuned / fast 切换
        params.GAIN_SETS。base = 出厂标量 k_p=16（旋转刚度只有平移 1/4，
        旋转主导极点 -0.54/s）；tuned = 与 C3 逐通道恒等的同预算设计点
        （K_ω=K_v=24, p_O=320, p_T=80，两律 d->e 传递函数完全相同）；
        fast = 敏感性档（极点 -6/-30）。整定过程见
        experiments/tune_tndq_gains.py。
     ④ 敏感条件维（层 3 结构敏感域对比，--condition；全部只改时间/测量/
        控制周期参数，路标几何与场景完全不变 -> 58 点 IK 限位验证仍有效、
        无穿模风险）：
          none          标准场景（缺省；文件名不变，兼容已有结果）
          highspeed     圆周 ω 1.0 -> 2.5 rad/s（CIRCLE_OMEGA_FAST，前馈/
                        向心项放大 6.25 倍 -> 暴露解析 vs 差分前馈差距）
          fast-transit  lift/retreat/transit/descend2 时长 ×0.5
                        （GRASP_FAST_PHASE_SCALE，快相位前馈精度）
          noise         编码器级测量噪声 σ_q=5e-5, σ_q̇=1e-3（NOISE_SIGMA_*，
                        控制器只见带噪测量、安全检查用真值；差分放大 ∝1/dt）
          coarse-dt     控制周期 5 -> 15 ms（GRASP_CTRL_DECIM=3，非控制步
                        ZOH 保持力矩；差分前馈一拍滞后 ×3，解析前馈不受影响）
        动机：准静态标准场景下各律同预算必然趋同（误差被 DC 刚度垄断）；
        敏感条件把结构差异（解析 vs 差分前馈、二阶通道有无、Aᵀ 整形）推到
        线性化失效/高频域，使其可观测。公平性：敏感条件下 C1 建议
        --gains tuned（与各基线同预算，残余差异纯属结构）。
     npz 齐备后自动打印：分相位误差/力矩/接触力的 空载↔带载、
     新律↔基线、整定前↔整定后 全交叉对比表（--compare-only 可单独重印，
     --plot 出图）。

相位时间线（config/params.py S3 参数节，全部路标经 58 点 IK 扫描验证）：

    [0, 2.0]     descend   hover(z=0.714) -> grasp(z=0.679) 指尖入杯 30 mm
    [2.0, 3.5]   hold      静止保持；t=2.5 s 刚性附着（load 模式）+1 s 静置
    [3.5, 5.0]   lift      垂直提杯到横穿高度 [0, 0.48, 0.718]
    [5.0, 6.0]   retreat   保持高度横移出杯口正上方 [0, 0.41, 0.718]
    [6.0, 8.0]   transit   斜线过渡到椅前自由空域 [0, 0.27, 0.68]
    [8.0, 9.5]   descend2  下降到圆心 [0, 0.27, 0.60]
    [9.5, 22.5]  circle    水平圆周（ramp 2 s + 稳态 >1.5 圈）

用法（TNDQ_sim 目录下，CoppeliaSim 需加载 KUKALBR4+_sim.ttt）：

    python3 experiments/run_grasp_circle.py --mode noload               # C1 空载
    python3 experiments/run_grasp_circle.py --mode load                 # C1 带载
    python3 experiments/run_grasp_circle.py --law dq-chandra --mode noload  # C2 空载
    python3 experiments/run_grasp_circle.py --law dq-chandra --mode load    # C2 带载
    python3 experiments/run_grasp_circle.py --law dq-hinf --mode noload # C3 空载
    python3 experiments/run_grasp_circle.py --law dq-hinf --mode load   # C3 带载
    python3 experiments/run_grasp_circle.py --law dq-ctc --mode noload  # C2-abl 空载
    python3 experiments/run_grasp_circle.py --law dq-ctc --mode load    # C2-abl 带载
    python3 experiments/run_grasp_circle.py --gains tuned --mode noload # C1 整定后
    python3 experiments/run_grasp_circle.py --gains tuned --mode load
    python3 experiments/run_grasp_circle.py --compare-only              # 只打印对比
    python3 experiments/run_grasp_circle.py --compare-only --plot       # 对比+出图

    # 敏感条件（每个条件建议跑齐各律；C1 用 --gains tuned 保同预算公平）：
    python3 experiments/run_grasp_circle.py --mode load --gains tuned --condition highspeed
    python3 experiments/run_grasp_circle.py --mode load --law dq-ctc  --condition highspeed
    python3 experiments/run_grasp_circle.py --mode load --law dq-chandra  --condition highspeed
    python3 experiments/run_grasp_circle.py --mode load --law dq-hinf --condition highspeed
    #（--condition ∈ {none, highspeed, fast-transit, noise, coarse-dt}）

输出：results/grasp_circle_[chandra_|dqctc_|dqhinf_|tuned_|fast_]{noload|load}
[_hspeed|_ftrans|_noise|_cdt].npz/.csv（逐步 CSV 含 d_hat_norm 列 =
反演的证书通道等效扰动 ‖d̂‖，§6.5(6)；纯诊断量，不进控制律）
+ 终端分相位统计、空载/带载 ×
控制律（C1/C2/C2-abl/C3）× 增益组 × 敏感条件对比表 + 全部已有结果的关键
指标汇总表 results/grasp_metrics_summary.csv（定量分析用，行 = law ×
gains × mode × condition × phase）；--plot 时另存
results/grasp_compare_*.png（含敏感条件分组柱状图
grasp_compare_conditions_*.png 与等效扰动图 grasp_compare_disturbance.png）。
γ 影响实验见 experiments/run_gamma_sweep.py。
"""

import argparse
import os
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))

from config import params
from config.lbr4_dynamics import (
    LBR4NominalDynamics, clip_torque, check_joint_limits,
)
from core.kinematics import TNDQSerialChain
from core.dq_algebra import dq_translation, dq_rotation
from control.error_system import full_error_state
from control.control_law import (
    geometric_computed_torque_law, damped_pinv,
    dq_hinf_kinematic_law, velocity_to_accel_ref, dq_ctc_law,
    dq_chandra2020_law,
)
from control.performance import (
    PerformanceAccumulator, check_hinf_condition_merged, pose_weight,
    ResidualDisturbanceEstimator,
)
from simdata.trajectory_generator import (
    CupCircleTrajectoryTNDQ, CompositeTrajectoryTNDQ,
    waypoint_sequence_trajectory, pose_dq,
)
from simdata.input_simulation import MeasurementNoise
from output.data_logger import DataLogger
from run_simulation import joint_safety_governor

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "results")

PHASE_NAMES = ["descend", "hold", "lift", "retreat", "transit",
               "descend2", "circle"]

# 控制律注册表：law id -> (显示名, npz 文件名前缀；tndq 保持无前缀，
# 兼容已有结果文件 grasp_circle_{mode}.npz）
LAWS = {
    "tndq": "C1 TNDQ 几何一致 CTC (5.2)",
    "dq-chandra": "C2 忠实 [Ch20] resolved-acceleration 律 (式 32–35)",
    "dq-hinf": "C3 一阶 DQ H∞ 运动学律 (hdq_hinf 原实现)",
    "dq-ctc": "C2-abl 朴素 twist 差消融律 (非 [Ch20] 理论)",
}

# 对比的基线列表（compare_laws / export_metrics_csv 共用）
BASELINE_LAWS = [("dq-chandra", "C2 dq-chandra"),
                 ("dq-hinf", "C3 dq-hinf"),
                 ("dq-ctc", "C2-abl dq-ctc")]


# 增益组（仅对 tndq 有意义；base 保持无前缀以兼容已有 npz）
GAIN_LABELS = {
    "base": "整定前 base（标量 k_p=16）",
    "tuned": "整定后 tuned（K_ω=K_v=24, p_O=320, p_T=80）",
    "fast": "敏感性档 fast（极点 -6/-30）",
}


# 结构敏感条件注册表（层 3 对比；只改时间/测量/控制周期，路标几何不变）
CONDITIONS = {
    "none": "标准场景（准静态基线，误差由 DC 刚度垄断）",
    "highspeed": (f"高速圆周 ω={params.CIRCLE_OMEGA:.1f} -> "
                  f"{params.CIRCLE_OMEGA_FAST:.1f} rad/s"
                  "（前馈/向心项 ×6.25，暴露解析 vs 差分前馈）"),
    "fast-transit": (f"搬运相位时长 ×{params.GRASP_FAST_PHASE_SCALE:.1f}"
                     "（lift/retreat/transit/descend2；快相位前馈精度）"),
    "noise": (f"测量噪声 σ_q={params.NOISE_SIGMA_Q:.0e}, "
              f"σ_q̇={params.NOISE_SIGMA_QDOT:.0e}"
              "（控制器只见带噪测量；差分放大 ∝1/dt）"),
    "coarse-dt": (f"控制周期 ×{params.GRASP_CTRL_DECIM}（5 -> 15 ms，"
                  "ZOH 保持力矩；差分一拍滞后 ×3，解析前馈不受影响）"),
}

# condition -> npz 文件名后缀（none 无后缀，兼容已有结果文件）
COND_TAG = {"none": "", "highspeed": "_hspeed", "fast-transit": "_ftrans",
            "noise": "_noise", "coarse-dt": "_cdt"}


def _npz_tag(law, mode, gains="base", condition="none"):
    suffix = COND_TAG[condition]
    if law == "dq-hinf":
        return f"dqhinf_{mode}{suffix}"
    if law == "dq-chandra":
        return f"chandra_{mode}{suffix}"
    if law == "dq-ctc":
        return f"dqctc_{mode}{suffix}"
    base = mode if gains == "base" else f"{gains}_{mode}"
    return f"{base}{suffix}"


def _gain_brief(K_d, k_p):
    """增益组的一行摘要（标量 / 矩阵 K_p 通用）。"""
    d = np.diag(np.atleast_2d(np.asarray(K_d, dtype=float)))
    kp = np.asarray(k_p, dtype=float)
    if kp.ndim == 2:
        p = np.diag(kp)
        return (f"K_ω={d[0]:.0f}, K_v={d[3]:.0f}, p_O={p[0]:.0f}, "
                f"p_T={p[3]:.0f}（矩阵 K_p，两通道独立配置）")
    return f"K_ω=K_v={d[0]:.0f}, k_p={float(kp):.0f}（标量 K_p=k_p·I）"


def build_grasp_trajectory(condition="none"):
    """抓取-搬运-圆周全程参考轨迹 + 相位时刻表。

    condition 只改时间参数（highspeed 换圆周 ω、fast-transit 压缩搬运
    相位时长），路标几何完全不变 -> 58 点 IK 限位验证仍有效、无穿模风险。
    返回 (trajectory, t_marks)：t_marks = [T1..T6]（相位边界），
    圆周段从 T6 起（CupCircleTrajectoryTNDQ 自带五次 ramp）。"""
    r = params.R_TOOL_DOWN
    x0 = pose_dq(params.GRASP_TIP_HOVER, r)
    # fast-transit：仅搬运四段时长 ×scale；descend/hold 不动 -> 附着时刻
    # 的准静态力学不变，附着瞬态对比基准保持干净
    s = (params.GRASP_FAST_PHASE_SCALE if condition == "fast-transit"
         else 1.0)
    t_lift = s * params.GRASP_T_LIFT
    t_retreat = s * params.GRASP_T_RETREAT
    t_transit = s * params.GRASP_T_TRANSIT
    t_descend2 = s * params.GRASP_T_DESCEND2
    legs = [
        (params.GRASP_TIP_GRASP, r, params.GRASP_T_DESCEND,
         params.GRASP_T_ATTACH_HOLD),
        (params.GRASP_TIP_LIFT, r, t_lift, 0.0),
        (params.GRASP_TIP_RETREAT, r, t_retreat, 0.0),
        (params.GRASP_TIP_TRANSIT, r, t_transit, 0.0),
        (params.GRASP_CIRCLE_CENTER, r, t_descend2, 0.0),
    ]
    chain_traj, t6 = waypoint_sequence_trajectory(x0, legs)
    omega = (params.CIRCLE_OMEGA_FAST if condition == "highspeed"
             else params.CIRCLE_OMEGA)
    circle = CupCircleTrajectoryTNDQ(
        params.GRASP_CIRCLE_CENTER, params.GRASP_CIRCLE_RADIUS,
        omega, r, ramp_time=params.CIRCLE_RAMP_TIME)
    traj = CompositeTrajectoryTNDQ(list(chain_traj.segments) + [(circle, t6)])
    T1 = params.GRASP_T_DESCEND
    T2 = T1 + params.GRASP_T_ATTACH_HOLD
    T3 = T2 + t_lift
    T4 = T3 + t_retreat
    T5 = T4 + t_transit
    return traj, [T1, T2, T3, T4, T5, t6]


def phase_of(t, marks):
    """时刻 -> 相位 id（0..6，PHASE_NAMES 顺序）。"""
    for i, m in enumerate(marks):
        if t < m:
            return i
    return len(marks)


def run(mode, t_end, law="tndq", gains="base", condition="none"):
    """跑一种组合（law × gains × mode × condition），返回 npz 路径；
    失败返回 None。"""
    from interfaces.coppeliasim_interface import (
        CoppeliaSimLBR4Interface, CoppeliaSimError,
    )

    # 增益组：tndq 可选 base/tuned/fast；C2/C2-abl/C3 基线用各自的
    # CH20_*/DQC_*/DQH_* 增益，仅借 base 权重作为 V 的参考存储函数
    # （基线无证书）。
    gset = params.GAIN_SETS[gains if law == "tndq" else "base"]
    K_d, k_p = gset["K_d"], gset["k_p"]

    print(f"[law] {LAWS[law]}")
    print(f"[condition] {condition}: {CONDITIONS[condition]}")
    if law == "tndq":
        print(f"[gains] {GAIN_LABELS[gains]}: {_gain_brief(K_d, k_p)}")
        ok, lam_min, level = check_hinf_condition_merged(
            K_d, params.KAPPA, params.GAMMA_A)
        print(f"Gain condition (5.6a): lambda_min(K_d)={lam_min:.3f} "
              f">= {level:.3f} required -> {'OK' if ok else 'NOT SATISFIED'}")
    elif law == "dq-chandra":
        print(f"Baseline gains (C2 忠实 [Ch20]): K_v={params.CH20_K_V[0, 0]:.0f} I6, "
              f"K_P={params.CH20_K_P[0, 0]:.0f} I6 "
              f"(ℓ̈+K_v ℓ̇+K_P ℓ=0 与 C1 tuned / C3 逐通道恒等，params.CH20_*)")
    elif law == "dq-ctc":
        p = np.diag(params.DQC_K_P)
        print(f"Baseline gains (C2-abl): K_d={params.DQC_K_D[0, 0]:.0f} I6, "
              f"p_O={p[0]:.0f}, p_T={p[3]:.0f} "
              f"(线性化通道与 C1 tuned / C3 逐通道恒等，params.DQC_*)")
    else:
        print(f"Baseline gains: kO=sqrt(2)/gamma_O={np.sqrt(2.0) / params.DQH_GAMMA_O:.1f}, "
              f"kT=sqrt(2)/gamma_T={np.sqrt(2.0) / params.DQH_GAMMA_T:.1f} "
              f"(带宽对齐 TNDQ 主导极点 -4 /s), K_servo={params.DQH_K_SERVO:.0f}")

    chain = TNDQSerialChain(params.KUKA_LBR4_DH)
    n = chain.n
    dyn_ctrl = LBR4NominalDynamics(params.KUKA_LBR4_DH, mismatch_scale=1.0)

    interface = CoppeliaSimLBR4Interface()
    try:
        interface.connect(torque_mode=True,
                          engine_dt=params.COPPELIA_DT_TARGET)
    except CoppeliaSimError as exc:
        print(f"[error] CoppeliaSim 连接失败：{exc}")
        print("        请启动 CoppeliaSim 并加载 TNDQ_sim/KUKALBR4+_sim.ttt。")
        return None
    dt = interface.sim_dt or params.DT

    # ---- 敏感条件装配（只碰测量/控制周期，轨迹由 build_grasp_trajectory
    # 处理）：noise = 控制器只见带噪测量（安全检查/治理器仍用真值，与
    # run_simulation.py E6 同构）；coarse-dt = 控制周期 dt×decim，非控制步
    # ZOH 重发上一拍力矩（差分前馈/内环差分用 dt_ctrl，解析前馈不受影响）。
    noise = (MeasurementNoise(n, sigma_q=params.NOISE_SIGMA_Q,
                              sigma_qdot=params.NOISE_SIGMA_QDOT)
             if condition == "noise" else None)
    decim = params.GRASP_CTRL_DECIM if condition == "coarse-dt" else 1
    dt_ctrl = dt * decim

    # ---- 场景准备（start 前）：挪杯到可达抓取位 + 净距监控集合 --------------
    interface.move_cup(params.CUP_POS_GRASP)
    interface.setup_clearance_monitor()
    interface.lock_gripper_fingers()
    q_init = params.Q_INIT_GRASP.copy()
    interface.set_joint_positions(q_init)
    interface.start()

    trajectory, marks = build_grasp_trajectory(condition)
    t_attach = params.GRASP_T_DESCEND + 0.5 * params.GRASP_T_ATTACH_HOLD
    q_nullspace_center = q_init.copy()

    logger = DataLogger()
    perf = PerformanceAccumulator(K_d, k_p,
                                  params.KAPPA, params.GAMMA_A)
    # 证书通道等效扰动反演器（§6.5(6)）：S3 不注入 w，故旧口径的 d≡0 使
    # (5.6)/(5.7) 全部空值；由闭环误差动态 (5.1e) 反演 d̂ 可以把定理 3 诚实
    # 条款里的全部扰动源（杯子的 ΔM/Δg、测量噪声、伪逆残差、限幅/治理器、
    # 离散化）归入证书口径。纯诊断量：不进控制律 -> 闭环轨迹逐比特不变。
    # 各基线借用 base 档证书增益，反演值额外含「实际反馈 − 证书反馈」的
    # 结构差，不可跟 C1 比绝对值（同律跨工况比仍公平，见 performance.py）。
    d_est = ResidualDisturbanceEstimator(K_d, k_p, dt_ctrl)
    # extra 通道（与 LOG_EVERY 同步采样）
    ex = {k: [] for k in ["t_ex", "phase", "grasp_F", "grasp_M",
                          "cup_contact", "clr_chair", "clr_cup", "cup_p"]}

    n_steps = int(round(t_end / dt))
    print(f"Running {n_steps} steps ({t_end}s, dt={dt}s"
          f"{', dt_ctrl=%.0f ms (ZOH)' % (dt_ctrl * 1e3) if decim > 1 else ''}"
          f"), law={law}, gains={gains}, mode={mode}, "
          f"condition={condition}, attach at t={t_attach:.2f}s"
          f"{'（刚性附着 + 满杯 %.2f kg）' % params.CUP_LOAD_MASS if mode == 'load' else '（不附着，空载基线）'}")

    saturated_steps = governed_steps = 0
    attached = False
    aborted = False
    qdot_cmd_prev = None       # 基线律差分前馈状态（C3 无二阶通道）
    xi_d_prev = None           # C2-abl 差分状态：ξ̇_d^num = Δξ_d/dt（忠实 C2 不用）
    J_prev = None              # C2-abl 差分状态：(J̇q̇)^num = ΔJ/dt · q̇
    tau_prev = None            # coarse-dt：非控制步 ZOH 重发的上一拍力矩

    try:
        for k in range(n_steps):
            t = k * dt

            # ---- 抓取事件：静止保持段中点刚性附着（load 模式） -----------
            if mode == "load" and not attached and t >= t_attach:
                attached = interface.attach_cup_rigid(params.CUP_LOAD_MASS)

            # ---- 传感层 -------------------------------------------------
            q, q_dot = interface.read_joint_states()

            over = check_joint_limits(q)
            if over:
                print(f"[abort] t={t:.3f}s 关节 {over} 超限位，安全终止")
                aborted = True
                break

            # coarse-dt：非控制步 ZOH 重发上一拍力矩（限位检查仍逐步；
            # 日志只在控制步采，采样间隔 = lcm(LOG_EVERY, decim) 步）
            if decim > 1 and k % decim != 0 and tau_prev is not None:
                interface.send_joint_targets(tau_prev, mode="torque")
                interface.step()
                continue

            # noise 条件：控制器（FK/误差/控制律/力矩装配）只见带噪测量；
            # 限位检查/安全治理器用真值（run_simulation.py E6 同构）
            if noise is not None:
                q_meas, qdot_meas = noise(q, q_dot)
            else:
                q_meas, qdot_meas = q, q_dot

            # ---- 控制器（FK + 误差 + 控制律 + 力矩装配，run_simulation 同构）
            tic = time.perf_counter()
            fk = chain.fk_outputs(q_meas, qdot_meas, q_ddot=None,
                                  with_jacobian=True)
            des = trajectory.evaluate(t)
            err = full_error_state(fk["x_breve"], des["x_breve_d"])

            damping = params.PINV_DAMPING
            sig_min = np.linalg.svd(fk["J"], compute_uv=False)[-1]
            if sig_min < params.SINGULARITY_TOL:
                damping = params.SINGULARITY_DAMPING

            if law == "tndq":
                # [C1] 式 (5.2) 几何一致计算力矩律（二阶，解析前馈）
                qddot_ref, _ = geometric_computed_torque_law(
                    err, des["xi_d"], des["xi_dot_d"],
                    fk["J"], fk["Jdot_qdot"],
                    K_d, k_p, damping=damping)
            elif law == "dq-chandra":
                # [C2] 忠实 [Ch20] resolved-acceleration 律（式 32–35）：
                # Ad 搬运 twist 误差 ω_e = −e_ξ + screw-log 位姿反馈
                # K_P·vec6(2 ln x̃)；ξ̇_d/J̇q̇ 按原文信息集为解析量（无差分）
                qddot_ref, _ = dq_chandra2020_law(
                    err, des["xi_d"], des["xi_dot_d"],
                    fk["J"], fk["Jdot_qdot"],
                    params.CH20_K_V, params.CH20_K_P, damping=damping)
            elif law == "dq-ctc":
                # [C2-abl] 朴素 twist 差消融律：数值差分 ξ̇_d/J̇q̇，无 Ad
                # 输运/Aᵀ 整形/证书（前馈一拍滞后 + 差分噪声；coarse-dt
                # 下差分步长 = dt_ctrl，滞后/噪声相应放大）
                xi_dot_d_num = (np.zeros(6) if xi_d_prev is None
                                else (des["xi_d"] - xi_d_prev) / dt_ctrl)
                Jdot_qdot_num = (np.zeros(6) if J_prev is None
                                 else (fk["J"] - J_prev) / dt_ctrl @ qdot_meas)
                qddot_ref, _ = dq_ctc_law(
                    err, fk["xi"], des["xi_d"], xi_dot_d_num, Jdot_qdot_num,
                    fk["J"], params.DQC_K_D, params.DQC_K_P, damping=damping)
                xi_d_prev = np.asarray(des["xi_d"], dtype=float).copy()
                J_prev = fk["J"].copy()
            else:
                # [C3] 一阶 DQ H∞ 运动学律：速度级指令 -> 同一力矩接口
                # （内环速度伺服 + 差分前馈；无 ξ̇_d/J̇q̇ 解析通道）
                task_vel = dq_hinf_kinematic_law(
                    err, des["xi_d"],
                    params.DQH_GAMMA_O, params.DQH_GAMMA_T)
                qdot_cmd = damped_pinv(
                    fk["J"], damping=max(damping, params.DQH_DAMPING)) @ task_vel
                qddot_ref = velocity_to_accel_ref(
                    qdot_cmd, qdot_cmd_prev, qdot_meas, dt_ctrl,
                    params.DQH_K_SERVO)
                qdot_cmd_prev = qdot_cmd

            Jp = damped_pinv(fk["J"], damping=damping)
            N_proj = np.eye(n) - Jp @ fk["J"]
            qddot_ref = qddot_ref + N_proj @ (
                params.NULLSPACE_K * (q_nullspace_center - q_meas)
                - params.NULLSPACE_D * qdot_meas)

            qn = np.linalg.norm(qddot_ref)
            if qn > params.QDDOT_MAX:
                qddot_ref = qddot_ref * (params.QDDOT_MAX / qn)
                saturated_steps += 1

            qddot_ref, governed = joint_safety_governor(q, q_dot, qddot_ref)
            governed_steps += int(governed)

            # 名义模型不含杯：load 模式下负载 = 模型失配扰动（实验变量）
            tau = dyn_ctrl.computed_torque(q_meas, qdot_meas, qddot_ref)
            runtime = time.perf_counter() - tic

            tau, sat = clip_torque(tau)
            saturated_steps += int(sat)
            interface.send_joint_targets(tau, mode="torque")
            interface.step()
            tau_prev = tau

            # ---- 监控层 ---------------------------------------------------
            # S3 不注入 w（d_inj≡0），扰动全部靠 (5.1e) 反演；差分噪声由
            # 反演器内部 20 Hz 低通滤掉，d̂ 不进控制律
            d_hat = d_est.update(err["e_xi"], err["e_z"], err["A"])
            V = perf.update(err["e_xi"], err["e_z"], d_hat, dt_ctrl,
                            d_injected=np.zeros(6))

            if k % params.LOG_EVERY == 0:
                logger.log(
                    t=t, e_z=err["e_z"], e_xi=err["e_xi"],
                    qddot_ref=qddot_ref, tau=tau,
                    x_d=des["x_d"], xi_d=des["xi_d"],
                    xi_dot_d=des["xi_dot_d"],
                    x=fk["x"], xi=fk["xi"],
                    V=V, c0=fk["c0"], c1=fk["c1"], c2=fk["c2"],
                    runtime=runtime, d_hat=d_hat)
                # 物理交互通道（用户需求 4）
                F, M = interface.read_grasp_wrench()
                d_chair, d_cup = interface.read_clearances(
                    cup_attached=attached)
                ex["t_ex"].append(t)
                ex["phase"].append(phase_of(t, marks))
                ex["grasp_F"].append(F)
                ex["grasp_M"].append(M)
                ex["cup_contact"].append(
                    interface.read_contact_force_norm(interface.cup_handle)
                    if interface.cup_handle is not None else 0.0)
                ex["clr_chair"].append(d_chair)
                ex["clr_cup"].append(d_cup)
                ex["cup_p"].append(
                    interface.read_object_position(interface.cup_handle)
                    if interface.cup_handle is not None else np.full(3, np.nan))

    except KeyboardInterrupt:
        print("\n[interrupt] 用户中断，保存已记录数据后安全退出 ...")
        aborted = True
    except Exception:
        print("[error] 仿真循环异常，安全断开后保留已有数据：")
        traceback.print_exc()
        aborted = True
    finally:
        interface.disconnect()

    if not logger.as_arrays()["t"].size:
        print("[error] 无有效数据（首步即失败），不生成输出文件。")
        return None

    extra = {k: np.asarray(v) for k, v in ex.items()}
    extra.update({"mode": mode, "law": law, "dt": dt,
                  "condition": condition, "dt_ctrl": dt_ctrl,
                  "t_marks": np.asarray(marks), "t_attach": t_attach,
                  "load_mass": params.CUP_LOAD_MASS if mode == "load" else 0.0,
                  "K_d": np.atleast_2d(np.asarray(K_d, dtype=float)),
                  "k_p": np.asarray(k_p, dtype=float),
                  # 稳定性指标（需求 4）：饱和/治理器触发计数归档
                  "sat_steps": saturated_steps, "gov_steps": governed_steps,
                  "n_steps": n_steps})
    tag = _npz_tag(law, mode, gains, condition)
    npz_path = logger.save_npz(
        os.path.join(RESULTS_DIR, f"grasp_circle_{tag}.npz"), extra=extra)
    csv_path = logger.save_csv(
        os.path.join(RESULTS_DIR, f"grasp_circle_{tag}.csv"))
    print(f"Saved raw data  : {npz_path}")
    print(f"Saved CSV       : {csv_path}")
    if saturated_steps:
        print(f"[note] 力矩/指令饱和步数: {saturated_steps}/{n_steps}")
    if governed_steps:
        print(f"[note] 安全治理器触发步数: {governed_steps}/{n_steps}")
    if aborted:
        print("[note] 本次运行提前终止，统计量基于已完成步数。")

    print_phase_table(npz_path)
    logger.print_summary(performance_summary=perf.summary())
    return npz_path


# ===========================================================================
# 定量分析（用户需求 3/4/5：分相位统计 + 空载/带载 + 控制律全交叉对比）
# ===========================================================================

def _d_hat_norm(d):
    """反演的证书通道等效扰动范数 ‖d̂(t)‖（§6.5(6)）。

    旧 npz（改动前跑的结果）没有 d_hat 通道 -> 返回全 nan，使统计/绘图
    里表现为 n/a 或空白，而不是被误读成「无扰动」的 0。"""
    if "d_hat" not in d:
        return np.full(len(d["t"]), np.nan)
    return np.linalg.norm(d["d_hat"], axis=1)


def _phase_stats(d):
    """按相位切片统计：返回 {phase: dict(指标)}；圆周段另给稳态切片
    （去掉 ramp + 1 s 过渡，只看极限环品质）。"""
    t = d["t"]
    phase = d["phase"]
    O = np.linalg.norm(d["e_z"][:, :3], axis=1)
    T = np.linalg.norm(d["e_z"][:, 3:], axis=1)
    exi = np.linalg.norm(d["e_xi"], axis=1)
    tau = np.linalg.norm(d["tau"], axis=1)
    gF = np.linalg.norm(d["grasp_F"], axis=1)
    dh = _d_hat_norm(d)      # 反演的证书通道等效扰动；旧 npz -> nan
    out = {}

    def _row(m):
        return dict(
            T_rms=np.sqrt(np.mean(T[m] ** 2)), T_max=T[m].max(),
            O_rms=np.sqrt(np.mean(O[m] ** 2)), O_max=O[m].max(),
            exi_rms=np.sqrt(np.mean(exi[m] ** 2)),
            tau_rms=np.sqrt(np.mean(tau[m] ** 2)), tau_max=tau[m].max(),
            gF_mean=np.mean(gF[m]), gF_max=gF[m].max(),
            d_hat_rms=np.sqrt(np.mean(dh[m] ** 2)), d_hat_max=dh[m].max(),
        )

    for pid, name in enumerate(PHASE_NAMES):
        m = phase == pid
        if not m.any():
            continue
        out[name] = _row(m)
    # 圆周稳态：t >= T6 + ramp + 1 s
    t6 = float(d["t_marks"][-1])
    m = t >= t6 + params.CIRCLE_RAMP_TIME + 1.0
    if m.any():
        out["circle-ss"] = _row(m)
    return out


def _load_npz(mode, law="tndq", gains="base", condition="none"):
    p = os.path.join(
        RESULTS_DIR,
        f"grasp_circle_{_npz_tag(law, mode, gains, condition)}.npz")
    if not os.path.exists(p):
        return None
    d = dict(np.load(p, allow_pickle=False))
    # extra 通道与主通道采样同步，但提前 abort 时长度可能差 1，对齐截断
    n = min(len(d["t"]), len(d["t_ex"]))
    for k in list(d.keys()):
        if d[k].ndim >= 1 and d[k].shape[0] in (len(d["t"]), len(d["t_ex"])):
            d[k] = d[k][:n]
    return d


# --- 跨增益组/跨控制律可比的汇总量 ------------------------------------------

def _V_common(d, k_p_ref=None):
    """用统一权重重算存储函数 V = ½‖e_ξ‖² + ½ e_zᵀK_p e_z。

    日志里的 V（npz/逐步 CSV 的 V 列）用各自增益的 K_p（与证书一致），
    权重不同时跨组比较没有意义；对比表/汇总 CSV/各图统一换算到 base 组
    权重（标量 k_p=16，即 K_p=16 I₆）后再比。

    口径提醒（§6.5(6) 水平集余度的比较必须同权重）：tuned 档的工作集
    门槛 c* 是在 tuned 权重（p_O=320, p_T=80）下给出的，而本函数给的是
    base 权重值；两者的位姿项权重差 p_O/16 = 20，因此拿本函数的
    V_ss/V_peak 去对 tuned 档的 c* 做余度比较前必须先乘 20（均方意义上的
    上界估算，旋转通道主导时紧）；不换算直接比会把余度高估 20 倍。"""
    W = pose_weight(params.GAIN_SETS["base"]["k_p"] if k_p_ref is None
                    else k_p_ref)
    e_xi, e_z = d["e_xi"], d["e_z"]
    return (0.5 * np.einsum("ij,ij->i", e_xi, e_xi)
            + 0.5 * np.einsum("ij,jk,ik->i", e_z, W, e_z))


# V 的统一权重口径标记（汇总 CSV / 图例 / 终端表头共用，避免误比）
V_WEIGHT_TAG = (f"base k_p={float(np.asarray(params.GAIN_SETS['base']['k_p'])):.0f}")


def _rms_bound(d):
    """均方（RMS）极限界 (5.7) 的实测校验：sup‖d̂‖/λ_min(K_d) vs 实测
    |e_ξ| 的 RMS。旧 npz（无 d̂ 通道）返回 None。

    (5.7) 是均方界而非逐点 ISS 极限球，左侧泛函就是 e_ξ 的 RMS，
    因此两者可直接比；margin = 界/实测 = 界的保守倍数。"""
    if "d_hat" not in d or "K_d" not in d:
        return None
    dh = _d_hat_norm(d)
    if not np.isfinite(dh).any():
        return None
    lam = float(np.min(np.linalg.eigvalsh(
        np.atleast_2d(np.asarray(d["K_d"], dtype=float)))))
    exi = np.linalg.norm(d["e_xi"], axis=1)
    rms = float(np.sqrt(np.mean(exi ** 2)))
    d_inf = float(np.nanmax(dh))
    bound = d_inf / lam
    return dict(d_inf=d_inf, bound=bound, rms=rms,
                margin=bound / max(rms, 1e-15))


def _V_metrics(d):
    """V 的收敛特性（需求 6）：稳态均值/峰值/附着后回落时间。"""
    V = _V_common(d)
    t = d["t"][:len(V)]
    V_ss = float(V[-max(1, len(V) // 5):].mean())
    i_pk = int(np.argmax(V))
    t_att = float(d["t_attach"])
    t_conv = np.nan
    idx = np.where(t >= t_att)[0]
    if idx.size:
        bad = idx[V[idx] > 2.0 * V_ss]     # 最后一次超出 2×V_ss 的时刻
        if not bad.size:
            t_conv = 0.0
        elif bad[-1] + 1 < len(V):
            t_conv = float(t[bad[-1] + 1] - t_att)
    return dict(V=V, t=t, V_ss=V_ss, V_peak=float(V[i_pk]),
                t_peak=float(t[i_pk]), t_conv=t_conv)


def _fmt_stab(d):
    """饱和步数 / 治理器触发次数（需求 4）。"""
    if "sat_steps" not in d:
        return "饱和/治理计数：旧 npz 未记录"
    n = int(d["n_steps"])
    return (f"饱和 {int(d['sat_steps'])}/{n} 步，"
            f"治理器 {int(d['gov_steps'])}/{n} 步")


def print_phase_table(npz_path):
    """单次运行的分相位统计表（无穿模审计 + 误差/力矩/抓握力）。"""
    d = dict(np.load(npz_path, allow_pickle=False))
    n = min(len(d["t"]), len(d["t_ex"]))
    for k in list(d.keys()):
        if d[k].ndim >= 1 and d[k].shape[0] in (len(d["t"]), len(d["t_ex"])):
            d[k] = d[k][:n]
    stats = _phase_stats(d)
    print("=" * 78)
    print(f"分相位统计（{os.path.basename(npz_path)}）")
    print("=" * 78)
    hdr = (f"{'phase':>9} {'|T|rms':>10} {'|T|max':>10} {'|O|rms':>10} "
           f"{'|tau|rms':>10} {'|F_grasp|mean':>14} {'max':>8}")
    print(hdr)
    print("-" * len(hdr))
    for name, s in stats.items():
        print(f"{name:>9} {s['T_rms']:10.3e} {s['T_max']:10.3e} "
              f"{s['O_rms']:10.3e} {s['tau_rms']:10.2f} "
              f"{s['gF_mean']:14.3f} {s['gF_max']:8.3f}")
    # 无穿模审计（用户核心要求）：零净距 + 邻域内无接触力 = 幽灵穿透
    # （FAIL）；零净距 + 邻域内有接触力 = 真实物理接触（带载模式杯离
    # 椅前的合法支撑；邻域 ±1 样本包容 200 Hz 接触振荡 vs 50 ms 采样）
    cc = d["cup_contact"]
    touching = cc > 1e-6
    near_touch = touching.copy()
    near_touch[:-1] |= touching[1:]
    near_touch[1:] |= touching[:-1]
    print("-" * len(hdr))
    for label, clr in (("机器人<->椅子", d["clr_chair"]),
                       ("杯相关对   ", d["clr_cup"])):
        m = np.isfinite(clr)
        if not m.any():
            continue
        zero = m & (clr <= 1e-6)
        ghost = zero & ~near_touch
        touch = zero & ~ghost
        pos = clr[m & ~zero]
        msg = (f"  净距 {label} : min正值 {pos.min() * 1e3:6.1f} mm"
               if pos.size else f"  净距 {label} : 无正值样本")
        if ghost.any():
            msg += (f"；幽灵穿透 {ghost.sum()} 样本"
                    f"（t={np.round(d['t_ex'][ghost], 2).tolist()}） <- 穿模！")
        elif touch.any():
            msg += (f"；真实接触 {touch.sum()} 样本"
                    f"（t∈[{d['t_ex'][touch].min():.2f},"
                    f"{d['t_ex'][touch].max():.2f}]s，接触力>0，合法支撑）"
                    " OK（无穿模）")
        else:
            msg += " OK（无穿模）"
        print(msg)
    if cc.max() > 0:
        i = int(cc.argmax())
        print(f"  杯子接触合力 max        : {cc.max():8.3f} N "
              f"@t={d['t_ex'][i]:.2f}s（附着/脱离椅面瞬态 + 支撑力，"
              f"需求 4 直接观测量；提杯后应归零）")
    else:
        print("  杯子接触合力 max        :    0.000 N（全程无接触）")
    print("=" * 78)


def compare(paths=("noload", "load"), law="tndq", gains="base"):
    """空载 vs 带载定量对比（用户需求 5，按控制律/增益组分开打印）。"""
    d0 = _load_npz(paths[0], law, gains)
    d1 = _load_npz(paths[1], law, gains)
    if d0 is None or d1 is None:
        if d0 is not None or d1 is not None:      # 未跑的组不刷提示
            missing = [m for m, d in zip(paths, (d0, d1)) if d is None]
            print(f"[note] [{law}/{gains}] 缺少 {missing} 结果，跑齐两种 --mode "
                  f"后自动输出对比表。")
        return
    s0, s1 = _phase_stats(d0), _phase_stats(d1)
    lm = float(d1.get("load_mass", params.CUP_LOAD_MASS))
    label = LAWS[law] + (f" | {GAIN_LABELS[gains]}" if law == "tndq" else "")
    print("=" * 78)
    print(f"[{label}] 空载 vs 带载（{lm:.2f} kg 满杯）对比 —— "
          f"物理交互对控制性能的影响")
    print("=" * 78)
    hdr = (f"{'phase':>9} {'metric':>9} {'noload':>11} {'load':>11} "
           f"{'ratio':>7}  note")
    print(hdr)
    print("-" * len(hdr))
    rows = [("T_rms", "位置误差 RMS [m]"), ("O_rms", "姿态误差 RMS"),
            ("exi_rms", "twist 误差 RMS"), ("tau_rms", "力矩 RMS [Nm]"),
            ("gF_mean", "抓握力均值 [N]"),
            ("d_hat_rms", "等效扰动 RMS（反演 d̂）")]
    for name in [n_ for n_ in ("hold", "lift", "transit", "circle",
                               "circle-ss") if n_ in s0 and n_ in s1]:
        for key, note in rows:
            a, b = s0[name][key], s1[name][key]
            if not (np.isfinite(a) and np.isfinite(b)):
                r_txt = f"{'n/a':>7}"          # 旧 npz 无 d̂ 通道
            else:
                r_txt = f"{b / a:7.2f}" if a > 1e-15 else f"{'inf':>7}"
            print(f"{name:>9} {key:>9} {a:11.3e} {b:11.3e} "
                  f"{r_txt}  {note}")
        print("-" * len(hdr))
    # 理论预期解读（定理 3(c)/(d)）：负载 = 未建模 ΔM/Δg 失配扰动
    F1 = np.linalg.norm(d1["grasp_F"], axis=1)
    t6 = float(d1["t_marks"][-1])
    m_ss = d1["t"] >= t6 + params.CIRCLE_RAMP_TIME + 1.0
    w_static = lm * 9.81
    print(f"  负载理论权重 m*g            : {w_static:.3f} N")
    if m_ss.any():
        print(f"  圆周稳态抓握力 mean/max     : "
              f"{F1[m_ss].mean():.3f} / {F1[m_ss].max():.3f} N "
              f"（超出 m*g 部分 = 向心/切向惯性力 ~ m*w^2*R = "
              f"{lm * params.CIRCLE_OMEGA ** 2 * params.GRASP_CIRCLE_RADIUS:.3f} N）")
    V0, V1 = _V_common(d0), _V_common(d1)
    print(f"  Lyapunov V 稳态均值         : noload {V0[-len(V0) // 5:].mean():.3e}"
          f" / load {V1[-len(V1) // 5:].mean():.3e}"
          f"（统一 {V_WEIGHT_TAG} 权重，有界）")
    for tag_, d_ in (("noload", d0), ("load", d1)):
        rb = _rms_bound(d_)
        if rb is not None:
            print(f"  (5.7) 均方界 {tag_:>6}        : "
                  f"sup‖d̂‖={rb['d_inf']:.3e} -> 界 {rb['bound']:.3e} "
                  f">= 实测 RMS {rb['rms']:.3e}（保守 ×{rb['margin']:.1f}）")
    print("  解读：控制器名义模型不含杯 -> 负载是 ΔM/Δg 模型失配扰动。")
    if _rms_bound(d0) is None and _rms_bound(d1) is None:
        # 旧 npz：无 d̂ 通道，不能声称 (5.6)/(5.7) 已非空值
        print("  注：本组结果早于 §6.5(6) 的 d̂ 反演改动，扰动口径仍为注入的"
              " J w（S3 不注入，故 d≡0），")
        print("  (5.6)/(5.7) 在本组仍为空值；重跑即可填充（d̂ 不进控制律，"
              "闭环轨迹不变）。")
    else:
        print("  失配已计入反演的 d̂，故 (5.6)/(5.7) 对带载工况也非空值"
              "（不再靠注入的 J w）。")
    print("=" * 78)


METRIC_ROWS = [("T_rms", "位置误差 RMS [m]"), ("O_rms", "姿态误差 RMS"),
               ("exi_rms", "twist 误差 RMS"), ("tau_rms", "力矩 RMS [Nm]"),
               ("gF_mean", "抓握力均值 [N]"),
               ("d_hat_rms", "等效扰动 RMS（反演 d̂）")]


def _print_metric_block(s0, s1, phases, hdr_len):
    """分相位指标表体（b/a 比值）—— 各对比表共用。"""
    for name in [n_ for n_ in phases if n_ in s0 and n_ in s1]:
        for key, note in METRIC_ROWS:
            a, b = s0[name][key], s1[name][key]
            if not (np.isfinite(a) and np.isfinite(b)):
                r_txt = f"{'n/a':>7}"        # 旧 npz 无 d̂ 通道
            elif a <= 1e-15 and b <= 1e-15:
                r_txt = f"{'n/a':>7}"        # 两边均为零（如附着前的抓握力）
            else:
                r_txt = f"{b / a:7.2f}" if a > 1e-15 else f"{'inf':>7}"
            print(f"{name:>9} {key:>9} {a:11.3e} {b:11.3e} "
                  f"{r_txt}  {note}")
        print("-" * hdr_len)


def _print_V_block(entries):
    """V 收敛特性 + 稳定性指标（统一 base 权重，跨组可比）。"""
    print(f"  V 收敛特性（统一换算到 {V_WEIGHT_TAG} 权重，故跨增益组可比；与"
          f" tuned 档的工作集门槛 c* 比余度前需先乘权重比 p_O/16=20）：")
    for tag_, d_ in entries:
        vm = _V_metrics(d_)
        rt = d_["runtime"]
        conv = ("已在带内" if vm["t_conv"] == 0.0
                else (f"{vm['t_conv']:.2f}s" if np.isfinite(vm["t_conv"])
                      else "未回落"))
        print(f"    {tag_:>16}: V_ss={vm['V_ss']:.3e}  "
              f"V_peak={vm['V_peak']:.3e}@{vm['t_peak']:.2f}s  "
              f"附着后回落 2×V_ss 用时 {conv}")
        print(f"    {'':>16}  {_fmt_stab(d_)}；单步耗时 mean/max "
              f"{rt.mean() * 1e3:.2f}/{rt.max() * 1e3:.2f} ms")
        rb = _rms_bound(d_)
        if rb is not None:
            print(f"    {'':>16}  (5.7) 均方界 {rb['bound']:.3e} >= 实测 RMS "
                  f"{rb['rms']:.3e}（保守 ×{rb['margin']:.1f}），"
                  f"sup‖d̂‖={rb['d_inf']:.3e}")


def compare_gains(mode, ref="base", cands=("tuned", "fast")):
    """整定前↔整定后对比（需求 6）：同律、同轨迹、同扰动、同安全机制，
    唯一变量 = (K_d, K_p)。"""
    d_ref = _load_npz(mode, "tndq", ref)
    if d_ref is None:
        return
    have = [(g, _load_npz(mode, "tndq", g)) for g in cands]
    have = [(g, d) for g, d in have if d is not None]
    if not have:
        print(f"[note] [{mode}] 整定后增益组结果缺失，跑 --gains tuned 后自动"
              f"输出整定前/后对比表。")
        return
    s_ref = _phase_stats(d_ref)
    for g, d in have:
        print("=" * 78)
        print(f"C1 增益整定对比（mode={mode}）：{GAIN_LABELS[ref]} → "
              f"{GAIN_LABELS[g]}")
        print(f"  唯一变量 = (K_d, K_p)；轨迹/初始条件/负载/dt/饱和预算/零空间"
              f"治理器全部相同")
        print("=" * 78)
        hdr = (f"{'phase':>9} {'metric':>9} {'整定前':>11} {'整定后':>11} "
               f"{'后/前':>7}  note")
        print(hdr)
        print("-" * len(hdr))
        _print_metric_block(s_ref, _phase_stats(d), PHASE_NAMES + ["circle-ss"],
                            len(hdr))
        _print_V_block([(f"{ref}", d_ref), (f"{g}", d)])
        print("=" * 78)


def compare_laws(mode, gains="base", condition="none"):
    """控制律同台对比（同 mode、同 condition、同轨迹、同力矩出口）：
    C1 TNDQ (5.2) vs C2 忠实 [Ch20] vs C3 DQ-H∞（+ C2-abl 消融档，若已有
    结果）；缺失的基线自动降级为可用子集对比表。"""
    d0 = _load_npz(mode, "tndq", gains, condition)
    if d0 is None:
        return
    base_avail = [(law, lab, _load_npz(mode, law, "base", condition))
                  for law, lab in BASELINE_LAWS]
    base_avail = [(law, lab, d) for law, lab, d in base_avail if d is not None]
    if not base_avail:
        cond_txt = "" if condition == "none" else f"/{condition}"
        print(f"[note] [{mode}{cond_txt}] 无基线（dq-chandra/dq-hinf/dq-ctc）结果，"
              f"跑齐 --law 后自动输出控制律对比表。")
        return
    s0 = _phase_stats(d0)
    stats = [(lab, _phase_stats(d)) for _, lab, d in base_avail]
    print("=" * 78)
    print(f"控制律对比（mode={mode}，condition={condition}，"
          f"C1 增益组={gains}）：C1 TNDQ (5.2) vs "
          + " vs ".join(lab for lab, _ in stats))
    if condition != "none":
        print(f"  敏感条件：{CONDITIONS[condition]}")
    print(f"  公平条件（需求 5）：同参考轨迹/同初始位姿/同负载/同 dt/"
          f"同力矩出口与安全预算；")
    print(f"  C2（K_v=24I, K_P=80I）/C2-abl（K_d=24I, p_O=160, p_T=80）与")
    print(f"  C1 tuned/C3 的线性化 d->e 传递函数逐通道恒等")
    print("=" * 78)
    cols = "".join(f" {lab:>11}" for lab, _ in stats)
    rcols = "".join(f" {lab.split()[0] + '/C1':>7}" for lab, _ in stats)
    hdr = f"{'phase':>9} {'metric':>9} {'C1 tndq':>11}{cols}{rcols}  note"
    print(hdr)
    print("-" * len(hdr))
    for name in [n_ for n_ in PHASE_NAMES + ["circle-ss"]
                 if n_ in s0 and all(n_ in s for _, s in stats)]:
        for key, note in METRIC_ROWS:
            a = s0[name][key]
            vals = [s[name][key] for _, s in stats]
            v_txt = "".join(f" {v:11.3e}" for v in vals)
            r_txt = ""
            for v in vals:
                if not (np.isfinite(a) and np.isfinite(v)):
                    r_txt += f" {'n/a':>7}"       # 旧 npz 无 d̂ 通道
                elif a <= 1e-15 and v <= 1e-15:
                    r_txt += f" {'n/a':>7}"
                else:
                    r_txt += (f" {v / a:7.2f}" if a > 1e-15 else f" {'inf':>7}")
            print(f"{name:>9} {key:>9} {a:11.3e}{v_txt}{r_txt}  {note}")
        print("-" * len(hdr))
    _print_V_block([(f"C1 {gains}", d0)]
                   + [(lab, d) for _, lab, d in base_avail])
    if condition != "none":
        print("  解读（层 3 结构敏感域）：本表唯一变量 = 敏感条件，同预算增益下的")
        print("  差异反映结构属性：C1/C2 解析二阶前馈不受控制周期/轨迹加速度影响；")
        print("  C2-abl/C3 差分前馈滞后与噪声放大 ∝ 1/dt_ctrl，随条件加剧。")
    elif gains == "base":
        print("  解读（忠于实测）：本任务准静态（圆周 ω=1 rad/s），带载稳态误差由")
        print("  对常值重力失配的等效直流刚度决定：C2（刚度 80）/C3（级联 kT*K_servo")
        print(f"  ≈ {np.sqrt(2.0) / params.DQH_GAMMA_T * params.DQH_K_SERVO:.0f}/s²）均高于 base 组的 k_p={float(np.asarray(params.K_P)):.0f} -> 稳态 |T| 更小（增益分配")
        print("  效应，非结构优势；该分配已由 --gains tuned 修正，见增益整定对比表）。")
    elif gains == "tuned":
        print("  解读（忠于实测）：tuned 组与 C2/C2-abl/C3 的线性化 d->e 传递函数逐通道恒等")
        print("  （极点 {-4,-20}、直流刚度 80），因此本表差异不含增益分配成分，只反映")
        print("  结构差别：C1 Aᵀ 整形+证书 vs C2 忠实 [Ch20] screw-log 位姿反馈（无证书）")
        print("  vs C2-abl 差分前馈+朴素 twist 差 vs C3 无二阶通道+内环差分桥接（证书失效）。")
    else:
        print("  解读（忠于实测）：fast 组把两通道极点推到 {-6,-30}（直流刚度 180），")
        print("  已超出各基线的预算，不再是同预算对比；给出的是同一安全约束下")
        print("  式 (5.2) 可达的上限，以及它的代价（附着瞬态抓握力上升）。")
    print("=" * 78)


def export_metrics_csv(path=None):
    """全部已有结果的关键指标汇总 CSV（需求 2：定量分析用数据表）。

    行 = law × gains × mode × condition × phase（含 circle-ss 稳态切片）；
    列 = 分相位误差/力矩/抓握力/等效扰动指标 + 跨组可比的 V 收敛特性
    （统一 base 权重，V_weight 列显式标注口径）+ (5.7) 均方界的实测校验
    + 稳定性计数（饱和/治理器）+ 单步耗时。与终端对比表同源
    （_phase_stats/_V_metrics/_rms_bound），保证图/表/CSV 三者数值一致。

    d_hat_* 列（§6.5(6) 反演的证书通道等效扰动）：对 C1 = 证书真正看到
    的扰动；对各基线（C2/C2-abl/C3）额外含「实际反馈 − 证书反馈」的结构差，因此绝对值
    不可跨律比（同律跨工况比仍公平）；旧 npz 无该通道 -> 空单元格。"""
    import csv
    if path is None:
        path = os.path.join(RESULTS_DIR, "grasp_metrics_summary.csv")
    cols = ["law", "gains", "mode", "condition", "phase",
            "T_rms", "T_max", "O_rms", "O_max", "exi_rms",
            "tau_rms", "tau_max", "gF_mean", "gF_max",
            "d_hat_rms", "d_hat_max",
            "V_ss", "V_peak", "t_peak", "t_conv", "V_weight",
            "d_hat_inf", "exi_rms_run", "rms_bound_5_7", "rms_margin",
            "sat_steps", "gov_steps", "n_steps",
            "runtime_mean_ms", "runtime_max_ms"]

    def _num(v):
        """float -> 科学计数字串；nan/inf（旧 npz 缺 d̂）-> 空单元格。"""
        if isinstance(v, float) and not np.isfinite(v):
            return ""
        return f"{v:.6e}" if isinstance(v, float) else v

    rows = []
    for law in LAWS:
        for gains in (list(GAIN_LABELS) if law == "tndq" else ["base"]):
            for mode in ("noload", "load"):
                for cond in CONDITIONS:
                    d = _load_npz(mode, law, gains, cond)
                    if d is None:
                        continue
                    vm = _V_metrics(d)
                    rb = _rms_bound(d) or {}
                    rt = d["runtime"]
                    common = dict(
                        V_ss=vm["V_ss"], V_peak=vm["V_peak"],
                        t_peak=vm["t_peak"], t_conv=vm["t_conv"],
                        # V 的权重口径（C-2）：与 tuned 档 c* 比余度前需 ×20
                        V_weight=V_WEIGHT_TAG,
                        d_hat_inf=rb.get("d_inf", np.nan),
                        exi_rms_run=rb.get("rms", np.nan),
                        rms_bound_5_7=rb.get("bound", np.nan),
                        rms_margin=rb.get("margin", np.nan),
                        sat_steps=(int(d["sat_steps"])
                                   if "sat_steps" in d else ""),
                        gov_steps=(int(d["gov_steps"])
                                   if "gov_steps" in d else ""),
                        n_steps=(int(d["n_steps"])
                                 if "n_steps" in d else ""),
                        runtime_mean_ms=float(rt.mean()) * 1e3,
                        runtime_max_ms=float(rt.max()) * 1e3)
                    for phase, s in _phase_stats(d).items():
                        row = dict(law=law,
                                   gains=(gains if law == "tndq" else "-"),
                                   mode=mode, condition=cond, phase=phase)
                        row.update({k: _num(v) for k, v in s.items()})
                        row.update({k: _num(v) for k, v in common.items()})
                        rows.append(row)
    if not rows:
        print("[note] 无已有结果，不生成指标汇总 CSV。")
        return None
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    print(f"Saved metrics CSV: {path}  ({len(rows)} 行，"
          f"law×gains×mode×condition×phase 全交叉）")
    return path


def compare_all():
    """全交叉对比：负载维 × 控制律维（C1/C2/C2-abl/C3）× 增益组维 ×
    敏感条件维 + 指标 CSV。负载/增益对比只看标准场景（none）；
    控制律对比表遍历全部条件（层 3 结构敏感域对比的主表）。"""
    for gains in GAIN_LABELS:
        compare(law="tndq", gains=gains)
    compare(law="dq-chandra")
    compare(law="dq-hinf")
    compare(law="dq-ctc")
    for mode in ("noload", "load"):
        compare_gains(mode)
    for cond in CONDITIONS:
        for mode in ("noload", "load"):
            for gains in GAIN_LABELS:
                if _load_npz(mode, "tndq", gains, cond) is not None:
                    compare_laws(mode, gains, cond)
    export_metrics_csv()


# ===========================================================================
# 可视化对比（用户需求 5，--plot 可选；默认不出图，保持纯数值传统）
# ===========================================================================

def plot_compare():
    """时序对比图（law × gains × mode）：|T|、|O|、|τ|、抓握力/净距、V、
    反演的等效扰动 ‖d̂‖。只画已存在的结果；图存
    results/grasp_compare_*.png（旧 npz 缺 d̂ 通道时该曲线自动缺席）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["WenQuanYi Micro Hei", "Noto Sans CJK JP",
                                   "Droid Sans Fallback", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    # (law, gains) -> (颜色, 图例前缀)；mode 用线型区分
    series = {("tndq", "base"): ("tab:blue", "C1 base"),
              ("tndq", "tuned"): ("tab:green", "C1 tuned"),
              ("tndq", "fast"): ("tab:olive", "C1 fast"),
              ("dq-chandra", "base"): ("tab:brown", "C2 [Ch20]"),
              ("dq-ctc", "base"): ("tab:purple", "C2-abl DQ-CTC"),
              ("dq-hinf", "base"): ("tab:red", "C3 DQ-H∞")}
    runs = {}
    for (law, gains), _ in series.items():
        for mode in ("noload", "load"):
            d = _load_npz(mode, law, gains)
            if d is not None:
                runs[(law, gains, mode)] = d
    if not runs:
        print("[note] 无可绘制的结果文件。")
        return

    marks = next(iter(runs.values()))["t_marks"]

    def _axvlines(ax):
        for m_ in marks:
            ax.axvline(float(m_), color="0.85", lw=0.6, zorder=0)

    specs = [
        ("grasp_compare_errors.png", 2, [
            ("位置误差 |T| [m]",
             lambda d: np.linalg.norm(d["e_z"][:, 3:], axis=1), True),
            ("姿态误差 |O|",
             lambda d: np.linalg.norm(d["e_z"][:, :3], axis=1), True),
        ]),
        ("grasp_compare_effort.png", 2, [
            ("力矩范数 |τ| [N·m]",
             lambda d: np.linalg.norm(d["tau"], axis=1), False),
            ("twist 误差 |e_ξ|",
             lambda d: np.linalg.norm(d["e_xi"], axis=1), True),
        ]),
        ("grasp_compare_interaction.png", 2, [
            ("抓握力 |F| [N]（力传感器）",
             lambda d: np.linalg.norm(d["grasp_F"], axis=1), False),
            ("最小净距 [m]（机器人↔椅）",
             lambda d: d["clr_chair"], False),
        ]),
        ("grasp_compare_lyapunov.png", 1, [
            (f"存储函数 V（统一 {V_WEIGHT_TAG} 权重，跨增益组可比；与 tuned 档"
             f" c* 比余度前需 ×p_O/16=20）", _V_common, True),
        ]),
        # §6.5(6)：反演的证书通道等效扰动——S3 不注入 w，故旧口径的
        # d≡0 使 (5.6)/(5.7) 空值；本图把定理 3 诚实条款的全部扰动源
        # （杯子的 ΔM/Δg、噪声、伪逆残差、限幅/治理器、离散化）可视化
        ("grasp_compare_disturbance.png", 1, [
            ("等效扰动 ‖d̂‖（反演，§6.5(6)；C1=证书所见扰动，"
             "各基线额外含结构差 -> 不跨律比绝对值）",
             _d_hat_norm, True),
        ]),
    ]
    for fname, nrow, panels in specs:
        fig, axes = plt.subplots(nrow, 1, figsize=(9, 3.2 * nrow),
                                 sharex=True, squeeze=False)
        axes = axes[:, 0]
        for ax, (title, fn, logscale) in zip(axes, panels):
            for (law, gains, mode), d in runs.items():
                c, lab = series[(law, gains)]
                ls = "-" if mode == "noload" else "--"
                y = fn(d)
                if not np.isfinite(y).any():
                    continue          # 旧 npz 无 d̂ 通道 -> 不画空曲线
                ax.plot(d["t"][:len(y)], y[:len(d["t"])], color=c, ls=ls,
                        lw=1.2, label=f"{lab} {mode}")
            if logscale:
                ax.set_yscale("log")
            _axvlines(ax)
            ax.set_title(title, fontsize=11)
            ax.grid(alpha=0.3)
            if ax.get_legend_handles_labels()[0]:
                ax.legend(fontsize=7, ncol=3)
            else:
                # 全部为旧 npz（无 d̂ 通道）：留空面板 + 明确提示需重跑
                ax.text(0.5, 0.5, "当前结果文件无 d̂ 通道（§6.5(6) 后新增），"
                                  "需重跑 run_grasp_circle.py 填充",
                        fontsize=9, color="0.4", ha="center", va="center",
                        transform=ax.transAxes)
        axes[-1].set_xlabel("t [s]")
        fig.tight_layout()
        out = os.path.join(RESULTS_DIR, fname)
        fig.savefig(out, dpi=150)
        plt.close(fig)
        print(f"Saved plot      : {out}")


def plot_condition_compare(mode="load"):
    """敏感条件分组柱状图（层 3 结构敏感域对比）：condition 组 × 各律柱。

    C1 优先取 tuned（与各基线同预算，残余差异纯属结构），缺失时回退
    base；只画已存在的结果。图存 results/grasp_compare_conditions_{mode}.png。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["WenQuanYi Micro Hei", "Noto Sans CJK JP",
                                   "Droid Sans Fallback", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    bars = [("tndq", "C1 TNDQ", "tab:green"),
            ("dq-chandra", "C2 [Ch20]", "tab:brown"),
            ("dq-ctc", "C2-abl DQ-CTC", "tab:purple"),
            ("dq-hinf", "C3 DQ-H∞", "tab:red")]
    # (相位, 指标, 面板标题)：稳态品质 ×3 + 快相位跟踪品质 ×1
    panels = [("circle-ss", "T_rms", "圆周稳态 |T| RMS [m]"),
              ("circle-ss", "O_rms", "圆周稳态 |O| RMS"),
              ("circle-ss", "exi_rms", "圆周稳态 |e_ξ| RMS"),
              ("transit", "exi_rms", "transit 相位 |e_ξ| RMS")]

    stats = {}          # (condition, law) -> _phase_stats
    c1_gains = {}       # condition -> 实际使用的 C1 增益组
    for cond in CONDITIONS:
        for law, _, _ in bars:
            if law == "tndq":
                for g in ("tuned", "base", "fast"):
                    d = _load_npz(mode, "tndq", g, cond)
                    if d is not None:
                        stats[(cond, law)] = _phase_stats(d)
                        c1_gains[cond] = g
                        break
            else:
                d = _load_npz(mode, law, "base", cond)
                if d is not None:
                    stats[(cond, law)] = _phase_stats(d)
    conds = [c for c in CONDITIONS
             if any((c, law) in stats for law, _, _ in bars)]
    if not conds:
        print(f"[note] [{mode}] 无可绘制的敏感条件结果（需先跑 --condition）。")
        return

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    x = np.arange(len(conds))
    w = 0.2
    for ax, (phase, key, title) in zip(axes.ravel(), panels):
        for i, (law, lab, color) in enumerate(bars):
            vals = [stats.get((c, law), {}).get(phase, {}).get(key, np.nan)
                    for c in conds]
            ax.bar(x + (i - 1.5) * w, vals, w, color=color, label=lab)
        ax.set_yscale("log")
        ax.set_xticks(x)
        ax.set_xticklabels(
            [c + (f"\n(C1 {c1_gains[c]})" if c1_gains.get(c) else "")
             for c in conds], fontsize=8)
        ax.set_title(title, fontsize=11)
        ax.grid(alpha=0.3, axis="y")
        ax.legend(fontsize=8)
    fig.suptitle(f"结构敏感条件 × 各控制律（mode={mode}；"
                 f"同预算增益，差异 = 结构属性）", fontsize=12)
    fig.tight_layout()
    out = os.path.join(RESULTS_DIR, f"grasp_compare_conditions_{mode}.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Saved plot      : {out}")


def main():
    ap = argparse.ArgumentParser(
        description="S3 抓取-搬运实验：抓杯 + 带载圆周 + 空载/带载 × 控制律"
                    "（C1 TNDQ vs C2 忠实[Ch20] vs C3 DQ-H∞，+ C2-abl 消融档）"
                    "全交叉对比")
    ap.add_argument("--mode", choices=["noload", "load"], default=None,
                    help="noload=同轨迹不附着（空载基线）；load=刚性附着+满杯")
    ap.add_argument("--law", choices=list(LAWS), default="tndq",
                    help="控制律：tndq=C1 几何一致 CTC（式 5.2）；"
                         "dq-chandra=C2 忠实 [Ch20] resolved-acceleration 律"
                         "（params.CH20_*）；dq-hinf=C3 一阶 DQ H∞ 基线"
                         "（hdq_hinf 原实现）；dq-ctc=C2-abl 朴素 twist 差"
                         "消融律（params.DQC_*，非 [Ch20] 理论）")
    ap.add_argument("--gains", choices=list(GAIN_LABELS), default="base",
                    help="C1 增益组（params.GAIN_SETS）：base=出厂标量 k_p=16；"
                         "tuned=整定后矩阵 K_p（与 C3 逐通道恒等）；"
                         "fast=敏感性档；整定过程见 tune_tndq_gains.py")
    ap.add_argument("--condition", choices=list(CONDITIONS), default="none",
                    help="结构敏感条件（层 3 对比，只改时间/测量/控制周期，"
                         "路标几何不变）：none=标准场景；highspeed=圆周 ω×2.5；"
                         "fast-transit=搬运相位时长×0.5；noise=编码器级测量"
                         "噪声；coarse-dt=控制周期×3（ZOH）；敏感条件下 C1 "
                         "建议配 --gains tuned 保同预算公平")
    ap.add_argument("--t-end", type=float, default=params.GRASP_T_END)
    ap.add_argument("--compare-only", action="store_true",
                    help="不跑仿真，只打印已有结果的全交叉对比表")
    ap.add_argument("--plot", action="store_true",
                    help="另存时序对比图 + 敏感条件柱状图 "
                         "results/grasp_compare_*.png")
    args = ap.parse_args()

    if args.compare_only:
        compare_all()
        if args.plot:
            plot_compare()
            for m in ("noload", "load"):
                plot_condition_compare(m)
        return 0
    if args.mode is None:
        ap.error("请指定 --mode noload / load（或 --compare-only）")
    if args.law != "tndq" and args.gains != "base":
        ap.error("--gains 仅对 --law tndq 有意义（C2/C2-abl/C3 基线分别用 "
                 "params.CH20_*/DQC_*/DQH_* 增益）")

    path = run(args.mode, args.t_end, law=args.law, gains=args.gains,
               condition=args.condition)
    if path is None:
        return 1
    compare_all()      # 已有结果齐备的维度自动输出对比
    if args.plot:
        plot_compare()
        for m in ("noload", "load"):
            plot_condition_compare(m)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
