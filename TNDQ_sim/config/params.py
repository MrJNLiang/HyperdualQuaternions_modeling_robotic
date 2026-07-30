"""
Central parameter file -- robot model, controller gains, simulation setup.

Everything a user is expected to tune lives here:
  - KUKA LBR4+ 7R modified/standard DH table (theoretical values)
  - controller gains K_d (block-diagonal, Theorem 3(c-2)), k_p
  - H-infinity design parameters kappa, gamma_a (formula (5.6a)/(5.6b))
  - simulation time step / horizon, pseudoinverse damping
"""

import numpy as np

# ---------------------------------------------------------------------------
# Robot: KUKA LBR4+ (7R serial chain), standard DH rows
#   [a, alpha, d, theta_offset, joint_type]   (type 0 = revolute)
# Joint variable theta_i = q_i + theta_offset_i acts about the local z axis
# (Appendix B.1 joint factor); the constant tail is Tz(d) Tx(a) Rx(alpha).
# Format matches core.kinematics.TNDQSerialChain.
# ---------------------------------------------------------------------------

# 2026-07 FK 对齐修正（experiments/diagnose_fk_alignment.py +
# measure_scene_kinematics.py 场景实测）：CoppeliaSim KUKALBR4+_sim.ttt 的
# LBR4p 模型 q=0 时关节 2/4/6 轴（基座系 -y 向）分别过 z=0.251/0.651/1.041，
# 末端 connection 位于 z=1.119、姿态 = I。旧表 (d1=0.340, d5=0.400,
# d7=0.126, alpha3/alpha4 反号) 使 FK 残差达 856 mm/160°，重力补偿方向错误
# 是力矩模式下机械臂塌落的直接根源。修正后残差 < 0.2 mm / 0.01°。
KUKA_LBR4_DH = np.array([
    #  a      alpha        d     theta0  type
    [0.0,  np.pi / 2, 0.251, 0.0, 0],
    [0.0, -np.pi / 2, 0.000, 0.0, 0],
    [0.0,  np.pi / 2, 0.400, 0.0, 0],
    [0.0, -np.pi / 2, 0.000, 0.0, 0],
    [0.0,  np.pi / 2, 0.390, 0.0, 0],
    [0.0, -np.pi / 2, 0.000, 0.0, 0],
    [0.0,  0.0,       0.078, 0.0, 0],
])

N_JOINTS = KUKA_LBR4_DH.shape[0]

# Home / initial joint configuration (away from wrist singularities)
Q_INIT = np.array([0.3, 0.4, -0.2, 0.8, 0.1, 0.6, -0.3])

# E4 大姿态误差初始位形（场景篇 §4.1 S1-c：姿态误差 ≥ 150°，
# 检验 unwinding 处置与参数化退化，总方案 §5.3 E4）。
# 数值由 FK 随机搜索得到：相对杯口上方目标姿态（R_TOOL_DOWN）误差 ~174°
# （近对拓，最严苛档），位置差 ~0.25 m，最小关节限位余量 ~56°
# （纯调节瞬态不致撞限位，区别于旧手选值在 0.75 s 内超限）
Q_INIT_LARGE_ERROR = np.array([-0.05, -0.95, 0.66, 1.11, -1.77, -0.16, -1.33])

# ---------------------------------------------------------------------------
# CoppeliaSim 场景参数（KUKALBR4+_sim.ttt，场景篇 §2/§4/§5）
# 杯子位置优先从场景实时读取（interfaces.cup_position），
# 连接失败/内部后端时回退到下列基座系默认值。
# ---------------------------------------------------------------------------

CUP_POS_DEFAULT = np.array([0.0, 0.625, 0.44])   # 杯口位置（基座系，m；
                                                 #  2026-07 场景实测 /Cup 相对
                                                 #  /LBR4p 位置，旧值为占位）
SETPOINT_HEIGHT = 0.10          # 定点目标 = 杯口上方 0.10 m（预抓取位姿，S1）
CIRCLE_HEIGHT = 0.10            # 圆心 = 杯口上方 0.10 m（S2；2026-07 IK 可达性
                                #  扫描：旧值 0.15 配 R=0.12 在 θ=90° 超出工具
                                #  朝下可达空间，致饱和/失踪/末端下坠穿模）
CIRCLE_RADIUS = 0.06            # 圆周半径 [m]（S2；同上，旧值 0.12 不可达；
                                #  R=0.06/H=0.10 整圈 IK 残差 0、姿态误差 0°、
                                #  最小限位余量 12.3°）
CIRCLE_OMEGA = 1.0              # 默认角速率 [rad/s]（中档；三档 0.25/1.0/2.5）
CIRCLE_OMEGA_FAST = 2.5         # E5 高速域挡（J̇q̇/C q̇ 不可忽略域）
CIRCLE_RAMP_TIME = 2.0          # 五次多项式起步时间 [s]
# 目标姿态：末端 z 轴竖直向下。本 DH 约定下（已用 FK 数值校验）对应
# 绕基座 y 轴旋 180° 的单位四元数 [w,x,y,z] = [0,0,1,0]：
# 修正后 DH 表下 q=0 时末端姿态 = I（工具 z 轴竖直向上），
# Q_INIT_TASK 的 FK 姿态与此目标重合（误差 0°，IK 求解结果）
R_TOOL_DOWN = np.array([0.0, 0.0, 1.0, 0.0])

# S1/S2 任务初始位形：工具朝下、位于定点目标（杯口上方 0.10 m）再上方
# 0.05 m 处的舒适位形（末端 p≈[0.00,0.625,0.59]，姿态误差 0°，最小限位
# 余量 14.5°(j6)；2026-07 基于修正后 DH 表用阻尼 IK + 零空间居中求解。
# 注：曾因 joint6 撞限位换成目标点本身的位形，但那是基座 dynamic
# 翻倒 bug 的连带症状且使趋近段退化为原地保持；根因修复后恢复
# “目标上方 5 cm”的设计语义，趋近段平滑下降可视、可评估）；
# 对接 CoppeliaSim 时也用作初始化关节角，场景篇 §4.1 S1-a 小误差档
Q_INIT_TASK = np.array([1.536, -0.673, 0.094, -0.628, -0.061, -1.842, 1.594])

# ---------------------------------------------------------------------------
# S3 抓取-搬运实验参数（experiments/run_grasp_circle.py）
#
# 几何事实（2026-07 场景实测，基座系）：杯半径 0.044、杯高 0.124（杯口
# z = 杯心 z + 0.062）；椅座面 y∈[0.425,0.875]、座面顶 z≈0.378；RG2 指尖
# 低于 tip 坐标系 0.207 m，两指内距 48 mm < 杯内径 ~80 mm。
#
# 抓取方案 = 内撑式：指尖从杯口开口垂直伸入 30 mm（全程走杯内空气，
# 距杯内壁 ~2 mm 间隙，无穿模），再用力传感器刚性附着模拟抓取（杯子
# 进入机器人动力学树 = 真实动载荷，readForceSensor 测附着交互力旋量）。
# 外夹式悬停（tip = 杯口 +0.28 m）经 IK 扫描证实全程不可达（LBR4 臂展）。
#
# 全部路标经 2026-07 阻尼 IK + 零空间居中逐点验证（相邻路标间线性插值
# 58 采样点 + 带载圆整圈 24 点：残差 < 0.01 mm、姿态误差 0°、最小限位
# 余量 5.4°，无 FAIL 点）。
# ---------------------------------------------------------------------------

# 杯子新位置：座面前缘内侧（仍在椅上，y=0.48 > 座面前缘 0.425），
# 比默认位 y=0.625 近 14.5 cm，使内撑式抓取链全程落在可达空间内
# （旧位 hover z=0.714 处 IK 残差 43 mm 不可达 -> 垂坠穿模的根源之一）
CUP_POS_GRASP = np.array([0.0, 0.48, 0.44])   # 杯心（基座系，杯口 z=0.502）

# 相位路标（tip 目标位置，姿态恒为 R_TOOL_DOWN）：
#   hover  指尖高于杯沿 5 mm：tip_z = 0.502 + 0.207 + 0.005 = 0.714
#   grasp  指尖伸入杯口 30 mm：tip_z = 0.502 - 0.030 + 0.207 = 0.679
#   lift   提杯到横穿高度 z=0.718（附着后垂直提杯 39 mm，杯底 0.417 >
#          椅面 0.378；指尖名义 0.511 > 杯沿 0.502，仿真内静态标定净距
#          10 mm，再叠加 lock_gripper_fingers 防手指下垂吃余量）
#   retreat 保持横穿高度移出杯口正上方（杯外壁 y=0.436 -> 退到 y=0.41，
#          横穿杯沿时段的净距不再被斜线下降提前吃掉）
#   transit 斜线下降到椅子前方自由空域（retreat y=0.41 已在椅前缘
#          y=0.425 之外，后续下降段不再经过椅面上空）
GRASP_TIP_HOVER = np.array([0.0, 0.48, 0.714])
GRASP_TIP_GRASP = np.array([0.0, 0.48, 0.679])
GRASP_TIP_LIFT = np.array([0.0, 0.48, 0.718])
GRASP_TIP_RETREAT = np.array([0.0, 0.41, 0.718])
GRASP_TIP_TRANSIT = np.array([0.0, 0.27, 0.68])

# 带载圆周：圆心在椅子前方自由空域（圆最大 y = 0.33 距椅前缘
# y=0.425 余 95 mm，包容带载横向跟踪误差；2026-07 实测 y=0.30 时
# 带载误差会让机构在圆周段近椅侧出现零净距 -> 内移到 y=0.27，
# 含整圈 45 点 IK 扫描验证限位余量 7.2°）
GRASP_CIRCLE_CENTER = np.array([0.0, 0.27, 0.60])
GRASP_CIRCLE_RADIUS = 0.06      # 同 S2 可达性结论（R=0.12 不可达）

# 相位时间线 [s]（五次/余弦平滑段，首尾速度加速度为 0，
# 附着瞬间发生在静止保持段中点 -> 刚性连接无速度跳变冲击）：
#   [0, T1]        hover -> grasp 下插
#   [T1, T2]       静止保持；中点 t=(T1+T2)/2 时刚性附着（带载模式）
#   [T2, T3]       grasp -> lift 垂直提升
#   [T3, T4]       lift -> retreat 保持高度横移（离开杯口/杯沿正上方）
#   [T4, T5]       retreat -> transit 斜线过渡
#   [T5, T6]       transit -> 圆心下降
#   [T6, t_end]    带载圆周（CIRCLE_OMEGA，ramp CIRCLE_RAMP_TIME）
GRASP_T_DESCEND = 2.0           # T1
GRASP_T_ATTACH_HOLD = 1.5       # T2-T1（附着发生在 t = T1 + 0.5 s，余下
                                # 1 s 静置让刚性焊接瞬态/杯摆动衰减）
GRASP_T_LIFT = 1.5              # T3-T2
GRASP_T_RETREAT = 1.0           # T4-T3
GRASP_T_TRANSIT = 2.0           # T5-T4
GRASP_T_DESCEND2 = 1.5          # T6-T5
GRASP_T_END = 22.5              # 总时长：圆周段 13 s（ramp 2 s + 稳态 >1.5 圈）

# 负载质量：附着时把杯质量从 0.05 kg 改写为此值（模拟装水杯，放大
# 负载效应；控制器名义模型不含杯 -> 负载 = 模型失配扰动，
# 由定理 3(c)/(d) H∞/ISS 证书兜底，正是空载/带载对比的实验变量；
# 2026-07 实测 0.5 kg 时带载下垂 ~25 mm 会吃光杯沿/椅面几何余量，
# 降为 0.25 kg（仍为空杯的 5 倍，负载效应清晰且无穿模）
CUP_LOAD_MASS = 0.25            # [kg]

# 抓取链初始位形 = hover 位形（IK 解，姿态误差 0°，最小限位余量 5.4°(j6)）
Q_INIT_GRASP = np.array([1.377, -0.327, 0.327, -0.828, -0.114, -2.0, 1.641])

# S3 结构敏感条件（experiments/run_grasp_circle.py --condition，2026-07）：
# 只改轨迹时间/测量/控制周期参数，路标几何与场景完全不变（IK 验证
# 仍有效，无穿模风险）——把三律的结构差异（解析 vs 差分前馈、Aᵀ 整形）
# 推到线性化失效/高频域，使其可观测（准静态任务下三律同预算必然趋同）。
GRASP_FAST_PHASE_SCALE = 0.5    # fast-transit：lift/retreat/transit/descend2 时长 ×0.5
GRASP_CTRL_DECIM = 3            # coarse-dt：控制周期 = 引擎 dt × 3（5→15 ms，ZOH
                                #  保持力矩；差分前馈一拍滞后 ×3，解析前馈不受影响）

# ---------------------------------------------------------------------------
# 扰动/实验条件参数（E1–E7，总方案 §5.3 + 场景篇 §6）
# ---------------------------------------------------------------------------

MISMATCH_SCALE = 1.2            # E3：控制器名义惯性参数整体高估 20%
NOISE_SIGMA_Q = 5.0e-5          # E6：关节角噪声 σ [rad]（16 bit 编码器量级）
NOISE_SIGMA_QDOT = 1.0e-3       # E6：关节速度噪声 σ [rad/s]（差分+低通残留）
CONTACT_T_ON = 4.0              # E7（内部后端仿真）：接触力脉冲起始 [s]
CONTACT_DURATION = 0.3          # E7：脉冲时长 [s]（场景篇 §6.3 接触时长量级）
# E7 接触建模：椅背擦碰 = 末端受到的有限支撑接触力 F_ext（基座系，N），
# 经 τ_ext = Jᵀ [0; F_ext] 映射到关节力矩（力臂自动决定各关节分担，
# 腕部小惯量关节不会被非物理地直接注入大力矩）；幅值 ~6 N 为
# 轻度擦碰量级，半正弦包络保证 C0 连续
CONTACT_FORCE = np.array([-5.0, 3.0, -2.0])   # 末端接触力峰值 [N]（基座系）

# TODO: POE (screw-axis) parameters of the same robot for the CoppeliaSim
# interface (interfaces/coppeliasim_interface.py) once the scene is fixed.

# ---------------------------------------------------------------------------
# Controller gains -- formula (5.2) and Theorem 3
# ---------------------------------------------------------------------------

# Block-diagonal K_d = diag(K_omega, K_v)  (enables split criterion (5.6b))
K_OMEGA = 8.0 * np.eye(3)      # rotational twist-error gain  [1/s]
K_V = 8.0 * np.eye(3)          # translational twist-error gain [1/s]
K_D = np.block([
    [K_OMEGA, np.zeros((3, 3))],
    [np.zeros((3, 3)), K_V],
])

K_P = 16.0                     # scalar pose-error gain (>0, Theorem 3)

# ---------------------------------------------------------------------------
# 增益整定（2026-07，control/gain_design.py 系统性设计 +
# experiments/tune_tndq_gains.py 报告；S3 实测数据驱动）
#
# 诊断：近单位误差处误差体系解耦为两条二阶通道（A -> A0 = diag(-I/2, I)）
#     旋转   Ö + K_ω Ȯ + (p_O/4) O = -d_ω/2
#     平移   T̈ + K_v Ṫ +  p_T   T = +d_v
# 标量 k_p（p_O = p_T = k_p）使旋转通道刚度只有平移的 1/4：上面的
# base 组（K_d=8I, k_p=16）平移临界阻尼（双重极点 -4）、但旋转极点
# 退化为 {-0.536, -7.46}，主导极点慢 7.5 倍、直流误差增益 0.125
# （C3 基线等效 0.00625，差 20 倍）—— 这正是 S3 带载稳态 |O| 落后
# C3 一个数量级的根因（实测比 0.08 vs 线性预测 0.05）。
#
# 定理 3 允许把 k_p 推广为对称正定矩阵 K_p：存储函数取
# V = ½‖e_ξ‖² + ½ e_zᵀ K_p e_z、反馈取 -Aᵀ K_p e_z 时交叉项仍精确
# 抵消（gain_design 模块首段推导 + tune_tndq_gains 数值核验），
# (5.6a)/(5.6b)、认证 L2 增益 1/λmin(K_d)、ISS 球 (5.7) 全部照搬。
#
# tuned 组 = “与 C3 同预算”设计点：令两条通道的特征多项式与扰动
# 输入系数都与 C3 等效模型 (s+4)(s+20) 完全相同 ->
#     K_ω = K_v = 24,  p_O = 320,  p_T = 80
# 即 d -> (O,T) 传递函数逐通道恒等（同 DC 刚度 80、同极点、同阻尼
# ζ=1.342），残余差异纯属结构差异（前馈精度/附着瞬态/证书）。
# 副产品：λmin(K_d) 8 -> 24，认证 L2 增益 0.125 -> 0.042、ISS 球缩紧 3 倍。
# 可行性：最快极点 -20 @ dt=5 ms -> |p|dt=0.1（C3 已在同 dt 下实测稳定）。
# fast 组（主导 -6、极点 {-6,-30}）用于参数敏感性验证：|p|dt=0.15 已到
# 显式积分余量边界，指令峰值预算亦更紧。
# ---------------------------------------------------------------------------

def _gain_set(K_omega, K_v, p_O, p_T):
    """K_d = diag(K_ω I3, K_v I3)、K_p = diag(p_O I3, p_T I3)。"""
    return {
        "K_d": np.diag(np.r_[np.full(3, K_omega), np.full(3, K_v)]),
        "k_p": np.diag(np.r_[np.full(3, p_O), np.full(3, p_T)]),
    }


GAIN_SETS = {
    # 原始出厂组（标量 k_p；run_simulation.py 回归结果对应此组）
    "base": {"K_d": K_D, "k_p": K_P},
    # 与 C3 同预算的整定组（推荐）
    "tuned": _gain_set(24.0, 24.0, 320.0, 80.0),
    # 敏感性档：主导极点再快 1.5 倍（{-6,-30}），逼近 dt 余量
    "fast": _gain_set(36.0, 36.0, 720.0, 180.0),
}

# H-infinity design parameters (Theorem 3(c)):
#   (5.6a) requires K_d >= 1/2 (kappa^-1 + gamma_a^-2) I
# With lambda_min(K_d) = 8: e.g. kappa = 1, gamma_a = 0.5 gives level
# 0.5*(1 + 4) = 2.5 <= 8  -> satisfied with margin.
KAPPA = 1.0                    # e_xi weighting in (5.6)
GAMMA_A = 0.5                  # prescribed L2 gain bound (>= theoretical best)

# Per-channel parameters for the split criterion (5.6b)
KAPPA_W, GAMMA_W = 1.0, 0.5
KAPPA_V, GAMMA_V = 1.0, 0.5

# ---------------------------------------------------------------------------
# 基线控制律 C3（一阶 DQ H∞ 运动学律，hdq_hinf_coppeliasim 原实现）
# —— S3 实验的控制律对比参数（experiments/run_grasp_circle.py --law dq-hinf）
#
# 公平性设计（总方案 §5.2：同一力矩出口、同一预算）：基线的名义线性化
# 误差衰减率对齐 TNDQ 律的主导极点 -4 /s（K_d=8, k_p=16 -> s²+8s+16 临界
# 阻尼双重极点）：
#   平移通道  Tdot ≈ -kT·T          -> kT = 4  => gamma_T = sqrt(2)/4
#   姿态通道  Odot ≈ -(kO/2)·O      -> kO = 8  => gamma_O = sqrt(2)/8
# （原仓库实验取 gamma_O=gamma_T=1.0，带宽仅 ~1.4 /s，直接对比会把差距
# 归因于调参而非结构；带宽对齐后剩余差距才是理论结构差异：无 e_ξ
# 阻尼通道、无 ξ̇_d/J̇q̇ 解析前馈、无 ad 输运修正）
DQH_GAMMA_O = np.sqrt(2.0) / 8.0   # 姿态通道 H∞ 水平 -> kO = 8
DQH_GAMMA_T = np.sqrt(2.0) / 4.0   # 平移通道 H∞ 水平 -> kT = 4
DQH_DAMPING = 1e-3                 # 基线原实现的阻尼伪逆 λ（原值保留）
# 速度级指令 -> 共享力矩接口的内环速度伺服：
#   q̈_ref = Δq̇_cmd/dt + K_SERVO (q̇_cmd - q̇)
# K_SERVO 高于外环带宽 5 倍以上且远低于采样频率 1/dt=200 Hz；
# 差分前馈的一拍滞后/差分噪声是 C3 缺二阶通道的真实属性（README C3 行）
DQH_K_SERVO = 20.0                 # 内环速度伺服增益 [1/s]

# ---------------------------------------------------------------------------
# 基线控制律 C2（二阶 DQ 计算力矩律，文献式 DQ CTC；总方案 §5.2 对比行
# “现有 DQ 动力学控制”）—— S3 三方对比（--law dq-ctc）
#
# 结构（control/control_law.py::dq_ctc_law）：
#   u_task = ξ̇_d^num + K_d (ξ_d − ξ) + [p_O·O; −p_T·T] − (J̇q̇)^num
# 与 C1 (5.2) 的差异：朴素 twist 差（无 Ad 输运 -> §4.1 伪项）、无 Aᵀ
# 整形（Lyapunov 交叉项不消 -> 无定理 3 证书）、前馈/J̇q̇ 用数值差分
# （一拍滞后 + 差分噪声，vs TNDQ σ² 通道免构造解析读出 (3.5)）。
#
# 公平性设计（同预算准则，与 tuned/C3 逐通道恒等）：C2 线性化通道为
#   旋转  Ö + K_ω Ȯ + (p_O/2) O = −d_ω/2   （只有 Ȯ=−ω̃/2 一个 ½ 因子）
#   平移  T̈ + K_v Ṫ +  p_T   T = +d_v
# 与 C1 tuned（旋转刚度 p_O/4 = 80）/ C3（DC 刚度 kO·K_SERVO/2 = 80）
# 对齐到同一特征多项式 (s+4)(s+20)、同 DC 刚度 80：
#   K_d = 24 I6,  p_O/2 = 80 -> p_O = 160,  p_T = 80
# 三方 d -> (O,T) 名义传递函数完全一致，残余差异纯属结构差异。
DQC_K_D = 24.0 * np.eye(6)                                  # twist 差增益
DQC_K_P = np.diag(np.r_[np.full(3, 160.0), np.full(3, 80.0)])  # 位姿增益


# ---------------------------------------------------------------------------
# Numerics
# ---------------------------------------------------------------------------

DT = 1e-3                      # integration step [s] (semi-implicit Euler)
COPPELIA_DT_TARGET = 5e-3      # CoppeliaSim 引擎步长目标值 [s]：场景默认 50 ms
                               #  对力矩闭环过粗（K_d=8/K_p=16 下发散撞限位），
                               #  connect() 时尝试改写并以实际读回值为控制周期
T_END = 10.0                   # simulation horizon [s]
PINV_DAMPING = 1e-6            # damped pseudoinverse lambda (Sec. 5, honesty (i))
QDDOT_MAX = 40.0               # q̈_ref 范数限幅 [rad/s²]（力矩/限位预算保护；
                               #  饱和残差计入 d(t)，定理 3 诚实条款）

# 关节级安全治理器（场景篇 §6.3 安全机制，E4 大姿态误差纯调节必需）：
# 1) 速度限幅：LWR4+ 手册额定关节速度，超速且指令仍加速时改为制动；
# 2) 限位预测制动：刹车距离 d_stop = q̇²/(2 A_BRAKE) 进入缓冲区即强制减速。
# 治理引入的指令偏差同样计入 d(t)（定理 3 诚实条款）。
QDOT_MAX = np.deg2rad(np.array([110.0, 110.0, 128.0, 128.0,
                                204.0, 184.0, 184.0]))   # 额定速度 [rad/s]
A_BRAKE = 25.0                 # 安全制动减速度 [rad/s²]（< QDDOT_MAX）
LIMIT_BUFFER = 0.08            # 限位缓冲区 [rad]（约 4.6°，含检测 margin）

# 零空间冗余消解（7R 臂 n=7 > m=6，任务一致投影 N = I - J⁺J）：
#   q̈_ref += N (K_NS (q_center - q) - D_NS q̇)
# 关节居中项推离限位（LWR4+ 限位对称，q_center = 0）；零空间分量
# 不改变 ξ = Jq̇ 的任务空间动态，定理 1–3 的 e_ξ/e_z 误差体系不受影响
# （阻尼伪逆下的泄漏项计入 d(t)，定理 3 诚实条款 (i)）。
NULLSPACE_K = 4.0              # 居中刚度 [1/s²]
NULLSPACE_D = 4.0              # 零空间阻尼 [1/s]
Q_CENTER = np.zeros(7)         # 关节居中参考（限位中点）
SINGULARITY_TOL = 1e-3         # 奇异监控：σ_min(J) 低于此值时提升阻尼并告警
SINGULARITY_DAMPING = 5e-2     # 奇异附近的加大阻尼（残差计入 d(t)，定理 3 诚实条款 (i)）
REPROJECT_EVERY = 50           # steps between (3.8) reprojections (Sec. 3.4)
LOG_EVERY = 10                 # log every k-th step to keep tables readable

# Unit-constraint alarm thresholds for the residual family (3.8)
C0_TOL = 1e-9
C1_TOL = 1e-9
C2_TOL = 1e-9
