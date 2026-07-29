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

KUKA_LBR4_DH = np.array([
    #  a      alpha        d     theta0  type
    [0.0,  np.pi / 2, 0.340, 0.0, 0],
    [0.0, -np.pi / 2, 0.000, 0.0, 0],
    [0.0, -np.pi / 2, 0.400, 0.0, 0],
    [0.0,  np.pi / 2, 0.000, 0.0, 0],
    [0.0,  np.pi / 2, 0.400, 0.0, 0],
    [0.0, -np.pi / 2, 0.000, 0.0, 0],
    [0.0,  0.0,       0.126, 0.0, 0],
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

CUP_POS_DEFAULT = np.array([0.55, 0.10, 0.25])   # 杯口位置（基座系，m，
                                                 #  位于 Q_INIT_TASK 末端前下方）
SETPOINT_HEIGHT = 0.10          # 定点目标 = 杯口上方 0.10 m（预抓取位姿，S1）
CIRCLE_HEIGHT = 0.15            # 圆心 = 杯口上方 0.15 m（S2）
CIRCLE_RADIUS = 0.12            # 圆周半径 [m]（S2）
CIRCLE_OMEGA = 1.0              # 默认角速率 [rad/s]（中档；三档 0.25/1.0/2.5）
CIRCLE_OMEGA_FAST = 2.5         # E5 高速域挡（J̇q̇/C q̇ 不可忽略域）
CIRCLE_RAMP_TIME = 2.0          # 五次多项式起步时间 [s]
# 目标姿态：末端 z 轴竖直向下。本 DH 约定下（已用 FK 数值校验）对应
# 绕基座 y 轴旋 180° 的单位四元数 [w,x,y,z] = [0,0,1,0]：
# 舒适位形 Q_INIT_TASK 的 FK 姿态距此目标仅 ~9°（tool_z ≈ [0.14,0,-0.99]）
R_TOOL_DOWN = np.array([0.0, 0.0, 1.0, 0.0])

# S1/S2 任务初始位形：前伸 + 工具朝下的舒适位形（末端 p≈[0.62,0,0.32]，
# 避免从竖直向上位形做 180° 翻转导致伪逆路径撞关节限位；
# 对接 CoppeliaSim 时也用作初始化关节角，场景篇 §4.1 S1-a 小误差档）
Q_INIT_TASK = np.array([0.0, -0.7, 0.0, 1.4, 0.0, -0.9, 0.0])

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
# Numerics
# ---------------------------------------------------------------------------

DT = 1e-3                      # integration step [s] (semi-implicit Euler)
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
