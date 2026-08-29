"""
TNDQ_b601 中央参数文件 -- 机器人模型 / 增益 / 任务几何 / 数值设置。

面向 Seeed Studio reBot Arm B601-DM（6R 桌面臂 + 平行夹爪）的 Isaac Sim
仿真验证与真机部署。用户可调参数集中于此。

数据源与验证链：
  - B601_DH_TABLE / B601_TOOL_ANGLE / B601_BASE_PREFIX：
    reBot-Isaacsim/urdf/reBot_B601_DM/urdf/reBot_B601_DM.urdf 提取，
    experiments/verify_dh.py 三路对账（URDF FK / DH 矩阵 FK / TNDQ DQ 链
    FK），200 随机 q 残差 < 3e-6 m / 2e-5 rad（残差源于 URDF 用 3.1416/
    -1.5708 近似 pi，固有差异）。
  - 关节限位 / 力矩上限：URDF <limit> 标签（j1-j3 effort 27 N*m、
    j4-j6 effort 7 N*m；速度标签 50/200 单位宽松，治理器另取保守值）。
  - 任务几何：experiments/design_task_geometry.py 阻尼 IK + 限位余量
    验证后回填（见各常量注释）。
"""

import os
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# 机器人：reBot B601-DM（6R），标准 DH 行 [a, alpha, d, theta_offset, type]
#   （type 0 = revolute；格式与 core.kinematics.TNDQSerialChain 一致）
#
# 提取自 URDF（q=0 逐对相邻关节轴几何；experiments/verify_dh.py 的
# extract_dh() 为数值参考实现，手写表与其输出逐字段对齐）：
#   - joint2 负轴 (0,0,-1) 由 DH 帧 z 轴直接取 URDF 轴方向（含符号）吸收；
#   - 平行轴（z2 // z3）公垂线滑动自由度按提取器的参考点连线分配；
#   - q=0 时 URDF gripper_link 姿态 = I（工具 rpy 逐段相消）、位置
#     (0.26031, 0, 0.1917)。
# ---------------------------------------------------------------------------

_L34 = np.hypot(0.2426, 0.054)              # joint3->joint4 轴间距离
_PHI34 = np.arctan2(0.054, 0.2426)          # 该连杆对 z 轴的倾角

B601_DH_TABLE = np.array([
    #  a          alpha            d          theta0             type
    [0.020084,   np.pi / 2,       0.05555,   0.0,                   0],  # joint1
    [0.264,      np.pi,          -0.031625,  np.pi,                 0],  # joint2
    [_L34,       0.0,            -0.001625,  np.pi - _PHI34,        0],  # joint3
    [-0.078308,  np.pi / 2,      -0.03,      _PHI34 - np.pi,        0],  # joint4
    [0.0,        np.pi / 2,       0.0025,    -np.pi / 2,            0],  # joint5
    [0.0,        np.pi / 2,       0.183402,  0.0,                   0],  # joint6
])

# DH 链末端帧 -> URDF gripper_link 的常量尾变换：纯旋转 E = Rz(pi/2)
# （原点重合：gripper_link 原点恰在关节 6 轴上，d_6 已吸收全部平移；
# E 不能并入 theta_6 偏置，因为 alpha_6 != 0 时两者绕不同轴）。
# 实验侧使用：x = dq_mul(chain.fkm(q), E_dq)，E_dq 由 tool_dq() 构造。
B601_TOOL_ANGLE = np.pi / 2

# 基座前缀：URDF base_link -> DH 链基座（原点 = joint1 轴上的 URDF 关节
# 原点，姿态恒等）。x 偏移 -8.416e-5 m 为制造装配偏心，无法并入 DH，
# 接口层（Isaac / 真机）读到的基座系位姿需先减去此前缀再进入 TNDQ 链。
B601_BASE_PREFIX = np.array([-8.416e-05, 0.0, 0.08465])

N_JOINTS = B601_DH_TABLE.shape[0]           # = 6

# tests/test_math_properties.py 的历史接口名（其测试为 TNDQ 代数性质，
# 与具体机器人无关，链仅作激励源；沿用 TNDQ_sim 的导入名以保持测试
# 副本逻辑一致，见 simdata.input_simulation.default_joint_sine_6r）。
KUKA_LBR4_DH = B601_DH_TABLE


def tool_dq():
    """尾变换 E 的单位 DQ（纯旋转 Rz(B601_TOOL_ANGLE)）。"""
    from core.dq_algebra import dq_rot_z
    return dq_rot_z(B601_TOOL_ANGLE)


# ---------------------------------------------------------------------------
# 关节限位 / 力矩上限 / 速度治理（URDF <limit>）
# ---------------------------------------------------------------------------

JOINT_LOWER = np.array([-2.8, -3.14, -3.14, -1.87, -1.57, -3.14])
JOINT_UPPER = np.array([2.8, 0.0, 0.0, 1.57, 1.57, 3.14])
JOINT_MID = 0.5 * (JOINT_LOWER + JOINT_UPPER)   # 限位中点（诊断参考）

TAU_MAX = np.array([27.0, 27.0, 27.0, 7.0, 7.0, 7.0])   # [N*m]（URDF effort）

# URDF velocity 标签（50/200）单位宽松不构成约束；安全治理器取桌面臂
# 保守值（圆周跟踪 ω=1 rad/s、R=0.06 m 时关节速度 << 1 rad/s）
QDOT_MAX = 3.0 * np.ones(6)              # 额定速度上限 [rad/s]
A_BRAKE = 20.0                           # 限位预测制动减速度 [rad/s^2]
LIMIT_BUFFER = 0.05                      # 限位缓冲区 [rad]（v14：原 0.08 时
                                            #   Q_INIT 落在缓冲区内部，起步
                                            #   瞬态正速度反复触发预测制动，
                                            #   与反馈互追成 ~15 Hz 极限环颤振；
                                            #   缩到 0.05 配合 Q_INIT=-0.10 使
                                            #   起点出缓冲区。高速保护主要靠
                                            #   q̇²/(2·A_BRAKE) 预测项，缓冲
                                            #   缩小不削弱运动段安全）

# ---------------------------------------------------------------------------
# 控制器增益 -- 公式 (5.2) 与定理 3
#
# B601 与 LBR4+ 的适配说明：
#   - 增益作用于误差动力学（极点配置），不直接依赖质量；B601 连杆总质量
#     ~1.87 kg（LBR4 ~17 kg），同增益下所需力矩小一个量级，27/7 N*m
#     预算充分；
#   - tuned 组（K_d=24, p_O=320, p_T=80 -> 通道极点 {-4,-20}）在
#     dt = 2 ms 下 |p|*dt = 0.04，显式积分余量充足（TNDQ_sim 在 5 ms
#     下已实测稳定）；
#   - B601 为 6R 非冗余臂（n = m = 6），无零空间投影（TNDQ_sim 的
#     NULLSPACE_* 仅对 7R 有意义，不移植）。
# ---------------------------------------------------------------------------

def _gain_set(K_omega, K_v, p_O, p_T):
    """K_d = diag(K_w I3, K_v I3)、K_p = diag(p_O I3, p_T I3)。"""
    return {
        "K_d": np.diag(np.r_[np.full(3, K_omega), np.full(3, K_v)]),
        "k_p": np.diag(np.r_[np.full(3, p_O), np.full(3, p_T)]),
    }


GAIN_SETS = {
    # 保守基线（TNDQ_sim 出厂组 K_d=8I、k_p=16 标量的矩阵等价形式）：
    # 平移通道临界阻尼双重极点 -4，旋转极点 {-0.536, -7.46}
    "base": _gain_set(8.0, 8.0, 16.0, 16.0),
    # KUKA tuned 组（两通道极点均 {-4, -20}）：大型臂高刚度设计点，
    # B601 实测偏激进（exp1 dither18/19：稳态 q̈_ref≈20、关节蠕动振荡）
    "tuned": _gain_set(24.0, 24.0, 320.0, 80.0),
    # B601 小臂适配组：通道极点 {-2, -8}（K=a+b=10、p=ab=16，旋转
    # p_O=4 p_T）。小型臂硬件精度低（材料/装配公差、弹性形变、
    # 传感器噪声），且真机反馈计算 ~10 ms：高刚度会把残差/噪声放大
    # 成振荡力矩签名（实测卡滞期 0.1~0.2 rad/s 蠕动、q̈_ref≈20）。
    # 降刚度后收敛慢一些但容差大；定理 3(c) 证书仍成立
    # （λ_min(K_d)=10 ≥ 2.5 水平，裕量充足）
    "small_arm": _gain_set(10.0, 10.0, 64.0, 16.0),
    # 抓取精度组（v10）：顶抓要求毫米级到位。small_arm p_T=16 下稳态
    # 偏差 e_ss=扰动/p_T≈15 mm（恒定扰动 ~0.24），抓不到方块。抓取位
    # 构型良态（sigma_min≈0.15、限位余量 47.7°）且无 dither，提刚度安全。
    # 平移通道临界阻尼双重极点 -8（K_v=16, p_T=64），e_ss 降到 ~4 mm；
    # 旋转同步 p_O=4 p_T 保双通道一致。
    "grasp": _gain_set(16.0, 16.0, 256.0, 64.0),
}
DEFAULT_GAIN_SET = "grasp"

# H-infinity 设计参数（定理 3(c)，(5.6a)：K_d >= 1/2 (kappa^-1+gamma_a^-2) I）：
# lambda_min(K_d) = 24 时 kappa=1、gamma_a=0.5 -> 水平 2.5 <= 24，裕量充足
KAPPA = 1.0
GAMMA_A = 0.5

# ---------------------------------------------------------------------------
# 任务几何（世界系 = URDF base_link 系；Isaac 场景中机器人 base_link 原点
# 与世界原点重合、立于地面上。任务目标进 TNDQ 链 / IK 前须减
# B601_BASE_PREFIX（纯平移），见 experiments/ik_lib.world_to_dh）
#
# 抓取方案（v9 斜向下抓，数据驱动演进 v1~v8 -> v9）：
#   v7 顶抓被用户 GUI 截图证伪（腕部 motor5 压方块、夹爪水平不对准）；
#   v8 纯侧抓（tilt=90 deg）被两个硬约束排除：
#     [a] 接触包络物理测绘（diag_envelope.py，探针立方体逐点扫描）：
#         gripper_link 原点 = 指尖端（闭合时指尖收到 x≈0），手指/
#         本体向 -x_g（腕后侧）延伸，指垫中心 ≈ 局部 (-0.015, 0,
#         -0.005)。纯顶抓时指尖刀片（y≈0）先捅方块顶面，几何排除；
#     [b] 深折叠构型（q2≈-2.3、q4≈-1.74 贴限位，margin 7.6 deg）在
#         力矩层完全保持不住（kinematic dp≈0.01 vs 力矩保持
#         dp≈0.07，diag_kinematic_hold.py），就位即被顶出。
#   v9 根因修复：斜向下抓（tilt=120 deg，x_g 斜向下 60 deg），
#     构型健康（margin 31.5 deg、q4=-0.61 远离限位）；抓取距离按
#     包络测绘重标：d=0.028（原点在方块面外 ~5 mm，指垫覆盖方块
#     面外侧 32 mm 区段；旧 d=0.054~0.12 全超出指垫伸展范围）。
#     Isaac 功能验证（diag_v9_grasp.py）：kinematic 逼近全程方块
#     移位 ≤1 mm（零碰撞）。
#   运动学形态（用户指定）：举高 -> 前移 -> 下降前段腕部转动使
#      夹爪朝下 -> 沿接近轴斜下插入 -> 闭合夹持 -> 带载提升。
#
# 工具约定（v9，接触包络物理测绘标定）：
#   - 控制末端 = URDF gripper_link 原点 = 指尖端平面（verify_dh 对账）；
#   - 手指沿 gripper -x_g 延伸（本体 x[-0.16,-0.07]、宽 ±0.10；
#     指尖收到原点 x≈0）；开合沿 ±y_g（prismatic 行程 0~0.0715 m）；
#   - 指垫中心 ≈ 原点 + (-0.015 x_g - 0.005 z_g)（闭合包络质心）；
#   - 抓取位：原点 = 方块中心 + 0.028 x_g + 0.005 z_g（垫心≈方块中心）；
#   - R_TOOL_QUAT：make_tool_pose(tilt=120 deg, phi=-14.4 deg)；
#   - CUBE_YAW：立方体绕 z 摆放角，一对侧面法向对准 ±y_g。
# ---------------------------------------------------------------------------

CUBE_SIZE = 0.045                          # 立方体边长 [m]（4.5 cm；恢复 v14
                                            #   口径。v15 接触抓取仿真验证裁决：
                                            #   45 mm 低于夹爪最小夹持宽度
                                            #   ~66 mm（楔形指内面过盈闭合时
                                            #   弹出方块）；放大到 70 mm 又因
                                            #   竖直棱投影 ±30.3 mm 超指平板
                                            #   带 ±19.6 mm，插入段即撞楔形
                                            #   刀片——立方体任何尺寸都无法被
                                            #   该夹爪 USD 模型平行夹持（指面
                                            #   视觉平整但碰撞体积多出无形部
                                            #   分）。用户裁决：接受仿真极限，
                                            #   直接上机验证（物理夹爪指面平
                                            #   整可夹），仿真保持 45 mm 口径。
                                            #   抓取尝试数据见 results/
                                            #   exp1_grasp_70mm_attempt.csv；
                                            #   GRIPPER_GRASP = CUBE_SIZE-2mm
                                            #   自动跟随）

# 支柱（静态立方体，Isaac 场景）：方块置于柱顶，斜向下抓从外侧
# 斜插入指，柱截面 0.035 小于开指间距，指/本体不碰柱
PEDESTAL_H = 0.06                          # 支柱高 [m]
PEDESTAL_SIZE = 0.035                      # 支柱截面边长 [m]

# 方块：径向 0.40 m、方位 -14.4 deg，置于支柱顶
CUBE_POS = np.array([0.3874, -0.0995, PEDESTAL_H + CUBE_SIZE / 2])
CUBE_YAW = np.deg2rad(75.6)                # 绕 z 摆放角（侧面法向对准
                                            #   ±y_g；phi=-14.4 deg -> 75.6 deg）
CUBE_MASS = 0.10                           # 立方体质量 [kg]
CUBE_FRICTION = 0.6                        # 静摩擦系数（Isaac 场景）

# 指垫偏置（精细包络测绘 diag_pad_fine，供实验二 TCP 定义用）：垫中心 =
# 原点 + PAD_OFFSET_X*x_g + PAD_OFFSET_Z*z_g。指垫占据接近轴
# x_ee ∈ [-20, 0] mm，中心约 -10 mm；开度 gap(s)=2s-25 mm。
PAD_OFFSET_X = -0.010                      # [m]（垫在原点后侧 ~10 mm）
PAD_OFFSET_Z = -0.005                      # [m]

# 任务姿态（单位四元数 [w,x,y,z]，世界系）= make_tool_pose(tilt=120 deg,
# phi=-14.4 deg) 的物理 gripper_link 姿态（v12 回归 v9 斜向下抓）：
#   x_g 斜向下 60 deg（接近轴），开合向水平。v10/v11 顶抓（tilt=160）被
#   STL 解剖+物理细扫双重证伪：(1) 顶抓时方块竖直棱在 gripper 系 z 向
#   跨 ±28.8 mm，超出手指平板 z 带 ±19.6 mm，指面够不到方块面；
#   (2) 指尖刀片结构在接近向原点附近横跨开合向，顶抓先撞方块顶。
#   tilt=120 时方块竖直棱投影 z_g=±19.5 mm 恰在平板带内（diag_v9_grasp
#   曾验证 kinematic 逼近零碰撞）。
R_TOOL_QUAT = np.array([0.958309, 0.032439, 0.256778, -0.121063])


# 物理 gripper 轴向（世界系；make_tool_pose(120, -14.4) 的列向量，
# 与 R_TOOL_QUAT 同一物理帧，直接硬编码避免重复构造；已逐分量对账，
# _TOOL_R 校验用）
TOOL_AXIS = np.array([0.838818, -0.215372, -0.5])      # +x = 接近向（斜向下 60 deg）
GRIPPER_Y = np.array([0.24869, 0.968583, 0.0])         # 手指开合向（水平）
_TOOL_Z = np.array([0.484292, -0.124345, 0.866025])    # gripper z（斜上外）


def _quat_axes(r):
    """单位四元数 [w,x,y,z] -> 旋转矩阵三列 [x | y | z]（纯 numpy）。"""
    w, x, y, z = r
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


_TOOL_R = _quat_axes(R_TOOL_QUAT / np.linalg.norm(R_TOOL_QUAT))  # 仅供校验用（TCP 约定）

# --- 任务路标（世界系 gripper 原点；v14 抓深度回归 v9 垫心几何） ---
GRASP_TCP = CUBE_POS.copy()                # 指垫中心 ≈ 方块中心（设计点）
# v14 裁决（e2e A/B + 几何核算 + teleport 深度扫描）：
#   v12c 深度（-0.042 z_g）下指垫接触盒（x_g∈[-20,0]，z_g∈±19.6 mm）
#   内无方块侧面面元，手指实际夹住支柱（stall 43.0 mm ≡ 支柱 y_g
#   投影宽 42.6 mm）。回归 v9/diag_pad_fine 几何：原点 = 方块中心
#   + 0.028 x_g + 0.005 z_g（垫心≈方块中心）：垫盒内有方块面面元，
#   接触开度 = 方块边长 45 mm，插入段零碰撞（e2e + diag_v9_grasp
#   实测方块移位 ≤0.1 mm）。但闭合阶段手指楔形内面（v12h 实测：
#   内面=斜坡，非竖直垫面）先于指垫碰方块并将其侧向弹出——与
#   diag_innerface 结论一致（45 mm 方块无法被该夹爪平行夹持，
#   指尖结构最小夹持宽度 ~66 mm）；pin/kinematic 两路线 e2e 行为
#   逐毫秒一致，排除轨迹因素，属夹爪硬件几何极限。
GRASP_POS = (CUBE_POS + 0.028 * TOOL_AXIS + 0.005 * _TOOL_Z)
                                            # 抓取位 gripper 原点（v14 裁决几何）
GRASP_APPROACH_POS = GRASP_POS - 0.08 * TOOL_AXIS
                                            # 接近退避位（沿接近轴后退 8 cm，
                                            #   即斜上方外侧；转腕完成点）

# 实验一（v9：直接夹到方块，用户要求）：SETPOINT = 抓取位
SETPOINT_HOVER_CLEARANCE = 0.0             # （保留常量名）
SETPOINT_POS = GRASP_POS
SETPOINT_HOLD_TIME = 3.0                   # 带载提升后保持时长 [s]

# 实验二（带载圆周）：接近插入 -> 闭合 -> 提升 -> 圆周
GRASP_APPROACH_TIME = 2.5                  # 退避位 -> 抓取位 插入 [s]
GRASP_ATTACH_HOLD = 1.5                    # 闭合后静置（附着瞬态衰减）[s]
GRASP_LIFT_HEIGHT = 0.08                   # 提升高度 [m]
GRASP_LIFT_TIME = 1.5                      # 提升 [s]
LIFT_POS = GRASP_POS - 0.02 * TOOL_AXIS + np.array([0.0, 0.0, GRASP_LIFT_HEIGHT])
                                            # 提升位（沿 -x_g 退 2 cm + 竖直 8 cm）

# 带载圆周（提升后水平圆）：圆心沿径向外偏半径，theta=0 = 提升位
# （消除 lift -> 圆周的切入突跳）
_RADIAL = np.array([CUBE_POS[0], CUBE_POS[1], 0.0])
_RADIAL = _RADIAL / np.linalg.norm(_RADIAL)   # 立方体水平方位单位向量
CIRCLE_RADIUS = 0.06                       # [m]
CIRCLE_CENTER = (GRASP_TCP + np.array([0.0, 0.0, GRASP_LIFT_HEIGHT])
                  + CIRCLE_RADIUS * _RADIAL)   # TCP 平面圆心（世界系）
CIRCLE_CENTER_Z = CIRCLE_CENTER[2]         # 圆周 TCP 高度 [m]
CIRCLE_OMEGA = 1.0                         # [rad/s]
CIRCLE_RAMP_TIME = 2.0                     # 五次多项式起步 [s]
CIRCLE_DURATION = 20.0                     # 圆周段时长 [s]（>3 圈）

# 初始位形：模型原生形态（USD/URDF 零位，用户要求）。j2/j3 的 URDF
# 上限即 0，纯零位恰在限位边界（首步即触发超限终止），故向内偏
# 0.10 rad（5.7°）。内偏须严格大于 check_joint_limits 的 margin=0.02
# （曾取 0.02 恰在判据边界，Isaac 读回数值抖动即触发 t=0 abort）。
# v14 实时化重构：原 -0.04 落在限位缓冲区（LIMIT_BUFFER 内）里，起步
# 无支撑下垂的恢复正速度反复触发预测制动 -> 与反馈互追成 ~15 Hz
# 极限环颤振（CSV governed 与 τ 尖峰交替佐证）；-0.10 使起点距软上限
# 0.05，需 >1.4 rad/s 正速度才会触发，起步瞬态（峰值 ~1 rad/s）不再
# 进入极限环。旧 Q_INIT（hover 上方 IK 解，q2=-2.6 深折叠）起点即贴
# 近锁滞区，已废弃。
Q_INIT = np.array([0.0, -0.10, -0.10, 0.0, 0.0, 0.0])

# exp1 goto 轨迹（任务空间分段路标，v9 斜向下抓重规划）：
#   零位(恒等) 举高 0.28 -> 恒等前移到接近退避位正上方 -> 转腕+下降到
#   退避位（斜上方外侧，沿接近轴 -x_g 距抓取位 8 cm）-> 沿接近轴斜下
#   插入到抓取位 -> 静置闭合 -> 带载提升（-x_g 退 2 cm + 竖直 8 cm）。
GOTO_TOP_Z = 0.28                          # 举高/平移段 gripper 高度 [m]
GOTO_T_LIFT = 3.0                          # 举高段时长 [s]
GOTO_T_TRANS = 4.0                         # 前移段时长 [s]
GOTO_T_ROTDESC = 4.0                       # 转腕+下降到退避位段时长 [s]
GOTO_T_DESC = 3.0                          # 沿接近轴插入段时长 [s]（8 cm；
                                            #   v9 首跑教训：原 1.5 s 太快，
                                            #   动态滞后 1~2 cm 使手指撞飞
                                            #   方块；降速后滞后 < 5 mm。
                                            #   v15 曾为 70 mm 方块降到 4.5 s，
                                            #   方块尺寸回退 45 mm 后恢复 3.0）
GOTO_GRASP_DWELL = 3.5                     # 抓取位静置 [s]（v9 三跑教训：
                                            #   长静置下深折叠构型重力残差
                                            #   慢漂 ~1 cm/s，夹爪漂离方块；
                                            #   改为到位即快速闭合，接触后再
                                            #   静置附着）
GOTO_T_LIFTLOAD = 3.0                      # 带载提升段时长 [s]
GOTO_RETURN_DWELL = 1.0                    # 回零落位静稳时长 [s]（v4：轨迹
                                           #   追加回零尾段，结束停 Q_INIT 邻域
                                           #   静稳，不再悬空）
GOTO_LIFTLOAD_H = 0.08                     # （保留常量名；提升位直接用 LIFT_POS）

# --- 短行程验证轨迹（--traj short，v5 首跑安全阀）---
SHORT_SWING_RAD = 0.12                     # j1（竖直底座轴）摆幅 [rad]：不改变
                                           #   各关节重力载荷分布，最小风险
SHORT_T_GO = 5.0                           # 去程时长 [s]（quintic 峰值速度
                                           #   1.875*0.12/5 ≈ 0.045 rad/s，极慢）
SHORT_T_BACK = 5.0                         # 回程时长 [s]（原路缓回 Q_INIT）

# ---------------------------------------------------------------------------
# Isaac Sim 场景（interfaces/isaac_interface.py）
# ---------------------------------------------------------------------------

# 机器人 USD 资产：引用 reBot-Isaacsim 仓库（相对本项目根定位，不硬编码
# 绝对路径；可用环境变量 B601_USD 覆盖；USD 含 payloads/ 相对引用，
# 复制会破坏引用关系，故采取引用方式）
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
ISAAC_ROBOT_USD = os.environ.get(
    "B601_USD",
    str(_PROJECT_ROOT.parent / "reBot-Isaacsim" / "usd" / "reBot_B601_DM"
        / "reBot_B601_DM.usda"),
)
ISAAC_ROBOT_PRIM = "/World/B601"           # 机器人在 stage 中的 prim 路径

# URDF（数据源，仅离线提取/交叉验证用；b601_dynamics 参数已硬编码）：
# B601_DH_TABLE / 惯性参数表均提取自此文件（experiments/verify_dh.py 与
# experiments/extract_inertia.py），可用环境变量 B601_URDF 覆盖
B601_URDF = os.environ.get(
    "B601_URDF",
    str(_PROJECT_ROOT.parent / "reBot-Isaacsim" / "urdf" / "reBot_B601_DM"
        / "urdf" / "reBot_B601_DM.urdf"),
)
ISAAC_PHYSICS_DT = 1.0 / 500.0             # 物理步长 [s]（2 ms，含 drive 清零）
ISAAC_RENDER_DT = 1.0 / 30.0               # 渲染步长 [s]（GUI 30 fps：step() 物理
                                            #   恒走 render=False 精确单步，每 17 物理步
                                            #   独立调 world.render() 渲一帧——渲染墙钟
                                            #   与物理节奏解耦。v14 原走 step(render=True)，
                                            #   落入 Kit 主循环墙钟配速（每调用推进仿真
                                            #   时间不恒 2 ms），控制时间基准失效致 GUI
                                            #   起步超限位 abort + 慢放；headless 不渲染
                                            #   不受影响）

# 夹爪（Isaac 位置 drive；USD PrismaticJoint 单指行程限位 [0, 0.0715] m，
# q=0 两指并拢，指间开度 ~= 2*单指行程，一阶近似不含指厚偏置）
GRIPPER_KP = 2.0e3                         # 手指 drive 刚度 [N/m]
GRIPPER_KD = 20.0                          # 手指 drive 阻尼 [N*s/m]
GRIPPER_OPENING = 0.14                     # 接近指间开度目标 [m]（set_gripper
                                            #   语义：目标 = 2*单指 stroke；
                                            #   stroke=0.07 ≈ 全开，真实开度
                                            #   = 2*stroke，diag_innerface 锤定：
                                            #   各档内面位置 ≡ stroke）；
                                            #   v9 首跑教训：开度 0.07 时每侧
                                            #   净空仅 12.5 mm，插入段跟踪偏移
                                            #   ~1 cm 即咬住方块面；全开后
                                            #   每侧净空 ~48 mm；三跑实测
                                            #   插入段方块零扰动、无自碰）
GRASP_INTERFERENCE = 0.002                 # 夹持过盈量 [m]（每侧 1 mm）
GRIPPER_GRASP = CUBE_SIZE - GRASP_INTERFERENCE
                                            # 夹持指间开度目标 [m]（v13 按
                                            #   立方体实际尺寸重标定）：接触
                                            #   开度 ≈ 方块边长 CUBE_SIZE
                                            #   =45 mm；旧值 0.063（63 mm）
                                            #   大于方块边长，手指闭合到位也
                                            #   接触不到方块，无法夹持。
                                            #   现取 43 mm = 过盈 1 mm/侧：
                                            #   KP*过盈 ≈ 2 N/指，摩擦 2μN
                                            #   ≈ 2.4 N > 方块自重 0.98 N。
                                            #   ★铁律：目标不得越过接触点
                                            #   （越过则 drive 强闭挤飞方块；
                                            #   更旧 0.040/0.045 时代对应的
                                            #   是 66 mm 大方块，已废弃）
GRIPPER_BASELINE_WIDTH = GRIPPER_OPENING    # 开指保持基线开度 [m]（0.14 =
                                            #   全开，每侧净空
                                            #   (140-45)/2 = 47.5 mm）。v14 首二跑
                                            #   教训：60 mm（净空 7.5 mm）t≈13 s
                                            #   撞落方块；100 mm（净空 27.5 mm）
                                            #   插入段仍有 ~8 mm 擦碰（跟踪滞后
                                            #   峰值 ~1 cm + 10° 倾角占有效净空
                                            #   ~5 mm + 指垫几何）；全开时 v9
                                            #   三跑已实证插入段方块零扰动；
                                            #   exp1 无接触基线对照模式恒保持
                                            #   此开度）

# ---------------------------------------------------------------------------
# 数值参数（主循环，两个实验共用）
# ---------------------------------------------------------------------------

DT = ISAAC_PHYSICS_DT                      # 物理/伺服步长 [s]
CTRL_DT = 1.0 / 100.0                      # 真机控制周期 [s]（10 ms）：反馈
                                            #   计算 ~10 ms，控制步长短于它则
                                            #   无法实时。真机伺服内环（电流环
                                            #   ~1 kHz + 插值）覆盖反馈间隙，
                                            #   而 Isaac 力矩直驱无内环，仿真
                                            #   中 10 ms 零阶保持离散化实测
                                            #   恶化收敛（0.024 -> 0.089 m）：
                                            #   仿真取 2 ms（性能上界），真机
                                            #   部署切 CTRL_USE_REAL_RATE=True
CTRL_USE_REAL_RATE = True                  # 当前为真机部署口径（TNDQ_real）：
                                            #   True = 按 CTRL_DT 真机反馈周期
                                            #   运行；仿真性能上界复测时改回
                                            #   False（每 2 ms 物理步控制，
                                            #   exp1 已记录的仿真指标即该口径）
CTRL_EVERY = (int(round(CTRL_DT / DT)) if CTRL_USE_REAL_RATE else 1)
PINV_DAMPING = 1e-6                        # 阻尼伪逆 lambda（诚实条款 (i)）
QDDOT_MAX = 30.0                           # q_ddot_ref 范数限幅 [rad/s^2]
SINGULARITY_TOL = 0.05                     # sigma_min(J) 阻尼提升阈值
                                            #   （exp1 首跑教训：原 1e-3 过低，
                                            #   Q_INIT 处 sigma_min=0.143，
                                            #   腕关节漂移使 sigma 降至 0.04
                                            #   时 J+ 放大正反馈指数发散）
SINGULARITY_DAMPING = 5e-2                 # 奇异附近加大阻尼

# 关节速度阻尼注入（exp1 首跑教训）：力矩直驱下 Isaac 无摩擦，mN·m 级
# 重力残差即可驱动轻腕关节（M6=4e-4 kg·m²）沿 task 反馈不敏感的
# Jacobian 小奇异值方向漂移。注入 tau -= D qd 提供关节空间耗散：
#   - 物理对应真机电机阻尼/减速器摩擦（部署时天然存在）；
#   - 耗散性扰动对 ISS/定理 3 稳定性裕度有利（d(t) 有界耗散项）；
#   - 圆周跟踪 qd~1 rad/s 时阻尼力矩 0.02 N·m，由 task 反馈补偿，
#     稳态跟踪误差 mm 级（后续 CSV 指标验证）。
JOINT_DAMPING = 0.02 * np.ones(6)          # [N·m·s/rad]

# 失速 dither（PhysX TGS 构型锁逃逸机制）总开关：v9 构型族健康
# （margin 31~40 deg、sigma 正常，非锁区）且 v9 四跑实测 dither 在
# 提升过渡误触发把方块打飞（抓取位慢漂造成的持续误差不属于构型
# 锁，dither 扰动只会破坏夹持）——关闭。
ENABLE_DITHER = False

REPROJECT_EVERY = 50                       # (3.8) 重投影周期 [步]
LOG_EVERY = 5                              # 每 k 步记录一行 CSV

# 单位约束残差族 (3.8) 告警阈值
C0_TOL = 1e-9
C1_TOL = 1e-9
C2_TOL = 1e-9
