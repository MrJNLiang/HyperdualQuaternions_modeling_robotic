"""
B601 IK 基础设施（experiments 设计工具共用库）。

从 design_task_geometry.py 抽取，保留其全部已验证结论：

  - DQ twist 线速度语义（core/kinematics.jacobian）：vec6(xi) = [w; v_O]，
    v_O = pdot - w x p 是速度场在基座原点的值（经典 screw 约定），不是
    末端原点速度 pdot。末端原点速度雅可比需转换（数值对账 4e-10）：
        J_p = [J[:3]; J[3:] - skew(p) @ J[:3]]。
  - 残差-雅可比配套：残差 = 目标 - 当前（四元数最短路径 + 位置差），
    一阶雅可比 = -point_jacobian（负号！）。矩阵差残差在大误差时导数含
    cos(theta) 因子会反号，与雅可比严重失配（实测差 >1），不可用。
  - scipy TRF 信赖域 + 边界约束 + 解析雅可比是最稳健配置：手写阻尼
    最小二乘在大初始姿态误差下线性化失效而震荡。

规范化任务姿态构造（make_tool_pose）：给定工具倾斜角 tilt（gripper x
轴与竖直的夹角）与倾斜方位 phi，构造 gripper y 轴（手指开合方向）严格
水平的姿态——保证倾斜工具仍能水平夹持地面上的立方体侧面（立方体绕 z
旋转 cube_yaw_for(phi) 对准手指）。

姿态约束来源（120k 采样结论）：竖直工具姿态在低空前方不可达（j2/j3
负限位 + 偏置腕），可行抓取域为"工具倾斜 <= ~22 deg + gripper 原点
z in (0.09, 0.16) + 链上 DH 帧离地 > 2 cm"。
"""

import numpy as np
from scipy.optimize import least_squares

from config.params import (
    B601_DH_TABLE, B601_TOOL_ANGLE, JOINT_LOWER, JOINT_UPPER,
)
from core.kinematics import TNDQSerialChain
from core.dq_algebra import dq_mul, dq_translation, dq_rotation, dq_rot_z

CHAIN = TNDQSerialChain(B601_DH_TABLE)
E_DQ = dq_rot_z(B601_TOOL_ANGLE)

# IK 内部限位（留 5 度安全边距，比控制环 LIMIT_BUFFER 严）
IK_LO = JOINT_LOWER + np.deg2rad(5.0)
IK_HI = JOINT_UPPER - np.deg2rad(5.0)


# ---------------------------------------------------------------------------
# 四元数工具（4 维 [w,x,y,z]，独立于 DQ 代数，仅 IK 内部使用）
# ---------------------------------------------------------------------------

def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_conj(a):
    return np.array([a[0], -a[1], -a[2], -a[3]])


def quat_angle(a, b):
    """两单位四元数的测地角 [rad]。"""
    return 2.0 * np.arccos(np.clip(min(1.0, abs(float(a @ b))), -1.0, 1.0))


def quat_to_R(r):
    """单位四元数 [w,x,y,z] -> 旋转矩阵。"""
    w, x, y, z = r
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_R(R):
    """旋转矩阵 -> 单位四元数 [w,x,y,z]（Shepperd 数值稳定分支）。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        q = np.array([0.25 * s,
                      (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s,
                      (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = np.array([(R[2, 1] - R[1, 2]) / s,
                      0.25 * s,
                      (R[0, 1] + R[1, 0]) / s,
                      (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = np.array([(R[0, 2] - R[2, 0]) / s,
                      (R[0, 1] + R[1, 0]) / s,
                      0.25 * s,
                      (R[1, 2] + R[2, 1]) / s])
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = np.array([(R[1, 0] - R[0, 1]) / s,
                      (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s,
                      0.25 * s])
    return q / np.linalg.norm(q)


# ---------------------------------------------------------------------------
# 规范化任务姿态（倾斜工具、手指开合方向水平）
# ---------------------------------------------------------------------------

def make_tool_pose(tilt, phi):
    """
    规范化倾斜工具姿态。

    gripper x 轴（工具轴）倾斜 tilt、水平投影方位 phi（从基座 +x 起）；
    gripper y 轴（手指开合方向）= normalize(ez x x_g) 严格水平；
    gripper z 轴 = x_g x y_g（右手系闭合）。
    tilt=0 时退化为绕竖直的滚转 phi（仍良定）。
    返回 (R, quat)。
    """
    xg = np.array([np.sin(tilt) * np.cos(phi),
                   np.sin(tilt) * np.sin(phi),
                   np.cos(tilt)])
    yg = np.array([-np.sin(phi), np.cos(phi), 0.0])
    zg = np.cross(xg, yg)
    R = np.column_stack((xg, yg, zg))
    return R, quat_from_R(R)


def cube_yaw_for(phi):
    """立方体绕 z 摆放角：本地 +x 侧面法向对准手指开合方向 y_g。"""
    return float(np.arctan2(np.cos(phi), -np.sin(phi)))


def make_side_pose(phi, z_up=True):
    """
    水平侧抓姿态：工具轴 x_g 水平方位 phi、手指开合方向 y_g 水平、
    z_g = +ez（z_up=True）或 -ez（绕 x_g 翻转 180 deg，腕构型变体）。

    依据：世界系低空全景扫描（60000 样本，TCP z<=0.05 + 防穿地）中
    近竖直域（tilt<30 deg）样本为 0——j2/j3 负限位下 B601 无法竖直
    下探；侧抓域（75~105 deg）3076 个样本，是抓地面物体的唯一方式。
    返回 (R, quat)。
    """
    xg = np.array([np.cos(phi), np.sin(phi), 0.0])
    yg = np.array([-np.sin(phi), np.cos(phi), 0.0])
    if not z_up:
        yg = -yg
    zg = np.cross(xg, yg)
    R = np.column_stack((xg, yg, zg))
    return R, quat_from_R(R)


def cube_yaw_for_gripper(yg):
    """侧抓立方体摆放角：本地 +x 侧面法向对准手指开合方向 yg（水平）。"""
    return float(np.arctan2(yg[1], yg[0]))


# ---------------------------------------------------------------------------
# FK / 雅可比
# ---------------------------------------------------------------------------

def _skew(v):
    """向量反对称矩阵（叉乘算子）。"""
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def fk_pose(q):
    """控制末端位姿（DH 链 FK * 尾变换 E）：(位置, 单位四元数)。

    注意：返回值为 DH 基座系（原点 = joint1 轴上的 URDF 关节原点，
    比 base_link / 世界系高 B601_BASE_PREFIX）。世界系目标进入 IK
    前必须用 world_to_dh() 转换（历史陷阱：曾因此把目标抬高 8.5 cm）。
    """
    x = dq_mul(CHAIN.fkm(q), E_DQ)
    return dq_translation(x), dq_rotation(x)


def world_to_dh(p_world):
    """世界 / base_link 系位置 -> DH 基座系（减基座前缀，纯平移）。"""
    from config.params import B601_BASE_PREFIX
    return np.asarray(p_world, dtype=float) - B601_BASE_PREFIX


def dh_to_world(p_dh):
    """DH 基座系位置 -> 世界 / base_link 系（加基座前缀）。"""
    from config.params import B601_BASE_PREFIX
    return np.asarray(p_dh, dtype=float) + B601_BASE_PREFIX


def _dh_mat(a, alpha, d, th):
    ct, st = np.cos(th), np.sin(th)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a],
        [st, ct * ca, -ct * sa, 0.0],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0]])


def dh_frames_world(q):
    """各 DH 帧原点的世界坐标（base_link 系，机器人立于地面时=世界系）。

    用于防穿地检查：帧 0..5 为关节轴原点（帧 5 = DH 链末端 =
    gripper_link 原点）。手指 / TCP 区域另行单独约束。
    """
    from config.params import B601_DH_TABLE, B601_BASE_PREFIX
    q = np.asarray(q, dtype=float).reshape(-1)
    T = np.eye(4)
    pts = []
    for i, row in enumerate(B601_DH_TABLE):
        a, alpha, d, th0, _ = row
        T = T @ _dh_mat(a, alpha, d, th0 + q[i])
        pts.append(T[:3, 3] + B601_BASE_PREFIX)
    return np.array(pts)


def point_jacobian(q, p=None):
    """
    末端原点速度雅可比 J_p：pdot = J_p qdot（基座系）。

    DQ twist 约定（core/kinematics.jacobian，与 screw theory 一致）：
        vec6(xi) = [w; v_O]，v_O = 速度场在世界原点的值 = pdot - w x p。
    IK 的位置误差对应 pdot，故需转换：pdot = v_O + w x p，即
        J_p = [J[:3]; J[3:] - skew(p) @ J[:3]]。
    右乘尾变换 E（纯旋转、原点重合）不改 twist，p 直接用含 E 的 FK。
    """
    J = CHAIN.jacobian(q)
    if p is None:
        p, _ = fk_pose(q)
    return np.vstack([J[:3], J[3:] - _skew(p) @ J[:3]])


def sigma_min(q):
    """point_jacobian 最小奇异值（奇异度监测）。"""
    return float(np.linalg.svd(point_jacobian(q), compute_uv=False)[-1])


# ---------------------------------------------------------------------------
# 信赖域最小二乘 IK
# ---------------------------------------------------------------------------

def joint_margin(q):
    """最小限位余量 [rad]。"""
    q = np.asarray(q, dtype=float)
    return float(np.min(np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)))


def pose_error(q, p_des, r_des):
    """
    IK 残差向量 e = [e_r; e_p]（基座系）。

    e_r = 2*vec(r_des (x) conj(r))（w<0 翻转取最短路径）：小误差时
    = theta*n，与角速度误差同阶，是 -point_jacobian 角速度行的精确
    一阶配套。e_p = p_des - p（末端原点，对应 point_jacobian 平移行）。
    """
    p, r = fk_pose(q)
    re = quat_mul(r_des, quat_conj(r))
    if re[0] < 0.0:
        re = -re
    return np.r_[2.0 * re[1:4], p_des - p]


def solve_ik(p_des, r_des, q0, max_nfev=300):
    """
    信赖域最小二乘 IK（scipy TRF，边界约束 + 解析雅可比）。

    残差 = pose_error，其雅可比（一阶）= -point_jacobian（负号：
    fk 朝目标运动时残差减小；行序一致）。
    返回 (q, 残差范数, 是否收敛)。
    """
    sol = least_squares(
        lambda q: pose_error(q, p_des, r_des),
        np.clip(np.asarray(q0, dtype=float), IK_LO, IK_HI),
        jac=lambda q: -point_jacobian(q),
        bounds=(IK_LO, IK_HI), method="trf",
        xtol=1e-14, ftol=1e-14, gtol=1e-14, max_nfev=max_nfev,
    )
    res = float(np.linalg.norm(sol.fun))
    return sol.x, res, bool(res < 1e-8)


def solve_ik_multi(p_des, r_des, n_init=24, seed=3, q_warm=None):
    """多初值 IK：限位内随机 + 热启动，返回限位余量最大的收敛解。"""
    rng = np.random.default_rng(seed)
    cands = []
    if q_warm is not None:
        cands.append(np.asarray(q_warm, dtype=float))
    cands.extend(rng.uniform(IK_LO, IK_HI, (n_init, 6)))
    best, best_margin, best_res = None, -np.inf, None
    for q0 in cands:
        q, res, ok = solve_ik(p_des, r_des, q0)
        if not ok:
            continue
        margin = joint_margin(q)
        if margin > best_margin:
            best, best_margin, best_res = q, margin, res
    return best, best_margin, best_res


def report(tag, q, p_des, r_des):
    """单点 IK 结果报告，返回 (dp, dtheta, margin, sigma_min)。"""
    p, r = fk_pose(q)
    dp = float(np.linalg.norm(p - p_des))
    dth = quat_angle(r, r_des)
    m = joint_margin(q)
    smin = sigma_min(q)
    print(f"    {tag:<22s} 残差 dp={dp:.2e} m  dtheta={dth:.2e} rad  "
          f"限位余量={np.rad2deg(m):6.2f} deg  sigma_min(J)={smin:.4f}")
    return dp, dth, m, smin
