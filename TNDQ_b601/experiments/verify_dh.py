"""
B601 标准DH表提取与三路对账验证。

数据源：reBot-Isaacsim/urdf/reBot_B601_DM/urdf/reBot_B601_DM.urdf
（该 URDF 的 FK 已在 2026-08 冒烟测试中与 Isaac Sim 场景实测、Pinocchio
三方对齐，残差 < 0.7%，可作黄金参考。）

三路对账（随机 q 采样）：
    1) URDF 链乘 FK      T = Trans(p1)·Rz(q1)·Trans(p2)·Rpy2·Rz(-q2)·...·Tool
    2) DH 齐次矩阵 FK    T = B · A_1(q1)·A_2(q2)·...·A_6(q6)
    3) TNDQ DQ 链 FK     TNDQSerialChain(B601_DH).fkm(q)   (core/kinematics)

DH 提取方法：q=0 时逐对相邻关节轴的几何量（公垂线/轴间角/轴向偏距），
关节 2 的负轴 (0,0,-1) 由 DH 帧 z 轴直接取 URDF 轴方向（含符号）自然
吸收（theta_2 = q_2 + pi 中的 q_2 与 URDF 的 q_2 同号）；
末端帧 z_6 取 URDF gripper_link 的 z 轴（工具接近向），x_6 = z_5 x z_6
（合法 DH 帧），DH 链末端与 URDF 末端之间残余常量尾变换 E = Rz(pi/2)
（B601_TOOL_ANGLE，实验侧 dq_mul 处理）；基座前缀 B = Trans(p_joint1)
留在链外（接口层处理）。手写表数值与数值提取器 extract_dh() 输出
逐字段对齐（等价的平行轴 d 分配/公垂线方向选择均采纳提取器版本）。

用法（TNDQ_b601 目录下）：
    python3 experiments/verify_dh.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ---------------------------------------------------------------------------
# URDF 原始数据（reBot_B601_DM.urdf，数值逐字段抄录）
# ---------------------------------------------------------------------------

# (name, origin_xyz, origin_rpy, axis) —— 6 个转动关节
URDF_JOINTS = [
    ("joint1", [-8.416e-05, 0.0, 0.08465], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
    ("joint2", [0.020084, 0.031625, 0.05555], [-1.5708, 0.0, 0.0], [0.0, 0.0, -1.0]),
    ("joint3", [-0.264, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
    ("joint4", [0.2426, -0.054, -0.001625], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
    ("joint5", [0.078308, -0.0375, -0.03], [-1.5708, 0.0, 0.0], [0.0, 0.0, 1.0]),
    ("joint6", [0.023692, 0.0, 0.04], [0.0, 1.5708, 0.0], [0.0, 0.0, 1.0]),
]

# gripper_joint（fixed）：link6 -> gripper_link（= DH 链末端帧）
URDF_TOOL_XYZ = np.array([0.0, 0.0, 0.15971])
URDF_TOOL_RPY = np.array([3.1416, -1.5708, 0.0])


def rpy_to_R(rpy):
    """URDF fixed-axis RPY：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def axis_rot(axis, q):
    """Rodrigues 旋转（axis 单位向量）。"""
    a = np.asarray(axis, dtype=float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1.0 - np.cos(q)) * (K @ K)


def urdf_fk(q):
    """URDF 链乘 FK：返回 base_link -> gripper_link 齐次变换。"""
    T = np.eye(4)
    for (_, xyz, rpy, axis), qi in zip(URDF_JOINTS, q):
        T = T @ np.block([[np.eye(3), np.array(xyz).reshape(3, 1)], [np.zeros((1, 3)), 1.0]])
        T = T @ np.block([[rpy_to_R(rpy), np.zeros((3, 1))], [np.zeros((1, 3)), 1.0]])
        T = T @ np.block([[axis_rot(axis, qi), np.zeros((3, 1))], [np.zeros((1, 3)), 1.0]])
    T = T @ np.block([[np.eye(3), URDF_TOOL_XYZ.reshape(3, 1)], [np.zeros((1, 3)), 1.0]])
    T = T @ np.block([[rpy_to_R(URDF_TOOL_RPY), np.zeros((3, 1))], [np.zeros((1, 3)), 1.0]])
    return T


# ---------------------------------------------------------------------------
# 标准DH表（提取结果；闭式表达式保留推导可追溯性）
# 注：q=0 时 URDF 末端（gripper_link）姿态 = I（工具 rpy 逐段相消），
# 位置 p = (0.26031, 0, 0.1917)。末端 z 轴 (0,0,1) 与关节 6 轴 (1,0,0)
# 垂直，DH 末端帧不可直接取 URDF 末端帧（x_6 = ee_x = (1,0,0) 与最后
# 关节轴平行，非法）；取 z_6 = ee_z、x_6 = z_5 x z_6，残余尾变换为
# 绕关节 6 轴的常量旋转 E = Rz(pi/2)（见 B601_TOOL_ROT）。
_L34 = np.hypot(0.2426, 0.054)              # joint3->joint4 轴间距离
_PHI34 = np.arctan2(0.054, 0.2426)          # 该连杆对 z 轴的倾角

B601_DH_TABLE = np.array([
    #  a          alpha            d          theta0   type
    [0.020084,   np.pi / 2,       0.05555,   0.0,                   0],  # joint1
    [0.264,      np.pi,          -0.031625,  np.pi,                 0],  # joint2
    [_L34,       0.0,            -0.001625,  np.pi - _PHI34,        0],  # joint3
    [-0.078308,  np.pi / 2,      -0.03,      _PHI34 - np.pi,        0],  # joint4
    [0.0,        np.pi / 2,       0.0025,    -np.pi / 2,            0],  # joint5
    [0.0,        np.pi / 2,       0.183402,  0.0,                   0],  # joint6
])

# DH 链末端帧 -> URDF gripper_link（TCP）的常量尾变换：纯旋转 Rz(pi/2)
# （原点重合：o_ee 恰在关节 6 轴上，d_6 已吸收全部平移）
B601_TOOL_ANGLE = np.pi / 2

# 基座前缀：URDF base_link -> DH 链基座（原点取 joint1 轴上 URDF 关节原点，
# 姿态恒等；x 偏移 -8.416e-5 m 为制造装配偏心，无法并入 DH，留在接口层）
B601_BASE_PREFIX = np.array([-8.416e-05, 0.0, 0.08465])


def dh_transform(a, alpha, d, theta):
    """标准 DH：A = Rz(theta) Tz(d) Tx(a) Rx(alpha)（与 core/kinematics 一致）。"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def dh_fk(q, dh_table=None, tool_angle=None):
    """DH 链 FK（含基座前缀与尾变换）：返回 base_link -> gripper_link。"""
    dh = B601_DH_TABLE if dh_table is None else dh_table
    ang = B601_TOOL_ANGLE if tool_angle is None else tool_angle
    T = np.block([[np.eye(3), B601_BASE_PREFIX.reshape(3, 1)],
                  [np.zeros((1, 3)), 1.0]])
    for i, row in enumerate(dh):
        T = T @ dh_transform(row[0], row[1], row[2], q[i] + row[3])
    # 尾变换 E = Rz(tool_angle)（纯旋转）
    c, s = np.cos(ang), np.sin(ang)
    T = T @ np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    return T


# ---------------------------------------------------------------------------
# 数值 DH 提取器（参考实现）：从 URDF q=0 关节轴几何直接提取标准 DH 表
# ---------------------------------------------------------------------------

def urdf_joint_frames():
    """q=0 时每个关节的轴参考系（世界 = base_link 系）：(原点, 轴方向)。"""
    frames = []
    T = np.eye(4)
    for (_, xyz, rpy, axis) in URDF_JOINTS:
        T = T @ np.block([[np.eye(3), np.array(xyz).reshape(3, 1)],
                          [np.zeros((1, 3)), 1.0]])
        T = T @ np.block([[rpy_to_R(rpy), np.zeros((3, 1))],
                          [np.zeros((1, 3)), 1.0]])
        frames.append((T[:3, 3].copy(), T[:3, :3] @ np.asarray(axis, dtype=float)))
    return frames


def signed_angle(v_from, v_to, around):
    """绕单位轴 around 从 v_from 到 v_to 的有符号角（弧度）。"""
    v_from = v_from - np.dot(v_from, around) * around
    v_to = v_to - np.dot(v_to, around) * around
    n_f = np.linalg.norm(v_from)
    n_t = np.linalg.norm(v_to)
    if n_f < 1e-12 or n_t < 1e-12:
        return 0.0
    v_from, v_to = v_from / n_f, v_to / n_t
    c = float(np.dot(v_from, v_to))
    s = float(np.dot(np.cross(v_from, v_to), around))
    return float(np.arctan2(s, c))


def extract_dh():
    """
    标准几何提取（Siciliano 约定 A_i = Rz(th) Tz(d) Tx(a) Rx(al)）：

    帧 i-1 已知 (o, x, y, z)，关节 i 的轴几何 (o_ref_i, z_i)：
      x_i  = z_{i-1} × z_i 的公垂线方向（平行时退化为参考点连线的垂直分量）
      al_i = 绕 x_i 从 z_{i-1} 到 z_i 的有符号角
      a_i  = (o_ref_i - o_{i-1})·x_i
      d_i  = (o_ref_i - o_{i-1})·z_{i-1}
      帧 i 原点 = o_{i-1} + d_i z_{i-1} + a_i x_i（URDF 参考点的轴向滑动
      分量自动进入下一帧的 d）
      th_i = 绕 z_{i-1} 从 x_{i-1} 到 x_i 的有符号角（= q_i 偏置）
    末端帧 n 的 z/x 直接取 URDF 末端帧姿态的列向量（工具尾零吸收）。
    """
    frames = urdf_joint_frames()
    # URDF 末端帧（gripper_link）
    T = np.eye(4)
    for (_, xyz, rpy, _) in URDF_JOINTS:
        T = T @ np.block([[np.eye(3), np.array(xyz).reshape(3, 1)],
                          [np.zeros((1, 3)), 1.0]])
        T = T @ np.block([[rpy_to_R(rpy), np.zeros((3, 1))],
                          [np.zeros((1, 3)), 1.0]])
    T = T @ np.block([[np.eye(3), URDF_TOOL_XYZ.reshape(3, 1)],
                      [np.zeros((1, 3)), 1.0]])
    T = T @ np.block([[rpy_to_R(URDF_TOOL_RPY), np.zeros((3, 1))],
                      [np.zeros((1, 3)), 1.0]])
    ee_R, ee_p = T[:3, :3], T[:3, 3]

    n = len(frames)
    # DH 帧 0：原点 = joint1 轴上的 URDF 关节点；z_0 = joint1 轴；
    # x_0 取关节 1→2 公垂线方向（使 theta_1 偏置 = 0 的自然选择）
    o0 = frames[0][0]
    z0 = frames[0][1] / np.linalg.norm(frames[0][1])

    dh = np.zeros((n, 5))
    o_prev, x_prev, z_prev = o0, None, z0
    for i in range(n):
        # DH 第 i 行（关节 i+1 的因子）：由关节 i 轴（z_prev，含 URDF 符号）
        # 与关节 i+1 轴（或末端帧）的几何决定
        if i + 1 < n:
            o_ref, z_next = frames[i + 1]
            z_next = z_next / np.linalg.norm(z_next)
            # 公垂线方向（含平行退化）
            cross = np.cross(z_prev, z_next)
            if np.linalg.norm(cross) > 1e-10:
                x_i = cross / np.linalg.norm(cross)
            else:  # 平行轴：参考点连线的垂直分量（公垂线沿轴滑动不唯一，
                # 取 URDF 参考点连线，轴向滑动量自动进入相邻行的 d）
                v = o_ref - o_prev
                v = v - np.dot(v, z_prev) * z_prev
                x_i = v / np.linalg.norm(v) if np.linalg.norm(v) > 1e-10 \
                    else np.cross(z_prev, [1.0, 0.0, 0.0])
        else:
            # 末端帧：z 取 URDF 末端 z（工具接近向），x = z_prev x z_end
            # （合法 DH 帧：x_n ⊥ z_{n-1} 且 ⊥ z_n）；残余尾变换 = 常量
            # 旋转 R_6^T R_ee（原点重合，见函数文档）
            o_ref = ee_p
            z_next = ee_R[:, 2]
            cross = np.cross(z_prev, z_next)
            if np.linalg.norm(cross) > 1e-10:
                x_i = cross / np.linalg.norm(cross)
            else:
                x_i = np.cross(z_prev, [0.0, 0.0, 1.0])
                x_i = x_i / np.linalg.norm(x_i)
        alpha = signed_angle(z_prev, z_next, x_i)
        a = float(np.dot(o_ref - o_prev, x_i))
        d = float(np.dot(o_ref - o_prev, z_prev))
        if i == 0:
            theta0 = 0.0  # x_0 定义为 x_1 本身（theta_1 偏置 = 0）
            x_prev = x_i
        else:
            theta0 = signed_angle(x_prev, x_i, z_prev)
            x_prev = x_i
        dh[i] = [a, alpha, d, theta0, 0]
        # DH 帧 i 原点（URDF 参考点的轴向滑动进入下一帧 d）
        o_prev = o_prev + d * z_prev + a * x_i
        z_prev = z_next
    # 尾变换 E：DH 帧 n -> URDF 末端帧（原点重合，纯旋转）
    R_n = np.column_stack([x_i, np.cross(z_prev, x_i), z_prev])
    tool_rot = R_n.T @ ee_R
    return dh, tool_rot


# ---------------------------------------------------------------------------
# 对账
# ---------------------------------------------------------------------------

def pose_residual(T1, T2):
    """位置误差 [m] 与旋转误差 [rad]（相对旋转的测地角）。"""
    dp = np.linalg.norm(T1[:3, 3] - T2[:3, 3])
    R = T1[:3, :3].T @ T2[:3, :3]
    dth = np.arccos(np.clip(0.5 * (np.trace(R) - 1.0), -1.0, 1.0))
    return dp, dth


def main():
    rng = np.random.default_rng(7)

    # ---- 0) 数值提取器输出参考表，与手写表逐关节对比 ---------------------
    dh_ref, tool_rot = extract_dh()
    print("[0] 数值提取 DH 表（a, alpha, d, theta0）：")
    for i in range(6):
        print(f"    joint{i+1}: "
              f"a={dh_ref[i,0]:+.6f}  alpha={dh_ref[i,1]:+.6f}  "
              f"d={dh_ref[i,2]:+.6f}  theta0={dh_ref[i,3]:+.6f}")
    print(f"    尾变换 tool_rot = \n{np.round(tool_rot, 6)}")
    for i in range(6):
        for j in range(4):
            if abs(dh_ref[i, j] - B601_DH_TABLE[i, j]) > 1e-5:
                print(f"    [diff] joint{i+1} 字段{j}: "
                      f"手写 {B601_DH_TABLE[i, j]:+.6f} vs 提取 {dh_ref[i, j]:+.6f}")

    # ---- 1) URDF FK vs DH FK：随机关节采样 -------------------------------
    n_test = 200
    worst_dp = worst_dth = 0.0
    for _ in range(n_test):
        q = rng.uniform(-2.0, 2.0, 6)
        Tu, Td = urdf_fk(q), dh_fk(q)
        dp, dth = pose_residual(Tu, Td)
        worst_dp = max(worst_dp, dp)
        worst_dth = max(worst_dth, dth)
    print(f"[1] URDF FK vs DH FK  ({n_test} 随机 q): "
          f"max|dp| = {worst_dp:.3e} m, max|dtheta| = {worst_dth:.3e} rad")

    # ---- 1.5) URDF FK vs 数值提取表 FK（含提取的尾变换）-----------------
    worst_dp0 = worst_dth0 = 0.0
    for _ in range(n_test):
        q = rng.uniform(-2.0, 2.0, 6)
        Td = dh_fk(q, dh_ref, tool_angle=np.arctan2(tool_rot[1, 0], tool_rot[0, 0]))
        dp, dth = pose_residual(urdf_fk(q), Td)
        worst_dp0 = max(worst_dp0, dp)
        worst_dth0 = max(worst_dth0, dth)
    print(f"[1.5] URDF FK vs 提取表 FK ({n_test} 随机 q): "
          f"max|dp| = {worst_dp0:.3e} m, max|dtheta| = {worst_dth0:.3e} rad")

    # ---- 2) URDF FK vs TNDQ DQ 链 FK -------------------------------------
    from core.kinematics import TNDQSerialChain
    from core.dq_algebra import dq_rotation, dq_translation, dq_mul

    chain = TNDQSerialChain(B601_DH_TABLE)
    # 尾变换 E 的 DQ（纯旋转）
    E_dq = np.r_[np.cos(B601_TOOL_ANGLE / 2), 0, 0, np.sin(B601_TOOL_ANGLE / 2),
                 np.zeros(4)]
    worst_dp2 = worst_dth2 = 0.0
    for _ in range(n_test):
        q = rng.uniform(-2.0, 2.0, 6)
        Tu = urdf_fk(q)
        x = dq_mul(chain.fkm(q), E_dq)
        p_dq = dq_translation(x)
        r_dq = dq_rotation(x)
        dp = np.linalg.norm(Tu[:3, 3] - B601_BASE_PREFIX - p_dq)
        # 姿态对账：把 URDF 旋转矩阵转四元数再比测地距离
        R = Tu[:3, :3]
        tr = np.trace(R)
        qw = np.sqrt(max(0.0, 1.0 + tr)) * 0.5
        if qw > 1e-8:
            qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
            qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
            qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)
            r_urdf = np.array([qw, qx, qy, qz])
        else:
            r_urdf = np.array([0.0, 1.0, 0.0, 0.0])
        dth = 2.0 * np.arccos(np.clip(min(1.0, abs(float(r_dq @ r_urdf))), -1.0, 1.0))
        worst_dp2 = max(worst_dp2, dp)
        worst_dth2 = max(worst_dth2, dth)
    print(f"[2] URDF FK vs TNDQ DQ FK ({n_test} 随机 q): "
          f"max|dp| = {worst_dp2:.3e} m, max|dtheta| = {worst_dth2:.3e} rad")

    # ---- 3) q=0 诊断输出 ---------------------------------------------------
    T0 = urdf_fk(np.zeros(6))
    print(f"[3] q=0 末端（URDF gripper_link，base_link 系）: "
          f"p = {np.round(T0[:3, 3], 6).tolist()}")

    # ---- 4) 限位内采样（真实可达域）---------------------------------------
    lo = np.array([-2.8, -3.14, -3.14, -1.87, -1.57, -3.14])
    hi = np.array([2.8, 0.0, 0.0, 1.57, 1.57, 3.14])
    worst_dp3 = worst_dth3 = 0.0
    for _ in range(n_test):
        q = rng.uniform(lo, hi)
        dp, dth = pose_residual(urdf_fk(q), dh_fk(q))
        worst_dp3 = max(worst_dp3, dp)
        worst_dth3 = max(worst_dth3, dth)
    print(f"[4] 限位域采样 URDF vs DH: "
          f"max|dp| = {worst_dp3:.3e} m, max|dtheta| = {worst_dth3:.3e} rad")

    ok = (worst_dp < 1e-4 and worst_dth < 1e-4
          and worst_dp2 < 1e-4 and worst_dth2 < 1e-4
          and worst_dp3 < 1e-4 and worst_dth3 < 1e-4)
    print(f"结论: {'PASS - DH 表与 URDF 运动学一致' if ok else 'FAIL - 需修正 DH 表'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
