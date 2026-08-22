"""
笛卡尔运动学轨迹生成器（v11，传统插值路线）。

设计定位（用户 v11 要求）：
    路径规划完全在任务空间（笛卡尔空间）中完成——位置用五次多项式
    样条、姿态用四元数球面插值（slerp，同一 quintic 时间标度），
    输出解析的 (p, ṗ, p̈, r, ṙ, r̈) 位姿序列。本模块不依赖任何
    TNDQ/HDQ 代数构造（纯 numpy），轨迹可解释、可独立验证。

    TNDQ 的用途被压缩到接口边界：KinematicTrajectoryAdapter 在每一
    步把 (p,r) 及其导数"表示"为控制律所需的 x_breve_d / xi_d /
    xi_dot_d（误差系统与控制律 (5.2) 的输入格式），这与 FK 层的
    TNDQ 使用同属"表示层"，不参与路径的数学构造。

与 simdata/trajectory_generator.py（TNDQ 解析路线，保留作对比备份）
的数学差异：
    - 位置：余弦斜坡 s=½-½cos(πu)  ->  五次多项式 10u³-15u⁴+6u⁵
      （两者都 C² 首尾零速零加速；quintic 中段速度更平、加速度峰值
      更低：max|s̈| = 5.77/T² vs 余弦 π²/2/T² ≈ 4.93/T²——实际两者
      接近，但 quintic 的 jerk 首尾为零，C³ 更平滑）；
    - 姿态：同样是定轴指数插值（短路径），实现独立于 core.dq_algebra。

段间连续性：每段首尾速度/加速度为 0（quintic 边界条件），路标处
C² 拼接；dwell 段恒位姿。最终时刻后恒保持末位姿（调节段）。
"""

import numpy as np


# ---------------------------------------------------------------------------
# 四元数基础运算（纯 numpy，独立实现，不 import core/）
# ---------------------------------------------------------------------------

def _qmul(a, b):
    """Hamilton 四元数乘法 [w,x,y,z]。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _qconj(a):
    return np.array([a[0], -a[1], -a[2], -a[3]])


def _normalize(a):
    return a / np.linalg.norm(a)


def quintic_s(t, T):
    """五次平滑步 s(u)=10u³-15u⁴+6u⁵ 及其解析导数（C²，首尾 v=a=0）。"""
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= T:
        return 1.0, 0.0, 0.0
    u = t / T
    s = 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5
    s_dot = (30.0 * u ** 2 - 60.0 * u ** 3 + 30.0 * u ** 4) / T
    s_ddot = (60.0 * u - 180.0 * u ** 2 + 120.0 * u ** 3) / T ** 2
    return s, s_dot, s_ddot


def _axis_angle_between(r0, r1):
    """r0 -> r1 的短路径相对旋转 (轴 n, 角 Θ)；退化时 Θ=0。"""
    r_rel = _qmul(r1, _qconj(r0))
    if r_rel[0] < 0.0:                    # 双覆盖取短路径
        r_rel = -r_rel
    vec = r_rel[1:]
    s_norm = np.linalg.norm(vec)
    theta = 2.0 * np.arctan2(s_norm, r_rel[0])
    axis = vec / s_norm if s_norm > 1e-12 else np.array([0.0, 0.0, 1.0])
    return axis, theta


def _slerp_derivatives(r0, axis, theta, theta_dot, theta_ddot):
    """定轴指数插值 r = exp((θ/2) n̂) ⊗ r0 的解析 (r, ṙ, r̈)。

    闭式（经典旋量运动学，与 TNDQ 无关）：
        ṙ  = ½ θ̇ n̂ r
        r̈  = ½ θ̈ n̂ r + ¼ θ̇² n̂² r      （n̂² = -1 的纯四元数性质）
    """
    if theta < 1e-15 and abs(theta_dot) < 1e-15:
        return r0, np.zeros(4), np.zeros(4)
    n_hat = np.r_[0.0, axis]
    half = 0.5 * theta
    q_rot = np.r_[np.cos(half), np.sin(half) * axis]
    r = _qmul(q_rot, r0)
    n_r = _qmul(n_hat, r)
    r_dot = 0.5 * theta_dot * n_r
    r_ddot = (0.5 * theta_ddot * n_r
              + 0.25 * theta_dot ** 2 * _qmul(n_hat, n_r))
    return r, r_dot, r_ddot


# ---------------------------------------------------------------------------
# 单段：位姿 quintic 插值（直线 + 定轴 slerp，同一时间标度）
# ---------------------------------------------------------------------------

class CartesianLeg:
    """(p0, r0) -> (p1, r1) 的 C² 平滑段（时长 T，首尾零速零加速）。

    位置：p(t) = p0 + s(t) Δp（直线，quintic 标度）；
    姿态：r(t) = slerp(r0, r1; s(t))（定轴短路径，同一 s(t)）——
    位置/姿态同步起止，无中间意外姿态摆动。
    """

    def __init__(self, p0, r0, p1, r1, duration):
        self.p0 = np.asarray(p0, dtype=float).reshape(3)
        self.dp = np.asarray(p1, dtype=float).reshape(3) - self.p0
        self.r0 = _normalize(np.asarray(r0, dtype=float).reshape(4))
        r1 = _normalize(np.asarray(r1, dtype=float).reshape(4))
        self.axis, self.theta = _axis_angle_between(self.r0, r1)
        self.T = float(duration)

    def evaluate(self, t):
        """返回 (p, p_dot, p_ddot, r, r_dot, r_ddot)。"""
        s, s_dot, s_ddot = quintic_s(t, self.T)
        p = self.p0 + s * self.dp
        p_dot = s_dot * self.dp
        p_ddot = s_ddot * self.dp
        r, r_dot, r_ddot = _slerp_derivatives(
            self.r0, self.axis,
            theta=s * self.theta,
            theta_dot=s_dot * self.theta,
            theta_ddot=s_ddot * self.theta)
        return p, p_dot, p_ddot, r, r_dot, r_ddot


# ---------------------------------------------------------------------------
# 多路标复合轨迹（含 dwell 保持；末端之后恒保持）
# ---------------------------------------------------------------------------

class CartesianWaypointTrajectory:
    """任务空间多路标轨迹：legs = [(p_i, r_i, T_i, dwell_i), ...]。

    从 x_start=(p0,r0) 依次 quintic 过渡到每个路标；dwell 内恒位姿；
    最后一段（含 dwell）结束后恒保持末位姿（调节段语义）。
    纯位姿输出，不做任何 TNDQ 构造。
    """

    def __init__(self, x_start, legs):
        p_cur, r_cur = x_start
        p_cur = np.asarray(p_cur, dtype=float).reshape(3)
        r_cur = _normalize(np.asarray(r_cur, dtype=float).reshape(4))
        self.segments = []       # (t0, T, dwell, leg)
        t = 0.0
        for p_t, r_t, duration, dwell in legs:
            p_t = np.asarray(p_t, dtype=float).reshape(3)
            r_t = _normalize(np.asarray(r_t, dtype=float).reshape(4))
            self.segments.append(
                (t, float(duration), float(dwell),
                 CartesianLeg(p_cur, r_cur, p_t, r_t, float(duration))))
            t += float(duration) + float(dwell)
            p_cur, r_cur = p_t, r_t
        self.p_final, self.r_final = p_cur, r_cur
        self.t_total = t

    def pose(self, t):
        """(p, ṗ, p̈, r, ṙ, r̈) —— 规划层唯一出口。"""
        for t0, T, dwell, leg in self.segments:
            if t < t0 + T + dwell:
                return leg.evaluate(t - t0)
        return (self.p_final, np.zeros(3), np.zeros(3),
                self.r_final, np.zeros(4), np.zeros(4))


# ---------------------------------------------------------------------------
# 控制器接口适配器（唯一的表示层边界：位姿导数 -> 控制律输入格式）
# ---------------------------------------------------------------------------

class KinematicTrajectoryAdapter:
    """把笛卡尔轨迹包装成 run_lib 期望的 evaluate(t) -> dict 接口。

    TNDQ 仅在此处作为"表示"出现：(p,r) 及解析导数按式 (2.1)/(3.3a)
    组装成 x_d / x_breve_d / xi_d / xi_dot_d（与 TNDQ 路线产出的字段
    完全同构），误差系统与控制律无需任何改动。规划数学（本模块上文）
    与 TNDQ 完全解耦。
    """

    def __init__(self, wp_traj):
        self.wp = wp_traj
        self.t_total = wp_traj.t_total

    def evaluate(self, t):
        from core.dq_algebra import dq_vec6
        from core.tndq_algebra import twist_from_tndq, twist_dot_from_tndq
        from simdata.trajectory_generator import _pose_tndq_from_rp_derivatives

        p, p_dot, p_ddot, r, r_dot, r_ddot = self.wp.pose(t)
        x_bar_d = _pose_tndq_from_rp_derivatives(
            r, r_dot, r_ddot, p, p_dot, p_ddot)
        return {
            "x_bar_d": x_bar_d,
            "x_breve_d": x_bar_d.to_hdq(),
            "x_d": x_bar_d.to_dq(),
            "xi_d": dq_vec6(twist_from_tndq(x_bar_d)),
            "xi_dot_d": dq_vec6(twist_dot_from_tndq(x_bar_d)),
        }
