"""
Desired-trajectory generator producing the TNDQ representation of the
desired curve -- paper Sec. 3.2 (formula (3.3a)) and Sec. 4.2.

The desired chain is modeled in TNDQ because the feedforward of the control
law (5.2) needs the sigma^2 channel (xi_dot_d); the error system then only
consumes the HDQ truncation x_breve_d (Proposition 2: lossless).

Both trajectory classes return analytic (p, p_dot, p_ddot) and
(r, r_dot, r_ddot), assembled into

    x_d      = r + eps (1/2) p r                       (formula (2.1))
    x_dot_d  = r_dot + eps (1/2)(p_dot r + p r_dot)      d/dt of (2.1)
    x_ddot_d = r_ddot + eps (1/2)(p_ddot r + 2 p_dot r_dot + p r_ddot)

so the unit-constraint family (3.8) holds exactly (no numeric differentiation).
Derived xi_d, xi_dot_d follow from formula (3.5).
"""

import numpy as np

from core.dq_algebra import q_mul, q_conj, dq_vec6
from core.tndq_algebra import TNDQ, twist_from_tndq, twist_dot_from_tndq


def _pose_tndq_from_rp_derivatives(r, r_dot, r_ddot, p, p_dot, p_ddot):
    """Assemble the TNDQ representation (3.3a) from analytic r/p derivatives."""
    pq = np.r_[0.0, p]
    pq_dot = np.r_[0.0, p_dot]
    pq_ddot = np.r_[0.0, p_ddot]

    x = np.r_[r, 0.5 * q_mul(pq, r)]
    x_dot = np.r_[r_dot, 0.5 * (q_mul(pq_dot, r) + q_mul(pq, r_dot))]
    x_ddot = np.r_[r_ddot,
                   0.5 * (q_mul(pq_ddot, r) + 2.0 * q_mul(pq_dot, r_dot) + q_mul(pq, r_ddot))]
    return TNDQ.from_pose_derivatives(x, x_dot, x_ddot)


def _rotation_derivatives(r0, axis, phi, phi_dot, phi_ddot):
    """
    Rotation curve r(t) = exp((phi/2) n_hat) r0 about a fixed unit axis n.
    Closed forms (same structure as Appendix B.1, screw = pure rotation):
        r_dot  = (1/2) phi_dot  n_hat r
        r_ddot = (1/2) phi_ddot n_hat r + (1/4) phi_dot^2 n_hat^2 r
    with n_hat the pure quaternion of the axis (n_hat^2 = -1).
    """
    n_hat = np.r_[0.0, axis]
    half = 0.5 * phi
    r_rot = np.r_[np.cos(half), np.sin(half) * axis]
    r = q_mul(r_rot, r0)
    n_r = q_mul(n_hat, r)
    r_dot = 0.5 * phi_dot * n_r
    r_ddot = 0.5 * phi_ddot * n_r + 0.25 * (phi_dot ** 2) * q_mul(n_hat, n_r)
    return r, r_dot, r_ddot


def _smooth_s(t, T):
    """C^inf ramp s = 1/2 - 1/2 cos(pi t/T) on [0, T]; analytic s_dot, s_ddot."""
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= T:
        return 1.0, 0.0, 0.0
    a = np.pi / T
    return (0.5 - 0.5 * np.cos(a * t),
            0.5 * a * np.sin(a * t),
            0.5 * a * a * np.cos(a * t))


class TrajectoryBase:
    """Common evaluation: (r,p)-derivatives -> TNDQ -> (x_d, xi_d, xi_dot_d)."""

    def _rp_derivatives(self, t):
        raise NotImplementedError

    def evaluate(self, t):
        """
        Returns dict:
            x_bar_d   : desired TNDQ (3.3a) -- sigma^2 channel feeds (5.2) feedforward
            x_breve_d : HDQ truncation (3.6) for the error layer (Theorem 1)
            x_d       : pose DQ
            xi_d      : vec6 desired twist        (3.5)
            xi_dot_d  : vec6 desired twist rate   (3.5)
        """
        r, r_dot, r_ddot, p, p_dot, p_ddot = self._rp_derivatives(t)
        x_bar_d = _pose_tndq_from_rp_derivatives(r, r_dot, r_ddot, p, p_dot, p_ddot)
        return {
            "x_bar_d": x_bar_d,
            "x_breve_d": x_bar_d.to_hdq(),
            "x_d": x_bar_d.to_dq(),
            "xi_d": dq_vec6(twist_from_tndq(x_bar_d)),
            "xi_dot_d": dq_vec6(twist_dot_from_tndq(x_bar_d)),
        }


class LineTrajectoryTNDQ(TrajectoryBase):
    """
    Straight-line tip motion with smooth start/stop:
        p_d(t) = p0 + s(t) (p1 - p0),  s = 1/2 - 1/2 cos(pi t/T)
    Optional simultaneous rotation of angle `rot_angle` about `rot_axis`
    with the same s(t) profile (exercises the rotational channels of (5.2)).
    """

    def __init__(self, x_start, delta_p, duration, rot_axis=None, rot_angle=0.0):
        from core.dq_algebra import dq_rotation, dq_translation
        self.r0 = dq_rotation(np.asarray(x_start, dtype=float))
        self.p0 = dq_translation(np.asarray(x_start, dtype=float))
        self.dp = np.asarray(delta_p, dtype=float).reshape(3)
        self.T = float(duration)
        if rot_axis is None:
            self.axis = None
        else:
            axis = np.asarray(rot_axis, dtype=float).reshape(3)
            self.axis = axis / np.linalg.norm(axis)
        self.rot_angle = float(rot_angle)

    def _rp_derivatives(self, t):
        s, s_dot, s_ddot = _smooth_s(t, self.T)
        p = self.p0 + s * self.dp
        p_dot = s_dot * self.dp
        p_ddot = s_ddot * self.dp

        if self.axis is None or abs(self.rot_angle) < 1e-15:
            r, r_dot, r_ddot = self.r0, np.zeros(4), np.zeros(4)
        else:
            r, r_dot, r_ddot = _rotation_derivatives(
                self.r0, self.axis,
                phi=s * self.rot_angle,
                phi_dot=s_dot * self.rot_angle,
                phi_ddot=s_ddot * self.rot_angle,
            )
        return r, r_dot, r_ddot, p, p_dot, p_ddot


class CircleTrajectoryTNDQ(TrajectoryBase):
    """
    Circle in the x-z plane through the start point, with a C^2 radius ramp
    (quintic smoothstep) during the first `ramp_time` seconds:
        p_d(t) = p0 + a(t) * R [cos(w t) - 1, 0, sin(w t)]
    Orientation held at r0 (omega_d = 0).
    """

    def __init__(self, x_start, radius, period, ramp_time):
        from core.dq_algebra import dq_rotation, dq_translation
        self.r0 = dq_rotation(np.asarray(x_start, dtype=float))
        self.p0 = dq_translation(np.asarray(x_start, dtype=float))
        self.R = float(radius)
        self.w = 2.0 * np.pi / float(period)
        self.Tr = float(ramp_time)

    def _ramp(self, t):
        """Quintic smoothstep a = 10u^3 - 15u^4 + 6u^5 (C^2 at both ends)."""
        if t <= 0.0:
            return 0.0, 0.0, 0.0
        if t >= self.Tr:
            return 1.0, 0.0, 0.0
        u = t / self.Tr
        a = 10 * u**3 - 15 * u**4 + 6 * u**5
        a_dot = (30 * u**2 - 60 * u**3 + 30 * u**4) / self.Tr
        a_ddot = (60 * u - 180 * u**2 + 120 * u**3) / self.Tr**2
        return a, a_dot, a_ddot

    def _rp_derivatives(self, t):
        a, a_dot, a_ddot = self._ramp(t)
        w, R = self.w, self.R
        c, s = np.cos(w * t), np.sin(w * t)

        offset = R * np.array([c - 1.0, 0.0, s])
        offset_dot = R * w * np.array([-s, 0.0, c])
        offset_ddot = R * w * w * np.array([-c, 0.0, -s])

        p = self.p0 + a * offset
        p_dot = a_dot * offset + a * offset_dot
        p_ddot = a_ddot * offset + 2.0 * a_dot * offset_dot + a * offset_ddot

        return self.r0, np.zeros(4), np.zeros(4), p, p_dot, p_ddot


class SetpointTrajectoryTNDQ(TrajectoryBase):
    """
    定点（调节）轨迹 —— 场景篇 §4 S1：杯口上方预抓取位姿。

    恒值目标 x_d = r* + ε·½ p* r*（式 2.1），ξ_d = ξ̇_d = 0：
    控制律 (5.2) 的前馈项自动退化为纯反馈 -K_d e_ξ - k_p Aᵀ e_z，
    控制栈无需任何改动（定理 3(b) 的调节问题特例）。
    """

    def __init__(self, p_target, r_target):
        self.p = np.asarray(p_target, dtype=float).reshape(3)
        r = np.asarray(r_target, dtype=float).reshape(4)
        self.r = r / np.linalg.norm(r)

    def _rp_derivatives(self, t):
        zeros4 = np.zeros(4)
        zeros3 = np.zeros(3)
        return self.r, zeros4, zeros4, self.p, zeros3, zeros3


class CupCircleTrajectoryTNDQ(TrajectoryBase):
    """
    绕杯水平圆周轨迹 —— 场景篇 §5 S2：

        p_d(t) = c + a(t) R [cos(ωt), sin(ωt), 0],  姿态恒为 r*（竖直向下）

    圆心 c = 杯口上方 CIRCLE_HEIGHT，圆平面水平（与 CircleTrajectoryTNDQ
    的 x-z 竖直圆不同）；半径经五次多项式 a(t) 缓启（C²，避免加速度跳变）。
    解析 (p, ṗ, p̈) 保证约束族 (3.8) 沿期望链精确成立；σ² 通道的 ξ̇_d
    携带向心加速度 ω²R，由式 (5.2) 前馈精确消化（引理 1；高速挡 E5
    依赖此项，总方案 §6.2 预言的差异来源）。
    """

    def __init__(self, center, radius, omega, r_target, ramp_time=2.0):
        self.c = np.asarray(center, dtype=float).reshape(3)
        self.R = float(radius)
        self.w = float(omega)
        r = np.asarray(r_target, dtype=float).reshape(4)
        self.r0 = r / np.linalg.norm(r)
        self.Tr = float(ramp_time)

    def _ramp(self, t):
        """五次平滑步 a = 10u³-15u⁴+6u⁵（与 CircleTrajectoryTNDQ 同构）。"""
        if t <= 0.0:
            return 0.0, 0.0, 0.0
        if t >= self.Tr:
            return 1.0, 0.0, 0.0
        u = t / self.Tr
        a = 10 * u**3 - 15 * u**4 + 6 * u**5
        a_dot = (30 * u**2 - 60 * u**3 + 30 * u**4) / self.Tr
        a_ddot = (60 * u - 180 * u**2 + 120 * u**3) / self.Tr**2
        return a, a_dot, a_ddot

    def start_point(self):
        """圆周起点（t=0，a=0 时位于圆心；实际进入圆周后的参考点）。"""
        return self.c + self.R * np.array([1.0, 0.0, 0.0])

    def _rp_derivatives(self, t):
        a, a_dot, a_ddot = self._ramp(t)
        w, R = self.w, self.R
        cwt, swt = np.cos(w * t), np.sin(w * t)

        offset = R * np.array([cwt, swt, 0.0])
        offset_dot = R * w * np.array([-swt, cwt, 0.0])
        offset_ddot = -w * w * offset          # 向心加速度 ω²R 指向圆心

        p = self.c + a * offset
        p_dot = a_dot * offset + a * offset_dot
        p_ddot = a_ddot * offset + 2.0 * a_dot * offset_dot + a * offset_ddot

        return self.r0, np.zeros(4), np.zeros(4), p, p_dot, p_ddot


def goto_trajectory(x_start, p_target, r_target, duration):
    """
    平滑趋近段（go-to 参考整形）：从当前位姿 x_start 平滑过渡到
    目标 (p*, r*)，到达后恒定保持（LineTrajectoryTNDQ 在 t ≥ T 后
    s≡1，各阶导数为 0 ⇒ 自动退化为定点目标）。

    工程动机（场景篇 §8 降级预案的对偶问题）：力矩模式下对 ~180°
    姿态误差直接做常值目标调节，纯反馈 -k_p Aᵀe_z 瞬时指令远超
    力矩/限位预算；参考整形把大误差分解为沿路小误差跟踪（定理 3
    证书逐步生效）；纯调节压力测试可用 --t-go 0 关闭。

    旋转路径：相对旋转 r_rel = r* r0⁻¹ 的轴角分解，取短路径
    （w<0 时翻转双覆盖符号，与定理 1(i) unwinding 处置同源）。
    """
    from core.dq_algebra import dq_rotation, dq_translation
    x_start = np.asarray(x_start, dtype=float)
    r0 = dq_rotation(x_start)
    p0 = dq_translation(x_start)
    r_t = np.asarray(r_target, dtype=float).reshape(4)
    r_t = r_t / np.linalg.norm(r_t)

    r_rel = q_mul(r_t, q_conj(r0))
    if r_rel[0] < 0.0:                      # 双覆盖取短路径
        r_rel = -r_rel
    vec = r_rel[1:]
    s_norm = np.linalg.norm(vec)
    angle = 2.0 * np.arctan2(s_norm, r_rel[0])
    axis = vec / s_norm if s_norm > 1e-12 else np.array([0.0, 0.0, 1.0])

    return LineTrajectoryTNDQ(
        x_start,
        delta_p=np.asarray(p_target, dtype=float) - p0,
        duration=float(duration),
        rot_axis=axis, rot_angle=angle)


class CompositeTrajectoryTNDQ:
    """分段轨迹：按时间顺序拼接多段 TrajectoryBase（场景篇 §5.1：
    "末端先经 S1 流程到达圆周起点，再切入跟踪"）。
    segments: [(trajectory, t_start), ...]，t_start 递增；
    evaluate(t) 分发到当前段并做时间平移。"""

    def __init__(self, segments):
        self.segments = sorted(segments, key=lambda s: s[1])

    def evaluate(self, t):
        traj, t0 = self.segments[0]
        for seg, seg_t0 in self.segments:
            if t >= seg_t0:
                traj, t0 = seg, seg_t0
        return traj.evaluate(t - t0)
