import numpy as np
from core.dq_math import dq_translation, q_mul


def dq_from_r_p(r, p):
    r = np.asarray(r, dtype=float).reshape(4)
    p = np.asarray(p, dtype=float).reshape(3)
    p_quat = np.r_[0.0, p]
    qd = 0.5 * q_mul(p_quat, r)
    return np.r_[r, qd]


def smoothstep(tau):
    tau = np.clip(tau, 0.0, 1.0)
    return 3 * tau**2 - 2 * tau**3


def smoothstep_dot(tau):
    tau = np.clip(tau, 0.0, 1.0)
    return 6 * tau - 6 * tau**2


class SmoothCircleTrajectory:
    """
    平滑启动圆轨迹：
    前 ramp_time 秒，圆半径从0平滑增大到 R；
    之后末端在 x-z 平面画圆。
    """

    def __init__(self, x_start, radius=0.08, period=8.0, ramp_time=2.0):
        self.x_start = np.asarray(x_start, dtype=float).reshape(8)
        self.r_start = self.x_start[:4].copy()
        self.p_start = dq_translation(self.x_start)

        self.radius = float(radius)
        self.period = float(period)
        self.omega = 2.0 * np.pi / self.period
        self.ramp_time = float(ramp_time)

    def evaluate(self, t):
        R = self.radius
        w = self.omega
        Tr = self.ramp_time

        if t < Tr:
            tau = t / Tr
            a = smoothstep(tau)
            a_dot = smoothstep_dot(tau) / Tr
        else:
            a = 1.0
            a_dot = 0.0

        theta = w * t

        circle_offset = np.array([
            R * (np.cos(theta) - 1.0),
            0.0,
            R * np.sin(theta)
        ])

        circle_vel = np.array([
            -R * w * np.sin(theta),
            0.0,
             R * w * np.cos(theta)
        ])

        p_d = self.p_start + a * circle_offset
        p_dot_d = a_dot * circle_offset + a * circle_vel

        xd = dq_from_r_p(self.r_start, p_d)

        xi_d = np.r_[0.0, 0.0, 0.0, p_dot_d]

        return xd, xi_d