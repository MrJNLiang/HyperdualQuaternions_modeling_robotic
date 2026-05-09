import numpy as np
from core.dq_math import q_mul


def dq_from_r_p(r, p):
    """
    根据旋转四元数 r 和位置 p 构造位姿DQ：
        x = r + eps * 1/2 * p * r
    """
    r = np.asarray(r, dtype=float).reshape(4)
    p = np.asarray(p, dtype=float).reshape(3)

    p_quat = np.r_[0.0, p]
    qd = 0.5 * q_mul(p_quat, r)

    return np.r_[r, qd]


class LineTrajectory:
    """
    末端沿直线运动：
        p_d(t) = p_start + s(t) * (p_end - p_start)

    姿态保持为 r_start。
    """

    def __init__(self, x_start, delta_p=None, duration=8.0):
        self.x_start = np.asarray(x_start, dtype=float).reshape(8)

        self.r_start = self.x_start[:4].copy()

        # 从DQ中取平移
        from core.dq_math import dq_translation
        self.p_start = dq_translation(self.x_start)

        if delta_p is None:
            # 默认让末端沿 x 和 z 方向移动一点
            delta_p = np.array([0.15, 0.0, 0.10], dtype=float)

        self.p_end = self.p_start + np.asarray(delta_p, dtype=float).reshape(3)
        self.duration = float(duration)

    def evaluate(self, t):
        """
        返回：
            xd: 期望末端位姿DQ
            xi_d: 期望末端空间twist，6维 [omega, v]
        """
        T = self.duration

        if t <= 0.0:
            s = 0.0
            s_dot = 0.0
        elif t >= T:
            s = 1.0
            s_dot = 0.0
        else:
            tau = t / T
            # 平滑起停：s = 0.5 - 0.5 cos(pi tau)
            s = 0.5 - 0.5 * np.cos(np.pi * tau)
            s_dot = 0.5 * np.pi / T * np.sin(np.pi * tau)

        dp = self.p_end - self.p_start
        p_d = self.p_start + s * dp
        p_dot_d = s_dot * dp

        xd = dq_from_r_p(self.r_start, p_d)

        # 姿态固定，omega_d = 0
        # 原文twist形式：xi = omega + eps(p_dot + p × omega)
        # omega=0时，dual part 就是 p_dot
        xi_d = np.r_[0.0, 0.0, 0.0, p_dot_d]

        return xd, xi_d