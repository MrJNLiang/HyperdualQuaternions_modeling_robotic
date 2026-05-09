import numpy as np


def joint_velocity_disturbance(t, n, scale=0.0):
    """
    关节速度扰动：
        实际发送给仿真器的是 qdot_cmd + disturbance

    scale=0 表示无扰动。
    """
    base = np.array([
        0.05 * np.sin(2.0 * np.pi * 0.40 * t),
        0.04 * np.cos(2.0 * np.pi * 0.50 * t),
        0.03 * np.sin(2.0 * np.pi * 0.60 * t),
        0.03 * np.cos(2.0 * np.pi * 0.70 * t),
        0.02 * np.sin(2.0 * np.pi * 0.80 * t),
        0.02 * np.cos(2.0 * np.pi * 0.90 * t),
        0.02 * np.sin(2.0 * np.pi * 1.00 * t),
    ], dtype=float)

    if n <= len(base):
        return scale * base[:n]

    extra = np.zeros(n - len(base))
    return scale * np.r_[base, extra]


def measurement_noise(n, pos_scale=0.0, vel_scale=0.0, rng=None):
    """
    测量噪声：
        q_meas = q + noise_q
        qdot_meas = qdot + noise_qdot
    """
    if rng is None:
        rng = np.random.default_rng()

    noise_q = pos_scale * rng.standard_normal(n)
    noise_qdot = vel_scale * rng.standard_normal(n)

    return noise_q, noise_qdot