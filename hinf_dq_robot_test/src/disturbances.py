import numpy as np


def disturbance_signals(t, scale=1.0):
    """
    Deterministic bounded disturbances.

    Return:
        vw: twist disturbance, 6D
        vc: pose uncertainty induced disturbance, 6D

    Format:
        [wx, wy, wz, vx, vy, vz]

    The first three components affect orientation error energy.
    The last three components affect translation error energy.
    """

    # twist disturbance
    vw = np.array([
        0.030 * np.sin(2.0 * np.pi * 0.70 * t),
        0.020 * np.cos(2.0 * np.pi * 0.50 * t),
        0.015 * np.sin(2.0 * np.pi * 0.90 * t + 0.3),

        0.015 * np.sin(2.0 * np.pi * 0.40 * t),
        0.012 * np.cos(2.0 * np.pi * 0.60 * t + 0.5),
        0.010 * np.sin(2.0 * np.pi * 0.80 * t)
    ], dtype=float)

    # pose uncertainty disturbance
    vc = np.array([
        0.015 * np.cos(2.0 * np.pi * 0.55 * t + 0.2),
        0.012 * np.sin(2.0 * np.pi * 0.75 * t),
        0.010 * np.cos(2.0 * np.pi * 0.35 * t),

        0.010 * np.cos(2.0 * np.pi * 0.45 * t),
        0.009 * np.sin(2.0 * np.pi * 0.65 * t + 0.4),
        0.008 * np.cos(2.0 * np.pi * 0.85 * t)
    ], dtype=float)

    return scale * vw, scale * vc