import numpy as np
from core.hdq_math import spatial_twist_from_hdq


def compute_hdq_outputs(robot, q, qdot):
    """
    HDQ版本：
        X = x + eps_star * x_dot = HDQ_FK(q, qdot)
        xi = 2 x_dot x*
    """
    q = np.asarray(q, dtype=float)
    qdot = np.asarray(qdot, dtype=float)

    X = robot.hdq_fkm_with_qdot(q, qdot)

    x = X.dq
    x_dot = X.hd
    xi = spatial_twist_from_hdq(X)

    return {
        "x": x,
        "x_dot": x_dot,
        "xi": xi,
    }