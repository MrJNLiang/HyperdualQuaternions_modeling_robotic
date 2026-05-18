import numpy as np
from core.hdq_math import spatial_twist_from_hdq, hdq_poe_chain_from_model


def compute_hdq_outputs(robot, q, qdot, method="chain"):
    """
    HDQ output:
        X = x + eps_star * x_dot
        xi = 2 x_dot x*

    Parameters
    ----------
    method : str
        "chain":
            Use pure HDQ/POE chain propagation when the robot has S, M, q_home.
            This does NOT explicitly form J(q). It uses qdot_i inside each
            joint HDQ factor and obtains the final x_dot from HDQ multiplication.

        "jacobian":
            Use robot.hdq_fkm_with_qdot(q, qdot), which in the current
            CoppeliaPOEModel implementation usually computes xi = J(q) qdot
            first and then constructs x_dot = 1/2 xi x.

        "auto":
            Use "chain" if the robot has S, M, q_home, otherwise fall back
            to "jacobian".
    """
    q = np.asarray(q, dtype=float)
    qdot = np.asarray(qdot, dtype=float)

    if method == "auto":
        method = "chain" if all(hasattr(robot, name) for name in ["S", "M", "q_home"]) else "jacobian"

    if method == "chain":
        if not all(hasattr(robot, name) for name in ["S", "M", "q_home"]):
            raise AttributeError(
                "method='chain' requires robot.S, robot.M and robot.q_home. "
                "Use method='jacobian' for the old DH/robot interface."
            )
        X = hdq_poe_chain_from_model(robot, q, qdot)

    elif method == "jacobian":
        X = robot.hdq_fkm_with_qdot(q, qdot)

    else:
        raise ValueError(f"Unknown HDQ method: {method}")

    x = X.dq
    x_dot = X.hd
    xi = spatial_twist_from_hdq(X)

    return {
        "x": x,
        "x_dot": x_dot,
        "xi": xi,
        "method": method,
    }
