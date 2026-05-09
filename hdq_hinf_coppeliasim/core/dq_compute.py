import numpy as np


def compute_dq_outputs(robot, q, qdot, jacobian_method="geometric"):
    """
    DQ版本：
        x = FK(q)
        J = J(q)
        xi = J(q) qdot
    """
    q = np.asarray(q, dtype=float)
    qdot = np.asarray(qdot, dtype=float)

    x = robot.fkm(q)

    if jacobian_method == "geometric":
        J = robot.pose_jacobian_geometric(q)
    elif jacobian_method == "numeric":
        J = robot.pose_jacobian_numeric(q)
    elif jacobian_method == "hdq_fast":
        J = robot.pose_jacobian_hdq_fast(q)
    else:
        raise ValueError(f"Unknown jacobian_method: {jacobian_method}")

    xi = J @ qdot

    return {
        "x": x,
        "J": J,
        "xi": xi,
    }