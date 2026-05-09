from core.dq_compute import compute_dq_outputs
from core.hdq_compute import compute_hdq_outputs


def compute_fk_outputs(robot, q, qdot, backend="dq", jacobian_method="geometric"):
    """
    统一接口：
        输入 q, qdot
        输出末端位姿 x 和末端速度 xi

    backend:
        "dq"  : x=FK(q), xi=J(q)qdot
        "hdq" : x, xdot, xi=HDQ_FK(q,qdot)
    """
    if backend == "dq":
        return compute_dq_outputs(
            robot,
            q,
            qdot,
            jacobian_method=jacobian_method
        )

    if backend == "hdq":
        return compute_hdq_outputs(robot, q, qdot)

    raise ValueError(f"Unknown backend: {backend}")