import numpy as np


def damped_pinv(J, damping=1e-4):
    """
    Damped least-squares pseudoinverse.
    For 6 x n Jacobian:
        J^+ = J^T (J J^T + lambda^2 I)^-1
    """
    J = np.asarray(J, dtype=float)
    m, n = J.shape

    return J.T @ np.linalg.inv(J @ J.T + (damping ** 2) * np.eye(m))


def hinf_setpoint_control(J, O, T, gamma_O=1.0, gamma_T=1.0, damping=1e-4):
    """
    Set-point version of the H∞ controller.

    Paper controller:
        q_dot = J^+ ( [kO O(z); -kT T(z)] + feedforward )

    For set-point:
        feedforward = 0

    If gamma_O1 = gamma_O2 = gamma_O:
        kO = sqrt(2) / gamma_O

    If gamma_T1 = gamma_T2 = gamma_T:
        kT = sqrt(2) / gamma_T
    """
    kO = np.sqrt(2.0) / gamma_O
    kT = np.sqrt(2.0) / gamma_T

    task_velocity = np.r_[kO * O, -kT * T]

    J_pinv = damped_pinv(J, damping=damping)
    q_dot = J_pinv @ task_velocity

    return q_dot, task_velocity, kO, kT