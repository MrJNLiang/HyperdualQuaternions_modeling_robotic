"""
Geometrically consistent computed-torque control law -- paper Section 5.

Reference acceleration (formula (5.2)):

    qddot_ref = J^+ ( vec6( Ad_{x_tilde} xi_dot_d + ad_{xi_tilde} Ad_{x_tilde} xi_d )
                      - K_d e_xi - k_p A^T(x_tilde) e_z - Jdot qdot )

    feedforward : transported desired acceleration + transport correction (Lemma 1)
    feedback    : -K_d e_xi - k_p A^T e_z  (A^T shaping -> exact Lyapunov
                                            cross-term cancellation, Thm 3(b))
    compensation: -Jdot qdot, obtained construction-free from the TNDQ chain (3.5)

The nominal computed-torque interface tau = M_hat qddot_ref + C_hat qdot + g_hat
(paper Sec. 2.4) is provided for completeness; the simulation operates at the
acceleration level qddot = qddot_ref + w_dyn (formula (5.1)).
"""

import numpy as np

from core.dq_algebra import dq_Ad, dq_ad, dq_vec6, vec6_to_pure_dq


def damped_pinv(J, damping=1e-6):
    """
    Damped least-squares pseudoinverse J^+ = J^T (J J^T + lambda^2 I)^-1.
    With damping > 0 near singularities J J^+ != I; the residual is part of
    the disturbance d(t) (Theorem 3, honesty remark (i)).
    """
    J = np.asarray(J, dtype=float)
    m = J.shape[0]
    return J.T @ np.linalg.inv(J @ J.T + (damping ** 2) * np.eye(m))


def feedforward_term(x_tilde, xi_tilde, xi_d_vec6, xi_dot_d_vec6):
    """
    Feedforward of formula (5.2), justified by Lemma 1 (formula (5.3)/(5.4)):

        vec6( Ad_{x_tilde} xi_dot_d + ad_{xi_tilde} Ad_{x_tilde} xi_d )

    Ad transports the desired acceleration to the current pose; the ad term
    is the transport correction so that the feedforward exactly cancels the
    non-feedback terms of the error dynamics (Theorem 3(a), proof step 3).
    All operations are DQ multiplications (paper Sec. 6.2).
    """
    xi_d = vec6_to_pure_dq(xi_d_vec6)
    xi_dot_d = vec6_to_pure_dq(xi_dot_d_vec6)

    Ad_xi_d = dq_Ad(x_tilde, xi_d)                 # Ad_{x_tilde} xi_d  (Sec. 2.2)
    Ad_xi_dot_d = dq_Ad(x_tilde, xi_dot_d)         # Ad_{x_tilde} xi_dot_d
    transport = dq_ad(xi_tilde, Ad_xi_d)           # ad_{xi_tilde} Ad_{x_tilde} xi_d

    return dq_vec6(Ad_xi_dot_d + transport)


def geometric_computed_torque_law(err, xi_d_vec6, xi_dot_d_vec6,
                                  J, Jdot_qdot, K_d, k_p, damping=1e-6):
    """
    Implementation of formula (5.2), the geometrically consistent
    computed-torque law.

    Parameters
    ----------
    err          : dict from control.error_system.full_error_state
                   (x_tilde, xi_tilde, e_xi, e_z, A -- Theorems 1/2)
    xi_d_vec6    : desired twist  vec6(xi_d)  (from desired TNDQ chain)
    xi_dot_d_vec6: desired twist rate vec6(xi_dot_d) (sigma^2 channel of x_bar_d)
    J            : geometric Jacobian (6 x n)
    Jdot_qdot    : vec6, read construction-free from the TNDQ chain (3.5)
    K_d          : 6x6 symmetric positive definite (block-diagonal for Thm 3(c-2))
    k_p          : scalar > 0

    Returns qddot_ref (n,) and the 6D task-space acceleration command.
    """
    # feedforward: transported desired acceleration + transport correction (5.2)
    u_ff = feedforward_term(err["x_tilde"], err["xi_tilde"], xi_d_vec6, xi_dot_d_vec6)

    # feedback: -K_d e_xi - k_p A^T(x_tilde) e_z   (5.2)
    u_fb = -K_d @ err["e_xi"] - k_p * (err["A"].T @ err["e_z"])

    # task-space command, then joint-space via pseudoinverse
    u_task = u_ff + u_fb - np.asarray(Jdot_qdot, dtype=float)
    qddot_ref = damped_pinv(J, damping=damping) @ u_task

    return qddot_ref, u_task


def nominal_computed_torque(M_hat, C_hat, g_hat, qddot_ref, q_dot):
    """
    Nominal computed-torque interface (paper Sec. 2.4):
        tau = M_hat qddot_ref + C_hat qdot + g_hat
    The actual acceleration then satisfies qddot = qddot_ref + w_dyn (5.1).
    TODO: connect to a rigid-body dynamics backend / CoppeliaSim torque mode
    (interfaces/coppeliasim_interface.py) when real M, C, g are available.
    """
    return M_hat @ qddot_ref + C_hat @ q_dot + g_hat
