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
    k_p          : pose gain -- scalar k_p > 0, or a symmetric positive definite
                   6x6 matrix K_p = diag(p_O I3, p_T I3).  The matrix form keeps
                   Theorem 3 exact provided the storage function uses the same
                   weight, V = 1/2||e_xi||^2 + 1/2 e_z^T K_p e_z: the feedback
                   -A^T K_p e_z then cancels the cross term e_z^T K_p A e_xi
                   identically (see control/gain_design.py).  It removes the
                   1/4 rotation-stiffness handicap of a single scalar gain.

    Returns qddot_ref (n,) and the 6D task-space acceleration command.
    """
    # feedforward: transported desired acceleration + transport correction (5.2)
    u_ff = feedforward_term(err["x_tilde"], err["xi_tilde"], xi_d_vec6, xi_dot_d_vec6)

    # feedback: -K_d e_xi - A^T(x_tilde) K_p e_z   (5.2; K_p inside A^T so that
    # the Lyapunov cross-term cancellation holds for any symmetric K_p)
    K_p = np.asarray(k_p, dtype=float)
    pose = K_p @ err["e_z"] if K_p.ndim == 2 else K_p * err["e_z"]
    u_fb = -K_d @ err["e_xi"] - err["A"].T @ pose

    # task-space command, then joint-space via pseudoinverse
    u_task = u_ff + u_fb - np.asarray(Jdot_qdot, dtype=float)
    qddot_ref = damped_pinv(J, damping=damping) @ u_task

    return qddot_ref, u_task


# ---------------------------------------------------------------------------
# Baseline: first-order DQ H-infinity kinematic law (paper Sec. 6 comparison
# row C3, ported verbatim from hdq_hinf_coppeliasim/core/controllers.py::
# hinf_tracking_control -- the "previous theory" of this repository).
#
#     qdot_cmd = J^+ ( [kO O; -kT T] + vec6( x_tilde xi_d x_tilde* ) ),
#     kO = sqrt(2)/gamma_O,  kT = sqrt(2)/gamma_T.
#
# Structural differences to (5.2) that the S3 comparison probes:
#   - velocity-level (1st order): no e_xi damping channel, no xi_dot_d
#     feedforward, no ad transport correction, no -Jdot qdot compensation;
#   - to share the same torque interface tau = M_hat qddot_ref + C_hat qdot
#     + g_hat, qdot_cmd must be bridged to qddot_ref by an inner velocity
#     servo with a *numerically differentiated* feedforward -- exactly the
#     "no second-order channel -> lagged feedforward" shortcoming listed in
#     the README C3 row (the TNDQ chain provides xi_dot_d/Jdot qdot
#     analytically instead, formula (3.5)).
# Error conventions are identical in both projects (O = -Im(r_tilde),
# T = p_tilde, x_tilde = x x_d*), so the port is sign-exact.
# ---------------------------------------------------------------------------

def dq_hinf_kinematic_law(err, xi_d_vec6, gamma_O, gamma_T):
    """
    Baseline C3: first-order DQ H-infinity tracking law (velocity level).

    Parameters
    ----------
    err       : dict from control.error_system.full_error_state
                (uses x_tilde, O, T -- same conventions as the source repo)
    xi_d_vec6 : desired twist vec6(xi_d)
    gamma_O/T : prescribed per-channel H-infinity levels; kO = sqrt(2)/gamma_O

    Returns the 6D task-velocity command; the caller maps it to joint space
    via the same damped pseudoinverse used by (5.2) (same budget, Sec. 5.2
    of the comparison plan).
    """
    kO = np.sqrt(2.0) / gamma_O
    kT = np.sqrt(2.0) / gamma_T

    feedback = np.r_[kO * err["O"], -kT * err["T"]]
    # feedforward vec6(x_tilde xi_d x_tilde*) == Ad_{x_tilde} xi_d
    feedforward = dq_vec6(dq_Ad(err["x_tilde"], vec6_to_pure_dq(xi_d_vec6)))
    return feedback + feedforward


# ---------------------------------------------------------------------------
# Baseline C2: second-order DQ computed-torque law (literature DQ-CTC form,
# comparison plan Sec. 5.2 row C2/C3 "现有 DQ 动力学控制" -- second-order
# task-space PD on the DQ pose error with *numerically differentiated*
# feedforward and Jdot, i.e. a DQ CTC without the TNDQ sigma^2 channels):
#
#     u_task = xi_dot_d_num + K_d (xi_d - xi) + [p_O O; -p_T T]
#     qddot_ref = J^+ ( u_task - (Jdot qdot)_num )
#
# Structural differences probed by the three-way S3 comparison:
#   - naive twist difference xi_d - xi (no Ad transport -> spurious term of
#     Sec. 4.1 grows with ||xi_d||);
#   - no A^T(x_tilde) shaping -> the Lyapunov cross terms do NOT cancel,
#     no H-inf/ISS certificate (unlike Theorem 3);
#   - xi_dot_d and Jdot qdot obtained by finite differences (one-step lag +
#     differentiation noise), vs the construction-free TNDQ channel (3.5).
# Near identity the linearised channels are
#     rotation    :  Oddot + K_omega Odot + (p_O/2) O = -d_omega/2
#     translation :  Tddot + K_v     Tdot +  p_T    T = +d_v
# (factor 1/2 instead of C1's 1/4: only one 1/2 from Odot = -omega/2, none
# from A^T).  Channel matching to C1 tuned therefore needs p_O = 2*a0.
# Pose/twist error conventions identical to C1/C3 (O=-Im(r_tilde), T=p_tilde).
# ---------------------------------------------------------------------------

def dq_ctc_law(err, xi_vec6, xi_d_vec6, xi_dot_d_num, Jdot_qdot_num,
               J, K_d, K_p, damping=1e-6):
    """
    Baseline C2: second-order DQ computed-torque law (acceleration level).

    Parameters
    ----------
    err            : dict from control.error_system.full_error_state
                     (uses O, T -- same conventions as C1/C3)
    xi_vec6        : measured twist vec6(xi)
    xi_d_vec6      : desired twist vec6(xi_d)
    xi_dot_d_num   : finite-difference desired twist rate
                     (xi_d - xi_d_prev)/dt -- the honest realisation of a
                     DQ CTC without the analytic sigma^2 channel
    Jdot_qdot_num  : finite-difference (J - J_prev)/dt @ qdot
    K_d            : 6x6 symmetric positive definite twist-error gain
    K_p            : 6x6 K_p = diag(p_O I3, p_T I3) or scalar pose gain

    Returns qddot_ref (n,) and the 6D task-space acceleration command.
    """
    K_p = np.asarray(K_p, dtype=float)
    Kp = K_p if K_p.ndim == 2 else K_p * np.eye(6)
    # pose feedback [ +p_O O ; -p_T T ] (sign convention identical to C3)
    u_pose = np.r_[Kp[:3, :3] @ err["O"], -(Kp[3:, 3:] @ err["T"])]
    # naive twist difference (no Ad transport -- the Sec. 4.1 spurious term)
    e_v = np.asarray(xi_d_vec6, dtype=float) - np.asarray(xi_vec6, dtype=float)

    u_task = (np.asarray(xi_dot_d_num, dtype=float) + K_d @ e_v + u_pose
              - np.asarray(Jdot_qdot_num, dtype=float))
    qddot_ref = damped_pinv(J, damping=damping) @ u_task
    return qddot_ref, u_task


def velocity_to_accel_ref(qdot_cmd, qdot_cmd_prev, q_dot, dt, k_servo):
    """
    Bridge a velocity-level command to the shared torque interface:

        qddot_ref = (qdot_cmd - qdot_cmd_prev)/dt + k_servo (qdot_cmd - qdot)

    The finite-difference feedforward is the honest realisation of C3's
    missing second-order channel (one-step lag + differentiation noise are
    genuine properties of the baseline, not implementation artefacts).
    First call (qdot_cmd_prev is None) uses servo term only.
    """
    qdot_cmd = np.asarray(qdot_cmd, dtype=float)
    ff = np.zeros_like(qdot_cmd) if qdot_cmd_prev is None else \
        (qdot_cmd - np.asarray(qdot_cmd_prev, dtype=float)) / dt
    return ff + k_servo * (qdot_cmd - np.asarray(q_dot, dtype=float))


def nominal_computed_torque(M_hat, C_hat, g_hat, qddot_ref, q_dot):
    """
    Nominal computed-torque interface (paper Sec. 2.4):
        tau = M_hat qddot_ref + C_hat qdot + g_hat
    The actual acceleration then satisfies qddot = qddot_ref + w_dyn (5.1).
    TODO: connect to a rigid-body dynamics backend / CoppeliaSim torque mode
    (interfaces/coppeliasim_interface.py) when real M, C, g are available.
    """
    return M_hat @ qddot_ref + C_hat @ q_dot + g_hat
