"""
DQ (dual quaternion) algebra layer -- paper Section 2.

Algebra:  H_hat = H + eps*H,  eps^2 = 0   (paper Sec. 2.1)

Array conventions (consistent with the existing project code base):
    quaternion q            : 4-array  [w, x, y, z]
    dual quaternion (DQ) x  : 8-array  [rw, rx, ry, rz, dw, dx, dy, dz]
                              = primary quaternion (4) + dual quaternion (4)
    unit pose DQ (2.1)      : x_hat = r + eps * (1/2) p r
    pure DQ twist (2.2)     : xi = omega + eps*v ,  vec6(xi) = [w; v]

All functions are pure numpy, no external dependency.
"""

import numpy as np


# ---------------------------------------------------------------------------
# Quaternion level  (paper Sec. 2.1, algebra H)
# ---------------------------------------------------------------------------

def q_mul(a, b):
    """Quaternion product in H.  q = [w, x, y, z]."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=float)


def q_conj(q):
    """Quaternion conjugate q* = eta - mu  (paper Sec. 2.1)."""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def q_normalize(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("Quaternion norm too small, cannot normalize.")
    return q / n


def q_exp_axis(n_axis, angle):
    """
    Unit quaternion for rotation of `angle` about unit axis `n_axis`:
        r = cos(phi/2) + n sin(phi/2)      (paper Sec. 2.1, Spin(3))
    """
    n_axis = np.asarray(n_axis, dtype=float).reshape(3)
    nn = np.linalg.norm(n_axis)
    if nn < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    n_axis = n_axis / nn
    half = 0.5 * float(angle)
    return np.r_[np.cos(half), np.sin(half) * n_axis]


def skew(v):
    """[v]_x  cross-product matrix, used in A(x_tilde) of Theorem 2."""
    v = np.asarray(v, dtype=float).reshape(3)
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ], dtype=float)


# ---------------------------------------------------------------------------
# Dual quaternion level  (paper Sec. 2.1, algebra H_hat = H + eps H, eps^2=0)
# ---------------------------------------------------------------------------

def dq_zero():
    return np.zeros(8, dtype=float)


def dq_identity():
    return np.array([1.0, 0, 0, 0, 0, 0, 0, 0], dtype=float)


def dq_mul(a, b):
    """
    DQ product with eps^2 = 0:
        (a0 + eps a1)(b0 + eps b1) = a0 b0 + eps (a0 b1 + a1 b0)
    This is the epsilon-level Leibniz rule (same shape as paper (2.3)/(3.2)).
    """
    ar, ad = a[:4], a[4:]
    br, bd = b[:4], b[4:]
    real = q_mul(ar, br)
    dual = q_mul(ar, bd) + q_mul(ad, br)
    return np.r_[real, dual]


def dq_conj(x):
    """DQ conjugate x* : component-wise quaternion conjugation (paper Sec. 2.1)."""
    return np.r_[q_conj(x[:4]), q_conj(x[4:])]


def dq_from_r_p(r, p):
    """
    Implementation of formula (2.1):
        x_hat = r + eps * (1/2) p r ,  p pure quaternion [0, px, py, pz].
    """
    r = np.asarray(r, dtype=float).reshape(4)
    p_quat = np.r_[0.0, np.asarray(p, dtype=float).reshape(3)]
    return np.r_[r, 0.5 * q_mul(p_quat, r)]


def dq_translation(x):
    """Recover translation p = 2 qd r*  from x = r + eps (1/2) p r (formula (2.1))."""
    p_quat = 2.0 * q_mul(x[4:], q_conj(x[:4]))
    return p_quat[1:4]


def dq_rotation(x):
    """Rotation quaternion part of a pose DQ."""
    return np.array(x[:4], dtype=float)


def dq_scalar_parts(x):
    """
    Sc(.) operator of paper Sec. 3.4:
    return [scalar part, dual-scalar part] = [x[0], x[4]].
    """
    return np.array([x[0], x[4]], dtype=float)


def dq_pure_part(x):
    """Project onto pure DQ subspace (zero scalar and dual-scalar parts)."""
    y = np.array(x, dtype=float)
    y[0] = 0.0
    y[4] = 0.0
    return y


def dq_vec6(xi):
    """vec6 isomorphism of paper Sec. 2.1: pure DQ -> R^6, [omega; v]."""
    return np.r_[xi[1:4], xi[5:8]]


def vec6_to_pure_dq(v):
    """Inverse isomorphism vec6^{-1}: R^6 -> pure DQ."""
    v = np.asarray(v, dtype=float).reshape(6)
    return np.array([0.0, v[0], v[1], v[2], 0.0, v[3], v[4], v[5]], dtype=float)


# ---------------------------------------------------------------------------
# Adjoint action and Lie bracket  (paper Sec. 2.2)
# ---------------------------------------------------------------------------

def dq_Ad(x, a):
    """
    Adjoint action of a unit DQ on a pure DQ (paper Sec. 2.2):
        Ad_x a = x a x*
    Transports a twist between reference poses; preserves purity and norm.
    """
    return dq_mul(dq_mul(x, a), dq_conj(x))


def dq_ad(a, b):
    """
    Lie bracket on pure DQs (paper Sec. 2.2):
        ad_a b = (1/2)(a b - b a)
    Result is again a pure DQ.
    """
    return 0.5 * (dq_mul(a, b) - dq_mul(b, a))


# ---------------------------------------------------------------------------
# Twist and pose kinematics  (paper Sec. 2.2)
# ---------------------------------------------------------------------------

def dq_twist(x, x_dot):
    """
    Implementation of formula (2.2):  spatial twist
        xi = 2 x_dot x*   (pure DQ, cf. Appendix A.1)
    """
    return 2.0 * dq_mul(x_dot, dq_conj(x))


def dq_pose_normalize(x):
    """
    0-th order re-projection of paper Sec. 3.4:
    normalize the rotation quaternion and rebuild the dual part from the
    current translation so that x x* = 1 holds again after numeric drift.
    """
    r = q_normalize(x[:4])
    p = (2.0 * q_mul(x[4:], q_conj(r)))[1:4]
    return dq_from_r_p(r, p)


def dq_unit_residual(x):
    """Constraint residual c0 = || x x* - 1 ||  (first member of family (3.8))."""
    return float(np.linalg.norm(dq_mul(x, dq_conj(x)) - dq_identity()))


def dq_log2_vec6(x):
    """
    Screw coordinates of a unit pose DQ:  vec6(2 ln x) = [phi*n; d*n + phi*m].

    Used by the faithful [Ch20] baseline (control/control_law.py::
    dq_chandra2020_law), whose pose feedback is -K_P * vec6(2 ln x_tilde)
    under the project's right-invariant error convention.

    With x = r + eps*q_d, q_d = (p r)/2 (formula (2.1)) and the screw form
    x = cos(phi_hat/2) + n_hat sin(phi_hat/2), phi_hat = phi + eps*d,
    n_hat = n + eps*m (n unit axis, m moment, d pitch), one obtains
        q_d = -(d/2) sin(phi/2) + sin(phi/2) m + (d/2) cos(phi/2) n
    hence the extraction below (d = n^T p follows because the transverse
    part of a screw displacement is perpendicular to the axis).
    Near identity vec6(2 ln x) -> [2 Im(r); p] = [-2 O; T] with the (O, T)
    pose error convention of the project (correction O(phi |p|)).  The
    derivative of the log map is singular at phi -> pi: the genuine
    large-error weakness of the [Ch20] pose feedback (paper Sec. 6.4,
    E4 differentiator).
    """
    r = np.asarray(x[:4], dtype=float)
    qd = np.asarray(x[4:], dtype=float)
    eta, mu = r[0], r[1:]
    s = float(np.linalg.norm(mu))
    p = (2.0 * q_mul(qd, q_conj(r)))[1:4]      # translation, formula (2.1)
    if s < 1e-10:                              # small angle: phi*n -> 2*mu
        return np.r_[2.0 * mu, p]
    phi = 2.0 * np.arctan2(s, eta)
    n = mu / s
    d = float(n @ p)                           # screw pitch
    m = (qd[1:] - 0.5 * d * eta * n) / s       # screw moment
    return np.r_[phi * n, d * n + phi * m]


# ---------------------------------------------------------------------------
# Elementary pose factors (used to build DH joint factors, Appendix B.1)
# ---------------------------------------------------------------------------

def dq_rot_z(theta):
    """Rotation about local z: r = [cos(t/2), 0, 0, sin(t/2)]."""
    return np.array([np.cos(theta / 2.0), 0, 0, np.sin(theta / 2.0),
                     0, 0, 0, 0], dtype=float)


def dq_rot_x(alpha):
    """Rotation about local x."""
    return np.array([np.cos(alpha / 2.0), np.sin(alpha / 2.0), 0, 0,
                     0, 0, 0, 0], dtype=float)


def dq_trans_z(d):
    """Translation d along local z: x = 1 + eps (1/2) d k."""
    return np.array([1, 0, 0, 0, 0, 0, 0, d / 2.0], dtype=float)


def dq_trans_x(a):
    """Translation a along local x: x = 1 + eps (1/2) a i."""
    return np.array([1, 0, 0, 0, 0, a / 2.0, 0, 0], dtype=float)
