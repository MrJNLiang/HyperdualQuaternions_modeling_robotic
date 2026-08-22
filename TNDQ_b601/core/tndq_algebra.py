"""
TNDQ / HDQ algebra layer -- paper Section 3 (and Sec. 2.3).

Truncation tower (paper Table 1):

    DQ    : 1 channel   a0                                (sigma^0)
    HDQ   : 2 channels  a0 + eps* a1                      (sigma^0, sigma^1)
    TNDQ  : 3 channels  a0 + sigma a1 + (1/2) sigma^2 a2  (full structure)

Storage convention
------------------
A TNDQ is a 3x8 numpy array `ch` such that the algebra element is

    a_bar = ch[0] + sigma * ch[1] + (1/2) sigma^2 * ch[2],     sigma^3 = 0.

For the TNDQ representation of a pose curve (formula (3.3a)) this means
    ch[0] = x_hat,  ch[1] = d/dt x_hat,  ch[2] = d^2/dt^2 x_hat,
i.e. channel 2 stores the *raw* second derivative (the 1/2 lives in the
basis element (1/2) sigma^2, exactly as in Definition 1 / formula (3.1)).

An HDQ is a 2x8 numpy array with element  a_breve = ch[0] + eps* ch[1]
(paper Sec. 2.3, algebra H_hat[eps*]/(eps*^2)).
"""

import numpy as np

from core.dq_algebra import (
    dq_mul, dq_conj, dq_identity, dq_zero,
    dq_vec6, dq_pure_part, dq_scalar_parts,
    dq_twist, dq_pose_normalize, dq_unit_residual,
)


# ---------------------------------------------------------------------------
# TNDQ:  A_2 = H_hat[sigma]/(sigma^3)          (Definition 1, formula (3.1))
# ---------------------------------------------------------------------------

class TNDQ:
    """
    Trident Number Dual Quaternion:
        a_bar = a0 + sigma a1 + (1/2) sigma^2 a2,  a_k in H_hat, sigma^3 = 0.
    Three DQ channels: pose / 1st derivative / 2nd derivative.
    """

    __slots__ = ("ch",)

    def __init__(self, a0, a1=None, a2=None):
        self.ch = np.zeros((3, 8), dtype=float)
        self.ch[0] = np.asarray(a0, dtype=float).reshape(8)
        if a1 is not None:
            self.ch[1] = np.asarray(a1, dtype=float).reshape(8)
        if a2 is not None:
            self.ch[2] = np.asarray(a2, dtype=float).reshape(8)

    # -- algebra ------------------------------------------------------------

    def __mul__(self, other):
        """
        Implementation of formula (3.2), the TNDQ product:

            a_bar b_bar = a0 b0
                        + sigma (a0 b1 + a1 b0)
                        + (1/2) sigma^2 (a0 b2 + 2 a1 b1 + a2 b0)

        With the storage convention (channel 2 = raw coefficient of
        (1/2) sigma^2) the product channels read:
            c0 = a0 b0
            c1 = a0 b1 + a1 b0
            c2 = a0 b2 + 2 a1 b1 + a2 b0
        The sigma^1 channel is the Leibniz rule, the sigma^2 channel is the
        second-order Leibniz rule -- this is Proposition 1's algebraic engine.
        """
        a0, a1, a2 = self.ch
        b0, b1, b2 = other.ch
        c0 = dq_mul(a0, b0)
        c1 = dq_mul(a0, b1) + dq_mul(a1, b0)
        c2 = dq_mul(a0, b2) + 2.0 * dq_mul(a1, b1) + dq_mul(a2, b0)
        return TNDQ(c0, c1, c2)

    def conj(self):
        """Channel-wise DQ conjugate (conjugation commutes with d/dt)."""
        return TNDQ(dq_conj(self.ch[0]), dq_conj(self.ch[1]), dq_conj(self.ch[2]))

    # -- truncation tower (paper Table 1, Proposition 2) ---------------------

    def to_hdq(self):
        """
        HDQ truncation, formula (3.6):
            a_bar|_HDQ = a0 + eps* a1
        Keep the first two channels only; lossless for all subsequent
        two-channel operations by Proposition 2 (formula (3.7)).
        """
        return HDQ(self.ch[0], self.ch[1])

    def to_dq(self):
        """DQ truncation: sigma^0 channel only (paper Table 1)."""
        return np.array(self.ch[0], dtype=float)

    # -- convenience ---------------------------------------------------------

    @staticmethod
    def identity():
        return TNDQ(dq_identity(), dq_zero(), dq_zero())

    @staticmethod
    def from_constant(x):
        """Constant (time-independent) pose: derivative channels vanish."""
        return TNDQ(x, dq_zero(), dq_zero())

    @staticmethod
    def from_pose_derivatives(x, x_dot, x_ddot):
        """
        TNDQ representation of a pose curve, formula (3.3a):
            x_bar = x_hat + sigma x_hat_dot + (1/2) sigma^2 x_hat_ddot
        """
        return TNDQ(x, x_dot, x_ddot)


# ---------------------------------------------------------------------------
# HDQ:  H_hat[eps*]/(eps*^2)                     (paper Sec. 2.3)
# ---------------------------------------------------------------------------

class HDQ:
    """
    Hyper Dual Quaternion:  a_breve = a0 + eps* a1,  eps*^2 = 0.
    Curve representation (Sec. 2.3): x_breve = x_hat + eps* x_hat_dot.
    """

    __slots__ = ("ch",)

    def __init__(self, a0, a1=None):
        self.ch = np.zeros((2, 8), dtype=float)
        self.ch[0] = np.asarray(a0, dtype=float).reshape(8)
        if a1 is not None:
            self.ch[1] = np.asarray(a1, dtype=float).reshape(8)

    def __mul__(self, other):
        """
        Implementation of formula (2.3), the HDQ product:
            (a0 + eps* a1)(b0 + eps* b1) = a0 b0 + eps* (a0 b1 + a1 b0)
        The eps* channel is exactly the Leibniz rule ([P1] eq.(14)(25)).
        """
        a0, a1 = self.ch
        b0, b1 = other.ch
        return HDQ(dq_mul(a0, b0), dq_mul(a0, b1) + dq_mul(a1, b0))

    def conj(self):
        """Channel-wise DQ conjugate; used for (x_breve_d)* in Theorem 1."""
        return HDQ(dq_conj(self.ch[0]), dq_conj(self.ch[1]))

    def to_dq(self):
        """DQ truncation: eps*^0 channel (paper Table 1)."""
        return np.array(self.ch[0], dtype=float)

    @staticmethod
    def identity():
        return HDQ(dq_identity(), dq_zero())


# ---------------------------------------------------------------------------
# Derived kinematic quantities  (formula (3.5))
# ---------------------------------------------------------------------------

def twist_from_tndq(x_bar):
    """
    Implementation of formula (3.5), first item:
        xi = 2 x_hat_dot x_hat*        (pure DQ, Appendix A.1)
    Returns the pure DQ (8-array).
    """
    return dq_twist(x_bar.ch[0], x_bar.ch[1])


def twist_dot_from_tndq(x_bar):
    """
    Implementation of formula (3.5), second item (derivation Appendix B.2):
        xi_dot = 2 x_hat_ddot x_hat* - (1/2) xi^2      (take pure part)
    and at vec6 level:  vec6(xi_dot) = Jdot qdot + J qddot.
    """
    x, x_dot, x_ddot = x_bar.ch
    xi = dq_twist(x, x_dot)
    xi_dot = 2.0 * dq_mul(x_ddot, dq_conj(x)) - 0.5 * dq_mul(xi, xi)
    return dq_pure_part(xi_dot)


def vec6_twists_from_tndq(x_bar):
    """Convenience: return (vec6 xi, vec6 xi_dot) from a chain output."""
    return dq_vec6(twist_from_tndq(x_bar)), dq_vec6(twist_dot_from_tndq(x_bar))


# ---------------------------------------------------------------------------
# Unitarity constraint family  (formula (3.8), paper Sec. 3.4)
# ---------------------------------------------------------------------------

def unit_constraint_residuals(x_bar):
    """
    Constraint residuals of the lifted unit condition, formula (3.8):

        c0 = || x x* - 1 ||
        c1 = || Sc(2 x_dot x*) ||
        c2 = || Sc(2 x_ddot x*) - (1/2) Sc(xi^2) ||

    Sc(.) takes the scalar and dual-scalar parts (Sec. 3.4).
    c2 sign: by formula (3.5), xi_dot = 2 x_ddot x* - (1/2) xi^2 is pure
    along a true curve, hence c2 = ||Sc(xi_dot)|| vanishes analytically;
    equivalently c2 = ||Sc(2 x_ddot x*) + (1/2) Sc(xi xi*)|| since
    xi xi* = -xi^2 for pure xi (matches the corrected (3.8) remark).
    Numeric drift monitor for the integrator.
    """
    x, x_dot, x_ddot = x_bar.ch

    c0 = dq_unit_residual(x)

    xi = dq_twist(x, x_dot)
    c1 = float(np.linalg.norm(dq_scalar_parts(xi)))

    xi_sq = dq_mul(xi, xi)
    acc_term = 2.0 * dq_mul(x_ddot, dq_conj(x))
    c2 = float(np.linalg.norm(dq_scalar_parts(acc_term) - 0.5 * dq_scalar_parts(xi_sq)))

    return c0, c1, c2


def reproject_tndq(x_bar):
    """
    Re-projection of paper Sec. 3.4 (triggered when residuals exceed a
    threshold):
        order 0: normalize the pose channel;
        order 1: rebuild x_dot from the projected pure twist,
                 x_dot <- (1/2) xi_proj x   (left kinematics, Sec. 2.2);
        order 2: rebuild x_ddot consistently from formula (3.5) inverted:
                 x_ddot = (1/2)(xi_dot + (1/2) xi^2) x  with pure xi_dot.
    """
    x, x_dot, x_ddot = x_bar.ch

    x_new = dq_pose_normalize(x)

    # order-1 projection: keep only the pure part of the measured twist
    xi_proj = dq_pure_part(dq_twist(x_new, x_dot))
    x_dot_new = 0.5 * dq_mul(xi_proj, x_new)

    # order-2 projection: keep only the pure part of the measured twist rate
    xi_dot_proj = dq_pure_part(
        2.0 * dq_mul(x_ddot, dq_conj(x_new)) - 0.5 * dq_mul(xi_proj, xi_proj)
    )
    x_ddot_new = 0.5 * dq_mul(xi_dot_proj + 0.5 * dq_mul(xi_proj, xi_proj), x_new)

    return TNDQ(x_new, x_dot_new, x_ddot_new)
