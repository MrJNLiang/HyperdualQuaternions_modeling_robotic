"""
Geometrically consistent error system -- paper Section 4 (Theorems 1 and 2).

Error pipeline (paper Sec. 6.2, error layer):

    one HDQ product:   x_breve_tilde = x_breve (x_breve_d)*     (Theorem 1, (4.1))
    -> x_tilde, x_tilde_dot                                        (4.2)
    -> geometrically consistent twist error xi_tilde = 2 x_tilde_dot x_tilde*  (4.3)
    -> e_z = [O; T]  (0-th channel, identical to [P2])
    -> output error kinematics  e_z_dot = A(x_tilde) e_xi        (Theorem 2, (4.5))

The error system lives on the HDQ truncation; Proposition 2 guarantees this
is lossless w.r.t. the TNDQ chain (Sec. 4.2).
"""

import numpy as np

from core.dq_algebra import (
    dq_mul, dq_conj, dq_translation, dq_vec6, dq_pure_part, skew,
)
from core.tndq_algebra import HDQ


def hdq_error(x_breve, x_breve_d):
    """
    Implementation of Theorem 1, formula (4.1):

        x_breve_tilde = x_breve (x_breve_d)*

    One HDQ multiplication (3 DQ products) simultaneously yields, per (4.2):
        channel 0: x_tilde     = x xd*                (right-invariant pose error, [P2])
        channel 1: x_tilde_dot = x_dot xd* + x xd_dot*  (its exact time derivative)

    Unwinding handling: xi_tilde is invariant under x_tilde -> -x_tilde
    (Theorem 1(i)), so we flip the sign of the whole HDQ element when the
    scalar part eta_tilde < 0 to stay in the working domain eta_tilde > 0.
    """
    x_breve_tilde = x_breve * x_breve_d.conj()

    # sign flip (both channels together): allowed by Theorem 1(i)
    if x_breve_tilde.ch[0][0] < 0.0:
        x_breve_tilde = HDQ(-x_breve_tilde.ch[0], -x_breve_tilde.ch[1])

    return x_breve_tilde


def twist_error_from_hdq(x_breve_tilde):
    """
    Implementation of formula (4.3):

        xi_tilde = 2 x_tilde_dot x_tilde*    (pure DQ, Theorem 1(i))
        e_xi     = vec6(xi_tilde)

    In the disturbance-free case this equals xi - Ad_{x_tilde} xi_d
    (formula (4.4)): actual twist minus desired twist *transported to the
    current pose* -- removing the spurious term of Sec. 4.1.
    """
    x_tilde, x_tilde_dot = x_breve_tilde.ch
    xi_tilde = 2.0 * dq_mul(x_tilde_dot, dq_conj(x_tilde))
    # purity holds analytically (Appendix A.1); project to guard numerics
    xi_tilde = dq_pure_part(xi_tilde)
    return dq_vec6(xi_tilde), xi_tilde


def output_error(x_tilde):
    """
    6D output error of [P2], used unchanged in Sec. 4.4:

        e_z = [O; T],   O = -Im(r_tilde),   T = p_tilde.
    """
    r_tilde = x_tilde[:4]
    O = -r_tilde[1:4]
    T = dq_translation(x_tilde)
    return np.r_[O, T], O, T


def A_matrix(x_tilde):
    """
    Implementation of Theorem 2, formula (4.5):

        A(x_tilde) = [ -1/2 (eta_tilde I3 + [O]_x)    0
                       -[T]_x                          I3 ]

    so that e_z_dot = A(x_tilde) e_xi.  As x_tilde -> 1,
    A -> A0 = diag(-1/2 I3, I3) with sigma_min(A0) = 1/2.
    """
    e_z, O, T = output_error(x_tilde)
    eta_tilde = x_tilde[0]

    A = np.zeros((6, 6))
    A[:3, :3] = -0.5 * (eta_tilde * np.eye(3) + skew(O))
    A[3:, :3] = -skew(T)
    A[3:, 3:] = np.eye(3)
    return A


def full_error_state(x_breve, x_breve_d):
    """
    Complete error layer in one call (paper Sec. 6.2 pipeline):

    Returns dict with
        x_breve_tilde : HDQ error element (4.1)
        x_tilde       : pose error DQ  (0-th channel, [P2] compatible)
        e_xi          : vec6 geometric twist error (4.3)
        e_z, O, T     : output error (Sec. 4.4)
        A             : A(x_tilde) of Theorem 2 (4.5)
    """
    x_breve_tilde = hdq_error(x_breve, x_breve_d)
    x_tilde = x_breve_tilde.to_dq()
    e_xi, xi_tilde = twist_error_from_hdq(x_breve_tilde)
    e_z, O, T = output_error(x_tilde)
    A = A_matrix(x_tilde)
    return {
        "x_breve_tilde": x_breve_tilde,
        "x_tilde": x_tilde,
        "xi_tilde": xi_tilde,
        "e_xi": e_xi,
        "e_z": e_z,
        "O": O,
        "T": T,
        "A": A,
    }
