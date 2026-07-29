"""
H-infinity / ISS performance guarantees -- paper Section 5.3 (Theorem 3).

Provides:
    - storage function V = 1/2 ||e_xi||^2 + k_p/2 ||e_z||^2  (Sec. 5.3)
    - gain design/verification per Theorem 3(c-1) (5.6a), (c-2) (5.6b)
      and the tightest certified L2 gain 1/lambda_min(K_d) (remark)
    - ISS ultimate bound (5.7), Theorem 3(d)
    - online energy accounting to measure the achieved L2 gain (experiment E4)
      and the Lyapunov decrease check dV <= -e_xi^T K_d e_xi + e_xi^T d
      (Theorem 3(b)/(c) proof, step 1).
"""

import numpy as np


# ---------------------------------------------------------------------------
# Storage function (Sec. 5.3)
# ---------------------------------------------------------------------------

def storage_function(e_xi, e_z, k_p):
    """V = 1/2 ||e_xi||^2 + k_p/2 ||e_z||^2  >= 0  (Sec. 5.3)."""
    e_xi = np.asarray(e_xi, dtype=float)
    e_z = np.asarray(e_z, dtype=float)
    return 0.5 * float(e_xi @ e_xi) + 0.5 * k_p * float(e_z @ e_z)


def storage_function_split(e_xi, e_z, k_p):
    """
    Channel-split storage functions of Theorem 3(c-2):
        V_omega = 1/2 ||omega_tilde||^2 + k_p/2 ||O||^2
        V_v     = 1/2 ||v_tilde||^2     + k_p/2 ||T||^2
    with V = V_omega + V_v.
    """
    e_xi = np.asarray(e_xi, dtype=float)
    e_z = np.asarray(e_z, dtype=float)
    V_w = 0.5 * float(e_xi[:3] @ e_xi[:3]) + 0.5 * k_p * float(e_z[:3] @ e_z[:3])
    V_v = 0.5 * float(e_xi[3:] @ e_xi[3:]) + 0.5 * k_p * float(e_z[3:] @ e_z[3:])
    return V_w, V_v


# ---------------------------------------------------------------------------
# Gain conditions (Theorem 3(c), formula (5.6a)/(5.6b) and tightest bound)
# ---------------------------------------------------------------------------

def check_hinf_condition_merged(K_d, kappa, gamma_a):
    """
    Theorem 3(c-1), Schur-complement criterion (5.6a):
        K_d >= 1/2 (kappa^-1 + gamma_a^-2) I_6   (as matrix inequality)
    Returns (satisfied, lambda_min(K_d), required scalar level).
    """
    lam_min = float(np.min(np.linalg.eigvalsh(np.asarray(K_d, dtype=float))))
    level = 0.5 * (1.0 / kappa + 1.0 / gamma_a ** 2)
    return lam_min >= level, lam_min, level


def check_hinf_condition_split(K_omega, K_v, kappa_w, gamma_w, kappa_v, gamma_v):
    """
    Theorem 3(c-2), per-channel criterion (5.6b) for block-diagonal
    K_d = diag(K_omega, K_v):
        K_omega >= 1/2 (kappa_w^-1 + gamma_w^-2) I_3
        K_v     >= 1/2 (kappa_v^-1 + gamma_v^-2) I_3
    Dimensionally homogeneous per channel (rotation vs translation).
    """
    lam_w = float(np.min(np.linalg.eigvalsh(np.asarray(K_omega, dtype=float))))
    lam_v = float(np.min(np.linalg.eigvalsh(np.asarray(K_v, dtype=float))))
    lvl_w = 0.5 * (1.0 / kappa_w + 1.0 / gamma_w ** 2)
    lvl_v = 0.5 * (1.0 / kappa_v + 1.0 / gamma_v ** 2)
    return (lam_w >= lvl_w) and (lam_v >= lvl_v), (lam_w, lvl_w), (lam_v, lvl_v)


def tightest_certified_l2_gain(K_d):
    """
    Tightest certifiable L2 gain of the Lyapunov path (Theorem 3 remark,
    Appendix C.3, theta* = sqrt(kappa)/(2 gamma_a)):
        certified L2 gain <= 1 / lambda_min(K_d).
    """
    lam_min = float(np.min(np.linalg.eigvalsh(np.asarray(K_d, dtype=float))))
    return 1.0 / lam_min


def iss_ultimate_bound(K_d, d_inf_norm):
    """
    Theorem 3(d), ISS ultimate ball radius (5.7):
        limsup ||e_xi|| <= ||d_b||_inf / lambda_min(K_d)
    Bias-type uncertainty only sets the steady-state ball radius.
    """
    lam_min = float(np.min(np.linalg.eigvalsh(np.asarray(K_d, dtype=float))))
    return float(d_inf_norm) / lam_min


def ez_cascade_bound(e_xi_bound, K_d, k_p):
    """
    Cascade transfer of the e_xi ball to e_z (Theorem 3(d), proof step 3):
    near x_tilde = 1, sigma_min(A0) = 1/2, and the quasi-steady state of
    dot e_xi = -K_d e_xi - k_p A^T e_z + d gives the ISS-type e_z estimate
        ||e_z|| <~ lambda_max(K_d) ||e_xi||_ss / (k_p sigma_min(A0)).
    Order-of-magnitude budget, not a tight bound (honesty remark (iv)).
    """
    lam_max = float(np.max(np.linalg.eigvalsh(np.asarray(K_d, dtype=float))))
    sigma_min_A0 = 0.5
    return lam_max * float(e_xi_bound) / (k_p * sigma_min_A0)


# ---------------------------------------------------------------------------
# Online performance accounting (experiments E3/E4/E5 of Sec. 6.3)
# ---------------------------------------------------------------------------

class PerformanceAccumulator:
    """
    Accumulates the L2 energies of Theorem 3(c):

        int ||e_xi||^2 dt  vs  gamma_a^2 int ||d||^2 dt + 2 V(0)   (5.6)

    plus per-channel energies of (5.6'), and checks the exact dissipation
    identity  dV = -e_xi^T K_d e_xi + e_xi^T d  (Theorem 3(c) proof step 1)
    against the numerically differentiated V.
    """

    def __init__(self, K_d, k_p, kappa, gamma_a):
        self.K_d = np.asarray(K_d, dtype=float)
        self.k_p = float(k_p)
        self.kappa = float(kappa)
        self.gamma_a = float(gamma_a)

        self.E_exi = 0.0        # int ||e_xi||^2 dt
        self.E_d = 0.0          # int ||d||^2 dt
        self.E_w = 0.0          # int ||omega_tilde||^2 dt   (c-2)
        self.E_dw = 0.0         # int ||d_omega||^2 dt
        self.E_v = 0.0          # int ||v_tilde||^2 dt
        self.E_dv = 0.0         # int ||d_v||^2 dt
        self.V0 = None
        self.d_inf = 0.0        # running sup ||d(t)||  (for ISS budget (5.7))

    def update(self, e_xi, e_z, d_vec6, dt):
        e_xi = np.asarray(e_xi, dtype=float)
        d = np.asarray(d_vec6, dtype=float)

        V = storage_function(e_xi, e_z, self.k_p)
        if self.V0 is None:
            self.V0 = V

        self.E_exi += float(e_xi @ e_xi) * dt
        self.E_d += float(d @ d) * dt
        self.E_w += float(e_xi[:3] @ e_xi[:3]) * dt
        self.E_dw += float(d[:3] @ d[:3]) * dt
        self.E_v += float(e_xi[3:] @ e_xi[3:]) * dt
        self.E_dv += float(d[3:] @ d[3:]) * dt
        self.d_inf = max(self.d_inf, float(np.linalg.norm(d)))
        return V

    def summary(self):
        """Measured vs certified performance (experiment E4/E5 criteria)."""
        eps = 1e-15
        # measured L2 gain: sqrt( int||e_xi||^2 / int||d||^2 )   vs 1/lambda_min(K_d)
        measured_gain = np.sqrt(self.E_exi / (self.E_d + eps))
        return {
            # merged H-inf inequality (5.6): kappa^-1 E_exi <= gamma_a^2 E_d + 2 V(0)
            "hinf_lhs_5_6": self.E_exi / self.kappa,
            "hinf_rhs_5_6": self.gamma_a ** 2 * self.E_d + 2.0 * (self.V0 or 0.0),
            "measured_l2_gain": measured_gain,
            "certified_l2_gain": tightest_certified_l2_gain(self.K_d),
            # per-channel gains (5.6')
            "measured_gain_omega": np.sqrt(self.E_w / (self.E_dw + eps)),
            "measured_gain_v": np.sqrt(self.E_v / (self.E_dv + eps)),
            # ISS budget (5.7)
            "iss_bound_e_xi": iss_ultimate_bound(self.K_d, self.d_inf),
            "d_inf": self.d_inf,
            "V0": self.V0 or 0.0,
        }
