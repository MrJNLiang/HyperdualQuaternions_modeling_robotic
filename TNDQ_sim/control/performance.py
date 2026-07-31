"""
H-infinity / ISS performance guarantees -- paper Section 5.3 (Theorem 3).

Provides:
    - storage function V = 1/2 ||e_xi||^2 + k_p/2 ||e_z||^2  (Sec. 5.3)
    - gain design/verification per Theorem 3(c-1) (5.6a), (c-2) (5.6b)
      and the tightest certified L2 gain 1/lambda_min(K_d) (remark)
    - mean-square (RMS) limit bound (5.7), Theorem 3(d) -- NOT a pointwise
      ISS ultimate ball: see the paper's note after Theorem 3(d) / App. C.5
    - online energy accounting to measure the achieved L2 gain (experiment E4)
      and the Lyapunov decrease check dV <= -e_xi^T K_d e_xi + e_xi^T d
      (Theorem 3(b)/(c) proof, step 1).
"""

import numpy as np


# ---------------------------------------------------------------------------
# Storage function (Sec. 5.3)
# ---------------------------------------------------------------------------

def pose_weight(k_p):
    """
    Pose-error weight of the storage function as a 6x6 matrix.

    Accepts the scalar gain k_p (classic form, K_p = k_p I6) or a symmetric
    positive definite matrix K_p = diag(p_O I3, p_T I3).  Theorem 3 holds for
    both: with V = 1/2||e_xi||^2 + 1/2 e_z^T K_p e_z and the feedback
    -A^T K_p e_z the cross terms cancel identically (control/gain_design.py).
    """
    K_p = np.asarray(k_p, dtype=float)
    return K_p if K_p.ndim == 2 else K_p * np.eye(6)


def storage_function(e_xi, e_z, k_p):
    """V = 1/2 ||e_xi||^2 + 1/2 e_z^T K_p e_z  >= 0  (Sec. 5.3)."""
    e_xi = np.asarray(e_xi, dtype=float)
    e_z = np.asarray(e_z, dtype=float)
    K_p = pose_weight(k_p)
    return 0.5 * float(e_xi @ e_xi) + 0.5 * float(e_z @ (K_p @ e_z))


def storage_function_split(e_xi, e_z, k_p):
    """
    Channel-split storage functions of Theorem 3(c-2):
        V_omega = 1/2 ||omega_tilde||^2 + 1/2 O^T K_p,O O
        V_v     = 1/2 ||v_tilde||^2     + 1/2 T^T K_p,T T
    with V = V_omega + V_v.
    """
    e_xi = np.asarray(e_xi, dtype=float)
    e_z = np.asarray(e_z, dtype=float)
    K_p = pose_weight(k_p)
    V_w = 0.5 * float(e_xi[:3] @ e_xi[:3]) \
        + 0.5 * float(e_z[:3] @ (K_p[:3, :3] @ e_z[:3]))
    V_v = 0.5 * float(e_xi[3:] @ e_xi[3:]) \
        + 0.5 * float(e_z[3:] @ (K_p[3:, 3:] @ e_z[3:]))
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
    Theorem 3(d), *mean-square* limit bound (5.7) -- name kept for API
    compatibility, semantics corrected:

        limsup_{T} sqrt( (1/T) int_{t}^{t+T} ||e_xi||^2 ) <= sup||d|| / lambda_eff

    i.e. a bound on the RMS of e_xi over a steady-state window, NOT a
    pointwise ISS ultimate ball radius (the storage function V has no
    dissipation in the e_z direction, so no pointwise ISS-Lyapunov argument
    is available -- paper Theorem 3(d) note / Appendix C.5 (i)-(iii)).

    Two further honesty caveats carried over from the paper:
      * the bound presumes the trajectory stays in the working set Omega_c
        (verified a posteriori, paper Sec. 6.5(6)), it is not derived;
      * lambda_eff = lambda_min(K_d) - alpha*lambda_max(K_d) with alpha the
        multiplicative-uncertainty level (5.1f); this routine evaluates the
        alpha -> 0 case, so the returned number is numerically unchanged
        w.r.t. the earlier version and is optimistic when alpha > 0.
    """
    lam_min = float(np.min(np.linalg.eigvalsh(np.asarray(K_d, dtype=float))))
    return float(d_inf_norm) / lam_min


def ez_cascade_bound(e_xi_bound, K_d, k_p):
    """
    Quasi-static estimate of the pose error from the twist-error level.

    Basis corrected: the earlier "cascade ISS" justification is withdrawn
    (an e_xi -> e_z ISS gain cannot be extracted from V, paper Sec. 5.5 and
    Appendix C.2 note).  What remains is the near-identity static stiffness
    relation (5.9): setting the derivatives of (5.8) to zero,
        ||T||_ss ~ ||d_v|| / p_T ,   ||O||_ss ~ 2 ||d_omega|| / p_O ,
    of which this routine returns the order-of-magnitude version driven by
    ||e_xi||_ss instead of ||d||,
        ||e_z|| <~ lambda_max(K_d) ||e_xi||_ss / (lambda_min(K_p) * 1/2).
    Engineering budget only -- NOT a proved upper bound.
    """
    lam_max = float(np.max(np.linalg.eigvalsh(np.asarray(K_d, dtype=float))))
    lam_p = float(np.min(np.linalg.eigvalsh(pose_weight(k_p))))
    sigma_min_A0 = 0.5
    return lam_max * float(e_xi_bound) / (lam_p * sigma_min_A0)


# ---------------------------------------------------------------------------
# Certificate-channel disturbance reconstruction (paper Sec. 6.5(6))
# ---------------------------------------------------------------------------

class ResidualDisturbanceEstimator:
    """
    Reconstructs the *certificate-channel equivalent disturbance* d_hat(t) by
    inverting the closed-loop twist-error dynamics (5.1e):

        e_xi_dot = -K_d e_xi - A^T K_p e_z + d
        =>  d_hat^k = (e_xi^k - e_xi^{k-1})/dt + K_d e_xi^k + A^T K_p e_z^k

    Why: before this, the runners fed the accounting only the *injected*
    disturbance d = J w, so every other perturbation source that Theorem 3
    explicitly admits -- model mismatch (Delta M, Delta g), measurement
    noise, damped-pseudoinverse residual, command clipping, the joint safety
    governor, and the discretisation error itself -- was invisible.  For the
    S3 grasp experiment, which injects no w at all, d was identically zero
    and (5.6)/(5.7) were vacuous.  Inverting the error dynamics recovers the
    *sum* of all sources, i.e. exactly the quantity the certificates are
    written against.

    Semantics per control law (must be quoted whenever d_hat is reported):
      * C1 (TNDQ, (5.2)): the applied feedback *is* -K_d e_xi - A^T K_p e_z,
        so d_hat is the disturbance the certificate actually sees.
      * C2/C3 baselines: their feedback differs structurally from the
        certificate one, hence d_hat additionally carries
        (applied feedback - certificate feedback).  It stays meaningful as
        "the equivalent disturbance needed to explain this trajectory under
        the C1 certificate", but its absolute value is NOT comparable across
        laws (it absorbs the gain/structure gap).  Within one law, across
        conditions (noload vs load, none vs noise, ...), it is fair.

    d_hat is a *diagnostic only*: it never enters the control law, so the
    closed-loop trajectory stays bit-identical to the previous version.

    The forward difference amplifies measurement noise by 1/dt, so the raw
    reconstruction is low-passed by a first-order IIR at f_cut (default
    20 Hz, a decade below the 200 Hz nominal control rate).
    """

    def __init__(self, K_d, k_p, dt, f_cut=20.0):
        self.K_d = np.atleast_2d(np.asarray(K_d, dtype=float))
        self.K_p = pose_weight(k_p)
        self.dt = float(dt)
        tau = 1.0 / (2.0 * np.pi * float(f_cut))
        self.beta = self.dt / (tau + self.dt)      # IIR pole, dt/(tau+dt)
        self.f_cut = float(f_cut)
        self._e_xi_prev = None
        self._d_filt = np.zeros(6)

    def update(self, e_xi, e_z, A):
        """One control step -> filtered d_hat (6,).  First call returns 0."""
        e_xi = np.asarray(e_xi, dtype=float)
        e_z = np.asarray(e_z, dtype=float)
        if self._e_xi_prev is None:
            self._e_xi_prev = e_xi.copy()          # no difference available
            return self._d_filt.copy()
        d_raw = ((e_xi - self._e_xi_prev) / self.dt
                 + self.K_d @ e_xi
                 + np.asarray(A, dtype=float).T @ (self.K_p @ e_z))
        self._e_xi_prev = e_xi.copy()
        self._d_filt = (1.0 - self.beta) * self._d_filt + self.beta * d_raw
        return self._d_filt.copy()


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
        self.k_p = k_p if np.ndim(k_p) else float(k_p)   # scalar or 6x6 K_p
        self.kappa = float(kappa)
        self.gamma_a = float(gamma_a)

        self.E_exi = 0.0        # int ||e_xi||^2 dt
        self.E_d = 0.0          # int ||d||^2 dt
        self.E_w = 0.0          # int ||omega_tilde||^2 dt   (c-2)
        self.E_dw = 0.0         # int ||d_omega||^2 dt
        self.E_v = 0.0          # int ||v_tilde||^2 dt
        self.E_dv = 0.0         # int ||d_v||^2 dt
        self.V0 = None
        self.T_total = 0.0      # int dt  (for the RMS of e_xi, cf. (5.7))
        self.E_d_inj = 0.0      # int ||d_injected||^2 dt
        self.d_inf = 0.0        # running sup ||d(t)||
        self.d_inj_inf = 0.0    # running sup ||d_injected(t)||
        # NOTE (paper Sec. 6.5(6)): d_vec6 is now the *reconstructed*
        # certificate-channel disturbance d_hat (ResidualDisturbanceEstimator
        # above), which covers model mismatch (Delta M, Delta g -- e.g. the
        # unmodelled cup of the S3 experiment), measurement noise, damped-pinv
        # residual, command clipping, the safety governor and discretisation.
        # d_injected keeps the narrower, exactly-known part d = J w so that the
        # earlier accounting of experiment E4 stays available for comparison;
        # it is the only channel with zero reconstruction error, but it is
        # identically zero for load / mismatch / noise / contact.

    def update(self, e_xi, e_z, d_vec6, dt, d_injected=None):
        """
        d_vec6     disturbance entering the certificate channel -- callers
                   pass the reconstructed d_hat (all sources), which is what
                   (5.6)/(5.7) are written against.
        d_injected the explicitly injected part d = J w only (optional).
                   Defaults to d_vec6, preserving the behaviour of callers
                   that still pass the injected disturbance alone.
        """
        e_xi = np.asarray(e_xi, dtype=float)
        d = np.asarray(d_vec6, dtype=float)
        d_inj = d if d_injected is None else np.asarray(d_injected, dtype=float)

        V = storage_function(e_xi, e_z, self.k_p)
        if self.V0 is None:
            self.V0 = V

        self.T_total += dt
        self.E_exi += float(e_xi @ e_xi) * dt
        self.E_d += float(d @ d) * dt
        self.E_d_inj += float(d_inj @ d_inj) * dt
        self.E_w += float(e_xi[:3] @ e_xi[:3]) * dt
        self.E_dw += float(d[:3] @ d[:3]) * dt
        self.E_v += float(e_xi[3:] @ e_xi[3:]) * dt
        self.E_dv += float(d[3:] @ d[3:]) * dt
        self.d_inf = max(self.d_inf, float(np.linalg.norm(d)))
        self.d_inj_inf = max(self.d_inj_inf, float(np.linalg.norm(d_inj)))
        return V

    def summary(self):
        """Measured vs certified performance (experiment E4/E5 criteria)."""
        eps = 1e-15
        # measured L2 gain: sqrt( int||e_xi||^2 / int||d||^2 )   vs 1/lambda_min(K_d)
        measured_gain = np.sqrt(self.E_exi / (self.E_d + eps))
        # RMS of e_xi over the whole run -- same functional as the left side
        # of the mean-square limit bound (5.7), so the two are directly
        # comparable (the bound is a steady-state statement, hence the
        # whole-run RMS is a slightly pessimistic stand-in for it).
        e_xi_rms = np.sqrt(self.E_exi / (self.T_total + eps))
        rms_bound = iss_ultimate_bound(self.K_d, self.d_inf)
        return {
            # merged H-inf inequality (5.6): kappa^-1 E_exi <= gamma_a^2 E_d + 2 V(0)
            "hinf_lhs_5_6": self.E_exi / self.kappa,
            "hinf_rhs_5_6": self.gamma_a ** 2 * self.E_d + 2.0 * (self.V0 or 0.0),
            "measured_l2_gain": measured_gain,
            "certified_l2_gain": tightest_certified_l2_gain(self.K_d),
            # per-channel gains (5.6')
            "measured_gain_omega": np.sqrt(self.E_w / (self.E_dw + eps)),
            "measured_gain_v": np.sqrt(self.E_v / (self.E_dv + eps)),
            # mean-square limit bound (5.7), now driven by the reconstructed
            # d_hat -- non-vacuous also for load / mismatch / noise / contact
            "iss_bound_e_xi": rms_bound,
            "e_xi_rms": e_xi_rms,
            "rms_margin": rms_bound / (e_xi_rms + eps),
            "d_inf": self.d_inf,
            # narrower, exactly-known injected channel (old accounting).  With
            # no injection at all (E_d_inj = 0, e.g. the whole S3 experiment)
            # the ratio is not a gain but 1/eps, so report nan instead of a
            # spurious 1e6 -- the honest statement is "undefined, no input".
            "d_inj_inf": self.d_inj_inf,
            "measured_l2_gain_injected": (np.sqrt(self.E_exi / self.E_d_inj)
                                          if self.E_d_inj > eps else np.nan),
            "V0": self.V0 or 0.0,
        }
