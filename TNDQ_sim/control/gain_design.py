"""
Gain design and budget alignment for the geometrically consistent
computed-torque law (5.2) -- paper Sec. 5.3 (Theorem 3) + Sec. 6.2 comparison.

Linearised channel models
-------------------------
Near the identity error x_tilde -> 1 the error system of Theorems 1/2

    edot_xi = -K_d e_xi - A^T(x_tilde) K_p e_z + d,     edot_z = A(x_tilde) e_xi

with A -> A0 = diag(-1/2 I3, I3) decouples into two SISO second-order
channels (K_d = diag(K_omega I3, K_v I3), K_p = diag(p_O I3, p_T I3)):

    rotation    :  Oddot + K_omega Odot + (p_O/4) O = -d_omega/2
    translation :  Tddot + K_v     Tdot +  p_T    T = +d_v

Note the factor 1/4 in the rotation stiffness: it comes from
Odot = -1/2 omega_tilde (row 1 of A0) *and* the 1/2 of A0^T acting on the
pose feedback.  A single scalar gain k_p (p_O = p_T = k_p) therefore gives
the rotation channel one quarter of the translational stiffness and a much
lower natural frequency -- the structural reason why the shipped set
(K_d = 8I, k_p = 16) is critically damped in translation but has a slow
dominant pole in rotation.

A symmetric positive definite K_p keeps Theorem 3 exact: with the storage
function V = 1/2 ||e_xi||^2 + 1/2 e_z^T K_p e_z the cross terms cancel
identically, since the feedback is -A^T K_p e_z (K_p inside):

    Vdot = e_xi^T(-K_d e_xi - A^T K_p e_z + d) + e_z^T K_p A e_xi
         = -e_xi^T K_d e_xi + e_xi^T d          (any symmetric K_p)

so (5.6a)/(5.6b), the certified L2 gain 1/lambda_min(K_d) and the mean-square
limit bound (5.7) all carry over verbatim; the scalar case is recovered for
K_p = k_p I.  (Note: (5.7) is an RMS bound over a steady-state window, not a
pointwise ISS ultimate ball -- see the paper's Theorem 3(d) correction note.)

C3 baseline equivalent
----------------------
The first-order DQ H-infinity law bridged by the inner velocity servo
(control_law.velocity_to_accel_ref) linearises to the *same* channel form,

    rotation    :  Oddot + (kO/2 + K_servo) Odot + (kO K_servo/2) O = -d_omega/2
    translation :  Tddot + (kT + K_servo)    Tdot + (kT K_servo)   T = +d_v

i.e. poles {-kO/2, -K_servo} and {-kT, -K_servo}.  Matching *dominant poles*
alone (the alignment used when the baseline was ported) does not equalise the
static disturbance rejection, because the cascade contributes a second fast
pole: at kO=8, kT=4, K_servo=20 both C3 channels have DC stiffness 80 while
the shipped C1 set has 16 (translation) and 4 (rotation).  For quasi-static
tasks such as S3 the DC stiffness -- not the dominant pole -- sets the error,
which is what `design_matching_c3` equalises exactly (identical d -> e
transfer functions in both channels).
"""

import numpy as np

SQRT2 = np.sqrt(2.0)


# ---------------------------------------------------------------------------
# Channel analysis
# ---------------------------------------------------------------------------

def channel_metrics(a1, a0, d_coeff, dt=None):
    """
    Metrics of one channel  edd + a1 ed + a0 e = d_coeff * d.

    Returns poles, damping ratio zeta, natural frequency wn, 2% settling
    time (4/|Re(dominant)|), static error gain |e_ss/d| = |d_coeff|/a0 and
    the discrete margin max|pole|*dt (explicit-integration sanity check).
    """
    poles = np.roots([1.0, a1, a0])
    wn = np.sqrt(a0)
    zeta = a1 / (2.0 * wn)
    dom = np.max(np.real(poles))              # closest to the imaginary axis
    metrics = dict(
        a1=a1, a0=a0, poles=poles, wn=wn, zeta=zeta,
        dominant_pole=dom, t_settle=4.0 / abs(dom),
        static_gain=abs(d_coeff) / a0,
    )
    if dt is not None:
        metrics["pole_dt"] = float(np.max(np.abs(poles))) * dt
    return metrics


def c1_channels(K_d, K_p, dt=None):
    """Linearised channels of law (5.2) for gains (K_d, K_p).

    K_d: 6x6 (block diagonal) or scalar; K_p: 6x6, length-2/6 vector or scalar.
    """
    K_omega, K_v = _dq_blocks(K_d)
    p_O, p_T = _dq_blocks(K_p)
    return {
        "rotation": channel_metrics(K_omega, p_O / 4.0, -0.5, dt),
        "translation": channel_metrics(K_v, p_T, 1.0, dt),
    }


def c3_channels(gamma_O, gamma_T, k_servo, dt=None):
    """Linearised channels of the C3 baseline + inner velocity servo."""
    kO = SQRT2 / gamma_O
    kT = SQRT2 / gamma_T
    return {
        "rotation": channel_metrics(kO / 2.0 + k_servo, kO * k_servo / 2.0,
                                    -0.5, dt),
        "translation": channel_metrics(kT + k_servo, kT * k_servo, 1.0, dt),
    }


def _dq_blocks(G):
    """Rotation/translation scalar of a scalar / 6-vector / 6x6 block gain."""
    G = np.asarray(G, dtype=float)
    if G.ndim == 0:
        return float(G), float(G)
    if G.ndim == 1:
        if G.size == 2:
            return float(G[0]), float(G[1])
        return float(np.mean(G[:3])), float(np.mean(G[3:]))
    ev = np.linalg.eigvalsh(G)
    if not np.allclose(G, np.diag(np.diag(G)), atol=1e-12):
        # non-diagonal: report the conservative (smallest) eigenvalue twice
        return float(ev.min()), float(ev.min())
    return float(np.mean(np.diag(G)[:3])), float(np.mean(np.diag(G)[3:]))


# ---------------------------------------------------------------------------
# Design rules
# ---------------------------------------------------------------------------

def design_from_poles(dominant, ratio=5.0):
    """
    Gains realising the channel polynomial (s+p1)(s+p2) in *both* channels,
    p1 = |dominant|, p2 = ratio*p1:

        K_omega = K_v = p1 + p2,     p_T = p1 p2,     p_O = 4 p1 p2

    ratio = 1 gives the critically damped design, ratio = 5 reproduces the
    C3 cascade pole pattern {-p1, -5p1}.
    """
    p1 = abs(float(dominant))
    p2 = ratio * p1
    a1, a0 = p1 + p2, p1 * p2
    return dict(K_omega=a1, K_v=a1, p_O=4.0 * a0, p_T=a0)


def design_matching_c3(gamma_O, gamma_T, k_servo):
    """
    Gains whose linearised channels coincide *identically* (poles, damping
    and d -> e static gain) with the C3 baseline -- the same-budget operating
    point of the comparison plan Sec. 5.2 for a quasi-static task.
    """
    ch = c3_channels(gamma_O, gamma_T, k_servo)
    return dict(
        K_omega=ch["rotation"]["a1"], p_O=4.0 * ch["rotation"]["a0"],
        K_v=ch["translation"]["a1"], p_T=ch["translation"]["a0"],
    )


def gains_to_matrices(g):
    """dict(K_omega, K_v, p_O, p_T) -> (K_d 6x6, K_p 6x6)."""
    K_d = np.diag(np.r_[np.full(3, g["K_omega"]), np.full(3, g["K_v"])])
    K_p = np.diag(np.r_[np.full(3, g["p_O"]), np.full(3, g["p_T"])])
    return K_d, K_p


# ---------------------------------------------------------------------------
# Feasibility screening (constraints of the design problem)
# ---------------------------------------------------------------------------

def screen(g, dt, kappa, gamma_a, qddot_max, e_xi_ref, e_z_ref,
           pole_dt_max=0.15, zeta_min=1.0):
    """
    Constrained evaluation of a candidate gain set.

    Constraints (all must hold):
      C-cert : lambda_min(K_d) >= 1/2(1/kappa + 1/gamma_a^2)     -- (5.6a)
      C-disc : max|pole| dt <= pole_dt_max                        -- 200 Hz loop
      C-damp : zeta >= zeta_min in both channels (no overshoot in contact)
      C-eff  : peak command proxy  lambda_max(K_d)|e_xi| + 1/2 lambda_max(K_p)|e_z|
               <= qddot_max                                      -- QDDOT_MAX budget

    The reference errors (e_xi_ref, e_z_ref) are measured transient magnitudes
    (S3 attach phase), so C-eff is a data-grounded saturation budget.
    """
    K_d, K_p = gains_to_matrices(g)
    ch = c1_channels(K_d, K_p, dt=dt)
    lam_min = float(np.min(np.diag(K_d)))
    lam_max = float(np.max(np.diag(K_d)))
    level = 0.5 * (1.0 / kappa + 1.0 / gamma_a ** 2)
    u_peak = lam_max * e_xi_ref + 0.5 * float(np.max(np.diag(K_p))) * e_z_ref

    ok = {
        "cert": lam_min >= level,
        "disc": max(ch["rotation"]["pole_dt"], ch["translation"]["pole_dt"])
        <= pole_dt_max,
        "damp": min(ch["rotation"]["zeta"], ch["translation"]["zeta"]) >= zeta_min,
        "eff": u_peak <= qddot_max,
    }
    return dict(
        gains=g, channels=ch, feasible=all(ok.values()), checks=ok,
        lam_min=lam_min, cert_level=level, l2_certified=1.0 / lam_min,
        u_peak=u_peak,
        static_O=ch["rotation"]["static_gain"],
        static_T=ch["translation"]["static_gain"],
        t_settle=max(ch["rotation"]["t_settle"], ch["translation"]["t_settle"]),
        pole_dt=max(ch["rotation"]["pole_dt"], ch["translation"]["pole_dt"]),
    )


def cost(entry, ref, w_ss=1.0, w_dyn=0.5, w_eff=0.25):
    """
    Transparent scalar index, all terms normalised by a reference design
    (the C3 equivalent), so J = 1 means "on par with the baseline budget":

        J = w_ss (G_O/G_O_ref + G_T/G_T_ref)/2
          + w_dyn (t_settle/t_settle_ref)
          + w_eff (u_peak/qddot_max)

    Infeasible candidates are reported with J = inf.
    """
    if not entry["feasible"]:
        return float("inf")
    j_ss = 0.5 * (entry["static_O"] / ref["static_O"]
                  + entry["static_T"] / ref["static_T"])
    return (w_ss * j_ss
            + w_dyn * entry["t_settle"] / ref["t_settle"]
            + w_eff * entry["u_peak"] / ref["u_peak"])


# ---------------------------------------------------------------------------
# Sensitivity
# ---------------------------------------------------------------------------

def sensitivity(g, keys=("K_omega", "K_v", "p_O", "p_T"), factors=(0.5, 2.0),
                **screen_kw):
    """
    One-at-a-time sensitivity: scale each gain by the given factors and report
    the resulting channel metrics.  Returns a list of (key, factor, entry).
    """
    out = []
    for k in keys:
        for f in factors:
            gg = dict(g)
            gg[k] = g[k] * f
            out.append((k, f, screen(gg, **screen_kw)))
    return out


if __name__ == "__main__":   # python3 -m control.gain_design
    from config import params

    dt = params.COPPELIA_DT_TARGET
    print("C1 shipped set (K_d = 8I, k_p = 16):")
    for name, m in c1_channels(params.K_D, params.K_P, dt=dt).items():
        print(f"  {name:11s} poles={np.round(m['poles'], 3)} zeta={m['zeta']:.3f} "
              f"DCgain={m['static_gain']:.4g} ts={m['t_settle']:.2f}s")
    print("C3 baseline equivalent (kO=8, kT=4, K_servo=20):")
    for name, m in c3_channels(params.DQH_GAMMA_O, params.DQH_GAMMA_T,
                               params.DQH_K_SERVO, dt=dt).items():
        print(f"  {name:11s} poles={np.round(m['poles'], 3)} zeta={m['zeta']:.3f} "
              f"DCgain={m['static_gain']:.4g} ts={m['t_settle']:.2f}s")
    print("C3-matched C1 design:")
    g = design_matching_c3(params.DQH_GAMMA_O, params.DQH_GAMMA_T,
                           params.DQH_K_SERVO)
    print("  ", {k: round(v, 3) for k, v in g.items()})
    for name, m in c1_channels(*gains_to_matrices(g), dt=dt).items():
        print(f"  {name:11s} poles={np.round(m['poles'], 3)} zeta={m['zeta']:.3f} "
              f"DCgain={m['static_gain']:.4g} ts={m['t_settle']:.2f}s")
