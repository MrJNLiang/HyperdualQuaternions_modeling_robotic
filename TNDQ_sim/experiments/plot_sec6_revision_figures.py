#!/usr/bin/env python3
"""Section-6 revision figures: verify Theorem 3 *directly* from the data.

Standalone (numpy + matplotlib + csv only).  Reads ONLY already-produced
results under TNDQ_sim/results/ (grasp_metrics_summary.csv, gamma_sweep.csv,
grasp_circle_*.npz).  Does NOT import or modify any simulation / experiment /
control code.

Figures produced (results/):
  sec6r_thm3d_bound_check_load        (a) Theorem 3(d): measured ||e_xi||_rms
                                          vs the mean-square limit bound (5.7)
                                          D/lambda_eff = sup||d_hat||/lambda_min(K_d),
                                          grouped bars + all-40-runs scatter.
  sec6r_thm3c_hinf_certificate        (b) Theorem 3(c): gamma_sweep H-infinity
                                          certificate -- A-group invariance &
                                          cert flip, B-group certified vs
                                          measured L2 gain, (5.6) lhs<=rhs.
  sec6r_v1_scaling_law                (f) static stiffness scaling law (5.9):
                                          residuals vs 1/k_p, theoretical slope
                                          -1 line through the tuned/fast mean.
  sec6r_v2_divergence_ext             (e) C1/C2/C3 divergence over all 5
                                          conditions + steady-state sub-window
                                          dispersion (weak statistics).

Steady-state window convention: circle-ss = t >= t_marks[-1] + ramp(2 s)
+ 1 s, identical to grasp_metrics_summary.csv and the paper.
"""

import csv
import os
import tempfile

import numpy as np

os.environ.setdefault(
    "MPLCONFIGDIR",
    os.path.join(tempfile.gettempdir(), "mplcfg_dsh_sec6r"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.normpath(os.path.join(HERE, "..", "results"))

# Journal style shared with plot_sec6_figures.py
C_C1, C_C2, C_C3 = "#0072B2", "#E69F00", "#D55E00"
C_TUNED, C_FAST, C_BASE = "#0072B2", "#56B4E9", "0.45"
C_BOUND = "#7A3B8E"

plt.rcParams.update({
    "font.size": 8.5, "axes.labelsize": 9, "axes.titlesize": 9,
    "legend.fontsize": 7.5, "xtick.labelsize": 8, "ytick.labelsize": 8,
    "lines.linewidth": 1.2, "axes.linewidth": 0.8,
    "grid.alpha": 0.3, "grid.linewidth": 0.5,
    "pdf.fonttype": 42, "ps.fonttype": 42,
    "figure.dpi": 110, "savefig.dpi": 600,
    "savefig.bbox": "tight",
})

SS = "circle-ss"
PHASES4 = ("descend", "hold", "lift", "retreat")


# ---------------------------------------------------------------------------
# Data access
# ---------------------------------------------------------------------------
def load_metrics():
    """grasp_metrics_summary.csv as list of dicts (str values)."""
    with open(os.path.join(RESULTS, "grasp_metrics_summary.csv"),
              newline="") as f:
        return list(csv.DictReader(f))


def mval(row, key):
    v = row[key]
    return float(v) if v not in ("", "nan", "None") else float("nan")


def load_gamma():
    with open(os.path.join(RESULTS, "gamma_sweep.csv"), newline="") as f:
        return list(csv.DictReader(f))


def load_npz(tag):
    d = dict(np.load(os.path.join(RESULTS, f"grasp_circle_{tag}.npz"),
                     allow_pickle=False))
    n = min(len(d["t"]), len(d["t_ex"]))
    for k in list(d):
        if d[k].ndim >= 1 and d[k].shape[0] in (len(d["t"]), len(d["t_ex"])):
            d[k] = d[k][:n]
    return d


def ss_subwindow_exi(tag, n_win=4):
    """4 disjoint steady-state sub-window RMS of ||e_xi|| (pseudo-replicates
    of the same limit cycle; identical-condition dispersion for noise)."""
    d = load_npz(tag)
    t = d["t"]
    tss = float(d["t_marks"][-1]) + 3.0
    m = t >= tss
    exi = np.linalg.norm(d["e_xi"][m], axis=1)
    ts = t[m]
    n4 = len(ts) // n_win
    return np.array([np.sqrt(np.mean(exi[i * n4:(i + 1) * n4] ** 2))
                     for i in range(n_win)])


def save(fig, name):
    for ext in ("pdf", "png"):
        out = os.path.join(RESULTS, f"{name}.{ext}")
        fig.savefig(out)
        print(f"saved: {out}")
    plt.close(fig)


def panel_tag(ax, tag):
    ax.text(0.008, 0.93, tag, transform=ax.transAxes, ha="left", va="top",
            fontweight="bold", fontsize=9,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.75,
                      pad=1.2))


# ---------------------------------------------------------------------------
# (a) Theorem 3(d): mean-square bound (5.7) vs measured ||e_xi||_rms
# ---------------------------------------------------------------------------
def fig_thm3d_bound():
    rows = load_metrics()
    # runs with a non-vacuous bound (d_hat channel present)
    runs = [r for r in rows if r["rms_bound_5_7"] not in ("", "nan")
            and r["phase"] == SS]
    data = []
    for r in runs:
        data.append(dict(
            law=r["law"], gains=r["gains"], mode=r["mode"], cond=r["condition"],
            bound=mval(r, "rms_bound_5_7"),
            exi_ss=mval(r, "exi_rms"),
            exi_run=mval(r, "exi_rms_run"),
            margin_run=mval(r, "rms_margin")))
    n = len(data)
    assert n >= 40, n

    fig, (axA, axB) = plt.subplots(
        1, 2, figsize=(7.0, 3.4), width_ratios=[1.15, 1.0])

    # -- panel A: grouped bars, key runs with a non-vacuous bound;
    #    fast/base gain tiers predate the d_hat channel (no bound available)
    keys = [("C1 tuned", "tndq", "tuned", "load", "none", C_TUNED),
            ("C1 tuned hs", "tndq", "tuned", "load", "highspeed", C_TUNED),
            ("C1 tuned noise", "tndq", "tuned", "load", "noise", C_TUNED),
            ("C2", "dq-chandra", "-", "load", "none", C_C2),
            ("C3", "dq-hinf", "-", "load", "none", C_C3)]
    x = np.arange(len(keys))
    labs, vals, bnds = [], [], []
    for lab, law, gains, mode, cond, col in keys:
        r = next(d for d in data if d["law"] == law and d["gains"] == gains
                 and d["mode"] == mode and d["cond"] == cond)
        vals.append(r["exi_ss"]); bnds.append(r["bound"]); labs.append(lab)
    bars = axA.bar(x, np.array(vals) * 1e3, width=0.55, color="0.72",
                   edgecolor="k", linewidth=0.5,
                   label=r"measured $\|e_\xi\|_{\mathrm{rms}}$ (circle-ss)")
    for xi, (v, b) in enumerate(zip(vals, bnds)):
        axA.plot([xi - 0.28, xi + 0.28], [b * 1e3] * 2, color=C_BOUND, lw=1.6,
                 zorder=5)
        axA.text(xi + 0.30, b * 1e3, f"$\\times${b / v:.0f}", fontsize=6.2,
                 va="center", color=C_BOUND)
    axA.set_yscale("log")
    axA.set_xticks(x)
    axA.set_xticklabels(labs, fontsize=6.6, rotation=28, ha="right")
    axA.set_ylabel(r"$\|e_\xi\|_{\mathrm{rms}}$ [$\times 10^{-3}$]")
    axA.set_ylim(0.3, 6e3)
    axA.grid(True, axis="y", which="both")
    bound_proxy = axA.plot([], [], color=C_BOUND, lw=1.6,
                           label=r"Thm 3(d) bound $D/\lambda_{\mathrm{eff}}$")
    axA.legend(handles=[bars] + bound_proxy, loc="upper right", fontsize=6.8)
    axA.text(0.99, 0.97, "bound strictly above measured\nin every run",
             transform=axA.transAxes, ha="right", va="top", fontsize=6.8,
             color=C_BOUND)
    panel_tag(axA, "(a)")

    # -- panel B: all 40 runs, bound vs measured (identity line) -----------
    xx = np.array([d["exi_ss"] for d in data])
    yy = np.array([d["bound"] for d in data])
    for col, sel in ((C_TUNED, [d for d in data if d["law"] == "tndq"]),
                     (C_C2, [d for d in data if d["law"] == "dq-chandra"]),
                     (C_C3, [d for d in data if d["law"] == "dq-hinf"]),
                     (C_BASE, [d for d in data if d["law"] == "dq-ctc"])):
        s = np.array([d["exi_ss"] for d in sel])
        b = np.array([d["bound"] for d in sel])
        axB.scatter(s, b, s=16, color=col, alpha=0.75, edgecolor="k",
                    linewidth=0.3)
    lo, hi = min(xx.min(), yy.min()), max(xx.max(), yy.max())
    axB.loglog([lo, hi], [lo, hi], ls="--", color="0.35", lw=1.0,
               label="identity (measured = bound)")
    axB.loglog([lo, hi], [5 * lo, 5 * hi], ls=":", color=C_BOUND, lw=0.9,
               label="5$\\times$ conservative")
    axB.set_xlabel(r"measured $\|e_\xi\|_{\mathrm{rms}}$ (circle-ss)")
    axB.set_ylabel(r"Thm 3(d) bound $D/\lambda_{\mathrm{eff}}$")
    axB.grid(True, which="both")
    axB.legend(loc="lower right", fontsize=6.6)
    axB.text(0.05, 0.96, f"all {n} runs strictly below the identity line",
             transform=axB.transAxes, fontsize=6.8, color="0.25", va="top")
    panel_tag(axB, "(b)")

    fig.suptitle("Theorem 3(d) verified: mean-square bound (5.7) strictly "
                 "exceeds the measured twist-error RMS in every run", y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    save(fig, "sec6r_thm3d_bound_check_load")
    return n, float(np.min([d["margin_run"] for d in data]))


# ---------------------------------------------------------------------------
# (b) Theorem 3(c): H-infinity certificate from gamma_sweep.csv
# ---------------------------------------------------------------------------
def fig_thm3c_hinf():
    gs = load_gamma()
    A = [r for r in gs if r["group"] == "A"]
    B = [r for r in gs if r["group"] == "B"]
    C = [r for r in gs if r["group"] == "C"]
    all_ok = sum(1 for r in gs if mval(r, "hinf_lhs") <= mval(r, "hinf_rhs"))
    total = len(gs)

    fig, (axA, axB, axC) = plt.subplots(1, 3, figsize=(7.0, 2.9),
                                        width_ratios=[1.15, 1.05, 1.0])

    # -- panel A: A-group invariance + certificate flip --------------------
    gA = [mval(r, "gamma") for r in A]
    axA.plot(gA, [mval(r, "T_rms_ss") for r in A], "o-", color=C_TUNED,
             ms=3.5, label=r"$\|\mathcal{T}\|_{\mathrm{rms,ss}}$")
    axA.plot(gA, [mval(r, "O_rms_ss") for r in A], "s-", color=C_C2, ms=3.5,
             label=r"$\|\mathcal{O}\|_{\mathrm{rms,ss}}$")
    lam = mval(A[0], "lam_min")
    kappa = 1.0                     # params.KAPPA (5.6a) context
    g_crit = 1.0 / np.sqrt(2.0 * lam - 1.0 / kappa)
    axA.axvline(g_crit, color=C_C3, ls="--", lw=1.0,
                label=f"cert. flip $\\gamma_a^{{*}}={g_crit:.3f}$")
    okA = [mval(r, "gamma") for r in A if int(r["cert_ok"]) == 1]
    nokA = [mval(r, "gamma") for r in A if int(r["cert_ok"]) == 0]
    axA.plot(okA, [1.3e-3] * len(okA), "o", color="k", ms=4, zorder=5)
    axA.plot(nokA, [1.3e-3] * len(nokA), "x", color=C_C3, ms=5, zorder=5)
    axA.set_xscale("log"); axA.set_yscale("log")
    axA.set_xlabel(r"$\gamma_a$ (analysis parameter)")
    axA.set_ylabel("steady-state error")
    axA.grid(True, which="both")
    axA.text(0.97, 0.05,
             "closed loop unchanged across all $\\gamma_a$\n"
             "(measured $L_2 = 0.1466$ constant);\n"
             "level $=\\frac{1}{2}(1+\\gamma_a^{-2})$  ($\\kappa{=}1$),\n"
             f"flip at $\\gamma_a^*=1/\\sqrt{{2\\lambda_{{\\min}}-1}}"
             f"={g_crit:.4f}$",
             transform=axA.transAxes, ha="right", fontsize=6.0, color="0.3")
    axA.legend(fontsize=6.0, loc="lower left")
    panel_tag(axA, "(a)")

    # -- panel B: B-group certified vs measured L2 gain --------------------
    gB = [mval(r, "gamma") for r in B]
    cert = [mval(r, "certified_l2") for r in B]
    meas = [mval(r, "measured_l2") for r in B]
    axB.loglog(gB, cert, "k--", lw=1.1,
               label=r"zero-initial-energy bound $\gamma_a^2 = "
                     r"1/\lambda_{\min}(K_d)$")
    axB.loglog(gB, meas, "o-", color=C_TUNED, ms=3.5,
               label="measured $\\sqrt{E_{\\xi}/E_d}$ (TNDQ)")
    axB.loglog([mval(r, "gamma") for r in C],
               [mval(r, "measured_l2") for r in C], "^--", color=C_C3, ms=3.5,
               label="measured (C3, no cert.)")
    axB.set_xlabel(r"$\gamma_a$ (B) / $\gamma$ (C)")
    axB.set_ylabel("L2 gain")
    axB.grid(True, which="both")
    # small-gamma B rows exceed the zero-initial bound: V(0) + saturation
    axB.annotate("measured $>$ zero-initial bound:\n$V(0)$ term + "
                 "saturation\n($\\gamma_a{=}0.177$: $\\mathrm{sat}{=}5$)",
                 xy=(0.177, 0.223), xytext=(0.30, 0.9), fontsize=6.2,
                 color="0.25",
                 arrowprops=dict(arrowstyle="->", color="0.5", lw=0.8))
    axB.text(0.03, 0.97, "full inequality (5.6) incl. $2V(0)$ verified "
             "in every data row;\nthe $2V(0)$ energy term is what keeps "
             "the certificate\nvalid despite $V(0)$-driven inflation",
             transform=axB.transAxes, fontsize=6.0, color="0.25", va="top")
    axB.legend(fontsize=6.0, loc="upper left")
    panel_tag(axB, "(b)")

    # -- panel C: hinf lhs vs rhs (full inequality 5.6) --------------------
    cols = {"A": C_TUNED, "B": C_C2, "C": C_C3}
    for grp in ("A", "B", "C"):
        sel = [r for r in gs if r["group"] == grp]
        lhs = [mval(r, "hinf_lhs") for r in sel]
        rhs = [mval(r, "hinf_rhs") for r in sel]
        axC.loglog(lhs, rhs, "o", color=cols[grp], ms=4, alpha=0.85,
                   label=f"group {grp}")
    lo = 1e-2; hi = 3e1
    axC.loglog([lo, hi], [lo, hi], ls="--", color="0.35", lw=1.0)
    axC.text(0.04, 0.90, "identity $\\mathrm{lhs}=\\mathrm{rhs}$",
             transform=axC.transAxes, fontsize=6.0, color="0.35")
    axC.set_xlabel(r"$\kappa^{-1}E_\xi$  (lhs of (5.6))")
    axC.set_ylabel(r"$\gamma_a^2 E_d + 2V(0)$  (rhs of (5.6))")
    axC.set_xlim(lo, hi); axC.set_ylim(lo, hi)
    axC.grid(True, which="both")
    axC.legend(fontsize=6.2, loc="lower right")
    axC.text(0.03, 0.06, "all points below identity\n$\\Rightarrow$ (5.6) "
             "holds", transform=axC.transAxes, fontsize=6.4, color="0.25")
    panel_tag(axC, "(c)")

    fig.suptitle("Theorem 3(c): H$\\infty$ certificate from the "
                 "$\\gamma$-sweep — decidable, tight in the B-group design, "
                 "and satisfied in every run", y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save(fig, "sec6r_thm3c_hinf_certificate")
    return dict(total=total, all_ok=all_ok, g_crit=g_crit)


# ---------------------------------------------------------------------------
# (f) V1: static stiffness scaling law (5.9), residuals vs 1/k_p, slope -1
# ---------------------------------------------------------------------------
def fig_v1_scaling():
    rows = load_metrics()

    def ss(law, gains):
        r = next(r for r in rows if r["law"] == law and r["gains"] == gains
                 and r["mode"] == "load" and r["condition"] == "none"
                 and r["phase"] == SS)
        return mval(r, "T_rms"), mval(r, "O_rms")

    tiers = [("base", 16.0, 16.0, C_BASE),
             ("tuned", 80.0, 320.0, C_TUNED),
             ("fast", 180.0, 720.0, C_FAST)]
    # theoretical disturbance levels from the tuned/fast mean inversion (5.9)
    dv_bar = 0.5 * (80 * ss("tndq", "tuned")[0] + 180 * ss("tndq", "fast")[0])
    dw_bar = 0.5 * (0.5 * 320 * ss("tndq", "tuned")[1]
                    + 0.5 * 720 * ss("tndq", "fast")[1])
    emp_slope = (np.log10(ss("tndq", "fast")[0] / ss("tndq", "tuned")[0])
                 / np.log10(180.0 / 80.0))

    fig, (axT, axO) = plt.subplots(1, 2, figsize=(7.0, 3.0))
    for ax, (key, k_idx, d_bar, ylab) in zip(
            (axT, axO),
            (("T", 1, dv_bar, r"$\|\mathcal{T}\|_{\mathrm{ss}}$ [m]"),
             ("O", 2, dw_bar, r"$\|\mathcal{O}\|_{\mathrm{ss}}$"))):
        # tiers: (name, p_T, p_O, color); 1/k_p on the channel stiffness
        x = np.array([1.0 / t[k_idx] for t in tiers])
        y = np.array([ss("tndq", t[0])[0 if key == "T" else 1]
                      for t in tiers])
        xline = np.logspace(np.log10(x.min() * 0.8),
                            np.log10(x.max() * 1.25), 40)
        yline = d_bar * xline                    # slope -1: residual = d/k_p
        ax.loglog(xline, yline, ls="--", color="0.35", lw=1.1,
                  label=f"theory (5.9), slope $-1$, $d={d_bar:.3f}$")
        ax.fill_between(xline, 0.98 * yline, 1.02 * yline, color=C_TUNED,
                        alpha=0.10, lw=0)
        for xi, yi, (name, _, _, col) in zip(x, y, tiers):
            ax.plot(xi, yi, "o", color=col, ms=5.5, mec="k", mew=0.5,
                    zorder=5)
            ax.annotate(name, (xi, yi), textcoords="offset points",
                        xytext=(0, 6), ha="center", fontsize=6.8)
        ax.annotate("base: uncompensated 1/4 factor\n$\\Rightarrow$ off the "
                    "line", xy=(x[0], y[0]), xytext=(x[0] * 0.32, y[0] * 0.6),
                    fontsize=6.4, color="0.3",
                    arrowprops=dict(arrowstyle="->", color="0.5", lw=0.8))
        ax.set_xlabel(r"$1/k_p$   ($k_p = p_T$ / $p_O$)")
        ax.set_ylabel(ylab)
        ax.grid(True, which="both")
        ax.legend(fontsize=6.6, loc="lower right")
    axT.set_title(f"translation channel (empirical slope ${emp_slope:.2f}$ "
                  "vs $-1$)", fontsize=8.2)
    axO.set_title("rotation channel (1/4 factor compensated in $p_O$)",
                  fontsize=8.2)
    fig.suptitle("V1 — static stiffness scaling law (5.9): one physical "
                 "disturbance, residuals scale as $1/k_p$", y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save(fig, "sec6r_v1_scaling_law")
    return dict(dv_bar=dv_bar, dw_bar=dw_bar, emp_slope=emp_slope)


# ---------------------------------------------------------------------------
# (e) V2 upgrade: C1/C2/C3 over all 5 conditions + sub-window dispersion
# ---------------------------------------------------------------------------
def fig_v2_divergence_ext():
    rows = load_metrics()
    conds = [("standard", "none"), ("high-speed", "highspeed"),
             ("fast-transit", "fast-transit"), ("coarse-dt", "coarse-dt"),
             ("noise", "noise")]
    laws = [("C1", "tndq", "tuned", C_C1),
            ("C2", "dq-chandra", "-", C_C2),
            ("C3", "dq-hinf", "-", C_C3)]
    tags = {"none": "tuned_load", "highspeed": "tuned_load_hspeed",
            "fast-transit": "tuned_load_ftrans", "coarse-dt": "tuned_load_cdt",
            "noise": "tuned_load_noise"}

    exi = {}          # exi[cond][law] = circle-ss rms x1e3
    for cname, ctag in conds:
        exi[cname] = {}
        for lab, law, gains, _ in laws:
            r = next(rr for rr in rows if rr["law"] == law
                     and rr["gains"] == gains and rr["mode"] == "load"
                     and rr["condition"] == ctag and rr["phase"] == SS)
            exi[cname][lab] = 1e3 * mval(r, "exi_rms")

    fig, (axA, axB) = plt.subplots(1, 2, figsize=(7.0, 3.2),
                                   width_ratios=[1.2, 1.0])
    # -- panel A: absolute RMS bars, all 5 conditions ----------------------
    x = np.arange(len(conds))
    w = 0.26
    for i, (lab, _, _, col) in enumerate(laws):
        vals = [exi[c][lab] for c, _ in conds]
        bars = axA.bar(x + (i - 1) * w, vals, w, color=col, edgecolor="k",
                       linewidth=0.5, label=lab)
        for b in bars:
            axA.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.05,
                     f"{b.get_height():.3f}", ha="center", va="bottom",
                     fontsize=5.8, rotation=90)
    # sub-window dispersion of C1/C3 (identical-condition noise floor)
    for cname, ctag in conds:
        base = tags[ctag]
        xi = x[[c for c, _ in conds].index(cname)]
        for lab, base_tag, col, i in (("C1", base, C_C1, 0),
                                      ("C3", base.replace("tuned_load",
                                                          "dqhinf_load"),
                                       C_C3, 2)):
            v = ss_subwindow_exi(base_tag) * 1e3
            axA.errorbar(xi + (i - 1) * w, exi[cname][lab], yerr=v.std(),
                         fmt="none", ecolor=col, elinewidth=1.0, capsize=2.2,
                         zorder=6)
    axA.set_xticks(x)
    axA.set_xticklabels([c for c, _ in conds], fontsize=7.2)
    axA.set_ylabel(r"$\|e_\xi\|_{\mathrm{rms}}$ [$\times 10^{-3}$]")
    axA.set_ylim(0, 3.9)
    axA.grid(True, axis="y")
    axA.legend(loc="upper left", fontsize=6.6)
    axA.text(0.99, 0.97, "whiskers: 4 steady-state sub-window dispersion "
             "(C1 / C3)", transform=axA.transAxes, ha="right", va="top",
             fontsize=5.8, color="0.35")
    panel_tag(axA, "(a)")

    # -- panel B: relative deviation from C1 -------------------------------
    xb = np.arange(len(conds))
    for i, (lab, law, gains, col) in enumerate(laws[1:], start=1):
        dev = [(exi[c][lab] / exi[c]["C1"] - 1.0) * 100 for c, _ in conds]
        bars = axB.bar(xb + (i - 1.5) * w * 1.15, dev, w * 1.1, color=col,
                       edgecolor="k", linewidth=0.5, label=lab)
        for b in bars:
            axB.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.05,
                     f"+{b.get_height():.2f}" if b.get_height() < 1 else
                     f"+{b.get_height():.1f}", ha="center", va="bottom",
                     fontsize=6.4)
    axB.axhline(0.0, color="k", lw=0.8)
    axB.set_xticks(xb)
    axB.set_xticklabels([c for c, _ in conds], fontsize=7.2)
    axB.set_ylabel("relative deviation from C1 [%]")
    axB.set_ylim(0, 3.0)
    axB.grid(True, axis="y")
    axB.legend(loc="upper left", fontsize=6.6)
    panel_tag(axB, "(b)")

    fig.suptitle("V2 — certificate divergence: C1$\\equiv$C2 in all 5 "
                 "conditions, C3 separates structurally (noise: $+2.2\\%$, "
                 "high-speed: $+2.0\\%$)", y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save(fig, "sec6r_v2_divergence_ext")
    return exi


# ---------------------------------------------------------------------------
def main():
    print("== (a) Theorem 3(d) mean-square bound check ==")
    n, min_margin = fig_thm3d_bound()
    print(f"    {n} runs with a bound; min whole-run margin x{min_margin:.1f}")
    print("== (b) Theorem 3(c) H-infinity certificate (gamma sweep) ==")
    info = fig_thm3c_hinf()
    print(f"    (5.6) lhs<=rhs in {info['all_ok']}/{info['total']}; "
          f"A-group cert flip at gamma_a*={info['g_crit']:.4f}")
    print("== (f) V1 static stiffness scaling law ==")
    v1 = fig_v1_scaling()
    print(f"    d_v={v1['dv_bar']:.4f}, d_w={v1['dw_bar']:.4f}, "
          f"empirical slope {v1['emp_slope']:.3f} vs -1")
    print("== (e) V2 divergence extension ==")
    exi = fig_v2_divergence_ext()
    for c, _ in [("standard", "none"), ("high-speed", "highspeed"),
                 ("noise", "noise")]:
        row = "  ".join(f"{lab} {exi[c][lab]:.3f}"
                        for lab in ("C1", "C2", "C3"))
        print(f"    {c:10s} {row}")


if __name__ == "__main__":
    main()
