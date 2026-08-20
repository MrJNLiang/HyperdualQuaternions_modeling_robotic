#!/usr/bin/env python3
"""Journal-grade figures for the Section-6 simulation study (load mode only).

Standalone (numpy + matplotlib): reads results/grasp_circle_*_load*.npz and
produces vector PDF + 600-dpi PNG pairs in results/:

  1. sec6_errors_timeseries_load          Task 1: ||T||, ||O||, ||e_xi|| vs t,
                                            C1 (tndq-tuned) / C2 (dq-chandra)
                                            / C3 (dq-hinf) only.
  2. sec6_c1_gain_tiers_load              Task 2: C1 tuned vs fast gain tiers
                                            (time series + steady-state RMS).
  3. sec6_v1_inversion_load               Task 3 (V1): static stiffness scaling
                                            law (5.9) disturbance inversion
                                            consistency across gain tiers.
  4. sec6_v2_certificate_divergence_load  Task 3 (V2): ||e_xi||_rms under
                                            standard / high-speed / noise.
  5. sec6_v3_level_set_margin_load        Task 3 (V3): storage function V(t)
                                            vs Theorem-3(b) level-set c*.

Steady-state window: t >= t_marks[-1] + ramp(2 s) + 1 s = 12.5 s (circle-ss,
same convention as grasp_metrics_summary.csv and Section 6).
"""

import os

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.transforms import blended_transform_factory

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "..", "results")

# ---------------------------------------------------------------------------
# Journal style: Okabe-Ito colorblind-safe palette, vector-friendly fonts
# ---------------------------------------------------------------------------
C_C1, C_C2, C_C3 = "#0072B2", "#E69F00", "#D55E00"   # blue / orange / vermil.
C_TUNED, C_FAST, C_BASE = "#0072B2", "#56B4E9", "0.45"
C_DV, C_DW = "#009E73", "#CC79A7"                    # d_v / d_omega channels

plt.rcParams.update({
    "font.size": 8.5, "axes.labelsize": 9, "axes.titlesize": 9,
    "legend.fontsize": 7.5, "xtick.labelsize": 8, "ytick.labelsize": 8,
    "lines.linewidth": 1.2, "axes.linewidth": 0.8,
    "grid.alpha": 0.3, "grid.linewidth": 0.5,
    "pdf.fonttype": 42, "ps.fonttype": 42,   # editable text in vector output
    "figure.dpi": 110, "savefig.dpi": 600,
    "savefig.bbox": "tight",
})

LAWS = [("C1", "tuned_load", "C1 TNDQ (tuned)", C_C1, "-"),
        ("C2", "chandra_load", "C2 [Ch20]", C_C2, (0, (5, 2))),
        ("C3", "dqhinf_load", r"C3 DQ-$H_\infty$", C_C3, "-.")]

CONDS = [("standard", ""), ("high-speed", "_hspeed"), ("noise", "_noise")]

# Gain tiers (params.GAIN_SETS): scalar base k_p = 16 I_6; tuned/fast matrices
GAIN_TIERS = [("tuned", "tuned_load", 320.0, 80.0, C_TUNED),
              ("fast", "fast_load", 720.0, 180.0, C_FAST),
              ("base", "load", 16.0, 16.0, C_BASE)]

K_P_TUNED = np.diag(np.r_[np.full(3, 320.0), np.full(3, 80.0)])
K_P_BASE = 16.0 * np.eye(6)
C_STAR = 160.0          # Theorem 3(b) level-set threshold (tuned weights)
T_ATTACH = 2.5          # rigid cup attachment (load mode)
PHASES_EN = ["descend", "hold", "lift", "retreat", "transit",
             "descend-2", "circle"]


# ---------------------------------------------------------------------------
# Data access helpers
# ---------------------------------------------------------------------------
def load(tag):
    """Load results/grasp_circle_<tag>.npz, truncating channels to a common
    length (aborted runs may differ by one sample across channel groups)."""
    d = dict(np.load(os.path.join(RESULTS, f"grasp_circle_{tag}.npz"),
                     allow_pickle=False))
    n = min(len(d["t"]), len(d["t_ex"]))
    for k in list(d):
        if d[k].ndim >= 1 and d[k].shape[0] in (len(d["t"]), len(d["t_ex"])):
            d[k] = d[k][:n]
    return d


def errors(d):
    """(||T||, ||O||, ||e_xi||) time series; e_z = [O; T] stacking."""
    T = np.linalg.norm(d["e_z"][:, 3:], axis=1)
    O = np.linalg.norm(d["e_z"][:, :3], axis=1)
    exi = np.linalg.norm(d["e_xi"], axis=1)
    return T, O, exi


def ss_mask(d):
    """Circle steady-state mask: t >= T_circle_start + ramp(2 s) + 1 s."""
    return d["t"] >= float(d["t_marks"][-1]) + 3.0


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x) ** 2)))


def ss_stats(tag):
    d = load(tag)
    T, O, exi = errors(d)
    m = ss_mask(d)
    return dict(T=rms(T[m]), O=rms(O[m]), exi=rms(exi[m]))


def mark_phases(ax, d, attach=True, names=False, ss_ref=True):
    """Phase-boundary lines, cup-attachment marker, steady-state shading."""
    marks = np.atleast_1d(d["t_marks"]).astype(float)
    for m in marks:
        if d["t"][0] < m < d["t"][-1]:
            ax.axvline(m, color="0.85", lw=0.6, zorder=0)
    if attach:
        ax.axvline(T_ATTACH, color="0.25", lw=0.9, ls=":", zorder=1)
    t_ss = float(d["t_marks"][-1]) + 3.0       # circle-ss start = 12.5 s
    ax.axvspan(t_ss, d["t"][-1], color="0.5", alpha=0.07, zorder=0)
    if ss_ref:
        ax.axvline(t_ss, color="0.35", lw=0.8, ls="--", zorder=1)
    b = [float(d["t"][0])] + list(marks) + [float(d["t"][-1])]
    if names and len(b) - 1 == len(PHASES_EN):
        for name, lo, hi in zip(PHASES_EN, b[:-1], b[1:]):
            ax.text(0.5 * (lo + hi), 1.012, name, fontsize=6, color="0.45",
                    ha="center", va="bottom", transform=ax.get_xaxis_transform())


def panel_tag(ax, tag, inside=False):
    if inside:
        ax.text(0.008, 0.925, tag, transform=ax.transAxes, ha="left",
                va="top", fontweight="bold", fontsize=9,
                bbox=dict(facecolor="white", edgecolor="none",
                          alpha=0.75, pad=1.2))
    else:
        ax.set_title(tag, loc="left", fontweight="bold", fontsize=9)


def save(fig, name):
    for ext in ("pdf", "png"):
        out = os.path.join(RESULTS, f"{name}.{ext}")
        fig.savefig(out)
        print(f"saved: {out}")
    plt.close(fig)


# ===========================================================================
# Task 1 - error time histories: C1 (tuned) vs C2 vs C3, load mode
# ===========================================================================
def fig_errors_timeseries():
    # layered styles: thick solid C1 on the bottom, dashed C2 letting the
    # C1 trace show through the gaps, thin dash-dot C3 on top — the near
    # equivalence of the three laws stays visible instead of overplotting
    style = {"C1": dict(ls="-", lw=1.7, zorder=3),
             "C2": dict(ls=(0, (5, 2)), lw=1.05, zorder=4),
             "C3": dict(ls=(0, (4, 1.4, 1, 1.4)), lw=0.9, zorder=5)}
    fig, axes = plt.subplots(3, 1, figsize=(7.0, 6.15), sharex=True)
    panels = [(r"$\|\mathcal{T}\|$ [mm]", lambda T, O, X: 1e3 * T),
              (r"$\|\mathcal{O}\|$", lambda T, O, X: O),
              (r"$\|e_\xi\|$", lambda T, O, X: X)]
    ref = None
    for ax, (ylab, fn) in zip(axes, panels):
        for law, tag, lab, col, _ in LAWS:
            d = load(tag)
            ref = d
            ax.plot(d["t"], fn(*errors(d)), color=col, label=lab,
                    **style[law])
        ax.set_yscale("log")
        ax.set_ylabel(ylab)
        ax.grid(True, which="major")
        mark_phases(ax, ref, names=ax is axes[0])
    tx = blended_transform_factory(axes[0].transData, axes[0].transAxes)
    axes[0].text(2.32, 0.95, "cup attached (0.25 kg)", transform=tx,
                 fontsize=6.8, color="0.3", va="top", ha="center",
                 rotation=90)
    axes[0].text(12.75, 1.6, "steady-state window ($t\\geq 12.5$ s)",
                 fontsize=6.5, color="0.35", va="center")
    axes[0].legend(loc="upper right", bbox_to_anchor=(0.995, 0.93),
                   framealpha=0.95)
    # quantitative circle-ss readout on the twist channel (previews V2)
    ss = {law: ss_stats(tag)["exi"] for law, tag, *_ in LAWS}
    axes[2].text(0.995, 0.93,
                 "circle-ss $\\|e_\\xi\\|_{\\mathrm{rms}}$ "
                 f"[$\\times 10^{{-3}}$]:  C1 {1e3*ss['C1']:.3f}  |  "
                 f"C2 {1e3*ss['C2']:.3f}  |  C3 {1e3*ss['C3']:.3f}",
                 transform=axes[2].transAxes, ha="right", va="top",
                 fontsize=6.8, color="0.25")
    for tag, ax in zip("(a) (b) (c)".split(), axes):
        panel_tag(ax, tag, inside=True)
    axes[-1].set_xlabel("$t$ [s]")
    axes[-1].set_xlim(0, ref["t"][-1])
    fig.tight_layout()
    save(fig, "sec6_errors_timeseries_load")


# ===========================================================================
# Task 2 - C1 gain tiers: tuned vs fast (stiffness -> steady-state residual)
# ===========================================================================
def fig_c1_gain_tiers():
    fig = plt.figure(figsize=(7.0, 5.85))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.45, 1.0], hspace=0.34,
                          wspace=0.40)
    axT = fig.add_subplot(gs[0, 0])
    axO = fig.add_subplot(gs[0, 1], sharex=axT)
    axbT = fig.add_subplot(gs[1, 0])
    axbO = fig.add_subplot(gs[1, 1])

    tiers = [("tuned", "tuned_load", C_TUNED, "-", "C1 tuned "
              "($p_O{=}320,\\, p_T{=}80$)"),
             ("fast", "fast_load", C_FAST, "-", "C1 fast "
              "($p_O{=}720,\\, p_T{=}180$)")]
    for name, tag, col, ls, lab in tiers:
        d = load(tag)
        T, O, _ = errors(d)
        axT.plot(d["t"], 1e3 * T, color=col, ls=ls, lw=1.1, label=lab)
        axO.plot(d["t"], O, color=col, ls=ls, lw=1.1, label=lab)
    d0 = load("tuned_load")
    for ax in (axT, axO):
        ax.set_yscale("log")
        ax.grid(True, which="both")
        mark_phases(ax, d0, attach=True)
        ax.set_xlim(0, d0["t"][-1])
    axT.set_ylabel(r"$\|\mathcal{T}\|$ [mm]")
    axO.set_ylabel(r"$\|\mathcal{O}\|$")
    axT.set_xlabel("$t$ [s]"), axO.set_xlabel("$t$ [s]")
    axT.legend(loc="lower right", framealpha=0.9)
    panel_tag(axT, "(a)"), panel_tag(axO, "(b)")

    stats = {name: ss_stats(tag) for name, tag, _, _, _ in tiers}
    for ax, key, lab, scale, ticklabs in (
            (axbT, "T", r"$\|\mathcal{T}\|_{\mathrm{rms}}$ [mm]", 1e3,
             ["tuned\n$p_T{=}80$", "fast\n$p_T{=}180$"]),
            (axbO, "O", r"$\|\mathcal{O}\|_{\mathrm{rms}}$ "
                        r"[$\times 10^{-3}$]", 1e3,
             ["tuned\n$p_O{=}320$", "fast\n$p_O{=}720$"])):
        vals = [scale * stats["tuned"][key], scale * stats["fast"][key]]
        bars = ax.bar([0, 1], vals, width=0.52,
                      color=[C_TUNED, C_FAST], edgecolor="k", linewidth=0.5)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v * 1.06, f"{v:.3f}",
                    ha="center", va="bottom", fontsize=7.5)
        ratio = vals[0] / vals[1]
        ax.annotate("", xy=(0.98, vals[1] * 1.5), xytext=(0.02, vals[0] * 1.5),
                    arrowprops=dict(arrowstyle="->", color="0.25", lw=0.9,
                                    connectionstyle="arc3,rad=-0.16"))
        ax.text(0.5, vals[0] * 2.1, f"$\\times{ratio:.2f}$ lower residual "
                f"(stiffness $\\times 2.25$)", ha="center", fontsize=7.2,
                color="0.2")
        ax.set_xticks([0, 1])
        ax.set_xticklabels(ticklabs, fontsize=7.5)
        ax.set_ylabel(lab)
        ax.set_yscale("log")
        ax.set_ylim(min(vals) * 0.55, max(vals) * 4.6)
        ax.grid(True, axis="y", which="both")
    panel_tag(axbT, "(c)"), panel_tag(axbO, "(d)")
    save(fig, "sec6_c1_gain_tiers_load")
    return stats


# ===========================================================================
# Task 3 (V1) - stiffness scaling law (5.9): disturbance inversion consistency
# ===========================================================================
def fig_v1_inversion(tier_stats=None):
    if tier_stats is None:
        tier_stats = {}
    inv = {}
    for name, tag, p_O, p_T, col in GAIN_TIERS:
        st = tier_stats.get(name) or ss_stats(tag)
        inv[name] = dict(dv=p_T * st["T"], dw=0.5 * p_O * st["O"],
                         T=st["T"], O=st["O"], color=col)

    ref_dv = 0.5 * (inv["tuned"]["dv"] + inv["fast"]["dv"])
    ref_dw = 0.5 * (inv["tuned"]["dw"] + inv["fast"]["dw"])
    cons = max(abs(inv["tuned"]["dv"] - inv["fast"]["dv"]) / ref_dv,
               abs(inv["tuned"]["dw"] - inv["fast"]["dw"]) / ref_dw) * 100

    fig, (axA, axB) = plt.subplots(1, 2, figsize=(7.0, 2.95), width_ratios=[1.15, 1.0])
    x = np.arange(3)
    w = 0.36
    tiers = [n for n, *_ in GAIN_TIERS]
    dv = [inv[n]["dv"] for n in tiers]
    dw = [inv[n]["dw"] for n in tiers]
    b1 = axA.bar(x - w / 2, dv, w, color=C_DV, edgecolor="k", linewidth=0.5,
                 label=r"$\|d_v\| = p_T\,\|\mathcal{T}\|_{\mathrm{ss}}$")
    b2 = axA.bar(x + w / 2, dw, w, color=C_DW, edgecolor="k", linewidth=0.5,
                 hatch="//", alpha=0.85,
                 label=r"$\|d_\omega\| = \frac{1}{2}\lambda(K_{p,O})\,"
                       r"\|\mathcal{O}\|_{\mathrm{ss}}$")
    for bars in (b1, b2):
        for b in bars:
            axA.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.012,
                     f"{b.get_height():.3f}", ha="center", va="bottom",
                     fontsize=6.8)
    axA.annotate(f"tuned vs fast:\n{cons:.2f}% consistent",
                 xy=(-0.30, 0.885), ha="left", va="top", fontsize=7.5,
                 color="0.15")
    axA.set_xticks(x)
    axA.set_xticklabels([f"{n}\n($p_O$={p_O:.0f}, $p_T$={p_T:.0f})"
                         for n, _, p_O, p_T, _ in GAIN_TIERS], fontsize=7.2)
    axA.set_ylabel("inverted disturbance magnitude")
    axA.set_ylim(0, 0.94)
    axA.grid(True, axis="y")
    axA.legend(loc="upper right", fontsize=6.6)
    panel_tag(axA, "(a)")

    dev_dv = [(v / ref_dv - 1.0) * 100 for v in dv]
    dev_dw = [(v / ref_dw - 1.0) * 100 for v in dw]
    axB.axhspan(-2.0, 2.0, color=C_DV, alpha=0.09, zorder=0)
    axB.text(0.0, -4.4, "tuned / fast agree within $\\pm 1.0$ %",
             ha="left", fontsize=6.8, color="0.3")
    b1 = axB.bar(x - w / 2, dev_dv, w, color=C_DV, edgecolor="k",
                 linewidth=0.5, label=r"$\|d_v\|$ channel")
    b2 = axB.bar(x + w / 2, dev_dw, w, color=C_DW, edgecolor="k",
                 linewidth=0.5, hatch="//", alpha=0.85,
                 label=r"$\|d_\omega\|$ channel")
    for bars in (b1, b2):
        for b in bars[2:]:
            h = b.get_height()
            axB.text(b.get_x() + b.get_width() / 2, h - 3.6,
                     f"{h:+.1f}%", ha="center", fontsize=7)
    axB.axhline(0.0, color="k", lw=0.8)
    axB.set_xticks(x)
    axB.set_xticklabels(tiers)
    axB.set_ylabel("deviation from tuned\u2013fast mean [%]")
    axB.set_ylim(-52, 14)
    axB.grid(True, axis="y")
    axB.legend(loc="lower left", fontsize=6.6)
    axB.text(2.02, 7.5, "base: uncompensated 1/4 rotation factor\n"
             "$\\Rightarrow$ scaling law (5.9) fails", ha="right",
             va="top", fontsize=6.8, color="0.3")
    panel_tag(axB, "(b)")
    fig.tight_layout()
    save(fig, "sec6_v1_inversion_load")
    return inv, cons


# ===========================================================================
# Task 3 (V2) - certificate divergence under extreme conditions
# ===========================================================================
def fig_v2_divergence():
    exi = {}                       # exi[cond][law] = steady-state ||e_xi||_rms
    for cname, suf in CONDS:
        exi[cname] = {}
        for law, tag, *_ in LAWS:
            exi[cname][law] = ss_stats(tag + suf)["exi"]

    fig, (axA, axB) = plt.subplots(1, 2, figsize=(7.0, 2.95),
                                   width_ratios=[1.2, 1.0])
    x = np.arange(len(CONDS))
    w = 0.26
    for i, (law, _, lab, col, _) in enumerate(LAWS):
        vals = [1e3 * exi[c][law] for c, _ in CONDS]
        bars = axA.bar(x + (i - 1) * w, vals, w, color=col, edgecolor="k",
                       linewidth=0.5, label=lab)
        for b in bars:
            axA.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.08,
                     f"{b.get_height():.3f}", ha="center", va="bottom",
                     fontsize=6.4, rotation=90)
    axA.set_xticks(x)
    axA.set_xticklabels(["standard\n($\\omega{=}1.0$)",
                         "high-speed\n($\\omega{=}2.5$)", "noise"],
                        fontsize=7.5)
    axA.set_ylabel(r"$\|e_\xi\|_{\mathrm{rms}}$ [$\times 10^{-3}$]")
    axA.set_ylim(0, 3.9)
    axA.grid(True, axis="y")
    axA.legend(loc="upper left", fontsize=6.8)
    panel_tag(axA, "(a)")

    xb = np.arange(len(CONDS))
    for i, (law, _, lab, col, _) in enumerate(LAWS[1:], start=1):
        dev = [(exi[c][law] / exi[c]["C1"] - 1.0) * 100 for c, _ in CONDS]
        bars = axB.bar(xb + (i - 1.5) * w * 1.15, dev, w * 1.1, color=col,
                       edgecolor="k", linewidth=0.5, label=lab)
        for b in bars:
            axB.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.05,
                     f"+{b.get_height():.2f}" if b.get_height() < 1 else
                     f"+{b.get_height():.1f}", ha="center", va="bottom",
                     fontsize=6.6)
    axB.axhline(0.0, color="k", lw=0.8)
    axB.set_xticks(xb)
    axB.set_xticklabels(["standard", "high-speed", "noise"], fontsize=7.5)
    axB.set_ylabel("relative deviation from C1 [%]")
    axB.set_ylim(0, 2.9)
    axB.grid(True, axis="y")
    axB.legend(loc="upper left", fontsize=6.8)
    panel_tag(axB, "(b)")
    fig.tight_layout()
    save(fig, "sec6_v2_certificate_divergence_load")
    return exi


# ===========================================================================
# Task 3 (V3) - level-set margin of Theorem 3(b)
# ===========================================================================
def fig_v3_level_set():
    d = load("tuned_load")
    t = d["t"]
    e_xi, e_z = d["e_xi"], d["e_z"]
    V_tuned = (0.5 * np.einsum("ij,ij->i", e_xi, e_xi)
           + 0.5 * np.einsum("ij,jk,ik->i", e_z, K_P_TUNED, e_z))
    V_base = (0.5 * np.einsum("ij,ij->i", e_xi, e_xi)
            + 0.5 * np.einsum("ij,jk,ik->i", e_z, K_P_BASE, e_z))
    i_pk = int(np.argmax(V_tuned))
    V_pk, t_pk = float(V_tuned[i_pk]), float(t[i_pk])
    margin = np.log10(C_STAR / V_pk)

    V_bound = 20.0 * V_base          # conservative tuned-weight bound, §6.3
    i_b = int(np.argmax(V_bound))
    V_bpk, t_bpk = float(V_bound[i_b]), float(t[i_b])
    margin_b = np.log10(C_STAR / V_bpk)

    fig, ax = plt.subplots(figsize=(7.0, 3.2))
    ax.plot(t, V_tuned, color=C_C1, lw=1.15, zorder=4,
            label="$V(t)$ — exact tuned certificate weights "
                  "($K_p{=}\\mathrm{diag}(320I_3, 80I_3)$)")
    ax.plot(t, V_bound, color="0.3", lw=0.9, ls="--", zorder=3,
            label="$20\\,V(t)$ base weights — conservative bound (§6.3)")
    ax.axhline(C_STAR, color=C_C3, lw=1.1, ls="--",
               label=f"level-set threshold $c^* = {C_STAR:.0f}$ (Thm. 3(b))")
    ax.axvline(T_ATTACH, color="0.4", lw=0.9, ls=":")
    ax.text(T_ATTACH + 0.15, 2.2e2, "cup attached", fontsize=7, color="0.35")
    ax.plot(t_bpk, V_bpk, "o", ms=4.5, color=C_C2, zorder=5)
    ax.annotate(f"bound peak $= {V_bpk:.3f}$\n"
                f"exact peak $= {V_pk:.3f}$ ($\\to$ ${margin:.1f}$ decades)",
                xy=(t_bpk, V_bpk), xytext=(4.2, 6.0), fontsize=7.5,
                arrowprops=dict(arrowstyle="->", color="0.25", lw=0.8))
    ax.annotate("", xy=(20.9, C_STAR), xytext=(20.9, V_bpk),
                arrowprops=dict(arrowstyle="<->", color=C_C3, lw=1.0))
    ax.text(20.35, 8.0, f"$\\approx {margin_b:.1f}$ decades of margin",
            fontsize=7.6, color=C_C3, ha="center", va="center",
            rotation=90)
    mark_phases(ax, d, attach=False, ss_ref=False)
    ax.set_yscale("log")
    ax.set_ylim(1e-5, 5e2)
    ax.set_xlim(0, t[-1])
    ax.set_xlabel("$t$ [s]")
    ax.set_ylabel("$V$")
    ax.grid(True, which="both")
    ax.legend(loc="lower left", fontsize=7.0, framealpha=0.95)
    panel_tag(ax, "(a)")

    ax_in = ax.inset_axes([0.55, 0.50, 0.30, 0.42])
    m = (t >= 2.4) & (t <= 4.4)
    ax_in.plot(t[m], V_tuned[m], color=C_C1, lw=1.1)
    ax_in.axvline(T_ATTACH, color="0.4", lw=0.8, ls=":")
    ax_in.set_yscale("log")
    ax_in.set_title("load-step transient", fontsize=7)
    ax_in.tick_params(labelsize=6.5)
    ax_in.grid(True, which="both", alpha=0.3)
    ax.indicate_inset_zoom(ax_in, edgecolor="0.5", lw=0.7)

    fig.tight_layout()
    save(fig, "sec6_v3_level_set_margin_load")
    return dict(V_pk=V_pk, t_pk=t_pk, V_bpk=V_bpk, margin=margin,
                margin_b=margin_b)


# ===========================================================================
def main():
    print("== Task 1: error time histories (C1/C2/C3, load) ==")
    fig_errors_timeseries()
    print("== Task 2: C1 gain tiers (tuned vs fast) ==")
    tier_stats = fig_c1_gain_tiers()
    print("== Task 3: V1 inversion / V2 divergence / V3 level set ==")
    inv, cons = fig_v1_inversion(tier_stats)
    exi = fig_v2_divergence()
    v3 = fig_v3_level_set()

    print("\n--- verification vs Section 6 numbers ---")
    for n in ("tuned", "fast", "base"):
        s = inv[n]
        print(f"V1 {n:5s}: ||T||ss={s['T']:.3e} ||O||ss={s['O']:.3e} -> "
              f"||d_v||={s['dv']:.3f}, ||d_w||={s['dw']:.3f}")
    print(f"V1 tuned-vs-fast consistency: {cons:.2f}% (paper: 1.93%)")
    for c, _ in CONDS:
        row = "  ".join(f"{law} {1e3*exi[c][law]:.3f}" for law, *_ in LAWS)
        print(f"V2 {c:10s} ||e_xi||_rms x1e3: {row}")
    print(f"V3 bound peak (20x base weights) = {v3['V_bpk']:.4f} "
          f"(paper: <=0.494), margin {v3['margin_b']:.2f} decades "
          f"(paper: ~2.5); exact tuned-weight peak = {v3['V_pk']:.4f} "
          f"at t={v3['t_pk']:.2f} s -> {v3['margin']:.2f} decades")


if __name__ == "__main__":
    main()
