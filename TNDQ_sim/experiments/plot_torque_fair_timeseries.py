"""Generate time-series plots for the 1% fair torque-level comparisons.

The gains are read from ``torque_fair_1pct_matches.csv``.  No new tuning is
performed here: each fairness anchor keeps the independently matched scale
that was selected by the full closed-loop experiment.

Run from the repository root:
    python3 TNDQ_sim/experiments/plot_torque_fair_timeseries.py
"""

import csv
import multiprocessing as mp
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

sys.path.insert(0, os.path.dirname(__file__))
from compare_metrics_portfolio import simulate  # noqa: E402


HERE = os.path.dirname(__file__)
RESULTS = os.path.join(HERE, "..", "results")
MATCHES = os.path.join(RESULTS, "torque_fair_1pct_matches.csv")
LAWS = ("C1", "C2", "C2abl", "C3")
LAW_LABEL = {"C1": "C1", "C2": "C2", "C2abl": "C2-abl", "C3": "C3"}
LAW_COLOR = {"C1": "#1f77b4", "C2": "#ff7f0e",
             "C2abl": "#2ca02c", "C3": "#d62728"}
ANCHORS = ("IAE_a_deg", "E_nongrav")
ANCHOR_LABEL = {
    "IAE_a_deg": "equal attitude IAE",
    "E_nongrav": "equal non-gravity torque effort",
}

_cjk_font = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if os.path.exists(_cjk_font):
    font_manager.fontManager.addfont(_cjk_font)
    _font = font_manager.FontProperties(fname=_cjk_font).get_name()
else:
    _font = "DejaVu Sans"
plt.rcParams["font.sans-serif"] = [_font, "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def read_matches():
    with open(MATCHES, newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {(row["anchor"], row["law"]): float(row["scale"])
            for row in rows}


def run_job(job):
    anchor, law, scale = job
    row, trace = simulate(law, "circle", "none", scale=scale, t_end=10.0,
                          c1_gain_set="base", return_trace=True)
    return anchor, law, scale, row, trace


def _plot_one_anchor(anchor, traces):
    fig, axes = plt.subplots(5, 1, figsize=(11, 14), sharex=True)
    for law in LAWS:
        trace = traces[law]
        t = trace["t"]
        color = LAW_COLOR[law]
        label = LAW_LABEL[law]
        axes[0].plot(t, trace["p_err"], color=color, label=label, linewidth=1.0)
        axes[1].plot(t, trace["a_err_deg"], color=color, label=label, linewidth=1.0)
        axes[2].plot(t, trace["tau_norm"], color=color, label=label, linewidth=1.0)
        axes[2].plot(t, trace["tau_peak_inst"], color=color, linestyle="--",
                     linewidth=0.7, alpha=0.7)
        axes[3].plot(t, trace["tau_slew_norm"], color=color, label=label,
                     linewidth=1.0)
        axes[4].plot(t, trace["qddot_norm"], color=color, label=label,
                     linewidth=1.0)

    axes[0].set_ylabel("position error [m]")
    axes[1].set_ylabel("attitude error [deg]")
    axes[2].set_ylabel("torque norm / peak [N m]")
    axes[3].set_ylabel("torque slew norm [N m/s]")
    axes[4].set_ylabel("reference acceleration [rad/s2]")
    axes[4].set_xlabel("time [s]")
    axes[2].text(0.01, 0.92, "solid: norm, dashed: instantaneous peak",
                 transform=axes[2].transAxes, fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.25)
    axes[0].legend(ncol=4, fontsize=8, loc="upper right")
    fig.suptitle("Torque-level fair comparison: " + ANCHOR_LABEL[anchor])
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    path = os.path.join(RESULTS, f"torque_fair_1pct_timeseries_{anchor}.png")
    fig.savefig(path, dpi=180)
    plt.close(fig)

    # A separate cumulative plot makes the fairness constraint visible in time.
    # Position and attitude have different units, so keep them on separate axes.
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    for law in LAWS:
        trace = traces[law]
        color = LAW_COLOR[law]
        label = LAW_LABEL[law]
        axes[0].plot(trace["t"], trace["p_iae_cum"], color=color,
                     label=label, linewidth=1.1)
        axes[1].plot(trace["t"], trace["a_iae_cum"], color=color,
                     label=label, linewidth=1.1)
        axes[2].plot(trace["t"], trace["E_nongrav_cum"], color=color,
                     label=label, linewidth=1.1)
        axes[3].plot(trace["t"], trace["E_qddot_cum"], color=color,
                     label=label, linewidth=1.1)
    axes[0].set_ylabel("position IAE [m s]")
    axes[1].set_ylabel("attitude IAE [deg s]")
    axes[2].set_ylabel("non-gravity effort")
    axes[3].set_ylabel("qddot effort")
    axes[3].set_xlabel("time [s]")
    axes[0].legend(ncol=4, fontsize=8, loc="upper left")
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.suptitle("Cumulative indicators: " + ANCHOR_LABEL[anchor])
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    path = os.path.join(RESULTS, f"torque_fair_1pct_cumulative_{anchor}.png")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main():
    scales = read_matches()
    jobs = [(anchor, law, scales[(anchor, law)])
            for anchor in ANCHORS for law in LAWS]
    nproc = min(len(jobs), 8, os.cpu_count() or 1)
    ctx = mp.get_context("fork")
    output = {}
    with ctx.Pool(nproc) as pool:
        for idx, (anchor, law, scale, row, trace) in enumerate(
                pool.imap(run_job, jobs), 1):
            output.setdefault(anchor, {})[law] = trace
            print(f"[{idx}/{len(jobs)}] {anchor} {law} scale={scale:.7g} "
                  f"IAE_p={row['IAE_p']:.7g} IAE_a={row['IAE_a_deg']:.7g}",
                  flush=True)

    os.makedirs(RESULTS, exist_ok=True)
    for anchor in ANCHORS:
        traces = output[anchor]
        np.savez_compressed(
            os.path.join(RESULTS, f"torque_fair_1pct_timeseries_{anchor}.npz"),
            **{f"{law}_{key}": value for law in LAWS
               for key, value in traces[law].items()})
        _plot_one_anchor(anchor, traces)
    print("saved torque_fair_1pct_timeseries_*.npz and *.png")


if __name__ == "__main__":
    main()
