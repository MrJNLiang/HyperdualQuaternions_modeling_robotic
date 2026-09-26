"""Plot and summarize the 1% torque-level fairness experiment."""

import csv
import os
import statistics
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

# Prefer an installed CJK font so Chinese labels in the reproducibility plots
# render as text rather than missing-glyph boxes.  The fallback keeps the
# script usable on machines without a CJK font.
_cjk_font = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if os.path.exists(_cjk_font):
    font_manager.fontManager.addfont(_cjk_font)
    _cjk_family = font_manager.FontProperties(fname=_cjk_font).get_name()
else:
    _cjk_family = "DejaVu Sans"
plt.rcParams["font.sans-serif"] = [_cjk_family, "SimHei", "NSimSun", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


HERE = os.path.dirname(__file__)
RESULTS = os.path.join(HERE, "..", "results")
MATRIX = os.path.join(RESULTS, "torque_fair_1pct_matrix.csv")
MATCHES = os.path.join(RESULTS, "torque_fair_1pct_matches.csv")

LAWS = ("C1", "C2", "C2abl", "C3")
LAW_LABEL = {"C1": "C1", "C2": "C2", "C2abl": "C2-abl", "C3": "C3"}
ANCHORS = ("E_nongrav", "tau_slew_rms", "tau_peak", "IAE_p", "IAE_a_deg")
OUTCOMES = ("IAE_p", "IAE_a_deg", "E_nongrav", "E_qddot",
            "tau_slew_rms", "tau_peak")
OUT_LABEL = {
    "IAE_p": "位置 IAE",
    "IAE_a_deg": "姿态 IAE",
    "E_nongrav": "非重力力矩能量",
    "E_qddot": "指令加速度能量",
    "tau_slew_rms": "力矩变化率 RMS",
    "tau_peak": "峰值力矩",
}
ANCHOR_LABEL = {
    "E_nongrav": "等非重力力矩能量",
    "tau_slew_rms": "等力矩变化率 RMS",
    "tau_peak": "等峰值力矩",
    "IAE_p": "等位置 IAE",
    "IAE_a_deg": "等姿态 IAE",
}


def read_csv(path):
    rows = []
    with open(path, newline="") as stream:
        for raw in csv.DictReader(stream):
            row = {}
            for key, value in raw.items():
                try:
                    row[key] = float(value)
                except (TypeError, ValueError):
                    row[key] = value
            rows.append(row)
    return rows


def main():
    rows = read_csv(MATRIX)
    matches = read_csv(MATCHES)
    scale = {(r["anchor"], r["law"]): r["scale"] for r in matches}
    os.makedirs(RESULTS, exist_ok=True)

    # Nominal tradeoff panels: each bar is normalized by C1 under the same
    # fairness anchor.  The anchor itself is not interpreted as an outcome.
    fig, axes = plt.subplots(len(ANCHORS), 1, figsize=(11, 16), sharex=True)
    x = np.arange(len(LAWS))
    width = 0.12
    for ax, anchor in zip(axes, ANCHORS):
        nominal = [r for r in rows if r["anchor"] == anchor and
                   r["scenario"] == "circle" and r["condition"] == "none"]
        by = {r["law"]: r for r in nominal}
        ref = by["C1"]
        for j, metric in enumerate(OUTCOMES):
            values = []
            for law in LAWS:
                value = by[law].get(metric, np.nan) / ref.get(metric, np.nan)
                values.append(value)
            ax.bar(x + (j - (len(OUTCOMES) - 1) / 2) * width, values,
                   width=width, label=OUT_LABEL[metric] if ax is axes[0] else None)
        ax.axhline(1.0, color="black", linewidth=0.8)
        ax.set_ylabel("相对 C1")
        ax.set_title(f"{ANCHOR_LABEL[anchor]}；C1={scale[(anchor, 'C1')]:.3g}")
        ax.grid(axis="y", alpha=0.25)
        ax.set_ylim(bottom=0)
    axes[-1].set_xticks(x, [LAW_LABEL[l] for l in LAWS])
    axes[0].legend(ncol=3, fontsize=8, loc="upper left")
    fig.suptitle("力矩级控制律：1%公平锚点下的标称性能比（越低越好）", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(os.path.join(RESULTS, "torque_fair_1pct_nominal.png"), dpi=180)
    plt.close(fig)

    # Condition sensitivity: median ratio across the six matrix conditions.
    fig, axes = plt.subplots(1, len(ANCHORS), figsize=(18, 5), sharey=True)
    heat_metrics = ("IAE_p", "IAE_a_deg", "E_nongrav", "tau_slew_rms")
    for ax, anchor in zip(axes, ANCHORS):
        values = np.full((len(heat_metrics), len(LAWS)), np.nan)
        for i, metric in enumerate(heat_metrics):
            for j, law in enumerate(LAWS):
                ratios = []
                for row in rows:
                    if row["anchor"] != anchor or row["law"] != law:
                        continue
                    ref = next(r for r in rows if r["anchor"] == anchor and
                               r["law"] == "C1" and
                               r["scenario"] == row["scenario"] and
                               r["condition"] == row["condition"])
                    if np.isfinite(row.get(metric, np.nan)) and np.isfinite(ref.get(metric, np.nan)):
                        ratios.append(row[metric] / ref[metric])
                if ratios:
                    values[i, j] = statistics.median(ratios)
        image = ax.imshow(values, aspect="auto", cmap="RdYlGn_r", vmin=0.2, vmax=2.0)
        ax.set_xticks(np.arange(len(LAWS)), [LAW_LABEL[l] for l in LAWS])
        ax.set_yticks(np.arange(len(heat_metrics)), [OUT_LABEL[m] for m in heat_metrics])
        ax.set_title(ANCHOR_LABEL[anchor])
        for i in range(values.shape[0]):
            for j in range(values.shape[1]):
                if np.isfinite(values[i, j]):
                    ax.text(j, i, f"{values[i, j]:.2f}", ha="center", va="center", fontsize=8)
    fig.colorbar(image, ax=axes.ravel().tolist(), label="相对 C1 中位数")
    fig.suptitle("六种工况下的相对性能（标称公平参数冻结）", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(RESULTS, "torque_fair_1pct_conditions.png"), dpi=180)
    plt.close(fig)

    # Match scales and residual errors.
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for anchor_i, anchor in enumerate(ANCHORS):
        vals = [scale[(anchor, law)] for law in LAWS]
        axes[0].plot(np.arange(len(LAWS)), vals, marker="o", label=ANCHOR_LABEL[anchor])
        errs = [next(r["rel_error"] for r in matches
                     if r["anchor"] == anchor and r["law"] == law) * 100 for law in LAWS]
        axes[1].plot(np.arange(len(LAWS)), errs, marker="o", label=ANCHOR_LABEL[anchor])
    axes[0].set_yscale("log")
    axes[0].set_ylabel("反馈尺度（对数）")
    axes[1].axhspan(-1, 1, color="green", alpha=0.12)
    axes[1].axhline(0, color="black", linewidth=0.8)
    axes[1].set_ylabel("标称锚点相对误差 [%]")
    for ax in axes:
        ax.set_xticks(np.arange(len(LAWS)), [LAW_LABEL[l] for l in LAWS])
        ax.grid(alpha=0.25)
    axes[0].legend(fontsize=8)
    fig.suptitle("经验增益尺度与 1% 匹配残差", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(os.path.join(RESULTS, "torque_fair_1pct_matching.png"), dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
