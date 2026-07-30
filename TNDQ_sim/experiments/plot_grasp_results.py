#!/usr/bin/env python3
"""S3 抓杯实验论文级出图脚本（仿真验证章节 图 6.0-6.6 配图）。

基于 results/grasp_circle_*.npz 原始数据出图，与 run_grasp_circle.py 的
--plot 快速对比图互补：本脚本聚焦"三律带载对比"视角（论文第 6 章），
默认 C1 取 tuned 档（与 C2/C3 同预算，残余差异纯属结构）。

生成图表（存 results/）：
  1. grasp_paper_errors_{mode}.png    位置|T|/姿态|O|/twist|e_ξ| 三误差时序
                                      三律对比（图 6.2，含相位分界与标注）
  2. grasp_paper_traj3d_{mode}.png    三维轨迹 + 圆周段俯视 + 径向偏差放大
                                      （图 6.0，轨迹重合故用径向偏差曝光差异）
  3. grasp_paper_effort_{mode}.png    力矩时序 + 分相位 τ_rms 柱状（图 6.3）
  4. grasp_paper_lyapunov_{mode}.png  统一权重 V(t) 半对数收敛曲线（图 6.5）
  5. grasp_paper_phase_bars_{mode}.png 分相位 T/O/e_ξ RMS 分组柱状图
  6. grasp_paper_disturbance_{mode}.png 反演的证书通道等效扰动 ‖d̂(t)‖
                                      + 分相位 d̂_rms + (5.7) 均方界的
                                      保守倍数（图 6.6，§6.5(6)）

口径说明（与论文修正后的定理 3 一致，切勿误读）：
  * V 的权重：本脚本全部 V 曲线按 base 权重（k_p=16）重算，故跨增益组
    可比；但 tuned 档的工作集门槛 c* 是在 tuned 权重（p_O=320）下给出的，
    拿图上数值去比余度前必须先乘权重比 p_O/16 = 20。
  * (5.7) 是均方（RMS）极限界，不是逐点 ISS 极限球；左侧泛函就是 e_ξ
    的 RMS，故可与实测 RMS 直接对比。
  * d̂ 对 C1 = 证书真正看到的扰动；对 C2/C3 额外含「实际反馈 − 证书反馈」
    的结构差，绝对值不可跨律比（同律跨工况比仍公平）。

用法：
    python3 experiments/plot_grasp_results.py                 # load 全部图
    python3 experiments/plot_grasp_results.py --mode noload
    python3 experiments/plot_grasp_results.py --gains fast --condition noise
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from core.dq_algebra import dq_translation
from run_grasp_circle import (
    RESULTS_DIR, PHASE_NAMES, _load_npz, _phase_stats, _V_common,
    _d_hat_norm, _rms_bound, V_WEIGHT_TAG,
)

plt.rcParams["font.family"] = ["WenQuanYi Micro Hei", "Noto Sans CJK JP",
                               "Droid Sans Fallback", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

# 三律统一配色（与 run_grasp_circle.plot_condition_compare 保持一致）
LAW_STYLE = [("tndq", "C1 TNDQ (式 5.2)", "tab:green"),
             ("dq-ctc", "C2 DQ-CTC", "tab:purple"),
             ("dq-hinf", "C3 DQ-H∞", "tab:red")]


def load_three_laws(mode, gains, condition):
    """加载三律结果：C1 用指定增益档（缺失回退），C2/C3 固定 base 档名。
    返回 {law: npz dict}（只含已存在的结果）。"""
    runs = {}
    for law, _, _ in LAW_STYLE:
        if law == "tndq":
            for g in (gains, "tuned", "base", "fast"):
                d = _load_npz(mode, "tndq", g, condition)
                if d is not None:
                    d["_c1_gains"] = g
                    runs[law] = d
                    break
        else:
            d = _load_npz(mode, law, "base", condition)
            if d is not None:
                runs[law] = d
    return runs


def ee_positions(d):
    """从日志的位姿 DQ 序列恢复末端平移轨迹 (N,3)。"""
    p = np.array([dq_translation(x) for x in d["x"]])
    p_d = np.array([dq_translation(x) for x in d["x_d"]])
    return p, p_d


def phase_bounds(d):
    """相位分界时间：[t0, ..., tend]，段数 = 7 时可标注相位名。"""
    t = d["t"]
    marks = np.atleast_1d(d["t_marks"]).astype(float)
    inner = [m for m in marks if t[0] < m < t[-1]]
    return [float(t[0])] + inner + [float(t[-1])]


def annotate_phases(ax, d, y_frac=0.96):
    """相位分界竖线 + 顶部相位名标注。"""
    b = phase_bounds(d)
    for m in b[1:-1]:
        ax.axvline(m, color="0.85", lw=0.6, zorder=0)
    if len(b) - 1 == len(PHASE_NAMES):
        for name, lo, hi in zip(PHASE_NAMES, b[:-1], b[1:]):
            ax.text(0.5 * (lo + hi), y_frac, name, fontsize=6.5,
                    color="0.45", ha="center", va="top",
                    transform=ax.get_xaxis_transform())


def _save(fig, fname):
    out = os.path.join(RESULTS_DIR, fname)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Saved plot      : {out}")


# ===========================================================================
# 图 1：三误差时序（图 6.2）——需求 1
# ===========================================================================

def plot_errors(runs, mode, tag):
    panels = [("位置误差 |T| [m]",
               lambda d: np.linalg.norm(d["e_z"][:, 3:], axis=1)),
              ("姿态误差 |O|",
               lambda d: np.linalg.norm(d["e_z"][:, :3], axis=1)),
              ("twist 误差 |e_ξ|",
               lambda d: np.linalg.norm(d["e_xi"], axis=1))]
    fig, axes = plt.subplots(3, 1, figsize=(9, 8.4), sharex=True)
    for ax, (title, fn) in zip(axes, panels):
        for law, lab, color in LAW_STYLE:
            if law not in runs:
                continue
            d = runs[law]
            if law == "tndq":
                lab = f"{lab} [{d['_c1_gains']}]"
            ax.plot(d["t"], fn(d), color=color, lw=1.1, label=lab)
        ax.set_yscale("log")
        annotate_phases(ax, next(iter(runs.values())))
        ax.set_title(title, fontsize=11)
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=8, loc="lower right")
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"三律误差时序对比（mode={mode}{tag}；同预算增益，"
                 "差异 = 结构属性）", fontsize=12)
    _save(fig, f"grasp_paper_errors_{mode}{tag}.png")


# ===========================================================================
# 图 2：三维圆周轨迹跟踪（图 6.0）——需求 2
# ===========================================================================

def circle_slice(d):
    """圆周稳态切片掩码（与 _phase_stats 的 circle-ss 同口径：
    最后一段相位起点 + ramp 2 s + 1 s 过渡之后）。"""
    b = phase_bounds(d)
    return d["t"] >= b[-2] + 3.0


def fit_circle_xy(pts):
    """最小二乘（Kåsa）圆拟合：非整数圈弧段也能得到无偏圆心/半径，
    避免用均值当圆心引入的伪径向偏差。返回 (center_xy, R)。"""
    x, y = pts[:, 0], pts[:, 1]
    A = np.c_[x, y, np.ones_like(x)]
    b = x ** 2 + y ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = 0.5 * sol[0], 0.5 * sol[1]
    R = np.sqrt(sol[2] + cx ** 2 + cy ** 2)
    return np.array([cx, cy]), float(R)


def plot_traj3d(runs, mode, tag):
    fig = plt.figure(figsize=(12, 4.6))
    ax3 = fig.add_subplot(1, 3, 1, projection="3d")
    ax_top = fig.add_subplot(1, 3, 2)
    ax_rad = fig.add_subplot(1, 3, 3)

    # -- 面板 A：全程 3D 轨迹（三律轨迹肉眼重合，取 C1 代表 + 参考） -----
    ref_law = "tndq" if "tndq" in runs else next(iter(runs))
    d0 = runs[ref_law]
    p, p_d = ee_positions(d0)
    ax3.plot(*p_d.T, color="0.4", lw=1.0, ls="--", label="参考轨迹")
    ax3.plot(*p.T, color="tab:green", lw=1.0, label="实际（C1）")
    ax3.scatter(*p[0], color="k", s=15)
    ax3.text(*p[0], "  start", fontsize=7)
    ax3.set_xlabel("x [m]"), ax3.set_ylabel("y [m]"), ax3.set_zlabel("z [m]")
    ax3.set_title("全程末端轨迹（7 相位）", fontsize=10)
    ax3.legend(fontsize=7, loc="upper left")
    ax3.view_init(elev=22, azim=-50)

    # -- 圆周段参考几何：由期望轨迹稳态段拟合圆心/半径（不依赖 params） --
    m0 = circle_slice(d0)
    c_xy, R_ref = fit_circle_xy(p_d[m0, :2])

    # -- 面板 B：圆周段俯视图（三律 vs 参考圆） --------------------------
    th = np.linspace(0, 2 * np.pi, 361)
    ax_top.plot(c_xy[0] + R_ref * np.cos(th), c_xy[1] + R_ref * np.sin(th),
                color="0.4", lw=1.2, ls="--", label=f"参考圆 R={R_ref:.3f} m")
    for law, lab, color in LAW_STYLE:
        if law not in runs:
            continue
        d = runs[law]
        pa, _ = ee_positions(d)
        m = circle_slice(d)
        ax_top.plot(pa[m, 0], pa[m, 1], color=color, lw=0.9, label=lab)
    ax_top.set_aspect("equal")
    ax_top.set_xlabel("x [m]"), ax_top.set_ylabel("y [m]")
    ax_top.set_title("圆周稳态段俯视（三律重合）", fontsize=10)
    ax_top.grid(alpha=0.3)
    ax_top.legend(fontsize=7)

    # -- 面板 C：径向偏差放大（轨迹重合时曝光跟踪精度差异） --------------
    for law, lab, color in LAW_STYLE:
        if law not in runs:
            continue
        d = runs[law]
        pa, _ = ee_positions(d)
        m = circle_slice(d)
        e_rad = np.linalg.norm(pa[m, :2] - c_xy, axis=1) - R_ref
        ax_rad.plot(d["t"][m], 1e3 * e_rad, color=color, lw=0.9, label=lab)
    ax_rad.axhline(0.0, color="0.4", lw=0.8, ls="--")
    ax_rad.set_xlabel("t [s]"), ax_rad.set_ylabel("径向偏差 [mm]")
    ax_rad.set_title("圆周稳态段径向偏差（放大）", fontsize=10)
    ax_rad.grid(alpha=0.3)
    ax_rad.legend(fontsize=7)

    fig.suptitle(f"圆周轨迹跟踪效果（mode={mode}{tag}）", fontsize=12)
    # 3D 子图的 z 轴刻度不被 tight_layout 计入，手动加大面板间距防重叠
    fig.tight_layout()
    fig.subplots_adjust(wspace=0.38)
    out = os.path.join(RESULTS_DIR, f"grasp_paper_traj3d_{mode}{tag}.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Saved plot      : {out}")


# ===========================================================================
# 图 3：力矩消耗（图 6.3）——需求 3 可选项
# ===========================================================================

def plot_effort(runs, mode, tag):
    fig, (ax_t, ax_b) = plt.subplots(2, 1, figsize=(9, 6.4))
    for law, lab, color in LAW_STYLE:
        if law not in runs:
            continue
        d = runs[law]
        ax_t.plot(d["t"], np.linalg.norm(d["tau"], axis=1),
                  color=color, lw=1.0, label=lab)
    annotate_phases(ax_t, next(iter(runs.values())))
    ax_t.set_xlabel("t [s]"), ax_t.set_ylabel("|τ| [N·m]")
    ax_t.set_title("力矩范数时序（三律基本重合 → 无 effort 代价）",
                   fontsize=11)
    ax_t.grid(alpha=0.3)
    ax_t.legend(fontsize=8)

    phases = PHASE_NAMES + ["circle-ss"]
    x = np.arange(len(phases))
    w = 0.26
    for i, (law, lab, color) in enumerate(LAW_STYLE):
        if law not in runs:
            continue
        st = _phase_stats(runs[law])
        vals = [st.get(ph, {}).get("tau_rms", np.nan) for ph in phases]
        ax_b.bar(x + (i - 1) * w, vals, w, color=color, label=lab)
    ax_b.set_xticks(x), ax_b.set_xticklabels(phases, fontsize=8)
    ax_b.set_ylabel("τ_rms [N·m]")
    ax_b.set_title("分相位力矩 RMS", fontsize=11)
    ax_b.grid(alpha=0.3, axis="y")
    ax_b.legend(fontsize=8)
    fig.suptitle(f"控制 effort 对比（mode={mode}{tag}）", fontsize=12)
    _save(fig, f"grasp_paper_effort_{mode}{tag}.png")


# ===========================================================================
# 图 4：Lyapunov 收敛（图 6.5）——需求 3 可选项
# ===========================================================================

def plot_lyapunov(runs, mode, tag):
    fig, ax = plt.subplots(figsize=(9, 3.8))
    for law, lab, color in LAW_STYLE:
        if law not in runs:
            continue
        d = runs[law]
        if law == "tndq":
            lab = f"{lab} [{d['_c1_gains']}]"
        ax.plot(d["t"], _V_common(d), color=color, lw=1.1, label=lab)
    d0 = next(iter(runs.values()))
    annotate_phases(ax, d0)
    t_att = float(d0.get("t_attach", np.nan))
    if np.isfinite(t_att) and t_att > 0:
        ax.axvline(t_att, color="tab:orange", lw=1.0, ls=":")
        ax.text(t_att, 0.05, " 附着(负载突变)", fontsize=7,
                color="tab:orange", transform=ax.get_xaxis_transform())
    ax.set_yscale("log")
    ax.set_xlabel("t [s]")
    # 权重口径（必须标注）：本图用 base 权重，而 tuned 档 c* 用 tuned 权重
    ax.set_ylabel(f"V（统一 {V_WEIGHT_TAG} 权重）")
    # 口径修正：定理 3(d) 给的是均方（RMS）极限界 (5.7)，不是逐点 ISS
    # 极限球，因此不能声称 V “回落至 ISS 极限球”（参见图 6.6 的界校验）
    ax.set_title(f"存储函数收敛：负载突变后指数回落至有界残差水平"
                 f"（均方界 (5.7) 校验见图 6.6；mode={mode}{tag}）",
                 fontsize=11)
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8)
    _save(fig, f"grasp_paper_lyapunov_{mode}{tag}.png")


# ===========================================================================
# 图 6：反演的证书通道等效扰动（图 6.6，§6.5(6)）
# ===========================================================================

def plot_disturbance(runs, mode, tag):
    """等效扰动 ‖d̂(t)‖ 时序 + 分相位 d̂_rms + (5.7) 均方界保守度。

    d̂ 由闭环误差动态 (5.1e) 反演（见 performance.ResidualDisturbance-
    Estimator）：S3 实验不注入 w，旧口径的 d≡0 使 (5.6)/(5.7) 全部空值；
    反演把定理 3 诚实条款承认的全部扰动源（杯子的 ΔM/Δg、测量噪声、
    阻尼伪逆残差、指令限幅与安全治理器、离散化）归入证书口径。

    面板 B 的文字标注给出 (5.7) 的保守倍数 = 界/实测 RMS，服务论文
    §6.5(6) 关于“界成立但保守”的诚实论证。旧 npz（无 d̂ 通道）跳过出图。"""
    avail = [(law, lab, color) for law, lab, color in LAW_STYLE
             if law in runs and np.isfinite(_d_hat_norm(runs[law])).any()]
    if not avail:
        print("[note] 结果文件无 d̂ 通道（§6.5(6) 后新增），跳过扰动图；"
              "重跑 run_grasp_circle.py 即可填充（d̂ 不进控制律，轨迹不变）。")
        return

    fig, (ax_t, ax_b) = plt.subplots(2, 1, figsize=(9, 6.8))

    # -- 面板 A：‖d̂(t)‖ 时序（含附着时刻标注） --------------------------
    for law, lab, color in avail:
        d = runs[law]
        if law == "tndq":
            lab = f"{lab} [{d['_c1_gains']}]"
        y = _d_hat_norm(d)
        ax_t.plot(d["t"][:len(y)], y, color=color, lw=1.0, label=lab)
    d0 = runs[avail[0][0]]
    annotate_phases(ax_t, d0)
    t_att = float(d0.get("t_attach", np.nan))
    if np.isfinite(t_att) and t_att > 0:
        ax_t.axvline(t_att, color="tab:orange", lw=1.0, ls=":")
        ax_t.text(t_att, 0.05, " 附着(负载突变 -> ΔM/Δg)", fontsize=7,
                  color="tab:orange", transform=ax_t.get_xaxis_transform())
    ax_t.set_yscale("log")
    ax_t.set_xlabel("t [s]"), ax_t.set_ylabel("‖d̂‖")
    ax_t.set_title("反演的证书通道等效扰动（含 ΔM/Δg、噪声、伪逆残差、"
                   "限幅/治理器、离散化）", fontsize=11)
    ax_t.grid(alpha=0.3, which="both")
    ax_t.legend(fontsize=8, loc="lower right")

    # -- 面板 B：分相位 d̂_rms + (5.7) 保守倍数 --------------------------
    phases = PHASE_NAMES + ["circle-ss"]
    x = np.arange(len(phases))
    w = 0.26
    have = {a[0] for a in avail}
    for i, (law, lab, color) in enumerate(LAW_STYLE):
        if law not in have:
            continue
        st = _phase_stats(runs[law])
        vals = [st.get(ph, {}).get("d_hat_rms", np.nan) for ph in phases]
        ax_b.bar(x + (i - 1) * w, vals, w, color=color, label=lab)
    ax_b.set_xticks(x), ax_b.set_xticklabels(phases, fontsize=8)
    ax_b.set_yscale("log")
    ax_b.set_ylabel("d̂_rms")
    ax_b.set_title("分相位等效扰动 RMS（C2/C3 含结构差 -> 不跨律比绝对值）",
                   fontsize=11)
    ax_b.grid(alpha=0.3, axis="y")
    ax_b.legend(fontsize=8)

    # (5.7) 均方界的实测校验（整段口径）：界 >= 实测 RMS，标注保守倍数
    lines = []
    for law, lab, _ in avail:
        rb = _rms_bound(runs[law])
        if rb is None:
            continue
        lines.append(f"{lab}: sup‖d̂‖={rb['d_inf']:.2e} -> 界 "
                     f"{rb['bound']:.2e} ≥ 实测 RMS {rb['rms']:.2e}"
                     f"（保守 ×{rb['margin']:.1f}）")
    if lines:
        ax_b.text(0.01, -0.42, "(5.7) 均方界实测校验：\n" + "\n".join(lines),
                  fontsize=7.5, color="0.25", ha="left", va="top",
                  transform=ax_b.transAxes)
    fig.suptitle(f"证书通道等效扰动与均方界校验（mode={mode}{tag}）",
                 fontsize=12)
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.22)
    out = os.path.join(RESULTS_DIR, f"grasp_paper_disturbance_{mode}{tag}.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Saved plot      : {out}")


# ===========================================================================
# 图 5：分相位误差指标柱状图 ——需求 3 可选项
# ===========================================================================

def plot_phase_bars(runs, mode, tag):
    panels = [("T_rms", "位置误差 RMS [m]"),
              ("O_rms", "姿态误差 RMS"),
              ("exi_rms", "twist 误差 RMS")]
    phases = PHASE_NAMES + ["circle-ss"]
    x = np.arange(len(phases))
    w = 0.26
    fig, axes = plt.subplots(3, 1, figsize=(9, 8.4), sharex=True)
    for ax, (key, title) in zip(axes, panels):
        for i, (law, lab, color) in enumerate(LAW_STYLE):
            if law not in runs:
                continue
            st = _phase_stats(runs[law])
            vals = [st.get(ph, {}).get(key, np.nan) for ph in phases]
            ax.bar(x + (i - 1) * w, vals, w, color=color, label=lab)
        ax.set_yscale("log")
        ax.set_title(title, fontsize=11)
        ax.grid(alpha=0.3, axis="y")
        ax.legend(fontsize=8)
    axes[-1].set_xticks(x)
    axes[-1].set_xticklabels(phases, fontsize=9)
    fig.suptitle(f"分相位性能指标（mode={mode}{tag}）", fontsize=12)
    _save(fig, f"grasp_paper_phase_bars_{mode}{tag}.png")


# ===========================================================================
# 入口
# ===========================================================================

def main():
    ap = argparse.ArgumentParser(
        description="S3 抓杯实验论文级出图（三律带载对比 + 3D 圆周轨迹）")
    ap.add_argument("--mode", choices=["noload", "load"], default="load")
    ap.add_argument("--gains", choices=["base", "tuned", "fast"],
                    default="tuned", help="C1 增益档（默认 tuned 与 C2/C3 "
                                          "同预算；缺失自动回退）")
    ap.add_argument("--condition", default="none",
                    choices=["none", "highspeed", "fast-transit",
                             "noise", "coarse-dt"])
    args = ap.parse_args()

    runs = load_three_laws(args.mode, args.gains, args.condition)
    if not runs:
        print(f"[error] 无可用结果文件（mode={args.mode}, "
              f"condition={args.condition}），请先运行 run_grasp_circle.py")
        return 1
    missing = [lab for law, lab, _ in LAW_STYLE if law not in runs]
    if missing:
        print(f"[note] 缺失结果（跳过）: {', '.join(missing)}")

    tag = "" if args.condition == "none" else f"_{args.condition}"
    plot_errors(runs, args.mode, tag)
    plot_traj3d(runs, args.mode, tag)
    plot_effort(runs, args.mode, tag)
    plot_lyapunov(runs, args.mode, tag)
    plot_phase_bars(runs, args.mode, tag)
    plot_disturbance(runs, args.mode, tag)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
