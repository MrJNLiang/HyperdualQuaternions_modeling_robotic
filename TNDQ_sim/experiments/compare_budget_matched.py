"""预算锚点匹配对比：等预算(1%)下测试哪种公平性指标能显形 C1 的优势。

从任何目录运行：python TNDQ_sim/experiments/compare_budget_matched.py

协议（替代 compare_metrics_portfolio.py 的线性化配平——那会使四律增益
等价、标称行为几乎相同）：

  1. 选一个"预算锚点"指标 B（公平性口径），取 C1 在 λ=1、circle/nominal
     全时程的 B 值为参考；
  2. 对每个基线律做增益尺度 λ 的二分标定（λ 同乘全部反馈/位姿增益），
     使 |B(λ)/B_ref − 1| ≤ 1%，标定后冻结；
  3. 冻结增益跑工况矩阵（circle none/l2/noise、line none、large none），
     比较结果指标：收敛速度（settle_s）、误差（IAE/peak）、稳定性
     （sat/gov/gamma）、执行器负担（E_nongrav/tau_slew）。

锚点集合：
  E_nongrav  ∫||τ-ĝ||²dt   重力参考努力（公平努力口径）
  E_qddot    ∫||q̈_ref||²dt  共享接口入口能量（仓库 E_cmd 口径的力矩版）
  IAE_p      ∫||T||dt       等精度口径（反锚点：比的是各律花多少努力
                            达到 C1 的精度）
  E_tau2     ∫||τ||²dt      退化锚点反例（重力主导，标定会把 λ 推到
                            极端——本身就是一个方法论发现）

C1 恒为参考（λ=1）。输出 results/budget_matched.csv 与终端判定表。
"""

import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from compare_metrics_portfolio import (  # noqa: E402
    LAWS, ROOT, simulate,
)

CAL_T_END = 10.0          # 标定时程 = 矩阵时程（预算口径完全一致）
TOL = 0.01                # 1% 预算带
ANCHORS = {
    "E_nongrav": +1,      # +1: 随 λ 递增
    "E_qddot": +1,
    "IAE_p": -1,          # -1: 随 λ 递减
    "E_tau2": +1,
}
CONDITIONS = (("circle", "none"), ("circle", "l2"), ("circle", "noise"),
              ("line", "none"), ("large", "none"))

OUTCOME_METRICS = ("IAE_p", "IAE_p_ss", "settle_s", "p_peak", "a_peak_deg",
                   "p_end", "E_nongrav", "tau_slew_rms", "E_qddot",
                   "tau_peak", "sat_frac", "gov_frac", "gamma_xi")


def _budget(law, anchor, scale):
    row = simulate(law, "circle", "none", scale=scale, t_end=CAL_T_END)
    return row[anchor], row


def calibrate(law, anchor, target):
    """二分标定 λ 使 |B(λ)/target-1| <= TOL；失败返回 (None, reason)。"""
    sense = ANCHORS[anchor]
    lo, hi = 0.25, 1.0

    def ok(v):
        return abs(v / target - 1.0) <= TOL

    # 向下扩下界：λ→小 使递增锚点 B 变小 / 递减锚点 B 变大
    v, _ = _budget(law, anchor, lo)
    while (v - target) * sense > 0:
        lo /= 2.0
        if lo < 0.02:
            return None, "cannot bracket low"
        v, _ = _budget(law, anchor, lo)
    if ok(v):
        return lo, "bracket hit"
    # 向上扩上界
    v, _ = _budget(law, anchor, hi)
    while (v - target) * sense < 0:
        hi *= 2.0
        if hi > 32.0:
            return None, "cannot bracket high (λ>32)"
        v, _ = _budget(law, anchor, hi)
    if ok(v):
        return hi, "bracket hit"
    for _ in range(12):
        mid = 0.5 * (lo + hi)
        v, _ = _budget(law, anchor, mid)
        if ok(v):
            return mid, "bisected"
        if (v - target) * sense < 0:
            lo = mid
        else:
            hi = mid
    lam = 0.5 * (lo + hi)
    v, _ = _budget(law, anchor, lam)
    return (lam, "bisected") if ok(v) else (None, f"residual {v/target-1:+.2%}")


def _calib_job(pack):
    """multiprocessing worker（模块级，可 pickle）：pack=(anchor, law, target)。"""
    anchor, law, target = pack
    lam, note = calibrate(law, anchor, target)
    return anchor, law, lam, note


def _run_job(pack):
    """multiprocessing worker：pack=(anchor, law, scenario, condition, lam)。"""
    anchor, law, scenario, condition, lam = pack
    r = simulate(law, scenario, condition, scale=lam)
    r["anchor"] = anchor
    return r


def main():
    import multiprocessing as mp

    # ---- 标定阶段（并行：每个 (anchor, law) 一个任务） ----------------------
    # IAE 锚点是"更严等精度"口径：目标 = C1 标称 IAE 的 90%（四律全部参与
    # 标定，含 C1——否则四律在 λ=1 处本就只差 1-2%，锚点无信息量）。
    ref_rows = {a: _budget("C1", a, 1.0)[1] for a in ANCHORS}
    targets = {a: ref_rows[a][a] for a in ANCHORS}
    targets["IAE_p"] *= 0.9
    calib_jobs = [(a, l, targets[a]) for a in ANCHORS for l in LAWS
                  if not (l == "C1" and a != "IAE_p")]

    lam_table = {}
    for a in ANCHORS:
        if a != "IAE_p":
            lam_table[(a, "C1")] = (1.0, "reference")
    try:
        ctx = mp.get_context("fork")
        with ctx.Pool(min(6, os.cpu_count() or 1)) as pool:
            for anchor, law, lam, note in pool.imap(_calib_job, calib_jobs):
                lam_table[(anchor, law)] = (lam, note)
                if lam is None:
                    print(f"[calib] {anchor:<9} {law:<5} FAILED: {note}",
                          flush=True)
                else:
                    print(f"[calib] {anchor:<9} {law:<5} λ={lam:.6f} ({note})",
                          flush=True)
    except Exception as exc:
        print(f"[warn] pool failed ({exc}), serial calibration", flush=True)
        for pack in calib_jobs:
            anchor, law, lam, note = _calib_job(pack)
            lam_table[(anchor, law)] = (lam, note)
            print(f"[calib] {anchor:<9} {law:<5} λ={lam} ({note})", flush=True)

    run_jobs = []
    for anchor in ANCHORS:
        print(f"=== anchor {anchor}: target = {targets[anchor]:.6g} "
              f"(C1 标称值的 {targets[anchor] / ref_rows[anchor][anchor]:.2f} 倍)"
              f" ===", flush=True)
        for law in LAWS:
            lam, note = lam_table[(anchor, law)]
            if lam is None:
                print(f"  {law:<5} skipped ({note})", flush=True)
                continue
            print(f"  {law:<5} λ={lam:.6f} ({note})", flush=True)
            for scenario, condition in CONDITIONS:
                run_jobs.append((anchor, law, scenario, condition, lam))

    rows = []
    try:
        ctx = mp.get_context("fork")
        with ctx.Pool(min(6, os.cpu_count() or 1)) as pool:
            for i, row in enumerate(pool.imap(_run_job, run_jobs)):
                rows.append(row)
                print(f"[{i + 1:>3}/{len(run_jobs)}] {row['anchor']:<9} "
                      f"{row['scenario']:>6}/{row['condition']:<6} "
                      f"{row['law']:<5} λ={row['scale']:.4f}"
                      f"  IAE_p={row['IAE_p']:.4g} settle={row['settle_s']}"
                      f"{'  ABORT' if row['aborted'] else ''}", flush=True)
    except Exception as exc:
        print(f"[warn] pool failed ({exc}), serial fallback", flush=True)
        for i, pack in enumerate(run_jobs):
            row = _run_job(pack)
            rows.append(row)
            print(f"[{i + 1:>3}/{len(run_jobs)}] done", flush=True)

    os.makedirs(ROOT, exist_ok=True)
    path = os.path.join(ROOT, "budget_matched.csv")
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    print("saved", path)

    lam_rows = [dict(anchor=a, law=l, lam=(lam_table[(a, l)][0] or np.nan),
                     note=lam_table[(a, l)][1]) for a in ANCHORS for l in LAWS]
    path2 = os.path.join(ROOT, "budget_matched_lambda.csv")
    with open(path2, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["anchor", "law", "lam", "note"])
        writer.writeheader()
        writer.writerows(lam_rows)
    print("saved", path2)
    _print_verdict(rows)
    return 0


def _print_verdict(rows):
    print("\n" + "=" * 88)
    print("等预算(1%)下的结果指标（相对 C1；lower=better；* = C1 最好）")
    print("=" * 88)
    by = {}
    for r in rows:
        by.setdefault((r["anchor"], r["scenario"], r["condition"]), {})[r["law"]] = r
    c1_wins = {a: {m: 0 for m in OUTCOME_METRICS} for a in ANCHORS}
    for (anchor, scenario, condition), grp in sorted(by.items()):
        print(f"\n--- anchor={anchor}  {scenario}/{condition} ---")
        hdr = f"{'metric':>14} " + "".join(f"{l:>10}" for l in ("C1", "C2", "C2abl", "C3"))
        print(hdr)
        for m in OUTCOME_METRICS:
            vals = {l: grp[l][m] for l in grp
                    if m in grp[l] and not np.isnan(grp[l][m])}
            if not vals:
                continue
            best = min(vals, key=vals.get)
            if best == "C1":
                c1_wins[anchor][m] += 1
            cells = []
            for l in ("C1", "C2", "C2abl", "C3"):
                if l not in vals:
                    cells.append(f"{'--':>10}")
                elif np.isnan(vals["C1"]) or vals["C1"] == 0:
                    cells.append(f"{vals[l]:10.4g}")
                else:
                    cells.append(f"{vals[l] / vals['C1']:10.3f}")
            star = "*" if best == "C1" else " "
            print(f"{m:>14} " + "".join(cells) + f" {star}"
                  f"   (C1={vals.get('C1', np.nan):.4g})")
    print("\n" + "=" * 88)
    print("C1 最优计数（按锚点 × 结果指标，共 5 工况）")
    print("=" * 88)
    for anchor in ANCHORS:
        line = [f"{m}:{n}" for m, n in c1_wins[anchor].items() if n]
        print(f"  {anchor:<10} -> {', '.join(line) if line else '(none)'}")


if __name__ == "__main__":
    raise SystemExit(main())
