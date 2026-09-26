"""Torque-level fair comparisons with independent 1% metric matching.

This experiment deliberately does not use the paper's linearized channel
matching.  C1 uses the original ``base`` gain set.  For each fairness anchor,
each competing law gets its own scalar feedback-gain scale selected from
closed-loop measurements so that the anchor differs from C1 by at most 1%.
The selected scale is then frozen and evaluated under the full condition
matrix.

Run from the repository root:
    python3 TNDQ_sim/experiments/compare_torque_fair_1pct.py
"""

import csv
import json
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from compare_metrics_portfolio import LAWS, ROOT, simulate  # noqa: E402


C1_GAIN_SET = "base"
CAL_T_END = 10.0
FULL_T_END = 10.0
TOL = 0.01

# Lower is better for every anchor.  The scale search is empirical; no
# monotonicity or linearized error model is assumed.
ANCHORS = (
    "E_nongrav",
    "tau_slew_rms",
    "tau_peak",
    "IAE_p",
    "IAE_a_deg",
)

CONDITIONS = (
    ("circle", "none"),
    ("circle", "noise"),
    ("circle", "mismatch"),
    ("circle", "friction2"),
    ("circle", "contact"),
    ("line", "none"),
)

# Broad enough to cover both low-gain C1/base behavior and high-gain laws.
# Full-horizon logarithmic grid.  The lower end is needed for error anchors
# because C1/base is deliberately much slower than the resolved-acceleration
# baselines; the upper end is needed for actuator-effort anchors.
CAL_SCALES = np.geomspace(0.003, 10.0, 31)

OUTCOME_METRICS = (
    "IAE_p", "IAE_a_deg", "IAE_p_ss", "a_rms_ss_deg", "p_peak",
    "a_peak_deg", "E_nongrav", "E_tau2", "E_qddot", "tau_peak",
    "tau_slew_rms", "sat_frac", "gov_frac", "p_end", "a_end_deg",
)


def _sim_job(job):
    if len(job) == 5:
        law, scenario, condition, scale, t_end = job
        anchor = None
    else:
        law, scenario, condition, scale, t_end, anchor = job
    row = simulate(law, scenario, condition, scale=scale, t_end=t_end,
                   c1_gain_set=C1_GAIN_SET)
    if anchor is not None:
        row["anchor"] = anchor
    return row


def _run_jobs(jobs, label):
    """Run independent simulations in parallel, preserving deterministic rows."""
    if not jobs:
        return []
    nproc = min(12, os.cpu_count() or 1)
    rows = []
    ctx = mp.get_context("fork")
    with ctx.Pool(nproc) as pool:
        for idx, row in enumerate(pool.imap(_sim_job, jobs), 1):
            rows.append(row)
            if idx == 1 or idx == len(jobs) or idx % max(1, len(jobs) // 8) == 0:
                print(f"[{label}] {idx}/{len(jobs)} {row['law']} "
                      f"{row['scenario']}/{row['condition']} "
                      f"scale={row['scale']:.6g}", flush=True)
    return rows


def _rel_error(value, target):
    if not np.isfinite(value) or not np.isfinite(target):
        return np.inf
    return abs(value / target - 1.0)


def _write_csv(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    keys = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def _read_numeric_csv(path):
    """Read a previous full-horizon calibration without trusting strings."""
    rows = []
    with open(path, newline="") as stream:
        for raw in csv.DictReader(stream):
            row = {}
            for key, value in raw.items():
                if value == "":
                    row[key] = np.nan
                else:
                    try:
                        row[key] = float(value)
                    except (TypeError, ValueError):
                        row[key] = value
            rows.append(row)
    return rows


def main():
    # C1 reference values are measured once using the original paper/base
    # gains.  This is the only reference; no pole/DC-gain equalization enters.
    ref = simulate("C1", "circle", "none", scale=1.0, t_end=FULL_T_END,
                   c1_gain_set=C1_GAIN_SET)
    targets = {anchor: float(ref[anchor]) for anchor in ANCHORS}
    print("C1 base reference:", json.dumps(targets, ensure_ascii=False))

    # Stage 1: full-horizon empirical screening.  This avoids treating a
    # short transient as a proxy for the requested fairness metric.
    calibration_path = os.path.join(ROOT, "torque_fair_1pct_calibration.csv")
    cached = []
    if os.path.exists(calibration_path):
        try:
            cached = _read_numeric_csv(calibration_path)
            cached = [row for row in cached if float(row.get("steps", 0)) > 0]
        except (OSError, ValueError, KeyError):
            cached = []
    expected_cached = len([law for law in LAWS if law != "C1"]) * len(CAL_SCALES)
    if len(cached) >= expected_cached and all(
            abs(float(row.get("scale", -1))) > 0.0 for row in cached):
        cal_rows = cached
        print(f"[cal] reusing {len(cal_rows)} cached full-horizon rows", flush=True)
    else:
        cal_jobs = [
            (law, "circle", "none", float(scale), CAL_T_END)
            for law in LAWS if law != "C1" for scale in CAL_SCALES
        ]
        cal_rows = _run_jobs(cal_jobs, "cal")
    by_law = {law: [] for law in LAWS if law != "C1"}
    for row in cal_rows:
        by_law[row["law"]].append(row)

    initial = {}
    for anchor in ANCHORS:
        for law in by_law:
            best = min(by_law[law], key=lambda row: _rel_error(row[anchor], targets[anchor]))
            initial[(anchor, law)] = float(best["scale"])

    # Refine each bracket on the full horizon.  Interpolation is used only to
    # propose the next measured scale; the accepted row is always a real
    # simulation, and no interpolated metric is reported.  Refinement rounds
    # are parallel because each proposal is independent.
    for _round in range(5):
        jobs = []
        job_keys = []
        for anchor in ANCHORS:
            for law in by_law:
                rows = by_law[law]
                target = targets[anchor]
                ordered = sorted(rows, key=lambda row: float(row["scale"]))
                pairs = []
                for left, right in zip(ordered[:-1], ordered[1:]):
                    yl = float(left[anchor]) - target
                    yr = float(right[anchor]) - target
                    if np.isfinite(yl) and np.isfinite(yr) and yl * yr <= 0.0:
                        pairs.append((abs(np.log(float(right["scale"]) /
                                           float(left["scale"]))), left, right))
                if not pairs:
                    continue
                _, left, right = min(pairs, key=lambda item: item[0])
                yl = float(left[anchor])
                yr = float(right[anchor])
                if abs(yr - yl) < 1e-14:
                    continue
                frac = np.clip((target - yl) / (yr - yl), 0.05, 0.95)
                log_left = np.log(float(left["scale"]))
                log_right = np.log(float(right["scale"]))
                scale = float(np.exp(log_left + frac * (log_right - log_left)))
                if any(abs(float(row["scale"]) - scale) < 1e-8 for row in rows):
                    continue
                key = (law, round(scale, 10))
                if key in job_keys:
                    continue
                job_keys.append(key)
                jobs.append((law, "circle", "none", scale, FULL_T_END))
        if not jobs:
            break
        new_rows = _run_jobs(jobs, f"refine{_round + 1}")
        for row in new_rows:
            by_law[row["law"]].append(row)

    selected = {}
    for anchor in ANCHORS:
        for law in by_law:
            selected[(anchor, law)] = min(
                by_law[law], key=lambda row: _rel_error(row[anchor], targets[anchor]))

    # C1 and all selected competitor scales are now frozen.  Evaluate the
    # actual comparison matrix.  The nominal circle row is already available.
    run_jobs = []
    for anchor in ANCHORS:
        # C1 reference row for this anchor and every condition.
        run_jobs.extend(("C1", scenario, condition, 1.0, FULL_T_END, anchor)
                        for scenario, condition in CONDITIONS)
        for law in by_law:
            scale = float(selected[(anchor, law)]["scale"])
            run_jobs.extend((law, scenario, condition, scale, FULL_T_END, anchor)
                            for scenario, condition in CONDITIONS)
    rows = _run_jobs(run_jobs, "matrix")
    for row in rows:
        target = targets[row["anchor"]]
        row["anchor_target"] = target
        row["anchor_rel_error"] = float(row[row["anchor"]] / target - 1.0)
        row["anchor_match_1pct"] = int(abs(row["anchor_rel_error"]) <= TOL)

    # Keep a compact parameter table with the full-horizon nominal match.
    match_rows = []
    for anchor in ANCHORS:
        match_rows.append(dict(anchor=anchor, law="C1", scale=1.0,
                               target=targets[anchor], value=targets[anchor],
                               rel_error=0.0, match_1pct=1))
        for law in by_law:
            row = selected[(anchor, law)]
            match_rows.append(dict(
                anchor=anchor, law=law, scale=float(row["scale"]),
                target=targets[anchor], value=float(row[anchor]),
                rel_error=float(row[anchor] / targets[anchor] - 1.0),
                match_1pct=int(_rel_error(row[anchor], targets[anchor]) <= TOL),
            ))

    _write_csv(os.path.join(ROOT, "torque_fair_1pct_matrix.csv"), rows)
    _write_csv(os.path.join(ROOT, "torque_fair_1pct_matches.csv"), match_rows)
    _write_csv(os.path.join(ROOT, "torque_fair_1pct_calibration.csv"), cal_rows)
    with open(os.path.join(ROOT, "torque_fair_1pct_meta.json"), "w") as stream:
        json.dump({"c1_gain_set": C1_GAIN_SET, "cal_t_end": CAL_T_END,
                   "full_t_end": FULL_T_END, "tol": TOL,
                   "anchors": ANCHORS, "conditions": CONDITIONS,
                   "targets": targets}, stream, indent=2)

    print("saved torque_fair_1pct_{matrix,matches,calibration}.csv")
    print("\nFull-horizon nominal matches:")
    for item in match_rows:
        print(f"  {item['anchor']:<13} {item['law']:<5} "
              f"scale={item['scale']:.6g} "
              f"relative={item['rel_error']:+.3%} "
              f"{'OK' if item['match_1pct'] else 'OUTSIDE 1%'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
