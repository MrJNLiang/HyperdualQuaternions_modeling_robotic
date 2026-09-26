"""Matched-speed FA versus screw-log H-infinity-style velocity feedback.

Plant: q[k+1] = q[k] + dt*(qdot_command + J_pinv*(w_external+w_control)).
The two injected task-twist inputs are kept separate in the recorded data.
"""

import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import params
from control.control_law import (
    damped_pinv, dq_hinf_kinematic_law, dq_log_hinf_kinematic_law,
)
from control.error_system import full_error_state
from core.dq_algebra import dq_log2_vec6
from core.kinematics import TNDQSerialChain
from simdata.trajectory_generator import LineTrajectoryTNDQ


DT = 0.01
T_END = 5.0
SCALES = (0.05, 0.08, 0.12)
RATIOS = (0.5, 1.0, 2.0)  # gamma_H / gamma_R
GAMMA_O, GAMMA_T = np.sqrt(2.0) / 8.0, np.sqrt(2.0) / 4.0
FIXED_GAMMA_R, FIXED_GAMMA_H = 0.300, 0.255
ROOT = os.path.join(os.path.dirname(__file__), "..", "results")


def disturbances(t, source):
    external = np.zeros(6)
    control = np.zeros(6)
    if source in ("external", "both") and 0.4 <= t <= 2.4:
        envelope = np.sin(np.pi * (t - 0.4) / 2.0) ** 2
        external = envelope * np.array([0.06, -0.04, 0.05, 0.015, -0.010, 0.012])
    if source in ("control", "both") and 0.8 <= t <= 2.8:
        envelope = np.sin(np.pi * (t - 0.8) / 2.0) ** 2
        carrier = np.sin(2.0 * np.pi * 1.5 * (t - 0.8))
        control = envelope * carrier * np.array([0.035, 0.025, -0.030, -0.009, 0.008, 0.006])
    return external, control


def sustained_time(values, limit):
    below = values < limit
    held = np.logical_and.accumulate(below[::-1])[::-1]
    indices = np.flatnonzero(held)
    return float(indices[0] * DT) if len(indices) else float("nan")


def simulate(law, scale, gamma_r=None, gamma_h=None, source="none", stationary=False,
             delta_p=(.15, .10, -.10), rotation=.5):
    chain = TNDQSerialChain(params.KUKA_LBR4_DH)
    q = params.Q_INIT.copy()
    if not stationary:
        q += scale * np.array([1, -1, 1, -1, 1, -1, 1])
    target = chain.fkm(params.Q_INIT)
    if stationary:
        traj = LineTrajectoryTNDQ(target, [0, 0, 0], 3.5)
    else:
        traj = LineTrajectoryTNDQ(target, delta_p, 3.5, [0, 0, 1], rotation)
    position, angle, log_r, log_h = [], [], [], []
    command, actual, external, control = [], [], [], []

    for step in range(round(T_END / DT)):
        t = step * DT
        fk = chain.fk_outputs(q, np.zeros(chain.n), with_jacobian=True)
        desired = traj.evaluate(t)
        err = full_error_state(fk["x_breve"], desired["x_breve_d"])
        if law == "FA":
            task = dq_hinf_kinematic_law(err, desired["xi_d"], GAMMA_O, GAMMA_T)
        else:
            task = dq_log_hinf_kinematic_law(
                err, desired["xi_d"], gamma_r, gamma_h)
        pinv = damped_pinv(
            fk["J"], damping=max(params.PINV_DAMPING, params.DQH_DAMPING))
        qdot_command = pinv @ task
        w_external, w_control = disturbances(t, source)
        qdot_actual = qdot_command + pinv @ w_external + pinv @ w_control
        ell = dq_log2_vec6(err["x_tilde"])
        rotation = err["x_tilde"][:4]
        position.append(np.linalg.norm(err["T"]))
        angle.append(2.0 * np.arctan2(np.linalg.norm(rotation[1:]), rotation[0]))
        log_r.append(ell[:3])
        log_h.append(ell[3:])
        command.append(qdot_command)
        actual.append(qdot_actual)
        external.append(w_external)
        control.append(w_control)
        q += DT * qdot_actual

    position, angle = np.asarray(position), np.asarray(angle)
    log_r, log_h = np.asarray(log_r), np.asarray(log_h)
    command, actual = np.asarray(command), np.asarray(actual)
    external, control = np.asarray(external), np.asarray(control)
    slew = np.diff(command, axis=0) / DT
    norms = np.linalg.norm(command, axis=1)
    metrics = dict(
        law=law, scale=scale, source=source, stationary=int(stationary),
        gamma_R=gamma_r if law == "log" else GAMMA_O,
        gamma_H=gamma_h if law == "log" else GAMMA_T,
        ratio_H_R=gamma_h / gamma_r if law == "log" else GAMMA_T / GAMMA_O,
        qdot_rms=float(np.sqrt(np.mean(norms ** 2))),
        qdot_peak=float(np.max(norms)),
        joint_peak=float(np.max(np.abs(command))),
        input_energy=float(DT * np.sum(norms ** 2)),
        actual_qdot_rms=float(np.sqrt(np.mean(np.sum(actual ** 2, axis=1)))),
        position_rms_m=float(np.sqrt(np.mean(position ** 2))),
        position_integral_m_s=float(DT * np.sum(position)),
        position_end_m=float(position[-1]),
        angle_rms_deg=float(np.rad2deg(np.sqrt(np.mean(angle ** 2)))),
        angle_integral_deg_s=float(np.rad2deg(DT * np.sum(angle))),
        angle_end_deg=float(np.rad2deg(angle[-1])),
        t5_s=sustained_time(angle, np.deg2rad(5)),
        t2_s=sustained_time(angle, np.deg2rad(2)),
        slew_rms=float(np.sqrt(np.mean(slew ** 2))),
        slew_peak=float(np.max(np.abs(slew))),
        log_R_L2=float(np.sqrt(DT * np.sum(log_r ** 2))),
        log_H_L2=float(np.sqrt(DT * np.sum(log_h ** 2))),
        external_R_L2=float(np.sqrt(DT * np.sum(external[:, :3] ** 2))),
        external_H_L2=float(np.sqrt(DT * np.sum(external[:, 3:] ** 2))),
        control_R_L2=float(np.sqrt(DT * np.sum(control[:, :3] ** 2))),
        control_H_L2=float(np.sqrt(DT * np.sum(control[:, 3:] ** 2))),
    )
    trace = dict(t=np.arange(len(position)) * DT, position=position,
                 angle_deg=np.rad2deg(angle), command_norm=norms,
                 joint_slew_norm=np.r_[0.0, np.linalg.norm(slew, axis=1)],
                 log_r=log_r, log_h=log_h)
    return metrics, trace


def match_rms(scale, ratio, target):
    # The RMS response need not be globally monotone.  Take the first
    # crossing from stronger to weaker feedback; reject ratios with no match.
    grid = np.geomspace(0.08, 8.0, 13)
    previous_gamma, previous_rms = None, None
    for gamma in grid:
        rms = simulate("log", scale, gamma, ratio * gamma)[0]["qdot_rms"]
        if previous_gamma is not None and previous_rms >= target >= rms:
            lo, hi = previous_gamma, gamma
            for _ in range(8):
                mid = (lo + hi) / 2.0
                value = simulate("log", scale, mid, ratio * mid)[0]["qdot_rms"]
                if value > target:
                    lo = mid
                else:
                    hi = mid
            return (lo + hi) / 2.0
        previous_gamma, previous_rms = gamma, rms
    return None


def write_csv(path, rows):
    with open(path, "w", newline="") as handle:
        fields = list(dict.fromkeys(key for row in rows for key in row))
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    os.makedirs(ROOT, exist_ok=True)
    nominal = []
    matched = {}
    matching = []
    for scale in SCALES:
        base, _ = simulate("FA", scale)
        base["rms_difference_pct"] = 0.0
        nominal.append(base)
        print("FA", scale, base["qdot_rms"], flush=True)
        for ratio in RATIOS:
            gamma = match_rms(scale, ratio, base["qdot_rms"])
            if gamma is None:
                matching.append(dict(scale=scale, ratio_H_R=ratio,
                                     status="NO_MATCH", gamma_R="", gamma_H="",
                                     rms_difference_pct=""))
                print("NO_MATCH", scale, ratio, flush=True)
                continue
            result, _ = simulate("log", scale, gamma, ratio * gamma)
            result["rms_difference_pct"] = 100.0 * (
                result["qdot_rms"] / base["qdot_rms"] - 1.0)
            if abs(result["rms_difference_pct"]) > 1.0:
                matching.append(dict(scale=scale, ratio_H_R=ratio,
                                     status="REJECT_RMS", gamma_R=gamma,
                                     gamma_H=ratio * gamma,
                                     rms_difference_pct=result["rms_difference_pct"]))
                print("REJECT_RMS", scale, ratio, result["rms_difference_pct"], flush=True)
                continue
            nominal.append(result)
            matched[(scale, ratio)] = (gamma, ratio * gamma)
            matching.append(dict(scale=scale, ratio_H_R=ratio,
                                 status="MATCH", gamma_R=gamma,
                                 gamma_H=ratio * gamma,
                                 rms_difference_pct=result["rms_difference_pct"]))
            print("MATCH", scale, ratio, gamma,
                  result["rms_difference_pct"], flush=True)

    fixed_rows = []
    for scale in SCALES:
        base, _ = simulate("FA", scale)
        candidate, _ = simulate(
            "log", scale, FIXED_GAMMA_R, FIXED_GAMMA_H)
        base["rms_difference_pct"] = 0.0
        candidate["rms_difference_pct"] = 100.0 * (
            candidate["qdot_rms"] / base["qdot_rms"] - 1.0)
        fixed_rows.extend((base, candidate))
        print("FIXED", scale, candidate["rms_difference_pct"], flush=True)

    # Fixed gains are reused in every disturbance run.
    disturbance_rows = []
    traces = {}
    for source in ("none", "external", "control", "both"):
        for law in ("FA", "log"):
            result, trace = simulate(
                law, 0.0, FIXED_GAMMA_R, FIXED_GAMMA_H,
                source, stationary=True)
            disturbance_rows.append(result)
            traces[(source, law)] = trace
            print("DISTURBANCE", source, law,
                  result["angle_integral_deg_s"],
                  result["position_integral_m_s"], flush=True)

    write_csv(os.path.join(ROOT, "direct_velocity_log_hinf_nominal.csv"), nominal)
    write_csv(os.path.join(ROOT, "direct_velocity_log_hinf_matching.csv"), matching)
    write_csv(os.path.join(ROOT, "direct_velocity_log_hinf_fixed.csv"), fixed_rows)
    validation = []
    for scale in (.06, .10):
        for law in ("FA", "log"):
            row, _ = simulate(law, scale, FIXED_GAMMA_R, FIXED_GAMMA_H,
                              delta_p=(.08, -.12, .06), rotation=-.35)
            validation.append(row)
    for scale in (.06, .10):
        base = next(r for r in validation if r["scale"] == scale and r["law"] == "FA")
        candidate = next(r for r in validation if r["scale"] == scale and r["law"] == "log")
        base["rms_difference_pct"] = 0.0
        candidate["rms_difference_pct"] = 100.0 * (
            candidate["qdot_rms"] / base["qdot_rms"] - 1.0)
        print("VALIDATION", scale, candidate["rms_difference_pct"], flush=True)
    write_csv(os.path.join(ROOT, "direct_velocity_log_hinf_validation.csv"), validation)
    write_csv(os.path.join(ROOT, "direct_velocity_log_hinf_disturbance.csv"),
              disturbance_rows)
    plot_results(nominal, fixed_rows, traces)


def plot_results(nominal, fixed_rows, traces):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    for scale, ax in zip(SCALES, axs.flat[:3]):
        rows = [r for r in nominal if r["scale"] == scale]
        labels = ["FA" if r["law"] == "FA" else f'log rho={r["ratio_H_R"]:g}'
                  for r in rows]
        values = [r["angle_integral_deg_s"] for r in rows]
        ax.bar(labels, values, color=["#4c6474"] + ["#b95f49"] * (len(rows) - 1))
        ax.set_title(f"Initial joint offset scale {scale:g}")
        ax.set_ylabel("Angle error integral (deg s)")
        ax.tick_params(axis="x", labelrotation=20)
    ax = axs.flat[3]
    for law, color in (("FA", "#4c6474"), ("log", "#b95f49")):
        trace = traces[("both", law)]
        ax.plot(trace["t"], trace["angle_deg"], label=law, color=color)
    ax.set_title("Two physical twist disturbances, zero initial error")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle error (deg)")
    ax.legend()
    fig.savefig(os.path.join(ROOT, "direct_velocity_log_hinf_comparison.png"), dpi=160)
    plt.close(fig)

    fig, axs = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    keys = ("position_integral_m_s", "angle_integral_deg_s", "qdot_peak", "slew_rms")
    units = ("m s", "deg s", "rad/s", "rad/s^2")
    for ax, key, unit in zip(axs.flat, keys, units):
        x = np.arange(len(SCALES))
        fa = [next(r[key] for r in fixed_rows if r["scale"] == s and r["law"] == "FA")
              for s in SCALES]
        log = [next(r[key] for r in fixed_rows if r["scale"] == s and r["law"] == "log")
               for s in SCALES]
        ax.bar(x - .18, fa, .36, label="FA", color="#4c6474")
        ax.bar(x + .18, log, .36, label="log", color="#b95f49")
        ax.set_xticks(x, ("12.5 deg", "20.0 deg", "30.2 deg"))
        ax.set_title(key.replace("_", " "))
        ax.set_ylabel(unit)
    axs.flat[0].legend()
    fig.savefig(os.path.join(ROOT, "direct_velocity_log_hinf_fixed.png"), dpi=160)
    plt.close(fig)

    fig, axs = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    for source, style in (("external", "-"), ("control", "--"), ("both", ":")):
        for law, color in (("FA", "#4c6474"), ("log", "#b95f49")):
            trace = traces[(source, law)]
            label = f"{law} {source}"
            for ax, key in zip(axs.flat, ("position", "angle_deg", "command_norm")):
                ax.plot(trace["t"], trace[key], style, color=color, label=label)
            axs.flat[3].plot(trace["t"], trace["joint_slew_norm"],
                             style, color=color, label=label)
    for ax, title, unit in zip(axs.flat,
                               ("Position error", "Orientation error", "Command norm", "Joint command slew"),
                               ("m", "deg", "rad/s", "rad/s^2")):
        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(unit)
    axs.flat[0].legend(fontsize=8, ncol=2)
    fig.savefig(os.path.join(ROOT, "direct_velocity_log_hinf_disturbance.png"), dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
