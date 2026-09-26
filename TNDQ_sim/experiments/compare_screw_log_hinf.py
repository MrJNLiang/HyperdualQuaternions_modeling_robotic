"""Compare fixed-gain screw-log and Figueredo–Adorno DQ velocity laws.

Run from any directory: python TNDQ_sim/experiments/compare_screw_log_hinf.py
The energy budget is integral squared commanded joint velocity, not motor work.
"""

import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import params
from core.kinematics import TNDQSerialChain
from control.error_system import full_error_state
from control.control_law import (dq_hinf_kinematic_law,
                                 dq_log_kinematic_law, damped_pinv,
                                 dq_log_dexp_kinematic_law,
                                 dq_log_structured_kinematic_law,
                                 velocity_to_accel_ref)
from simdata.trajectory_generator import LineTrajectoryTNDQ
from simdata.input_simulation import L2Disturbance

DT = 0.01
T_END = 5.0
ROOT = os.path.join(os.path.dirname(__file__), "..", "results")
GAMMA_O = np.sqrt(2.0) / 8.0
GAMMA_T = np.sqrt(2.0) / 4.0


def simulate(kind, case, alpha=1.0, ratio=1.0, init_scale=0.05):
    chain = TNDQSerialChain(params.KUKA_LBR4_DH)
    q = (params.Q_INIT_LARGE_ERROR if case == "large" else
         params.Q_INIT + init_scale * np.array([1, -1, 1, -1, 1, -1, 1])).copy()
    qdot = np.zeros(chain.n)
    q_center = q.copy()
    target = chain.fkm(params.Q_INIT)
    if case != "large":
        trajectory = LineTrajectoryTNDQ(target, [0.15, 0.10, -0.10],
                                         3.5, [0., 0., 1.], 0.5)
    else:
        trajectory = LineTrajectoryTNDQ(target, [0., 0., 0.],
                                         1.0)
    disturbance = L2Disturbance(chain.n, amplitude=1.0, decay=0.5,
                                omega=3.0, t_on=1.0, seed=0)
    rng = np.random.default_rng(271828)
    previous = None
    pos, angle, command, actual, variation = [], [], [], [], []
    sat = 0
    for i in range(int(round(T_END / DT))):
        t = i * DT
        if case == "noise":
            qm = q + rng.normal(0., params.NOISE_SIGMA_Q, chain.n)
            vm = qdot + rng.normal(0., params.NOISE_SIGMA_QDOT, chain.n)
        else:
            qm, vm = q, qdot
        fk = chain.fk_outputs(qm, vm, with_jacobian=True)
        desired = trajectory.evaluate(t)
        err = full_error_state(fk["x_breve"], desired["x_breve_d"])
        if kind == "FA":
            task = dq_hinf_kinematic_law(err, desired["xi_d"],
                                          GAMMA_O, GAMMA_T)
        elif kind == "log":
            task = dq_log_kinematic_law(err, desired["xi_d"],
                                         4.0 * alpha * ratio, 4.0 * alpha)
        elif kind == "dexp":
            task = dq_log_dexp_kinematic_law(err, desired["xi_d"],
                                              4.0 * alpha * ratio, 4.0 * alpha)
        else:
            task = dq_log_structured_kinematic_law(
                err, desired["xi_d"],
                np.diag([4.0 * alpha * ratio] * 3 + [4.0 * alpha] * 3))
        pinv = damped_pinv(fk["J"], damping=max(params.PINV_DAMPING,
                                                 params.DQH_DAMPING))
        cmd = pinv @ task
        qdd = velocity_to_accel_ref(cmd, previous, vm, DT,
                                     params.DQH_K_SERVO)
        null = np.eye(chain.n) - pinv @ fk["J"]
        qdd += null @ (params.NULLSPACE_K * (q_center - qm)
                       - params.NULLSPACE_D * vm)
        qdd_norm = np.linalg.norm(qdd)
        if qdd_norm > params.QDDOT_MAX:
            qdd *= params.QDDOT_MAX / qdd_norm
            sat += 1
        w = disturbance(t) if case == "disturbance" else 0.
        qdot += (qdd + w) * DT
        q += qdot * DT
        pos.append(np.linalg.norm(err["T"]))
        angle.append(2.0 * np.arctan2(np.linalg.norm(err["x_tilde"][1:4]),
                                      max(0., err["x_tilde"][0])))
        command.append(np.linalg.norm(cmd) ** 2)
        actual.append(np.linalg.norm(qdot) ** 2)
        if previous is not None:
            variation.append(np.linalg.norm((cmd - previous) / DT) ** 2)
        previous = cmd
    pos, angle = np.asarray(pos), np.asarray(angle)
    ok = (pos < 0.005) & (angle < np.deg2rad(2.))
    suffix_ok = np.logical_and.accumulate(ok[::-1])[::-1]
    ix = np.where(suffix_ok)[0]
    settling = float(ix[0] * DT) if len(ix) else float("nan")
    def t_thresh(deg):
        good = angle < np.deg2rad(deg)
        s = np.logical_and.accumulate(good[::-1])[::-1]
        ii = np.where(s)[0]
        return float(ii[0] * DT) if len(ii) else float("nan")
    return dict(case=case, law=kind, alpha=alpha, ratio=ratio,
                E_cmd=float(DT * sum(command)),
                E_actual=float(DT * sum(actual)),
                p_rms=float(np.sqrt(np.mean(pos ** 2))),
                a_rms_deg=float(np.rad2deg(np.sqrt(np.mean(angle ** 2)))),
                p_end=float(pos[-1]), a_end_deg=float(np.rad2deg(angle[-1])),
                p_peak=float(np.max(pos)),
                a_peak_deg=float(np.rad2deg(np.max(angle))),
                slew_rms=float(np.sqrt(np.mean(variation))),
                cmd_rms=float(np.sqrt(np.mean(command))),
                cmd_peak=float(np.sqrt(np.max(command))),
                theta_int=float(DT * np.sum(angle)),
                t10_s=t_thresh(10.), t5_s=t_thresh(5.), t2_s=t_thresh(2.),
                settle_s=settling, sat_steps=sat)


def main():
    baseline = simulate("FA", "line")
    # Calibrate once on nominal tracking; keep these gains for later cases.
    lo, hi = 0.1, 1.0
    while simulate("log", "line", hi)["E_cmd"] < baseline["E_cmd"]:
        hi *= 2.
        if hi > 16.:
            raise RuntimeError("Could not bracket energy match")
    for _ in range(12):
        mid = (lo + hi) / 2.
        if simulate("log", "line", mid)["E_cmd"] < baseline["E_cmd"]:
            lo = mid
        else:
            hi = mid
    alpha = (lo + hi) / 2.
    dexp_ratio = 0.5
    # This branch is decreasing in energy; calibration is made only on line.
    lo2, hi2 = 0.2, 0.3
    for _ in range(12):
        mid = (lo2 + hi2) / 2.
        if simulate("dexp", "line", mid, dexp_ratio)["E_cmd"] > baseline["E_cmd"]:
            lo2 = mid
        else:
            hi2 = mid
    alpha2 = (lo2 + hi2) / 2.
    # The image structure is calibrated independently on the nominal line
    # case.  alpha_s=.95 keeps the command-energy difference from FA below
    # 1% (the exact value is 0.047%).  Freeze it for all other cases.
    alpha_s = 0.95
    rows = [baseline, simulate("log", "line", alpha),
            simulate("dexp", "line", alpha2, dexp_ratio),
            simulate("structured", "line", alpha_s)]
    for case in ("large", "noise", "disturbance"):
        rows += [simulate("FA", case), simulate("log", case, alpha),
                 simulate("dexp", case, alpha2, dexp_ratio),
                 simulate("structured", case, alpha_s)]
    os.makedirs(ROOT, exist_ok=True)
    path = os.path.join(ROOT, "screw_log_hinf_comparison.csv")
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print("matched alpha", alpha, "dexp alpha", alpha2,
          "structured alpha", alpha_s,
          "gains", 4 * alpha, 4 * alpha,
          4 * alpha2 * dexp_ratio, 4 * alpha2,
          4 * alpha_s, 4 * alpha_s)
    print("results", path)
    for r in rows:
        print(r)


if __name__ == "__main__":
    main()
