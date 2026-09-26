"""Non-ideal torque-level comparison of FA and screw-log velocity laws.

The plant has RNEA rigid-body dynamics, gravity compensation, actuator torque
limits, a finite-bandwidth joint velocity servo, and object-side viscous plus
smoothed Coulomb friction. Friction coefficients are sensitivity assumptions,
not identified KUKA specifications.
"""

import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import params
from config.lbr4_dynamics import (
    LBR4NominalDynamics, LBR4_VISCOUS_FRICTION, LBR4_COULOMB_FRICTION,
    clip_torque, check_joint_limits,
)
from core.kinematics import TNDQSerialChain
from control.error_system import full_error_state
from control.control_law import dq_hinf_kinematic_law, dq_log_kinematic_law, damped_pinv
from simdata.trajectory_generator import LineTrajectoryTNDQ

DT = 0.01
T_END = 5.0
SERVO_GAIN = np.array([65., 65., 48., 40., 22., 14., 12.])
FRICTION_LEVELS = (0.5, 1.0, 2.0)
ROOT = os.path.join(os.path.dirname(__file__), "..", "results")
GAMMA_O = np.sqrt(2.0) / 8.0
GAMMA_T = np.sqrt(2.0) / 4.0


def simulate(kind, case="nominal", alpha=1.0, friction_scale=1.0):
    chain = TNDQSerialChain(params.KUKA_LBR4_DH)
    q = (params.Q_INIT + 0.05 * np.array([1, -1, 1, -1, 1, -1, 1])).copy()
    qdot = np.zeros(chain.n)
    q_center = q.copy()
    trajectory = LineTrajectoryTNDQ(
        chain.fkm(params.Q_INIT), [0.15, 0.10, -0.10], 3.5, [0., 0., 1.], 0.5)
    plant = LBR4NominalDynamics(
        params.KUKA_LBR4_DH, friction=True,
        viscous_friction=friction_scale * LBR4_VISCOUS_FRICTION,
        coulomb_friction=friction_scale * LBR4_COULOMB_FRICTION)
    rng = np.random.default_rng(271828)
    tau_sq, servo_sq, tau_slew, mech_pos, mech_neg, cmd_slew, cmd_energy = [], [], [], [], [], [], []
    pos_err, rot_err, tau_peak = [], [], np.zeros(chain.n)
    prev_cmd = None
    prev_tau = None
    sat_count = 0
    steps = int(round(T_END / DT))

    for i in range(steps):
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
            task = dq_hinf_kinematic_law(err, desired["xi_d"], GAMMA_O, GAMMA_T)
        else:
            task = dq_log_kinematic_law(err, desired["xi_d"], 4.0 * alpha, 4.0 * alpha)

        pinv = damped_pinv(fk["J"], damping=max(params.PINV_DAMPING,
                                                 params.DQH_DAMPING))
        cmd = pinv @ task
        null = np.eye(chain.n) - pinv @ fk["J"]
        cmd += null @ (params.NULLSPACE_K * (q_center - qm) - params.NULLSPACE_D * vm)
        cmd = np.clip(cmd, -params.QDOT_MAX, params.QDOT_MAX)
        cmd_energy.append(float(cmd @ cmd))

        # Same gravity-compensated, finite-bandwidth velocity servo for both laws.
        tau_servo = SERVO_GAIN * (cmd - vm)
        tau_raw = plant.gravity_vector(qm) + tau_servo
        tau, saturated = clip_torque(tau_raw)
        sat_count += int(saturated)
        tau_peak = np.maximum(tau_peak, np.abs(tau))
        tau_ext = np.zeros(chain.n)
        if case == "disturbance" and 1.0 <= t <= 3.0:
            tau_ext = 1.5 * np.sin(2.0 * np.pi * 1.7 * t + np.arange(chain.n))
        qddot = plant.forward_dynamics(q, qdot, tau + tau_ext)
        qdot += qddot * DT
        q += qdot * DT

        tau_sq.append(float(tau @ tau))
        servo_sq.append(float(tau_servo @ tau_servo))
        if prev_tau is not None:
            tau_slew.append(float(np.sum(((tau - prev_tau) / DT) ** 2)))
        prev_tau = tau.copy()
        power = float(tau @ qdot)
        mech_pos.append(max(0.0, power))
        mech_neg.append(max(0.0, -power))
        if prev_cmd is not None:
            cmd_slew.append(float(np.sum(((cmd - prev_cmd) / DT) ** 2)))
        prev_cmd = cmd.copy()
        pos_err.append(float(np.linalg.norm(err["T"])))
        rot_err.append(float(2.0 * np.arctan2(
            np.linalg.norm(err["x_tilde"][1:4]), max(0., err["x_tilde"][0]))))
        if check_joint_limits(q):
            break

    pos_err, rot_err = np.asarray(pos_err), np.asarray(rot_err)
    # E_qdot_cmd is the primary fairness metric for a velocity-level
    # controller. Torque and mechanical-work quantities are secondary
    # actuator-load diagnostics.
    return dict(case=case, law=kind, friction_scale=friction_scale, alpha=alpha,
                steps=len(pos_err), E_tau2=DT * sum(tau_sq),
                E_servo2=DT * sum(servo_sq),
                E_qdot_cmd=DT * sum(cmd_energy),
                E_mech_pos=DT * sum(mech_pos), E_mech_neg=DT * sum(mech_neg),
                p_rms=float(np.sqrt(np.mean(pos_err ** 2))),
                rot_rms_deg=float(np.rad2deg(np.sqrt(np.mean(rot_err ** 2)))),
                p_peak=float(np.max(pos_err)),
                rot_peak_deg=float(np.rad2deg(np.max(rot_err))),
                p_end=float(pos_err[-1]), rot_end_deg=float(np.rad2deg(rot_err[-1])),
                cmd_slew_rms=float(np.sqrt(np.mean(cmd_slew))) if cmd_slew else 0.,
                tau_slew_rms=float(np.sqrt(np.mean(tau_slew))) if tau_slew else 0.,
                tau_peak_max=float(np.max(tau_peak)), saturated_steps=sat_count,
                settled=bool(pos_err[-1] < 0.01 and rot_err[-1] < np.deg2rad(3.)))


def main():
    base = simulate("FA")
    lo, hi = 0.05, 2.0
    for _ in range(10):
        mid = 0.5 * (lo + hi)
        if simulate("log", alpha=mid)["E_qdot_cmd"] < base["E_qdot_cmd"]:
            lo = mid
        else:
            hi = mid
    alpha = 0.5 * (lo + hi)
    matched = simulate("log", alpha=alpha)
    energy_gap = abs(matched["E_qdot_cmd"] / base["E_qdot_cmd"] - 1.0)
    rows = [base, matched]
    for case in ("noise", "disturbance"):
        rows.extend([simulate("FA", case=case),
                     simulate("log", case=case, alpha=alpha)])
    for scale in FRICTION_LEVELS:
        rows.extend([simulate("FA", friction_scale=scale),
                     simulate("log", alpha=alpha, friction_scale=scale)])
    os.makedirs(ROOT, exist_ok=True)
    path = os.path.join(ROOT, "screw_log_torque_friction.csv")
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"log alpha={alpha:.6f}; primary velocity-input energy gap="
          f"{energy_gap:.3%} (target <= 1.000%)")
    if energy_gap > 0.01:
        print("WARNING: velocity-input energy matching is outside the 1% "
              "fairness band; do not use this run for performance claims.")
    print("assumed friction Bv=", LBR4_VISCOUS_FRICTION.tolist(),
          "Tc=", LBR4_COULOMB_FRICTION.tolist(), "scales=", FRICTION_LEVELS)
    print("results", path)
    for row in rows:
        print(row)


if __name__ == "__main__":
    main()
