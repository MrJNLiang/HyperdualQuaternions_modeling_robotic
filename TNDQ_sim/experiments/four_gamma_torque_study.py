"""Finite-energy torque test and four-parameter, torque-budget comparison.

Run from the repository root:
    PYTHONDONTWRITEBYTECODE=1 python3 TNDQ_sim/experiments/four_gamma_torque_study.py

The certificate test uses the exact-matched rigid-body plant without friction.
The comparison gives C1, Chandra's C2, and a decoupled quaternion-vector PD
baseline the same four rotational/translational P/D degrees of freedom.
"""

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import params
from config.lbr4_dynamics import LBR4NominalDynamics, clip_torque, check_joint_limits
from core.dq_algebra import dq_rotation, dq_translation
from core.kinematics import TNDQSerialChain
from control.control_law import (
    damped_pinv, dq_chandra2020_law, feedforward_term,
    geometric_computed_torque_law,
)
from control.error_system import full_error_state
from simdata.trajectory_generator import LineTrajectoryTNDQ, SetpointTrajectoryTNDQ

# This import validates and enables the same 3-vector cross-product fast path
# used by the existing torque-level portfolio experiment.
from compare_metrics_portfolio import governor  # noqa: E402


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "results"))
ETA0 = 0.8
WBAR = 1.0
BASE = {"gwxi": 0.2, "gwz": 0.05, "gvxi": 0.3, "gvz": 0.1}
LAWS = ("C1", "C2", "C4")
COLORS = {"C1": "#087e8b", "C2": "#d1495b", "C4": "#6d597a"}


def gains_from_gamma(gamma, eta0=ETA0, wbar=WBAR):
    """Equation (14) of the four-Gamma derivation; gamma order is explicit."""
    gwxi, gwz, gvxi, gvz = (float(gamma[k]) for k in
                               ("gwxi", "gwz", "gvxi", "gvz"))
    if min(gwxi, gwz, gvxi, gvz) <= 0 or not 0 < eta0 < 1 or wbar <= 0:
        raise ValueError("Gamma and wbar must be positive; eta0 must lie in (0,1)")
    bw, bv = 2 / gwxi, 2 / gvxi
    po = gwxi**2 / (eta0 * gwz**2) + (1 + 1 / (2 - eta0)) / (eta0 * gwxi**2)
    pt = gvxi**2 / gvz**2 + 1 / (2 * gvxi**2) + wbar**2 / 4
    return np.array([bw, bv, po, pt], dtype=float)


def matrices(law, values):
    """Match local angular/linear acceleration stiffness and damping."""
    bw, bv, po, pt = map(float, values)
    kd = np.diag(np.r_[np.full(3, bw), np.full(3, bv)])
    if law == "C1":
        kp = np.diag(np.r_[np.full(3, po), np.full(3, pt)])
    elif law == "C2":
        # C1: angular acceleration = (po/2) O = -(po/4) angle locally.
        kp = np.diag(np.r_[np.full(3, po / 4), np.full(3, pt)])
    elif law == "C4":
        # Decoupled quaternion-vector / translation feedback, same local poles.
        kp = np.diag(np.r_[np.full(3, po / 2), np.full(3, pt)])
    else:
        raise ValueError(law)
    return kd, kp


def storage(err, values):
    """Equation (6), evaluated on the current dual-quaternion errors."""
    bw, bv, po, pt = values
    O, T, w, v = err["O"], err["T"], err["e_xi"][:3], err["e_xi"][3:]
    eta = err["x_tilde"][0]
    ww = bw * (0.5 * w @ w + 0.5 * po * O @ O + bw**2 * (1 - eta)
               - 0.5 * bw * O @ w)
    wv = bv * (0.5 * v @ v + 0.5 * (pt + bv**2 / 4) * T @ T
               + 0.25 * bv * T @ v)
    return np.array([ww, wv])


def supply_residual(err, d, values, gamma):
    """Exact continuous-time dW + weighted output - input (Eq. 8)."""
    bw, bv, po, pt = values
    O, T, w, v = err["O"], err["T"], err["e_xi"][:3], err["e_xi"][3:]
    eta = err["x_tilde"][0]
    dw = (-bw**2 * (1 - eta / 4) * (w @ w)
          - bw**2 * po * eta / 4 * (O @ O)
          + bw * (w - bw * O / 2) @ d[:3])
    dv = (-3 * bv**2 / 4 * (v @ v)
          - bv**2 * pt / 4 * (T @ T)
          - bv**2 / 4 * T @ np.cross(w, v)
          + bv * (v + bv * T / 4) @ d[3:])
    out = np.array([(O @ O) / gamma["gwz"]**2 + (w @ w) / gamma["gwxi"]**2,
                    (T @ T) / gamma["gvz"]**2 + (v @ v) / gamma["gvxi"]**2])
    return np.array([dw, dv]) + out - np.array([d[:3] @ d[:3], d[3:] @ d[3:]])


def _task_command(law, err, des, fk, kd, kp):
    J, jdot = fk["J"], fk["Jdot_qdot"]
    if law == "C1":
        return geometric_computed_torque_law(
            err, des["xi_d"], des["xi_dot_d"], J, jdot, kd, kp,
            damping=params.PINV_DAMPING)
    if law == "C2":
        return dq_chandra2020_law(
            err, des["xi_d"], des["xi_dot_d"], J, jdot, kd, kp,
            damping=params.PINV_DAMPING)
    ff = feedforward_term(err["x_tilde"], err["xi_tilde"],
                          des["xi_d"], des["xi_dot_d"])
    u = ff - kd @ err["e_xi"] + np.r_[kp[0, 0] * err["O"],
                                      -kp[3, 3] * err["T"]] - jdot
    return damped_pinv(J, params.PINV_DAMPING) @ u, u


def simulate(law, values, gamma=None, axis="rot", scenario="setpoint",
             dt=0.005, horizon=4.0, pulse=True, trace=False):
    """One matched-model rigid-body torque run, with a fixed joint torque pulse."""
    chain = TNDQSerialChain(params.KUKA_LBR4_DH)
    plant = LBR4NominalDynamics(params.KUKA_LBR4_DH, friction=False)
    q0 = params.Q_INIT.copy()
    x0 = chain.fkm(q0)
    if scenario == "setpoint":
        traj = SetpointTrajectoryTNDQ(dq_translation(x0), dq_rotation(x0))
    elif scenario == "line":
        traj = LineTrajectoryTNDQ(x0, [0.08, 0.05, -0.04], 2.0,
                                  [0, 0, 1], 0.3)
    else:
        raise ValueError(scenario)
    q, qdot = q0.copy(), np.zeros_like(q0)
    kd, kp = matrices(law, values)
    J0 = chain.jacobian(q0)
    M0 = plant.mass_matrix(q0)
    task_axis = np.zeros(6)
    if axis == "mixed":
        task_axis[:] = [0.25, 0.0, 0.35, 0.35, 0.2, -0.25]
    else:
        task_axis[2 if axis == "rot" else 3] = 0.5
    torque_direction = M0 @ damped_pinv(J0, params.PINV_DAMPING) @ task_axis
    e_out = np.zeros(2)
    e_in = np.zeros(2)
    w0 = np.zeros(2)
    max_prefix = np.full(2, -np.inf)
    max_supply = np.full(2, -np.inf)
    max_gap = 0.0
    eta_min, omega_max, sigma_min = 1.0, 0.0, np.inf
    iae_p = iae_a = iae_w = iae_v = 0.0
    e_nongrav = slew_sq = 0.0
    peak_tau = peak_ng = peak_ext = max_coupling = 0.0
    sat = qdd_sat = gov = 0
    prev_tau = None
    samples = {k: [] for k in ("t", "prefix_rot", "prefix_trans", "O", "T",
                                "omega", "v", "domega", "dv", "tau")}
    aborted = False
    steps = int(round(horizon / dt))
    for i in range(steps):
        t = i * dt
        fk = chain.fk_outputs(q, qdot, q_ddot=None, with_jacobian=True)
        des = traj.evaluate(t)
        err = full_error_state(fk["x_breve"], des["x_breve_d"])
        J = fk["J"]
        sigma_min = min(sigma_min, float(np.linalg.svd(J, compute_uv=False)[-1]))
        eta_min = min(eta_min, float(err["x_tilde"][0]))
        omega_max = max(omega_max, float(np.linalg.norm(err["e_xi"][:3])))
        qdd_ref, u_task = _task_command(law, err, des, fk, kd, kp)
        if np.linalg.norm(qdd_ref) > params.QDDOT_MAX:
            qdd_ref *= params.QDDOT_MAX / np.linalg.norm(qdd_ref)
            qdd_sat += 1
        qdd_ref, governed = governor(q, qdot, qdd_ref)
        gov += int(governed)
        M = plant.mass_matrix(q)
        h = plant.coriolis_plus_gravity(q, qdot)
        tau_cmd, limited = clip_torque(M @ qdd_ref + h)
        sat += int(limited)
        envelope = 0.0
        if pulse and 0.5 <= t < 1.5:
            envelope = np.sin(np.pi * (t - 0.5))**2
        tau_ext = envelope * torque_direction
        qdd = np.linalg.solve(M, tau_cmd + tau_ext - h)
        d_phys = J @ np.linalg.solve(M, tau_ext)
        d_total = J @ qdd - u_task
        max_gap = max(max_gap, float(np.linalg.norm(d_total - d_phys)))
        peak_ext = max(peak_ext, float(np.max(np.abs(tau_ext))))
        peak_tau = max(peak_tau, float(np.max(np.abs(tau_cmd))))
        ng = tau_cmd - plant.gravity_vector(q)
        peak_ng = max(peak_ng, float(np.max(np.abs(ng))))
        e_nongrav += float(ng @ ng) * dt
        if prev_tau is not None:
            slew_sq += float(np.sum(((tau_cmd - prev_tau) / dt)**2))
        prev_tau = tau_cmd.copy()
        O, T = np.linalg.norm(err["O"]), np.linalg.norm(err["T"])
        w, v = np.linalg.norm(err["e_xi"][:3]), np.linalg.norm(err["e_xi"][3:])
        max_coupling = max(max_coupling, abs(float(err["T"] @ np.cross(
            err["e_xi"][:3], err["e_xi"][3:]))))
        iae_p += T * dt
        iae_a += 2 * np.arctan2(O, max(err["x_tilde"][0], 0)) * dt
        iae_w += w * dt
        iae_v += v * dt
        if gamma is not None:
            if i == 0:
                w0 = storage(err, values)
            e_out += np.array([O**2 / gamma["gwz"]**2 + w**2 / gamma["gwxi"]**2,
                               T**2 / gamma["gvz"]**2 + v**2 / gamma["gvxi"]**2]) * dt
            e_in += np.array([d_total[:3] @ d_total[:3],
                              d_total[3:] @ d_total[3:]]) * dt
            max_supply = np.maximum(max_supply,
                                    supply_residual(err, d_total, values, gamma))
        qdot = qdot + qdd * dt
        q = q + qdot * dt
        if check_joint_limits(q):
            aborted = True
            break
        if gamma is not None:
            fk_next = chain.fk_outputs(q, qdot, q_ddot=None, with_jacobian=False)
            des_next = traj.evaluate(t + dt)
            err_next = full_error_state(fk_next["x_breve"], des_next["x_breve_d"])
            prefix = e_out + storage(err_next, values) - w0 - e_in
            max_prefix = np.maximum(max_prefix, prefix)
            if trace:
                for key, value in (("t", t + dt), ("prefix_rot", prefix[0]),
                                   ("prefix_trans", prefix[1]), ("O", O),
                                   ("T", T), ("omega", w), ("v", v),
                                   ("domega", np.linalg.norm(d_total[:3])),
                                   ("dv", np.linalg.norm(d_total[3:])),
                                   ("tau", peak_tau)):
                    samples[key].append(value)
    result = dict(law=law, axis=axis, scenario=scenario, steps=i + 1,
                  aborted=int(aborted), bw=values[0], bv=values[1],
                  po=values[2], pt=values[3], eta_min=eta_min,
                  omega_max=omega_max, sigma_min=sigma_min,
                  IAE_p=iae_p, IAE_a_deg=np.rad2deg(iae_a),
                  IAE_omega=iae_w, IAE_v=iae_v,
                  E_nongrav=e_nongrav, tau_peak=peak_tau,
                  tau_nongrav_peak=peak_ng, max_coupling=max_coupling,
                  tau_ext_peak=peak_ext,
                  tau_slew_rms=np.sqrt(slew_sq / max(i, 1)),
                  sat_steps=sat, qdd_sat_steps=qdd_sat, gov_steps=gov,
                  max_d_gap=max_gap)
    if gamma is not None:
        result.update(W0_rot=w0[0], W0_trans=w0[1],
                      E_out_rot=e_out[0], E_out_trans=e_out[1],
                      E_d_rot=e_in[0], E_d_trans=e_in[1],
                      max_prefix_rot=max_prefix[0], max_prefix_trans=max_prefix[1],
                      final_prefix_rot=prefix[0], final_prefix_trans=prefix[1],
                      max_supply_rot=max_supply[0], max_supply_trans=max_supply[1],
                      ratio_rot=np.sqrt(e_out[0] / e_in[0]) if e_in[0] > 1e-14 else np.nan,
                      ratio_trans=np.sqrt(e_out[1] / e_in[1]) if e_in[1] > 1e-14 else np.nan,
                      domain_ok=int(eta_min >= ETA0 and omega_max <= WBAR))
    return result, {key: np.asarray(value) for key, value in samples.items()}


def _write_csv(name, rows):
    os.makedirs(ROOT, exist_ok=True)
    path = os.path.join(ROOT, name)
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    return path


def profiles(seed=20260927, extra=8):
    rows = [("base", BASE.copy())]
    for key in BASE:
        for factor in (0.75, 1.25):
            g = BASE.copy()
            g[key] *= factor
            rows.append((f"{key}_{factor:g}", g))
    rng = np.random.default_rng(seed)
    for i in range(extra):
        g = {key: value * np.exp(rng.uniform(np.log(0.75), np.log(1.25)))
             for key, value in BASE.items()}
        rows.append((f"joint_{i:02d}", g))
    return rows


def certificate_study(dt, horizon):
    rows, traces = [], {}
    for name, gamma in profiles(extra=0):
        values = gains_from_gamma(gamma)
        for axis in ("rot", "trans", "mixed"):
            row, trace = simulate("C1", values, gamma, axis, "setpoint",
                                  dt, horizon, trace=(name == "base"))
            row.update(profile=name, **gamma)
            rows.append(row)
            if name == "base":
                traces[axis] = trace
            print(f"[certificate] {name:16s} {axis:5s} "
                  f"max_supply={max(row['max_supply_rot'], row['max_supply_trans']):.2e} "
                  f"max_prefix={max(row['max_prefix_rot'], row['max_prefix_trans']):.2e}",
                  flush=True)
    _write_csv("four_gamma_certificate.csv", rows)
    np.savez_compressed(os.path.join(ROOT, "four_gamma_certificate_traces.npz"),
                        **{f"{axis}_{key}": val for axis, trace in traces.items()
                           for key, val in trace.items()})
    fig, axes = plt.subplots(1, 3, figsize=(14, 3.6))
    for ax, axis in zip(axes, ("rot", "trans", "mixed")):
        tr = traces[axis]
        ax.plot(tr["t"], tr["prefix_rot"], label="rotation budget")
        ax.plot(tr["t"], tr["prefix_trans"], label="translation budget")
        ax.axhline(0, color="black", lw=0.8)
        ax.set(xlabel="Time (s)", ylabel="LHS + W(T) - W(0) - input energy",
               title=f"{axis} torque pulse")
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, "four_gamma_certificate.png"), dpi=170)
    plt.close(fig)
    return rows


def _independent_values(seed, n, reference):
    rng = np.random.default_rng(seed)
    values = np.array([gains_from_gamma(g) for _, g in reference])
    lo = values.min(axis=0) * 0.8
    hi = values.max(axis=0) * 1.2
    for i in range(n):
        yield f"independent_{i:02d}", np.exp(rng.uniform(np.log(lo), np.log(hi)))


def _dominates(a, b):
    keys = ("IAE_p", "IAE_a_deg", "IAE_omega", "IAE_v")
    return all(a[key] <= b[key] for key in keys) and any(a[key] < b[key] for key in keys)


def comparison_study(dt, horizon, extra):
    designs = profiles(extra=extra)
    ref_values = gains_from_gamma(BASE)
    ref, _ = simulate("C1", ref_values, axis="mixed", scenario="line",
                      dt=dt, horizon=horizon)
    energy_budget = ref["E_nongrav"] * 1.01
    peak_budget = ref["tau_nongrav_peak"] * 1.05
    rows = []
    for name, gamma in designs:
        values = gains_from_gamma(gamma)
        for law in LAWS:
            row, _ = simulate(law, values, axis="mixed", scenario="line",
                              dt=dt, horizon=horizon)
            row.update(profile=name, c1_certificate_applicable=int(law == "C1"),
                       **gamma)
            rows.append(row)
            print(f"[comparison] {law} {name:16s} E={row['E_nongrav']:.4g} "
                  f"p={row['IAE_p']:.4g} a={row['IAE_a_deg']:.4g}", flush=True)
    for name, values in _independent_values(1000, extra, designs):
        for law in LAWS:
            row, _ = simulate(law, values, axis="mixed", scenario="line",
                              dt=dt, horizon=horizon)
            row.update(profile=name, c1_certificate_applicable=0,
                       gwxi=np.nan, gwz=np.nan,
                       gvxi=np.nan, gvz=np.nan)
            rows.append(row)
            print(f"[comparison] {law} {name:16s} E={row['E_nongrav']:.4g} "
                  f"p={row['IAE_p']:.4g} a={row['IAE_a_deg']:.4g}", flush=True)
    for row in rows:
        feasible = (not row["aborted"] and row["sat_steps"] == 0
                    and row["qdd_sat_steps"] == 0 and row["gov_steps"] == 0
                    and row["E_nongrav"] <= energy_budget
                    and row["tau_nongrav_peak"] <= peak_budget)
        row.update(energy_budget=energy_budget, peak_budget=peak_budget,
                   feasible=int(feasible))
    good = [r for r in rows if r["feasible"]]
    for row in rows:
        row["pareto4"] = int(bool(row["feasible"]) and
                             not any(_dominates(other, row) for other in good
                                     if other is not row))
    _write_csv("four_gamma_tradeoff.csv", rows)
    fig, ax = plt.subplots(figsize=(6, 4.8))
    for law in LAWS:
        group = [r for r in good if r["law"] == law]
        ax.scatter([r["IAE_p"] for r in group], [r["IAE_a_deg"] for r in group],
                   label=f"{law} (n={len(group)})", s=34, alpha=0.8,
                   c=COLORS[law])
        frontier = [r for r in group if r["pareto4"]]
        ax.scatter([r["IAE_p"] for r in frontier],
                   [r["IAE_a_deg"] for r in frontier],
                   facecolors="none", edgecolors=COLORS[law], s=105, lw=1.4)
    ax.set(xlabel="Position IAE (m s)", ylabel="Orientation IAE (deg s)",
           title="Common non-gravity torque budgets; lower is better")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, "four_gamma_tradeoff.png"), dpi=170)
    plt.close(fig)
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dt", type=float, default=0.005)
    parser.add_argument("--horizon", type=float, default=4.0)
    parser.add_argument("--extra", type=int, default=8,
                        help="additional joint four-parameter settings per law")
    args = parser.parse_args()
    if args.dt <= 0 or args.horizon <= 1.5 or args.extra < 0:
        parser.error("dt > 0, horizon > 1.5 and extra >= 0 are required")
    cert = certificate_study(args.dt, args.horizon)
    comp = comparison_study(args.dt, args.horizon, args.extra)
    good = [r for r in comp if r["feasible"]]
    print(f"certificate rows={len(cert)}; comparison rows={len(comp)}; "
          f"feasible={len(good)}")
    print("certificate max continuous residual:",
          max(max(r["max_supply_rot"], r["max_supply_trans"]) for r in cert))
    print("certificate max discrete prefix residual:",
          max(max(r["max_prefix_rot"], r["max_prefix_trans"]) for r in cert))


if __name__ == "__main__":
    main()
