#!/usr/bin/env python3
"""稳态腕部失修诊断 —— qddot_ref 巨大但 τ≈0 的根因。

exp1_slant_v4_dither7 稳态（t>7s）：
    pos_err=0.023 m  ori_err=0.30 rad
    qddot_ref_norm≈23.5（持续巨大）但 tau5=0.004、tau6=-0.005（≈0），
    meas==cmd（力矩通道正常），qd5/qd6≈0（腕不动）。
矛盾点：τ = M̂ q̈_ref + Ĉ q̇ + ĝ，M̂ SPD，‖q̈_ref‖=23.5 应有大 τ，
但 CSV 记 ‖τ‖≈6（≈重力水平）。逐步离线复算定位抵消/记录错位：

  [A] 取 CSV t=8s 行 q/qd，复算 FK / err / 控制律 / τ，打印各量；
  [B] 打印 qddot_ref 逐分量、M̂ qddot_ref 逐分量、g、τ 逐分量；
  [C] 打印 J 的奇异值与 J^# 各行范数（腕列是否被阻尼吃掉）。

运行：
    /home/liang/miniconda3/envs/dq_hinf/bin/python \
        TNDQ_b601/experiments/diag_wrist_stall.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    B601_BASE_PREFIX, B601_DH_TABLE, DEFAULT_GAIN_SET, GAIN_SETS,
    PINV_DAMPING, QDDOT_MAX, R_TOOL_QUAT, SETPOINT_POS,
)
from config.b601_dynamics import B601NominalDynamics
from control.control_law import geometric_computed_torque_law
from control.error_system import full_error_state
from core.dq_algebra import dq_rotation, dq_translation
from experiments.run_lib import B601TCPChain, w2dh
from simdata.trajectory_generator import goto_trajectory

CSV = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "results", "exp1_setpoint.csv")


def main():
    chain = B601TCPChain(B601_DH_TABLE)
    dyn = B601NominalDynamics()
    gains = GAIN_SETS[DEFAULT_GAIN_SET]
    K_d, k_p = gains["K_d"], gains["k_p"]

    hdr = open(CSV).readline().strip().split(",")
    col = {c: i for i, c in enumerate(hdr)}
    d = np.loadtxt(CSV, delimiter=",", skiprows=1)
    i = int(np.argmin(np.abs(d[:, col["t"]] - 8.0)))
    q = d[i, [col[f"q{j + 1}"] for j in range(6)]]
    # 腕速度按差分（CSV 无 qd 列）
    dt = d[i, col["t"]] - d[i - 1, col["t"]]
    qd = (d[i, [col[f"q{j + 1}"] for j in range(6)]]
          - d[i - 1, [col[f"q{j + 1}"] for j in range(6)]]) / dt
    print(f"[取点] t={d[i, col['t']]:.2f}s  q={np.round(q, 4)}", flush=True)
    print(f"       CSV: qdd_ref={d[i, col['qddot_ref_norm']]:.3f} "
          f"tau={d[i, col['tau_norm']]:.3f} "
          f"tau56=[{d[i, col['tau5']]:.4f},{d[i, col['tau6']]:.4f}]",
          flush=True)

    # 期望：goto 已完成（t=8 > T_MOVE=3），x_d = SETPOINT 常值
    traj = goto_trajectory(chain.fkm(np.array(
        [-0.251958, -2.607660, -1.617296, -1.077630, 0.000041, -0.000004])),
        w2dh(SETPOINT_POS), R_TOOL_QUAT, 3.0)
    des = traj.evaluate(d[i, col["t"]])

    fk = chain.fk_outputs(q, qd, q_ddot=None, with_jacobian=True)
    err = full_error_state(fk["x_breve"], des["x_breve_d"])
    sig = np.linalg.svd(fk["J"], compute_uv=False)
    qddot_ref, _ = geometric_computed_torque_law(
        err, des["xi_d"], des["xi_dot_d"], fk["J"], fk["Jdot_qdot"],
        K_d, k_p, damping=PINV_DAMPING)
    print("\n[B] 限幅前 qddot_ref =", np.round(qddot_ref, 3),
          f" ‖·‖={np.linalg.norm(qddot_ref):.3f}", flush=True)
    qn = float(np.linalg.norm(qddot_ref))
    if qn > QDDOT_MAX:
        qddot_ref = qddot_ref * (QDDOT_MAX / qn)
        print(f"    限幅后 ‖·‖={np.linalg.norm(qddot_ref):.3f} "
              f"(QDDOT_MAX={QDDOT_MAX})", flush=True)
    M = dyn.mass_matrix(q)
    g = dyn.gravity_vector(q)
    cg = dyn.coriolis_plus_gravity(q, qd)
    tau = dyn.computed_torque(q, qd, qddot_ref)
    print("    M̂ q̈_ref =", np.round(M @ qddot_ref, 3), flush=True)
    print("    Ĉq̇+ĝ   =", np.round(cg, 3), flush=True)
    print("    τ       =", np.round(tau, 3),
          f" ‖τ‖={np.linalg.norm(tau):.3f}", flush=True)
    print(f"\n[C] σ(J) = {np.round(sig, 4)}", flush=True)
    Jpinv = fk["J"].T @ np.linalg.inv(fk["J"] @ fk["J"].T
                                      + PINV_DAMPING ** 2 * np.eye(6))
    print("    ‖J^# 各行‖ =", np.round(
        np.linalg.norm(Jpinv, axis=1), 3), flush=True)
    print("    e_xi =", np.round(err["e_xi"], 4), flush=True)
    print("    e_z  =", np.round(err["e_z"], 4), flush=True)


if __name__ == "__main__":
    main()
