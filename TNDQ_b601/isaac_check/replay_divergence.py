#!/usr/bin/env python3
"""发散拐点离线复现 —— 用 CSV 记录量重建 t≈1.09s 控制律各分量（纯 Python）。

exp1 现象：t=1.08 -> 1.09 之间 e_xi_norm 从 0.031 突跳到 0.253（8 倍），
而位姿误差 e_z 基本不变（pos_err 仍 1.2mm）。e_xi = ξ − Ad ξ_d，ξ = J q̇
——若记录的 q 平滑而 e_xi 突跳，则突跳来自控制环内的 q̇ 测量（Isaac
get_joint_state 的速度通道），而非臂的真实运动。

本脚本用记录的 q 做中心差分重建 q̇_fd，逐步复现：
    FK 层 -> 误差层 -> (5.2) 各分量（u_ff / u_fb / Jdot qdot / qddot_ref）
并与 CSV 记录值对比：
    复现 e_xi << 记录 e_xi  -> Isaac q̇ 测量噪声是根因；
    复现 e_xi ≈ 记录 e_xi   -> 臂真的突跳，回看力矩/物理层。

运行：
    python TNDQ_b601/isaac_check/replay_divergence.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    B601_DH_TABLE, DEFAULT_GAIN_SET, GAIN_SETS, PINV_DAMPING, Q_INIT,
    R_TOOL_QUAT, SETPOINT_POS,
)
from control.control_law import (
    damped_pinv, feedforward_term, geometric_computed_torque_law,
)
from control.error_system import full_error_state
from simdata.trajectory_generator import goto_trajectory

from experiments.run_lib import B601TCPChain, w2dh

CSV = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "results", "exp1_setpoint.csv")
T_MOVE = 3.0
DT_LOG = 0.01


def main():
    d = np.genfromtxt(CSV, names=True, delimiter=",")
    t = d["t"]
    Q = np.stack([d[f"q{k}"] for k in range(1, 7)], axis=1)

    chain = B601TCPChain(B601_DH_TABLE)
    traj = goto_trajectory(chain.fkm(Q_INIT), w2dh(SETPOINT_POS),
                           R_TOOL_QUAT, T_MOVE)
    gains = GAIN_SETS[DEFAULT_GAIN_SET]
    K_d, k_p = gains["K_d"], gains["k_p"]

    # ---- 先看记录的 q 在拐点附近是否平滑（差分速度）----
    sel = (t >= 0.95) & (t <= 1.35)
    idx = np.where(sel)[0]
    print("t        |dq/dt|max(FD,100Hz)  记录e_xi  复现e_xi(FD qd)"
          "   记录qdd_ref  复现qdd_ref")
    for i in idx:
        i0, i1 = max(i - 1, 0), min(i + 1, len(t) - 1)
        qd_fd = (Q[i1] - Q[i0]) / (t[i1] - t[i0])
        q = Q[i]
        fk = chain.fk_outputs(q, qd_fd, q_ddot=None, with_jacobian=True)
        des = traj.evaluate(t[i])
        err = full_error_state(fk["x_breve"], des["x_breve_d"])
        e_xi_replay = float(np.linalg.norm(err["e_xi"]))
        qdd_ref, u_task = geometric_computed_torque_law(
            err, des["xi_d"], des["xi_dot_d"], fk["J"], fk["Jdot_qdot"],
            K_d, k_p, damping=PINV_DAMPING)
        print(f"{t[i]:7.3f}  {float(np.max(np.abs(qd_fd))):8.4f}"
              f"          {d['e_xi_norm'][i]:8.4f}  {e_xi_replay:8.4f}"
              f"      {d['qddot_ref_norm'][i]:9.3f}"
              f"   {np.linalg.norm(qdd_ref):9.3f}")

    # ---- 拐点时刻 u_task 分量分解（复现侧）----
    print("\n== t=1.09 附近 u_task 分量分解（复现，FD qd）==")
    for i in idx:
        if not (1.05 <= t[i] <= 1.13):
            continue
        i0, i1 = max(i - 1, 0), min(i + 1, len(t) - 1)
        qd_fd = (Q[i1] - Q[i0]) / (t[i1] - t[i0])
        q = Q[i]
        fk = chain.fk_outputs(q, qd_fd, q_ddot=None, with_jacobian=True)
        des = traj.evaluate(t[i])
        err = full_error_state(fk["x_breve"], des["x_breve_d"])
        u_ff = feedforward_term(err["x_tilde"], err["xi_tilde"],
                                des["xi_d"], des["xi_dot_d"])
        K_p = np.asarray(k_p, dtype=float)
        pose = K_p @ err["e_z"] if K_p.ndim == 2 else K_p * err["e_z"]
        u_fb = -K_d @ err["e_xi"] - err["A"].T @ pose
        jdq = np.asarray(fk["Jdot_qdot"], dtype=float)
        print(f"t={t[i]:.3f}: |u_ff|={np.linalg.norm(u_ff):.3f} "
              f"|u_fb|={np.linalg.norm(u_fb):.3f} "
              f"|Jdot qdot|={np.linalg.norm(jdq):.3f} "
              f"e_xi={np.round(err['e_xi'], 3).tolist()}")
        print(f"         xi_d={np.round(des['xi_d'], 3).tolist()}")
        # Jacobian 条件
        s = np.linalg.svd(fk["J"], compute_uv=False)
        print(f"         sigma(J)={np.round(s, 3).tolist()}")


if __name__ == "__main__":
    main()
