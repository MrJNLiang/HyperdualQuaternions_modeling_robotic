#!/usr/bin/env python3
"""冻结点开环探针 —— 区分"物理锁"与"控制律反向驱动"。

exp1 现象：t<1.08s 完美跟踪（误差 1.2mm），随后臂反向爬升、误差以
参考速度线性增长（臂速度=0），力矩 ~4.5 N·m 持续施加不饱和、无接触、
非限位、自碰撞/求解器迭代均无差异。

本探针：teleport 到 CSV 冻结时刻的构型 q_f，开环施加同一时刻的记录
力矩 tau_f，观察 2 s：
    不动  -> PhysX 物理层构型锁（与开环 q_ss+tau_cmd 探针呼应）；
    动了  -> 控制闭环在反向驱动（FK/误差/Jacobian 符号链问题）。
对照：施加 -tau_f 与纯重力补偿 0 力矩，确认方向敏感性。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/diag_openloop_freeze.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

RESULTS = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "results", "exp1_setpoint_nosc.csv")


def load_freeze_state(t_probe):
    d = np.genfromtxt(RESULTS, names=True, delimiter=",")
    t = d["t"]
    i = int(np.argmin(np.abs(t - t_probe)))
    q_f = np.array([d[f"q{k}"][i] for k in range(1, 7)])
    tau_f = np.array([d[f"tau{k}"][i] for k in range(1, 7)])
    return q_f, tau_f


def run_openloop(backend, q0, tau, hold_s=2.0, tag=""):
    backend.reset_to(q0)
    n = int(round(hold_s / backend.physics_dt))
    qs = []
    for k in range(n):
        q, _ = backend.get_joint_state()
        qs.append(q.copy())
        backend.apply_arm_torques(tau)
        backend.step()
    qs = np.array(qs)
    dq = qs[-1] - qs[0]
    print(f"[{tag}] tau={np.round(tau, 3).tolist()}", flush=True)
    print(f"        dq(2s) = {np.round(dq, 4).tolist()}  "
          f"max|dq|={float(np.max(np.abs(dq))):.4f} rad", flush=True)
    return dq


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    try:
        for t_probe, label in [(9.9, "冻结终态 t=9.9"), (1.2, "发散拐点 t=1.2")]:
            q_f, tau_f = load_freeze_state(t_probe)
            print(f"\n===== {label}: q_f = {np.round(q_f, 4).tolist()}")
            print(f"          tau_f = {np.round(tau_f, 3).tolist()} =====")
            run_openloop(backend, q_f, tau_f, tag="A] +tau_f")
            run_openloop(backend, q_f, -tau_f, tag="B] -tau_f")
            run_openloop(backend, q_f, np.zeros(6), tag="C] 零力矩")
            tau_j2 = np.zeros(6)
            tau_j2[1] = tau_f[1] if abs(tau_f[1]) > 0.05 else -1.0
            run_openloop(backend, q_f, tau_j2, tag="D] 仅j2 tau")
    finally:
        backend.close()
    print("=== DIAG OPENLOOP FREEZE DONE ===")


if __name__ == "__main__":
    main()
