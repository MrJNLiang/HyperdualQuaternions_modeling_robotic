#!/usr/bin/env python3
"""B 力矩分量累加扫描 —— 定位锁死的最小分量组合。

probe_config_scan：单关节构型维扫描无锁死（组合效应）。
probe_joint_grid：B 去 j2 能动、仅 j2 能动、B 全组合锁死。
本探针在 Q_F 上逐步累加 B 的分量（顺序：j2 -> +j3 -> +j4 -> +j5
-> +j6 -> +j1；另做 2 分量组合矩阵），找出锁死的最小组合。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_combo_scan.py
"""
import os
import sys
from itertools import combinations

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_F = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])
TAU_FB = np.array([0.033, -1.347, 0.625, 0.281, -0.004, 0.003])


def response(backend, dyn, tau_extra, n_steps=50):
    backend.reset_to(Q_F)
    tau = dyn.gravity_vector(Q_F) + tau_extra
    q_start, _ = backend.get_joint_state()
    for _ in range(n_steps):
        backend.apply_arm_torques(tau)
        backend.step()
    q_end, _ = backend.get_joint_state()
    return float(np.max(np.abs(q_end - q_start)))


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        print("=== 累加序列（j2 -> +j3 -> +j4 -> +j5 -> +j6 -> +j1）===")
        acc = np.zeros(6)
        for i in [1, 2, 3, 4, 5, 0]:
            acc[i] = TAU_FB[i]
            r = response(backend, dyn, acc)
            print(f"  +j{i + 1}({TAU_FB[i]:+.3f}) -> |dq|max={r:.4f}  "
                  f"tau={np.round(acc, 3).tolist()}")
        print("=== 2 分量组合（|tau_i|>=0.2 的关节）===")
        big = [i for i in range(6) if abs(TAU_FB[i]) >= 0.2]
        for a, b in combinations(big, 2):
            t = np.zeros(6)
            t[a], t[b] = TAU_FB[a], TAU_FB[b]
            r = response(backend, dyn, t)
            print(f"  j{a + 1}+j{b + 1}({TAU_FB[a]:+.3f},{TAU_FB[b]:+.3f})"
                  f" -> |dq|max={r:.4f}")
        print("=== 对照：等幅单/双分量 ===")
        t = np.zeros(6)
        t[1] = -1.35
        print(f"  j2 单独 -1.35        -> {response(backend, dyn, t):.4f}")
        t[3] = 0.28
        print(f"  j2+j4 (-1.35,+0.28)  -> {response(backend, dyn, t):.4f}")
    finally:
        backend.close()
    print("=== COMBO SCAN DONE ===")


if __name__ == "__main__":
    main()
