#!/usr/bin/env python3
"""构型锁死关节定位 —— 沿 Q_INIT->Q_F 各关节逐维扫描。

已知：B 力矩 @Q_INIT 能动（|dq|max=0.157），@Q_F 锁死（|dq|max≈0.002），
接触报告全程为空、自碰撞/求解器迭代无关 -> 构型依赖锁死。
扫描：对每个关节 i，其余保持 Q_INIT，q_i 线性扫向 Q_F[i]（5 档），
测 B 力矩 0.1s 响应 |dq|max。锁死首次出现的关节角即肇事维度。
再对肇事关节做细扫（Q_F 其余关节固定、该关节从 Q_INIT 值扫到
Q_F 值 9 档）收窄区间。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_config_scan.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import Q_INIT

Q_F = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])
TAU_FB = np.array([0.033, -1.347, 0.625, 0.281, -0.004, 0.003])


def response(backend, dyn, q0, tau_extra, n_steps=50):
    backend.reset_to(q0)
    tau = dyn.gravity_vector(q0) + tau_extra
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
    qI = np.asarray(Q_INIT, float)
    try:
        print("=== 基准 ===")
        print(f"  B@Q_INIT |dq|max = {response(backend, dyn, qI, TAU_FB):.4f}")
        print(f"  B@Q_F    |dq|max = {response(backend, dyn, Q_F, TAU_FB):.4f}")
        print("=== 逐关节扫描（其余=Q_INIT）===")
        culprit = []
        for i in range(6):
            row = []
            for s in np.linspace(0.0, 1.0, 5):
                q = qI.copy()
                q[i] = qI[i] + s * (Q_F[i] - qI[i])
                row.append(response(backend, dyn, q, TAU_FB))
            print(f"  j{i + 1}: {np.round(row, 4).tolist()}")
            if min(row) < 0.02:
                culprit.append(i)
        print("  肇事关节:", [i + 1 for i in culprit] or "无（组合效应）")
        for i in culprit:
            print(f"=== j{i + 1} 细扫（其余=Q_F）===")
            row = []
            for s in np.linspace(0.0, 1.0, 9):
                q = Q_F.copy()
                q[i] = qI[i] + s * (Q_F[i] - qI[i])
                row.append(response(backend, dyn, q, TAU_FB))
            print(f"  j{i + 1}: {np.round(row, 4).tolist()}")
            print(f"  q_i 档位: {np.round(np.linspace(qI[i], Q_F[i], 9), 3).tolist()}")
    finally:
        backend.close()
    print("=== CONFIG SCAN DONE ===")


if __name__ == "__main__":
    main()
