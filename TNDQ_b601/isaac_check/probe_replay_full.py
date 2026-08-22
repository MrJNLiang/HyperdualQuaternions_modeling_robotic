#!/usr/bin/env python3
"""闭环力矩序列全回放 —— 判定 exp1_slant 停滞是"力矩错"还是"执行环境错"。

probe_openloop_v4 矛盾点：
  [C] 单帧回放（t=2s 力矩常数 0.5 s）-> 臂朝正确方向快动（0.19 rad）；
  但 exp1 闭环中同样的力矩持续 9 s 只走 0.05 rad。

本探针：从 Q_INIT 起、按 CSV 逐行施加记录力矩（10 s，与 exp1 同口径），
若复现停滞轨迹 => 控制律产生的力矩序列本身导致停滞（闭环自洽均衡）；
若臂正常到达 SETPOINT => 闭环执行环境（传感/时序）有问题。

附带 [G]：零力矩自由落体 0.1 s 测 Q_INIT 物理重力（与名义对账）。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_replay_full.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_INIT = np.array([-0.251958, -2.607660, -1.617296,
                   -1.077630, 0.000041, -0.000004])


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        print("=== [G] 零力矩 0.1 s 测物理重力 ===", flush=True)
        backend.reset_to(Q_INIT)
        print("  reset 完成", flush=True)
        _, qd0 = backend.get_joint_state()
        for _ in range(50):
            backend.apply_arm_torques(np.zeros(6))
            backend.step()
        print("  50 步完成", flush=True)
        _, qd1 = backend.get_joint_state()
        # M qdd ≈ tau_g（零外力）：g_phys ≈ M * (qd1-qd0)/dt
        M = dyn.mass_matrix(Q_INIT)
        print("  M 计算完成", flush=True)
        g_phys = M @ ((qd1 - qd0) / 0.1)
        g_nom = dyn.gravity_vector(Q_INIT)
        print(f"  g_nom  = {np.round(g_nom, 3).tolist()}", flush=True)
        print(f"  g_phys = {np.round(g_phys, 3).tolist()}", flush=True)
        print(f"  残差   = {np.round(g_phys - g_nom, 3).tolist()}",
              flush=True)

        print("=== [R] CSV 力矩全回放 10 s ===", flush=True)
        csv = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "results", "exp1_setpoint.csv")
        with open(csv) as f:
            header = f.readline().strip().split(",")
        data = np.loadtxt(csv, delimiter=",", skiprows=1)
        ic = {c: i for i, c in enumerate(header)}
        # CSV 为 100 Hz（LOG_EVERY=5、DT=0.002），每行展开 5 步
        backend.reset_to(Q_INIT)
        n_rows = int(min(10.0, data[-1, ic["t"]]) / 0.01)
        q_hist = []
        for k in range(n_rows):
            tau = np.array([data[k, ic[f"tau{i}"]] for i in range(1, 7)])
            for _ in range(5):
                backend.apply_arm_torques(tau)
                backend.step()
            q, _ = backend.get_joint_state()
            q_hist.append(q.copy())
            if k % 100 == 0:
                q_csv = np.array([data[k, ic[f"q{i}"]] for i in range(1, 7)])
                print(f"  t={k*0.01:5.2f}s  q_replay-q_csv="
                      f"{np.round(q - q_csv, 4).tolist()}", flush=True)
        q_end = q_hist[-1]
        q_csv_end = np.array([data[n_rows - 1, ic[f"q{i}"]]
                              for i in range(1, 7)])
        print(f"  终点差 replay-csv = {np.round(q_end - q_csv_end, 4).tolist()}")
        # 回放终点 FK 位置 vs SETPOINT
        from experiments.ik_lib import fk_pose, dh_to_world
        p, _ = fk_pose(q_end)
        print(f"  回放终点末端位置(DH系) = {np.round(p, 4).tolist()}")
        print(f"  SETPOINT(DH系) 期望约  = "
              f"{np.round(np.array([0.6647, -0.1711, 0.0416]) - np.array([-8.416e-05, 0, 0.08465]), 4).tolist()}")
    finally:
        backend.close()
    print("=== REPLAY FULL DONE ===")


if __name__ == "__main__":
    main()
