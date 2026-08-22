#!/usr/bin/env python3
"""干净环境回放 + 物理重力标定 —— 判定闭环停滞根因。

probe_replay_full 矛盾：
  [G] 零力矩 0.1 s 测得 g_phys ≈ [0, -0.5, -0.3, -0.13]，与名义
      [0, 5.7, -2.3, -0.66] 严重不符；但 [A] 名义重力补偿 1 s 仅漂
      0.06 rad -> 若 g_phys 真 ≈0，臂应高速塌缩。矛盾 -> 测量受
      teleport 后求解器脏状态污染（qdd 估计被抑制）。
  [R] CSV 力矩回放塌缩，但原闭环同力矩保持。回放也在 teleport 后
      立即开跑 -> 同类污染嫌疑。

本探针：
  [1] 世界完全复位（world.reset）后重放 CSV 力矩 3 s：若仍塌缩 ->
      闭环力矩确实驱动运动，停滞是控制律问题；若保持 -> 原闭环
      环境物理一致，停滞需另寻解释；
  [2] 干净环境下零力矩自由落体 0.5 s（逐步记录 qd，FD 对账，
      不做 M 求逆），标定物理重力方向与量级。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_replay_clean.py
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
        # ---------- [2] 干净环境零力矩自由落体 ----------
        print("=== [2] world.reset 后零力矩 0.5 s ===", flush=True)
        backend.world.reset()
        backend.articulation.initialize()
        backend.reset_to(Q_INIT)
        qds = []
        for k in range(250):
            backend.apply_arm_torques(np.zeros(6))
            backend.step()
            if k % 5 == 0:
                _, qd = backend.get_joint_state()
                qds.append(qd.copy())
        qds = np.array(qds)   # (50, 6)，每行间隔 0.01 s
        qdd_est = np.diff(qds, axis=0) / 0.01
        print(f"  qdd 前 3 行均值 = {np.round(qdd_est[:3].mean(axis=0), 2).tolist()}")
        print(f"  qdd 全程均值    = {np.round(qdd_est.mean(axis=0), 2).tolist()}")
        qdd_nom = dyn.forward_dynamics(Q_INIT, np.zeros(6), np.zeros(6))
        print(f"  FD 名义（tau=0） = {np.round(qdd_nom, 2).tolist()}")
        q, _ = backend.get_joint_state()
        print(f"  0.5 s 后 q 漂移 = {np.round(q - Q_INIT, 3).tolist()}")

        # ---------- [1] 干净环境 CSV 力矩回放 ----------
        print("=== [1] world.reset 后 CSV 力矩回放 3 s ===", flush=True)
        backend.world.reset()
        backend.articulation.initialize()
        backend.reset_to(Q_INIT)
        csv = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "results", "exp1_setpoint.csv")
        with open(csv) as f:
            header = f.readline().strip().split(",")
        data = np.loadtxt(csv, delimiter=",", skiprows=1)
        ic = {c: i for i, c in enumerate(header)}
        n_rows = int(3.0 / 0.01)
        for k in range(n_rows):
            tau = np.array([data[k, ic[f"tau{i}"]] for i in range(1, 7)])
            for _ in range(5):
                backend.apply_arm_torques(tau)
                backend.step()
            if k % 50 == 0:
                q_now, _ = backend.get_joint_state()
                q_csv = np.array([data[k, ic[f"q{i}"]] for i in range(1, 7)])
                print(f"  t={k*0.01:4.2f}s  q_now-q_csv="
                      f"{np.round(q_now - q_csv, 4).tolist()}", flush=True)
        q_end, _ = backend.get_joint_state()
        q_csv_end = np.array([data[n_rows - 1, ic[f"q{i}"]]
                              for i in range(1, 7)])
        print(f"  3s 终点差 replay-csv = {np.round(q_end - q_csv_end, 4).tolist()}")
    finally:
        backend.close()
    print("=== REPLAY CLEAN DONE ===")


if __name__ == "__main__":
    main()
