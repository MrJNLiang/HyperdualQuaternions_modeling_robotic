#!/usr/bin/env python3
"""冻结力矩响应 v2 —— 长时窗 + 位置记录 + 逐步阻尼辨识。

probe_freeze_impulse 发现：单关节扰动（C/D）在 Q_F 持续加速（物理健康），
但闭环净反馈力矩组合（B）前 5 步有响应、0.1 s 内衰减归零——疑似
速度相关阻尼或"先动后锁"。本探针：

  [1] B 案例延长到 1 s，记录位置：持续加速 / 饱和限速 / 回摆锁死；
  [2] 若饱和限速：稳态 qd × 阻尼系数 ≈ 净力矩 -> 辨识等效阻尼来源；
  [3] D 案例（+0.5 e_j4）同样延长，对照是否存在相同衰减。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_freeze_impulse2.py [--self-collide 0]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_F = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])
TAU_FB = np.array([0.033, -1.347, 0.625, 0.281, -0.004, 0.003])


def run_case(backend, dyn, tag, tau_extra, n_steps=500):
    backend.reset_to(Q_F)
    g = dyn.gravity_vector(Q_F)
    tau = g + tau_extra
    q0, _ = backend.get_joint_state()
    qd_hist, q_hist = [], []
    for k in range(n_steps):
        backend.apply_arm_torques(tau)
        backend.step()
        q, qd = backend.get_joint_state()
        qd_hist.append(qd.copy())
        q_hist.append(q.copy())
    qd_hist, q_hist = np.array(qd_hist), np.array(q_hist)
    print(f"[{tag}] extra={np.round(tau_extra, 3).tolist()}")
    for k in [4, 9, 24, 49, 99, 249, 499]:
        if k < n_steps:
            print(f"    t={(k + 1) * 0.002:6.3f}s  qd={np.round(qd_hist[k], 3).tolist()}")
    dq = q_hist[-1] - q0
    print(f"    净位移 dq(1s)={np.round(dq, 3).tolist()}")
    print(f"    末段 qd 均值(0.9-1.0s)={np.round(qd_hist[450:].mean(0), 3).tolist()}")
    return qd_hist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-collide", type=int, default=1)
    ap.add_argument("--pos-iter", type=int, default=None)
    ap.add_argument("--vel-iter", type=int, default=None)
    args = ap.parse_args()

    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    print(f"=== self_collide={bool(args.self_collide)} "
          f"pos_iter={args.pos_iter} vel_iter={args.vel_iter} ===")
    backend = IsaacB601Backend(headless=True,
                               arm_self_collision=bool(args.self_collide),
                               solver_pos_iter=args.pos_iter,
                               solver_vel_iter=args.vel_iter)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        qd_b = run_case(backend, dyn, "B +闭环净反馈(1s)", TAU_FB)
        qd_d = run_case(backend, dyn, "D +0.5 e_j4 (1s)", 0.5 * np.eye(6)[3])
        # 稳态速度 -> 等效阻尼辨识（若存在限速）
        print("\n模型预言（无阻尼）1 s 后 qd:")
        Minv = np.linalg.inv(dyn.mass_matrix(Q_F))
        print(f"  B: {np.round(Minv @ TAU_FB, 1).tolist()} (线性增长，1s 应达此值)")
        print(f"  D: {np.round(Minv @ (0.5 * np.eye(6)[3]), 1).tolist()}")
    finally:
        backend.close()
    print("=== PROBE FREEZE IMPULSE2 DONE ===")


if __name__ == "__main__":
    main()
