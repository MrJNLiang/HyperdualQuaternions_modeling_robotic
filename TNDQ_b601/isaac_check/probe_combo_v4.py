#!/usr/bin/env python3
"""v4 停滞点 Q_S 组合锁复现 —— probe_combo_scan 方法移植到新构型。

exp1_slant_v4 停滞态（t≈2s 后 9 s 蠕变 <0.02 rad）：
    Q_S ≈ [-0.252, -2.70, -1.63, -1.24, ~0, ~0]
    闭环净反馈力矩 TAU_NET = tau_cmd - g(Q_S) ≈ [0, -1.6, +0.7, +0.27, 0, 0]
开环证据：同力矩序列在名义模型/Isaac 回放都驱动快速运动，但闭环
实际停滞；probe_openloop_v4 [B] j2 单分量扰动剧烈运动。
假设：与 Q_F 同族的"多关节力矩组合锁"（PhysX TGS 病理）。

测试（Q_S 上，重力补偿基线 + 增量）：
  [1] 闭环净反馈全组合 TAU_NET；
  [2] 单分量 j2 / j3 / j4；
  [3] 两两组合 j2+j3 / j2+j4 / j3+j4；
  [4] 5x 放大全组合。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_combo_v4.py \
        [--pos-iter N] [--vel-iter N]
"""
import argparse
import os
import sys
from itertools import combinations

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 停滞点（exp1 CSV t=2s）
Q_S = np.array([-0.2519, -2.6989, -1.6336, -1.2405, 0.0001, -0.0020])
# 闭环净反馈力矩（tau_cmd[2s] - g_nom(Q_S)；运行中现算 g）
TAU_NET = np.array([0.02, -1.61, 0.72, 0.27, 0.0, 0.0])


def response(backend, dyn, tau_extra, n_steps=50):
    backend.reset_to(Q_S)
    tau = dyn.gravity_vector(Q_S) + tau_extra
    q_start, _ = backend.get_joint_state()
    for _ in range(n_steps):
        backend.apply_arm_torques(tau)
        backend.step()
    q_end, _ = backend.get_joint_state()
    # TCP 位移（DH 链 FK，不含尾变换：仅看位置）
    from core.kinematics import TNDQSerialChain
    from config.params import B601_DH_TABLE
    from core.dq_algebra import dq_translation
    ch = TNDQSerialChain(B601_DH_TABLE)
    p0 = dq_translation(ch.fk_tndq(q_start).ch[0])
    p1 = dq_translation(ch.fk_tndq(q_end).ch[0])
    return float(np.max(np.abs(q_end - q_start))), p1 - p0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pos-iter", type=int, default=None)
    ap.add_argument("--vel-iter", type=int, default=None)
    ap.add_argument("--no-self-collide", action="store_true")
    args = ap.parse_args()
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True,
                               arm_self_collision=not args.no_self_collide,
                               solver_pos_iter=args.pos_iter,
                               solver_vel_iter=args.vel_iter)
    backend.setup()
    dyn = B601NominalDynamics()
    print(f"solver iter: pos={args.pos_iter} vel={args.vel_iter} "
          f"self_collide={not args.no_self_collide}", flush=True)
    try:
        g = dyn.gravity_vector(Q_S)
        print(f"g(Q_S) = {np.round(g, 3).tolist()}", flush=True)

        def rep(label, extra):
            r, dp = response(backend, dyn, extra)
            print(f"  {label:14s} -> |dq|max={r:.4f} rad  "
                  f"dp=[{dp[0]:+.4f},{dp[1]:+.4f},{dp[2]:+.4f}]",
                  flush=True)

        print("=== [1] 闭环净反馈全组合 ===", flush=True)
        rep("TAU_NET", TAU_NET)
        print("=== [2] 单分量 ===", flush=True)
        for i in (1, 2, 3):
            t = np.zeros(6)
            t[i] = TAU_NET[i]
            rep(f"j{i + 1}({TAU_NET[i]:+.2f})", t)
        print("=== [3] 两两组合 ===", flush=True)
        for a, b in combinations((1, 2, 3), 2):
            t = np.zeros(6)
            t[a], t[b] = TAU_NET[a], TAU_NET[b]
            rep(f"j{a + 1}+j{b + 1}", t)
        print("=== [4] 5x 放大全组合 ===", flush=True)
        rep("5*TAU_NET", 5.0 * TAU_NET)
        print("=== [5] 反签名全组合（假设：锁与符号绑定） ===", flush=True)
        rep("-TAU_NET", -TAU_NET)
        print("=== [6] 反签名子组合 ===", flush=True)
        for a, b in combinations((1, 2, 3), 2):
            t = np.zeros(6)
            t[a], t[b] = -TAU_NET[a], -TAU_NET[b]
            rep(f"-j{a + 1}-j{b + 1}", t)
    finally:
        backend.close()
    print("=== COMBO V4 DONE ===")


if __name__ == "__main__":
    main()
