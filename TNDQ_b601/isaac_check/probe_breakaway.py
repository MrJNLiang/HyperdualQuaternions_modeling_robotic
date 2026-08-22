#!/usr/bin/env python3
"""组合锁突破后行为 —— dither 解锁方案可行性验证。

probe_combo_v4：Q_S 处闭环净反馈全组合 [1] 锁死（0.0094 rad），
单分量/两两组合能动，5x 放大能动（0.306）。

本探针验证"脉冲突破 + 回归组合力矩"序列：
  [1] 对照：TAU_NET 常数 2 s（应全程锁）；
  [2] TAU_NET + 前 0.2 s j2 脉冲（+2.5 N·m），之后纯 TAU_NET：
      若突破后持续运动 => dither 解锁闭环可行；
  [3] 纯 j2 常数 -1.61 对照（应持续动）。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_breakaway.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_S = np.array([-0.2519, -2.6989, -1.6336, -1.2405, 0.0001, -0.0020])
TAU_NET = np.array([0.02, -1.61, 0.72, 0.27, 0.0, 0.0])


def run_seq(backend, dyn, seq, label):
    """seq: [(n_steps, tau_extra), ...]；返回各段末 |dq|max 与总位移。"""
    backend.reset_to(Q_S)
    g = dyn.gravity_vector(Q_S)
    q0, _ = backend.get_joint_state()
    out = []
    for n_steps, extra in seq:
        for _ in range(n_steps):
            backend.apply_arm_torques(g + extra)
            backend.step()
        q_now, qd_now = backend.get_joint_state()
        out.append((float(np.max(np.abs(q_now - q0))),
                    float(np.max(np.abs(qd_now)))))
    print(f"  {label}: 各段(|dq|max, |qd|max) = "
          f"{[(round(a, 3), round(b, 3)) for a, b in out]}", flush=True)
    return out


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        print("=== [1] TAU_NET 常数 2 s ===", flush=True)
        run_seq(backend, dyn, [(1000, TAU_NET)], "对照")

        print("=== [2] j2 脉冲 0.2 s -> TAU_NET 1.8 s ===", flush=True)
        pulse = TAU_NET.copy()
        pulse[1] -= 2.5                      # 与反馈同向（j2 负向）
        run_seq(backend, dyn, [(100, pulse), (900, TAU_NET)],
                "脉冲突破")

        print("=== [3] 纯 j2 常数 2 s ===", flush=True)
        tj2 = np.zeros(6)
        tj2[1] = TAU_NET[1]
        run_seq(backend, dyn, [(1000, tj2)], "j2 对照")
    finally:
        backend.close()
    print("=== BREAKAWAY DONE ===")


if __name__ == "__main__":
    main()
