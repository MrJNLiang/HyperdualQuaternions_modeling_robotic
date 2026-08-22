#!/usr/bin/env python3
"""腕部失修物理层验证 —— 停滞构型下开环腕力矩响应。

diag_wrist_stall：闭环稳态 qddot_ref[5]=-22.9（腕应大加速），但
τ5/τ6≈0.005（M̂66≈3e-4），且实测腕不动（qd5/qd6≈0）。名义模型
已聚合 gripper（与 URDF 一致），故排除惯量失配——怀疑腕部也有
物理层锁/摩擦病理。

测试（停滞构型 q_W，重力补偿基线）：
  [1] j6 常值 +0.5 N·m，1 s：测位移/末速度；
  [2] j6 常值 -0.5 N·m，1 s；
  [3] j5 常值 +0.5 N·m，1 s；
  [4] j5+j6 组合（+0.5, -0.5），1 s；
  [5] j2+j3+j4+j5+j6 五关节（闭环签名 + 腕），1 s。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_wrist_v4.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_W = np.array([-0.2554, -2.7817, -1.6417, -1.2471, -0.0693, 0.2860])
DT = 1.0 / 500


def run_case(backend, dyn, extra, n_steps=500):
    backend.reset_to(Q_W)
    q0, _ = backend.get_joint_state()
    g = dyn.gravity_vector(Q_W)
    for _ in range(n_steps):
        backend.apply_arm_torques(g + extra)
        backend.step()
    q1, qd1 = backend.get_joint_state()
    return q1 - q0, qd1


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        cases = [
            ("j6 +0.5",      [0, 0, 0, 0, 0, 0.5]),
            ("j6 -0.5",      [0, 0, 0, 0, 0, -0.5]),
            ("j5 +0.5",      [0, 0, 0, 0, 0.5, 0]),
            ("j5+j6 组合",   [0, 0, 0, 0, 0.5, -0.5]),
            ("五关节组合",   [0, -1.61, 0.72, 0.27, 0.5, -0.5]),
        ]
        for label, v in cases:
            extra = np.array(v, dtype=float)
            dq, qd = run_case(backend, dyn, extra)
            print(f"  {label:12s} -> dq={np.round(dq, 4).tolist()}  "
                  f"|dq|max={np.max(np.abs(dq)):.4f} rad  "
                  f"|qd|max={np.max(np.abs(qd)):.3f}", flush=True)
    finally:
        backend.close()
    print("=== WRIST V4 DONE ===")


if __name__ == "__main__":
    main()
