#!/usr/bin/env python3
"""名义 M̂ vs Isaac 物理 M 系统对账 —— 脉冲法逐关节有效惯量。

现象链（exp1_slant_v4_dither8）：J̄（名义 M̂ 加权）下 q̈_ref=19.2
但 τ≈重力水平、腕 τ5/τ6≈0，位置/姿态修正双双失速——反馈扭矩在
物理空间是零向量。probe_wrist_inertia 已示 M̂66=2.7e-4 vs 物理
M66_eff≈7e-3（25 倍）。本探针系统测量：两构型（Q_INIT、q_W 停滞
态）× 六关节，0.01 N·m/0.05 s 脉冲（其余关节重力补偿保持），测
脉冲末速度反推 M_eff，对照名义 M̂ 对角元与比值。

若比值显著偏离 1：名义模型缺物理资产附加惯量（USD 转换附加碰撞
体/视觉网格密度），需修正 b601_dynamics 惯量表或在接口层扣除。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_mass_compare.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_W = np.array([-0.2540, -2.6400, -1.5480, -1.1730, 0.0160, -0.0050])
Q_I = np.array([-0.251958, -2.607660, -1.617296, -1.077630, 0.000041,
                -0.000004])
DT = 1.0 / 500
PULSE_N = 25          # 0.05 s
TAU_PULSE = 0.01      # N·m


def measure(backend, dyn, q_cfg, joint):
    backend.reset_to(q_cfg)
    extra = np.zeros(6)
    extra[joint] = TAU_PULSE
    for _ in range(PULSE_N):
        q_now, _ = backend.get_joint_state()
        # 全关节重力补偿 + 目标关节脉冲
        backend.apply_arm_torques(dyn.gravity_vector(q_now) + extra)
        backend.step()
    _, qd_pulse = backend.get_joint_state()
    v = qd_pulse[joint]
    M_eff = TAU_PULSE * PULSE_N * DT / v if abs(v) > 1e-9 else float("inf")
    return M_eff


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        for name, q_cfg in (("Q_INIT", Q_I), ("q_W", Q_W)):
            M = dyn.mass_matrix(q_cfg)
            print(f"--- {name} ---", flush=True)
            for j in range(6):
                M_eff = measure(backend, dyn, q_cfg, j)
                ratio = M_eff / M[j, j] if np.isfinite(M_eff) else float(
                    "inf")
                print(f"  j{j + 1}: M̂={M[j, j]:.6f}  "
                      f"M_phys={M_eff:.6f}  ratio={ratio:8.2f}", flush=True)
    finally:
        backend.close()
    print("=== MASS COMPARE DONE ===")


if __name__ == "__main__":
    main()
