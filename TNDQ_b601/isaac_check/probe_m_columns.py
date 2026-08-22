#!/usr/bin/env python3
"""M 列直测 —— 开环脉冲法逐列测量物理质量矩阵。

probe_physx_mass 的 get_mass_matrices 返回异常（j1=j2=j3 完全相等、
对角放大 10~989x），与 USD 层直读质量（与 URDF 一致）矛盾。本探针
用物理真值验证：Q_INIT 处逐关节施小幅常力矩脉冲（其余重力补偿），
测加速度向量 a，得 M 一列 ≈ τ·e_j / a 意义下的线性方程组解。

方法：
  τ = g(q) + δ·e_j（δ=0.05 N·m），持续 PULSE_STEPS 步，
  qd 从 ~0 起步，M q̈ ≈ δ e_j ⇒ q̈ ≈ δ M⁻¹ e_j。
  取中后段速度差分/步长作为 q̈ 估计（避开启动瞬态）。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_m_columns.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import Q_INIT

DELTA = 0.5
PULSE_STEPS = 6
WARMUP = 2


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    dt = backend.physics_dt
    M_meas = np.zeros((6, 6))
    try:
        for j in range(6):
            backend.reset_to(Q_INIT)
            backend.apply_arm_torques(np.zeros(6))
            for _ in range(3):
                backend.step()
            _, qd0 = backend.get_joint_state()
            tau = np.zeros(6)
            tau[j] = DELTA
            qds = []
            for k in range(PULSE_STEPS):
                q_now, _ = backend.get_joint_state()
                backend.apply_arm_torques(dyn.gravity_vector(q_now) + tau)
                backend.step()
                _, qd = backend.get_joint_state()
                qds.append(qd.copy())
            # 取后半段差分，避开启动瞬态
            qdd = (qds[-1] - qds[WARMUP]) / ((PULSE_STEPS - 1 - WARMUP) * dt)
            # M q̈ = δ e_j → 列估计：对满阵做最小二乘不行（单方向），
            # 但若 q̈_j 主导则 M[:, j] ≈ δ e_j / q̈_j 无意义；
            # 正确解读：q̈ = δ M⁻¹ e_j → Minv[:, j] = q̈/δ
            Minv_col = qdd / DELTA
            print(f"j{j + 1}: q̈={np.round(qdd, 3).tolist()}  "
                  f"|q̈|max={np.max(np.abs(qdd)):.3f}", flush=True)
            M_meas[:, j] = Minv_col
        Minv = M_meas
        print("Minv 实测（列 = q̈/δ）：", flush=True)
        for row in Minv:
            print("    " + "  ".join(f"{v:9.3f}" for v in row), flush=True)
        try:
            M_est = np.linalg.inv(Minv)
            M_nom = dyn.mass_matrix(Q_INIT)
            print("M 实测（inv(Minv)）vs 名义 对角比：", flush=True)
            for j in range(6):
                print(f"    j{j + 1}: M_est={M_est[j, j]:.6f}  "
                      f"M̂={M_nom[j, j]:.6f}  "
                      f"ratio={M_est[j, j] / M_nom[j, j]:6.2f}", flush=True)
            print("    M_est =", np.round(M_est, 5).tolist(), flush=True)
        except np.linalg.LinAlgError:
            print("    Minv 奇异，无法求逆", flush=True)
    finally:
        backend.close()
    print("=== M COLUMNS DONE ===")


if __name__ == "__main__":
    main()
