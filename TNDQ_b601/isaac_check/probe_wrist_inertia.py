#!/usr/bin/env python3
"""腕部有效惯量实测 —— M̂ vs Isaac 物理层。

diag_wrist_stall：闭环腕指令 τ5/τ6≈0.004 N·m（= M̂ q̈_ref，q̈_ref 腕
分量 6/-23 rad/s²）却无腕加速度（qd5/qd6≈0）；probe_wrist_v4：
0.5 N·m 开环腕能动 => 非锁。唯一自洽解释：物理腕有效惯量 >> 名义
（M̂55/M̂66≈3e-4 kg m²），如 USD 场景 gripper 碰撞体带密度。

测量：q_W 与 Q_INIT 两处，重力补偿基线 + j5/j6 各施 0.05 N·m
脉冲 0.1 s -> 自由滑行，由 qdd ≈ tau / M_eff 反推 M_eff（取脉冲
末速度与后续加速度拟合）。对照名义 M̂(q) 的 55/66 分量。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_wrist_inertia.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_W = np.array([-0.2554, -2.7817, -1.6417, -1.2471, -0.0693, 0.2860])
Q_I = np.array([-0.251958, -2.607660, -1.617296, -1.077630, 0.000041,
                -0.000004])
DT = 1.0 / 500
PULSE_N = 25          # 0.05 s
TAU_PULSE = 0.01      # N·m（小脉冲防飞：M=3e-4 时末速 ≈1.7 rad/s）


def measure(backend, dyn, q_cfg, joint):
    backend.reset_to(q_cfg)
    extra = np.zeros(6)
    extra[joint] = TAU_PULSE
    # 脉冲段（重力补偿逐步用当前 q 重算，防臂塔落污染）
    for _ in range(PULSE_N):
        q_now, _ = backend.get_joint_state()
        backend.apply_arm_torques(dyn.gravity_vector(q_now) + extra)
        backend.step()
    _, qd_pulse = backend.get_joint_state()
    # 滑行段（纯重力补偿）0.4 s，记录速度
    v_hist = []
    for _ in range(200):
        q_now, _ = backend.get_joint_state()
        backend.apply_arm_torques(dyn.gravity_vector(q_now))
        backend.step()
        _, qd = backend.get_joint_state()
        v_hist.append(qd[joint])
    v_pulse = qd_pulse[joint]
    # 脉冲末速度 ≈ (tau/M_eff) * t_pulse  =>  M_eff ≈ tau t / v
    M_eff = TAU_PULSE * PULSE_N * DT / v_pulse if abs(v_pulse) > 1e-9 \
        else float("inf")
    return v_pulse, M_eff, np.mean(np.abs(np.diff(v_hist)) / DT)


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        for name, q_cfg in (("q_W", Q_W), ("Q_INIT", Q_I)):
            M = dyn.mass_matrix(q_cfg)
            print(f"--- {name}: 名义 M55={M[4, 4]:.5f} M66={M[5, 5]:.5f} "
                  f"M56={M[4, 5]:.5f}", flush=True)
            for joint in (4, 5):
                v_p, M_eff, decel = measure(backend, dyn, q_cfg, joint)
                print(f"    j{joint + 1} 脉冲: v_pulse={v_p:.4f} rad/s  "
                      f"M_eff={M_eff:.5f} kg m²  "
                      f"滑行减速度={decel:.3f} rad/s²", flush=True)
    finally:
        backend.close()
    print("=== WRIST INERTIA DONE ===")


if __name__ == "__main__":
    main()
