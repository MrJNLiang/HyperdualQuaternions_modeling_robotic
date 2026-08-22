#!/usr/bin/env python3
"""稳态冻结根因探针 —— exp1 反馈力矩"拉不动"Isaac 关节的机制检验。

exp1（TCP 修复后）稳态离线复算：
    qdd_ref = [-0.19, -0.43, -0.17, 7.04, -1.97, 15.54]（集中腕部）
    M qdd_ref = [-0.007, -0.184, 0.073, 0.059, -0.004, 0.006] N*m
而悬挂实验中 mN·m 级残差即可驱动 j2 漂 0.034 rad/s —— 0.184 N·m
的持续力矩却使关节冻结，物理上矛盾。

三个实验分离机制：
    [1] q_ss 悬挂 + j2 恒偏置 -0.184 N*m（exp1 稳态同款纠正力矩）
        动 -> exp1 冻结另有原因（如反馈力矩被噪声相消）
        不动 -> PhysX 层面静摩擦/约束（查 USD 关节属性）
    [2] 同款 +0.184 N*m（方向对照）
    [3] 纯重力补偿悬挂下连续 60 步 qd 读数（速度噪声量级，
        e_xi=0.031 恒定的来源假设检验）

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/diag_stall_probe.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import Q_INIT
from config.b601_dynamics import B601NominalDynamics

# exp1（TCP 修复后首跑）稳态构型
Q_SS = np.array([-0.594, -2.395, -0.697, -1.575, -0.978, -0.237])
HOLD_S = 1.0


def run_bias(backend, dyn, tag, bias):
    backend.reset_to(Q_SS)
    n = int(round(HOLD_S / backend.physics_dt))
    hist = []
    for k in range(n):
        q, _ = backend.get_joint_state()
        backend.apply_arm_torques(dyn.gravity_vector(q) + bias)
        backend.step()
        if k % int(0.1 / backend.physics_dt) == 0:
            hist.append((k * backend.physics_dt, q.copy()))
    print(f"[{tag}] j2 轨迹（每 0.1s）: "
          f"{[round(h[1][1], 4) for h in hist]}")
    print(f"[{tag}] 末态 q = {np.round(hist[-1][1], 4).tolist()}")


def main():
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()

    try:
        bias = np.zeros(6)
        bias[1] = -0.184
        run_bias(backend, dyn, "1] j2 偏置 -0.184 N*m", bias)
        bias[1] = +0.184
        run_bias(backend, dyn, "2] j2 偏置 +0.184 N*m", bias)

        # [3] 速度读数噪声（Q_INIT 纯重力补偿悬挂）
        backend.reset_to(Q_INIT)
        qd_log = []
        for k in range(60):
            q, qd = backend.get_joint_state()
            qd_log.append(qd.copy())
            backend.apply_arm_torques(dyn.gravity_vector(q))
            backend.step()
        qd_log = np.array(qd_log)
        print("[3] 悬挂 60 步 qd 读数统计：")
        print(f"    均值   = {np.round(qd_log.mean(axis=0), 5).tolist()}")
        print(f"    RMS    = {np.round(np.sqrt((qd_log**2).mean(axis=0)), 5).tolist()}")
        print(f"    max|qd| = {np.round(np.abs(qd_log).max(axis=0), 5).tolist()}")
    finally:
        backend.close()
    print("=== DIAG STALL PROBE DONE ===")


if __name__ == "__main__":
    main()
