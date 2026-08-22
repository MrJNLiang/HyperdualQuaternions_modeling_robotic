#!/usr/bin/env python3
"""冻结锁死定位 —— 逐关节力矩响应矩阵 + 构型依赖对照。

probe_freeze_impulse2 结论：闭环净反馈力矩组合（B）在 Q_F 锁死（1s
净位移≈0），自碰撞关闭/求解器迭代 64/32 均无效；单关节 e_j4（D）
却剧烈运动。需定位：

  [1] Q_F 处逐关节 ±tau0 响应矩阵：哪个关节顶死（或哪个耦合对失效）；
  [2] B 力矩在 Q_INIT 施加：能动 -> 构型相关锁死；仍不动 -> 方向相关；
  [3] B 力矩在 Q_F 但去掉 j2 分量：定位锁死是否由 j2 分量拖住整个链。

每案例 0.1 s（50 步）开环常值力矩，末步 |qd| 与净位移。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_joint_grid.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import Q_INIT

Q_F = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])
TAU_FB = np.array([0.033, -1.347, 0.625, 0.281, -0.004, 0.003])


def run_case(backend, dyn, q0, tag, tau_extra, n_steps=50):
    backend.reset_to(q0)
    tau = dyn.gravity_vector(q0) + tau_extra
    q0_meas, _ = backend.get_joint_state()
    qd_last = None
    for _ in range(n_steps):
        backend.apply_arm_torques(tau)
        backend.step()
        q, qd = backend.get_joint_state()
        qd_last = qd
    dq = q - q0_meas
    print(f"[{tag}]")
    print(f"    末步 qd  ={np.round(qd_last, 3).tolist()}")
    print(f"    净位移dq ={np.round(dq, 4).tolist()}  |dq|max={np.max(np.abs(dq)):.4f}")
    return qd_last, dq


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    tau0 = 0.5
    try:
        print("=== [1] Q_F 逐关节 ±%.1f N·m（0.1s）===" % tau0)
        for i in range(6):
            for s in (+1.0, -1.0):
                extra = np.zeros(6)
                extra[i] = s * tau0
                run_case(backend, dyn, Q_F, f"j{i + 1} {'+' if s > 0 else '-'}",
                         extra)
        print("=== [2] B 力矩在 Q_INIT ===")
        run_case(backend, dyn, np.asarray(Q_INIT, float), "B@Q_INIT", TAU_FB)
        print("=== [3] B 力矩变体在 Q_F ===")
        no2 = TAU_FB.copy()
        no2[1] = 0.0
        run_case(backend, dyn, Q_F, "B 去 j2 分量", no2)
        only2 = np.zeros(6)
        only2[1] = TAU_FB[1]
        run_case(backend, dyn, Q_F, "仅 j2 分量(-1.35)", only2)
        half = TAU_FB * 5.0
        run_case(backend, dyn, Q_F, "B ×5 放大", half)
    finally:
        backend.close()
    print("=== PROBE JOINT GRID DONE ===")


if __name__ == "__main__":
    main()
