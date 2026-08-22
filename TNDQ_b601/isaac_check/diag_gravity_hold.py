#!/usr/bin/env python3
"""重力补偿悬挂漂移诊断 —— exp1 稳态误差根因分离。

exp1（TCP 约定修复后）稳定收敛但稳态误差 20mm/0.28 rad：反馈 q̈_ref
持续 17 rad/s^2 而末端冻结（e_xi=0.031 恒定爬行），形如"反馈力矩被
持续扰动平衡"。

本脚本在 Isaac 中对两个构型做纯重力补偿悬挂（与主循环同款每步
ĝ(q)），量化各关节漂移：
    [1] Q_INIT   （exp1 起始构型，对照）
    [2] q_ss     （exp1 末态构型，误差冻结点）

判读：
    - 漂移 ~ mrad/s 且无定向 -> 重力模型 OK，误差另有来源；
    - 某关节定向持续漂移 -> 该通道重力/摩擦失配，漂移率 x 轻惯量
      即等效失配力矩（exp1 稳态反馈须平衡它）。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/diag_gravity_hold.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import Q_INIT
from config.b601_dynamics import B601NominalDynamics

# exp1（TCP 修复后首跑）稳态构型：results/exp1_setpoint.csv 末段
Q_SS = np.array([-0.594, -2.395, -0.697, -1.575, -0.978, -0.237])

HOLD_S = 2.0          # 每构型悬挂时长


def main():
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()

    try:
        for tag, q0 in [("Q_INIT", Q_INIT), ("q_ss", Q_SS)]:
            backend.reset_to(q0)
            n_steps = int(round(HOLD_S / backend.physics_dt))
            drift = np.zeros((n_steps, 6))
            for k in range(n_steps):
                q, _ = backend.get_joint_state()
                backend.apply_arm_torques(dyn.gravity_vector(q))
                backend.step()
                q_now, _ = backend.get_joint_state()
                drift[k] = q_now - q0
            rate = (drift[-1] - drift[int(n_steps / 2)]) / (HOLD_S / 2.0)
            print(f"[{tag}] 悬挂 {HOLD_S:.1f}s：")
            print(f"    末漂移 dq = {np.round(drift[-1], 4).tolist()} rad")
            print(f"    后半段漂移率 = {np.round(rate, 4).tolist()} rad/s")
            tau_hold = dyn.gravity_vector(Q_SS if tag == "q_ss" else Q_INIT)
            print(f"    ĝ(q0) = {np.round(tau_hold, 3).tolist()} N*m")
    finally:
        backend.close()
    print("=== DIAG GRAVITY HOLD DONE ===")


if __name__ == "__main__":
    main()
