#!/usr/bin/env python3
"""Q_F 接触对账 —— AABB 相交检测复现（diag_selfcollide 技术）。

冻结 TCP 高度仅 pz=0.105 m（姿态 tilt 96.5°，指尖朝下），嫌疑：
指尖/夹爪触地或近地。probe_freeze_impulse B 案例"先动后灭"+逐步
数据显示 step 3 起剧烈反摆——符合小间隙接触渐进锁死特征。

  [1] teleport Q_F，静置 1 步 -> 全 link AABB 相交检测（含地面）；
  [2] 施加 B 力矩 20 步（臂微动）-> 再次检测：新出现的相交对即
      锁死约束来源。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_contact_check.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

Q_F = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])
TAU_FB = np.array([0.033, -1.347, 0.625, 0.281, -0.004, 0.003])


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend
    from diag_selfcollide import _bbox_overlaps

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        backend.reset_to(Q_F)
        backend.apply_arm_torques(dyn.gravity_vector(Q_F))
        backend.step()
        print("=== [1] Q_F 静置 AABB ===")
        _bbox_overlaps(backend, "Q_F 静置")

        backend.reset_to(Q_F)
        g = dyn.gravity_vector(Q_F)
        for _ in range(20):
            backend.apply_arm_torques(g + TAU_FB)
            backend.step()
        q, qd = backend.get_joint_state()
        print("=== [2] B 力矩 20 步后（dq=%s）AABB ==="
              % np.round(q - Q_F, 4).tolist())
        _bbox_overlaps(backend, "B 驱动 20 步")
    finally:
        backend.close()
    print("=== CONTACT CHECK DONE ===")


if __name__ == "__main__":
    main()
