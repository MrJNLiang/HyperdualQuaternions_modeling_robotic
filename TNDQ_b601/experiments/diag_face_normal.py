#!/usr/bin/env python3
"""v12h 接触面法向直测：探针置于指-块接触点，释放读弹出方向。

v12g 裁决：笛卡尔提升（姿态恒定）方块仍沿 +x_g 渐移后弹飞 ->
接触面本身含接近向斜坡（夹紧力分解出 +x_g 分量）。本脚本在
v12c 抓取位夹紧状态下，将 1 cm 探针逐点放在方块两侧面接触区
（gripper 系 x=-28 mm、y=±22 mm、z_g 网格），teleport 释放 6 步
后读位移方向，直接给出接触面法向 -> 判定斜坡来源（刀片/垫/轨）。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_face_normal.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik, world_to_dh
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True, probe_size=0.01)
    backend.setup()

    q0 = np.array([-0.3, -1.6, -0.6, -0.6, 0.0, 0.0])
    grasp = P.GRASP_POS
    q, res, ok = solve_ik(world_to_dh(grasp), P.R_TOOL_QUAT, q0)
    assert ok, f"IK 失败 {res}"
    backend.reset_to(q, gripper_width=P.GRIPPER_GRASP - 0.010)
    # 保持夹持构型（手指 drive 压紧）
    for _ in range(200):
        backend.articulation.set_joint_positions(
            backend._row(q), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)
        backend.set_gripper(P.GRIPPER_GRASP - 0.010)
        backend.step()
    p_g, q_g = backend.get_ee_pose()
    w, x, y, z = q_g
    Rg = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    print(f"ee=({p_g[0]:.4f},{p_g[1]:.4f},{p_g[2]:.4f})")
    print(f"x_g={np.round(Rg[:,0],3)}  y_g={np.round(Rg[:,1],3)}  "
          f"z_g={np.round(Rg[:,2],3)}")

    # 探针点（gripper 系）：方块面深度 x=-28，开合向 ±22，z 网格
    for z_mm in (10, 20, 30, 42):
        for side in (+1, -1):
            p_local = np.array([-0.028, side * 0.022, z_mm * 1e-3])
            p_w = p_g + Rg @ p_local
            backend.probe.set_world_pose(p_w, np.array([1.0, 0, 0, 0.0]))
            backend.probe.set_linear_velocity(np.zeros(3))
            backend.probe.set_angular_velocity(np.zeros(3))
            for _ in range(8):
                backend.articulation.set_joint_positions(
                    backend._row(q), joint_indices=backend.arm_idx)
                backend.articulation.set_joint_velocities(
                    backend._row(np.zeros(8)), joint_indices=None)
                backend.set_gripper(P.GRIPPER_GRASP - 0.010)
                backend.step()
            pp, _ = backend.get_probe_pose()
            d_w = pp - p_w
            # 分解到 gripper 系
            d_g = Rg.T @ d_w
            print(f"z={z_mm:2d}mm y={side*22:+3d}mm: "
                  f"弹出=({d_g[0]*1e3:+6.1f},{d_g[1]*1e3:+6.1f},"
                  f"{d_g[2]*1e3:+6.1f})mm_g")
    backend.close()


if __name__ == "__main__":
    main()
