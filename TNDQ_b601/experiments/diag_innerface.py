#!/usr/bin/env python3
"""手指内面地面真值测绘（v12b 裁决）。

细扫事实：真实开度 ~114 mm 处方块被推（STL 内面模型预言 45 mm）。
本脚本零几何假设：抓取构型保持，各档 stroke 下用 1 cm 探针立方体沿
开合向(y)逐点 teleport（pad 深度 x=-10 mm、z=0 与 z=-20 mm 两行），
6 步后读位移判占据，直接测出内面位置 y_inner(stroke)。

判定模型：
  - 若 y_inner ≈ stroke_mm（frame 平面内面）→ 接触开度=物宽，45 mm 夹持可行；
  - 若 y_inner ≈ stroke_mm - 34（指尖内伸 34 mm）→ 最小夹持宽度 ~66 mm，
    45 mm 方块无法被平行夹持，需改任务设计（或放弃夹持改托举）。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_innerface.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik, world_to_dh
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


def _quat_to_R(r):
    w, x, y, z = r
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True, probe_size=0.01)
    backend.setup()

    # 抓取方块挪走（防干扰）
    cube0, cq0 = backend.get_cube_pose()
    backend.cube.set_world_pose(cube0 + np.array([2.0, 0, 0]), cq0)

    q0 = np.array([-0.3, -1.6, -0.6, -0.6, 0.0, 0.0])
    q, res, ok = solve_ik(world_to_dh(P.SETPOINT_POS), P.R_TOOL_QUAT, q0)
    assert ok, f"IK 失败 {res}"
    backend.reset_to(q, gripper_width=P.GRIPPER_OPENING)

    THR = 0.0030   # 占据判据 [m]

    for stroke in (0.060, 0.050, 0.040, 0.030, 0.020):
        # 保持抓取构型 + 手指 teleport 到该档
        for _ in range(20):
            backend.articulation.set_joint_positions(
                backend._row(q), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_positions(
                backend._row(np.full(2, stroke)), joint_indices=backend.grip_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.step()
        p_g, q_g = backend.get_ee_pose()
        Rg = _quat_to_R(q_g)

        for z_local in (0.0, -0.020):
            row = []
            for y_mm in range(0, 71, 3):
                y_local = y_mm * 1e-3
                p_local = np.array([-0.010, y_local, z_local])
                p_w = p_g + Rg @ p_local
                backend.probe.set_world_pose(p_w, np.array([1, 0, 0, 0.0]))
                backend.probe.set_linear_velocity(np.zeros(3))
                backend.probe.set_angular_velocity(np.zeros(3))
                for _ in range(6):
                    backend.articulation.set_joint_positions(
                        backend._row(q), joint_indices=backend.arm_idx)
                    backend.articulation.set_joint_positions(
                        backend._row(np.full(2, stroke)),
                        joint_indices=backend.grip_idx)
                    backend.articulation.set_joint_velocities(
                        backend._row(np.zeros(8)), joint_indices=None)
                    backend.step()
                pp, _ = backend.get_probe_pose()
                moved = np.linalg.norm(pp - p_w) > THR
                row.append('#' if moved else '.')
            print(f"stroke={stroke*1e3:4.1f}mm z={z_local*1e3:+5.1f}mm "
                  f"y=0..70mm: {''.join(row)}")
    backend.close()


if __name__ == "__main__":
    main()
