#!/usr/bin/env python3
"""v12f 抓取几何二维网格扫描：定位面接触夹持区。

v12e 裁决：仅改 tilt 无效——接触结构是窄轨（沿接近向延伸、
frame 后 recess 10.7 mm），stall 开度 = 方块宽 + 2*recess = 66 mm，
与深度/tilt 弱相关；z_g=-0.042 使方块 z_g 中心 +42 mm，只有下角
（z_g=+22.5）擦到平板带下缘 -> 点接触 -> 提升必被推飞。

面接触判据：方块面心 z_g=c_z 落入平板带中心 -> 需 c_z≈20 mm
（z 偏置 ≈ -0.020）。本脚本扫 (d_x, z_off) 网格，每格硬复位 +
慢闭读 stall 开度与扰动；stall < 55 mm 且扰动 < 2 mm 者入围。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_grasp_grid.py
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
    backend = IsaacB601Backend(headless=True)
    backend.setup()

    q0 = np.array([-0.3, -1.6, -0.6, -0.6, 0.0, 0.0])
    q_prev = q0
    x_g, z_g = P.TOOL_AXIS, P._TOOL_Z

    def teleport_arm(qa, n=30):
        for _ in range(n):
            backend.articulation.set_joint_positions(
                backend._row(qa), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.step()

    cy = P.CUBE_YAW
    cube_q = np.array([np.cos(0.5 * cy), 0.0, 0.0, np.sin(0.5 * cy)])
    cube_p0 = np.array(P.CUBE_POS, dtype=float)
    cube_p0[2] += 1e-3

    candidates = []
    for d_x in (0.012, 0.020, 0.028):
        for z_off in (-0.015, -0.020, -0.025, -0.030):
            grasp = P.CUBE_POS + d_x * x_g + z_off * z_g
            q, res, ok = solve_ik(world_to_dh(grasp), P.R_TOOL_QUAT, q_prev)
            if not ok:
                print(f"d_x={d_x*1e3:3.0f} z={z_off*1e3:+4.0f}: IK 失败")
                continue
            q_prev = q
            backend.cube.set_world_pose(cube_p0, cube_q)
            backend.cube.set_linear_velocity(np.zeros(3))
            backend.cube.set_angular_velocity(np.zeros(3))
            backend.reset_to(q, gripper_width=P.GRIPPER_OPENING)
            teleport_arm(q, 60)
            c0, _ = backend.get_cube_pose()
            # 慢闭到 46 mm 目标，1.2 s
            w_target = 0.046
            for _ in range(600):
                backend.articulation.set_joint_positions(
                    backend._row(q), joint_indices=backend.arm_idx)
                backend.articulation.set_joint_velocities(
                    backend._row(np.zeros(8)), joint_indices=None)
                backend.set_gripper(w_target)
                backend.step()
            w_stall = backend.get_gripper_width()
            c, _ = backend.get_cube_pose()
            d = np.abs(c - c0).max()
            print(f"d_x={d_x*1e3:3.0f} z={z_off*1e3:+4.0f}: "
                  f"stall={w_stall*1e3:5.1f}mm 扰动={d*1e3:5.1f}mm")
            if w_stall < 0.055 and d < 0.002:
                candidates.append((d_x, z_off, w_stall, q.copy()))
    print(f"\n入围（stall<55mm 且扰动<2mm）：{len(candidates)} 格")
    for d_x, z_off, ws, _ in candidates:
        print(f"  d_x={d_x*1e3:.0f}mm z_off={z_off*1e3:+.0f}mm "
              f"stall={ws*1e3:.1f}mm")
    backend.close()


if __name__ == "__main__":
    main()
