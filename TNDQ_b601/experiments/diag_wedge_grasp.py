#!/usr/bin/env python3
"""v12i 楔形夹持验证：方块中心降到斜坡顶面高度，楔形对称举升。

v12h 形状裁决（空中探针，排除支柱）：
  手指内面 = 楔形斜坡，顶面 z_top 随深度上升：
    x=-16 -> z=8, x=-22 -> 10, x=-28 -> 12, x=-34 -> 14 mm；
  z>16 带内无竖直内面（y 26-50 全空）。
  -> v12c（c_z=42）接触发生在斜坡低段，方块坐支柱上，楔形侧向
     分力无处可去 -> 沿 +x_g 挤出（全部提升失败根因）。

楔形举升方案：c_z ≈ 斜坡顶面 12-14 mm，方块下半在楔面之上，
对称楔形夹紧的合力竖直向上 + y 向自定心 -> 方块被举离支柱。
扫 z_off ∈ {-0.008, -0.012, -0.016}（d_x=0.028）：慢闭 46 mm
目标，观察方块 dz（期望 +）与水平漂移（期望 ~0）。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_wedge_grasp.py
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
    cy = P.CUBE_YAW
    cube_q = np.array([np.cos(0.5 * cy), 0.0, 0.0, np.sin(0.5 * cy)])
    cube_p0 = np.array(P.CUBE_POS, dtype=float)
    cube_p0[2] += 1e-3

    for z_off in (-0.008, -0.012, -0.016):
        grasp = P.CUBE_POS + 0.028 * x_g + z_off * z_g
        q, res, ok = solve_ik(world_to_dh(grasp), P.R_TOOL_QUAT, q_prev)
        if not ok:
            print(f"z_off={z_off*1e3:+.0f}mm: IK 失败 {res}")
            continue
        q_prev = q
        backend.cube.set_world_pose(cube_p0, cube_q)
        backend.cube.set_linear_velocity(np.zeros(3))
        backend.cube.set_angular_velocity(np.zeros(3))
        backend.reset_to(q, gripper_width=P.GRIPPER_OPENING)
        for _ in range(60):
            backend.articulation.set_joint_positions(
                backend._row(q), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.step()
        c0, _ = backend.get_cube_pose()

        w_target = 0.046
        for k in range(1000):
            backend.articulation.set_joint_positions(
                backend._row(q), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.set_gripper(w_target)
            backend.step()
            if k % 250 == 249:
                ck, _ = backend.get_cube_pose()
                dk = ck - c0
                print(f"  z={z_off*1e3:+4.0f} t={2*(k+1):4d}ms "
                      f"位移=({dk[0]*1e3:+6.1f},{dk[1]*1e3:+6.1f},"
                      f"{dk[2]*1e3:+6.1f})mm "
                      f"开度={backend.get_gripper_width()*1e3:4.1f}mm")
        c, _ = backend.get_cube_pose()
        d = c - c0
        print(f"z_off={z_off*1e3:+.0f}mm: 末态 dz={d[2]*1e3:+6.1f}mm"
              f"（+即楔形举升） 水平={np.linalg.norm(d[:2])*1e3:5.1f}mm "
              f"开度={backend.get_gripper_width()*1e3:.1f}mm")
    backend.close()


if __name__ == "__main__":
    main()
