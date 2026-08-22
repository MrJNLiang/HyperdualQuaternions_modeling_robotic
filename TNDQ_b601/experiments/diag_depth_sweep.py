#!/usr/bin/env python3
"""v12d 抓取深度扫描：量化 stall 开度随深度的变化，找可压紧+可提升的几何。

v12c 裁决事实：当前 GRASP_POS（0.028 x_g - 0.042 z_g）下实测 stall
开度 66~68 mm ≈ 接触点开度，驱动目标 58 mm 压不下去（开度保持
68.4 mm）-> 零过盈、摩擦不足以提升（方块滑落 dz=-60）。

机理：方块面位于指 frame 后 recess 内 10.7 mm，recess 深度固定，
方块深度 c_x 决定接触 stroke = 22.5 + (10.7 - c_x_mm)。c_x 越大
（gripper 沿 x_g 退得越多）接触 stroke 越小、可压紧余量越大。

本脚本：对 d_x ∈ {0.024, 0.020, 0.016}（z_g 偏置保持 -0.042），
各档 IK -> teleport -> 慢闭（drive 收敛读 stall 开度）-> 关节插值
提升 5 cm 看方块随动。全部 teleport 级，不经过 TNDQ 控制链。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_depth_sweep.py
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

    def teleport_arm(qa, n=30):
        for _ in range(n):
            backend.articulation.set_joint_positions(
                backend._row(qa), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.step()

    for d_x in (0.024, 0.020, 0.016):
        grasp = P.CUBE_POS + d_x * P.TOOL_AXIS - 0.042 * P._TOOL_Z
        q, res, ok = solve_ik(world_to_dh(grasp), P.R_TOOL_QUAT, q_prev)
        if not ok:
            print(f"d_x={d_x*1e3:.0f}mm: IK 失败 {res}")
            continue
        q_prev = q
        # 硬复位（无插值扫掠，避免指体扫过方块）+ 方块归位 + 全开保持
        cy = P.CUBE_YAW
        cube_q = np.array([np.cos(0.5 * cy), 0.0, 0.0, np.sin(0.5 * cy)])
        cube_p = np.array(P.CUBE_POS, dtype=float)
        cube_p[2] += 1e-3
        backend.cube.set_world_pose(cube_p, cube_q)
        backend.cube.set_linear_velocity(np.zeros(3))
        backend.cube.set_angular_velocity(np.zeros(3))
        backend.reset_to(q, gripper_width=P.GRIPPER_OPENING)
        teleport_arm(q, 60)
        c0, _ = backend.get_cube_pose()

        # 慢闭：drive 目标 40 mm 开度（预期 stall 于接触点），2 s
        w_target = 0.040
        for _ in range(1000):
            backend.articulation.set_joint_positions(
                backend._row(q), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.set_gripper(w_target)
            backend.step()
        w_stall = backend.get_gripper_width()
        c, _ = backend.get_cube_pose()
        d_close = c - c0
        pe0, _ = backend.get_ee_pose()
        print(f"d_x={d_x*1e3:4.0f}mm: stall 开度={w_stall*1e3:5.1f}mm "
              f"闭合扰动=({d_close[0]*1e3:+5.1f},{d_close[1]*1e3:+5.1f},"
              f"{d_close[2]*1e3:+5.1f})mm")
        print(f"  闭合后 ee=({pe0[0]:.4f},{pe0[1]:.4f},{pe0[2]:.4f}) 目标"
              f" grasp=({grasp[0]:.4f},{grasp[1]:.4f},{grasp[2]:.4f}) "
              f"差={np.linalg.norm(pe0 - grasp)*1e3:.1f}mm")

        # 提升：保持 drive 目标，关节插值 5 cm，1 s
        p_lift = grasp + np.array([0.0, 0.0, 0.05])
        ql, rl, okl = solve_ik(world_to_dh(p_lift), P.R_TOOL_QUAT, q)
        if not okl:
            print(f"  提升位 IK 失败")
            continue
        n = 500
        for k in range(n):
            a = k / (n - 1)
            qi = q + a * (ql - q)
            backend.articulation.set_joint_positions(
                backend._row(qi), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.set_gripper(w_target)
            backend.step()
            if k % 100 == 99:
                ck, _ = backend.get_cube_pose()
                dk = ck - c0
                print(f"  t={2*(k+1):4d}ms 方块位移=({dk[0]*1e3:+6.1f},"
                      f"{dk[1]*1e3:+6.1f},{dk[2]*1e3:+6.1f})mm "
                      f"开度={backend.get_gripper_width()*1e3:4.1f}mm")
        c, _ = backend.get_cube_pose()
        dz = c[2] - c0[2]
        dh = np.linalg.norm(c[:2] - c0[:2])
        pe, _ = backend.get_ee_pose()
        print(f"  提升后: dz={dz*1e3:+6.1f}mm（随动≈+50 即夹住）"
              f" 水平={dh*1e3:5.1f}mm  开度={backend.get_gripper_width()*1e3:.1f}mm")
        print(f"  TCP 校验: ee=({pe[0]:.4f},{pe[1]:.4f},{pe[2]:.4f}) "
              f"目标 lift=({p_lift[0]:.4f},{p_lift[1]:.4f},{p_lift[2]:.4f}) "
              f"差={np.linalg.norm(pe - p_lift)*1e3:.1f}mm")
        # 闭合后即刻读方块相对 TCP（判断方块是否真的在指间）
        print(f"  闭合后方块相对 ee: {(c0 - pe0)*1e3} mm")
    backend.close()


if __name__ == "__main__":
    main()
