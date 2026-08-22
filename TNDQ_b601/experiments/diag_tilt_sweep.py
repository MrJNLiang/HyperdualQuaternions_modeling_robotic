#!/usr/bin/env python3
"""v12e 倾角扫描：让方块竖直棱落入手指平板带内，实现面接触夹持。

v12d 裁决：tilt=120 下方块竖直棱 z_g 投影 ±19.5 mm 恰在平板带
±19.6 mm 之外 2.9 mm -> 接触发生在棱/角点（不对称），一提升即
被打飞（三档深度全部复现，方向恒为 +x_g 外侧）。

几何判据：方块半宽 22.5 mm，竖直棱投影 = ±22.5*|cos(tilt-90)|；
平板带 ±19.6 mm -> 需 cos(tilt-90) <= 0.871 -> tilt <= ~116.7 deg。
本脚本扫 tilt ∈ {116, 112, 108}（z_g 偏置保持 -0.042，d_x=0.028），
各档：IK -> 硬复位 -> 慢闭读 stall -> 目标 0.058 提升 5 cm 看随动。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_tilt_sweep.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik, world_to_dh, make_tool_pose
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()

    q0 = np.array([-0.3, -1.6, -0.6, -0.6, 0.0, 0.0])
    q_prev = q0
    phi = -14.4

    def teleport_arm(qa, n=30):
        for _ in range(n):
            backend.articulation.set_joint_positions(
                backend._row(qa), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.step()

    for tilt in (116.0, 112.0, 108.0):
        R, quat = make_tool_pose(np.deg2rad(tilt), np.deg2rad(phi))
        x_g, z_g = R[:, 0], R[:, 2]
        grasp = P.CUBE_POS + 0.028 * x_g - 0.042 * z_g
        q, res, ok = solve_ik(world_to_dh(grasp), quat, q_prev)
        if not ok:
            print(f"tilt={tilt}: IK 失败 {res}")
            continue
        q_prev = q
        # 方块归位 + 硬复位 + 全开保持
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

        # 慢闭：drive 目标 58 mm，2 s（过盈 ~4 mm/侧）
        w_target = 0.058
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
        print(f"tilt={tilt:5.1f}: stall 开度={w_stall*1e3:5.1f}mm "
              f"闭合扰动=({d_close[0]*1e3:+5.1f},{d_close[1]*1e3:+5.1f},"
              f"{d_close[2]*1e3:+5.1f})mm")

        # 提升 5 cm（竖直），保持 drive，1 s
        p_lift = grasp + np.array([0.0, 0.0, 0.05])
        ql, rl, okl = solve_ik(world_to_dh(p_lift), quat, q)
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
        print(f"  提升后: dz={dz*1e3:+6.1f}mm（随动≈+50 即夹住）"
              f" 水平={dh*1e3:5.1f}mm")
    backend.close()


if __name__ == "__main__":
    main()
