#!/usr/bin/env python3
"""v14 抓取深度 d_z 扫描（teleport 级裁决）。

v14 事实链：
  - v12c（d_z=-0.042）：指垫接触盒内无方块面元，手指夹住支柱
    （stall 43 mm ≡ 支柱 y_g 投影宽 42.6 mm）；
  - v14a（d_z=+0.005，垫心≈方块中心）：e2e 闭合阶段方块被推飞
    （刀片/指尖结构先于指垫接触，gw 140→92 mm 间方块移位 7 mm）。
本脚本对 d_z ∈ {+0.005, +0.011, +0.015, -0.002}（d_x=0.028 固定）：
  IK -> teleport 抓取位 -> kinematic 插入核对（方块移位）-> 慢闭到
  43 mm（读 stall 与扰动）-> 关节插值提升 5 cm（读随动）。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_depth_sweep_v14.py
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

    for d_z in (0.005, 0.011, 0.015, -0.002):
        grasp = P.CUBE_POS + 0.028 * P.TOOL_AXIS + d_z * P._TOOL_Z
        q, res, ok = solve_ik(world_to_dh(grasp), P.R_TOOL_QUAT, q_prev)
        if not ok:
            print(f"d_z={d_z*1e3:+5.1f}mm: IK 失败 {res}")
            continue
        # 退避位（沿接近轴退 8 cm）
        q_off, res_o, ok_o = solve_ik(
            world_to_dh(grasp - 0.08 * P.TOOL_AXIS), P.R_TOOL_QUAT, q)
        if not ok_o:
            print(f"d_z={d_z*1e3:+5.1f}mm: 退避位 IK 失败 {res_o}")
            continue
        q_prev = q
        # 方块归位 + teleport 退避位（硬复位避免扫掠）
        cy = P.CUBE_YAW
        cube_q = np.array([np.cos(0.5 * cy), 0.0, 0.0, np.sin(0.5 * cy)])
        cube_p = np.array(P.CUBE_POS, dtype=float)
        cube_p[2] += 1e-3
        backend.cube.set_world_pose(cube_p, cube_q)
        backend.cube.set_linear_velocity(np.zeros(3))
        backend.cube.set_angular_velocity(np.zeros(3))
        backend.reset_to(q_off, gripper_width=P.GRIPPER_OPENING)
        teleport_arm(q_off, 60)
        c0, _ = backend.get_cube_pose()
        # kinematic 插入核对（碰撞表现为方块移位）
        max_shift = 0.0
        for a in np.linspace(0.0, 1.0, 61):
            qi = q_off + a * (q - q_off)
            backend.articulation.set_joint_positions(
                backend._row(qi), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.step()
            ck, _ = backend.get_cube_pose()
            max_shift = max(max_shift, float(np.linalg.norm(ck - c0)))
        print(f"d_z={d_z*1e3:+5.1f}mm: 插入段方块最大移位={max_shift*1e3:.1f}mm")

        # 慢闭：drive 目标 43 mm（CUBE_SIZE-2 mm 过盈），2 s
        w_target = P.GRIPPER_GRASP
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
        print(f"d_z={d_z*1e3:+5.1f}mm: stall={w_stall*1e3:5.1f}mm "
              f"闭合扰动=({d_close[0]*1e3:+5.1f},{d_close[1]*1e3:+5.1f},"
              f"{d_close[2]*1e3:+5.1f})mm")

        # 提升：保持 drive，关节插值 5 cm，1 s
        p_lift = grasp + np.array([0.0, 0.0, 0.05])
        ql, rl, okl = solve_ik(world_to_dh(p_lift), P.R_TOOL_QUAT, q)
        if not okl:
            print("  提升位 IK 失败")
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
        c, _ = backend.get_cube_pose()
        dz = c[2] - c0[2]
        dh = np.linalg.norm(c[:2] - c0[:2])
        verdict = "夹住" if dz > 0.03 else ("滑脱" if dz < 0.01 else "部分")
        print(f"  提升后: dz={dz*1e3:+6.1f}mm 水平={dh*1e3:5.1f}mm "
              f"开度={backend.get_gripper_width()*1e3:.1f}mm -> {verdict}")
    backend.close()


if __name__ == "__main__":
    main()
