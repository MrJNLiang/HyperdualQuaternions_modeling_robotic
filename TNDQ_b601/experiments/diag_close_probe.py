#!/usr/bin/env python3
"""闭合探针：在抓取位逐步闭合，手指碰到什么？

v9 exp1 三跑现象：几何对位完美（垫心对方块中心偏 ≤6 mm），但手指
从 0.14 闭到目标 0.04 全程无 stall（没停在方块 0.045）——说明该
高度上手指截面没有方块。本诊断 kinematic 硬设抓取位，逐步下发闭合
目标，记录手指行程 stall 值；再 teleport 方块离开重复，对比空闭行程。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_close_probe.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik_multi, world_to_dh, make_tool_pose
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True, solver_pos_iter=32,
                               solver_vel_iter=16)
    backend.setup()
    phi = np.deg2rad(-14.4)
    R, quat = make_tool_pose(np.deg2rad(120), phi)
    xg, zg = R[:, 0], R[:, 2]
    p_grasp = P.CUBE_POS + 0.028 * xg + 0.005 * zg
    q_grasp, m, _ = solve_ik_multi(world_to_dh(p_grasp), quat,
                                   n_init=64, seed=5)
    print(f"抓取位 IK margin={np.rad2deg(m):.1f}deg q={np.round(q_grasp,3)}")
    cube0, cq0 = backend.get_cube_pose()

    def kin_set(q):
        backend.articulation.set_joint_positions(
            backend._row(q), joint_indices=backend.arm_idx)
        # 仅清零手臂速度；手指速度留给位置 drive（全零会抑制闭合）
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(6)), joint_indices=backend.arm_idx)

    def close_and_report(tag):
        # 先全开稳定
        backend.set_gripper(0.14)
        for _ in range(50):
            kin_set(q_grasp)
            backend.step()
        widths = []
        for target in np.arange(0.135, -0.005, -0.005):
            backend.set_gripper(max(target, 0.0))
            for _ in range(30):
                kin_set(q_grasp)
                backend.step()
            widths.append((target, backend.get_gripper_width()))
        print(f"[{tag}] 目标->实测:")
        for tg, w in widths:
            flag = ""
            if tg <= 0.045 and abs(w - tg) > 0.004:
                flag = f"  <- stall（实测 {w:.4f} > 目标）"
            print(f"  {tg:.3f} -> {w:.4f}{flag}")
        c, _ = backend.get_cube_pose()
        print(f"[{tag}] 方块移位 {np.linalg.norm(c - cube0):.4f} m")

    close_and_report("有方块")
    # 方块传送走，重复空闭
    backend.cube.set_world_pose(cube0 + np.array([0.8, 0.6, 0.05]), cq0)
    for _ in range(20):
        backend.step()
    close_and_report("无方块(对照)")
    backend.close()


if __name__ == "__main__":
    main()
