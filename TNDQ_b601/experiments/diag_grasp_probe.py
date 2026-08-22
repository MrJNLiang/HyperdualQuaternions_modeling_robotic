#!/usr/bin/env python3
"""功能性抓取探针 —— 用物理结果直接定抓取深度，绕开 mesh 代数。

标定已定：+stroke=张开，指距 D=2*stroke（width 映射本身正确）；
gripper 原点处两指原点重合（原点在指根端），手指沿 gripper +x
（抓取姿态下朝下）伸出。剩余未知量：指垫有效长度/接触几何。
本脚本做功能实验：

Phase 1（下降接触剖面，开指 0.08）：z_o 0.13 -> 0.05 步进 2.5 mm，
teleport+重力保持，打印 dp（USD vs FK）与立方体位移，识别谁在挡。

Phase 2（功能抓取扫描）：z_o in {0.130..0.100}，每档
  开指 teleport -> 保持 -> 闭合到 0.025 -> 保持 -> 提升 6 cm，
  读立方体 z：若被夹起则报告 LIFTED，确定可用抓取深度。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_grasp_probe.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config.params as P
import experiments.ik_lib as IK
from config.params import B601_BASE_PREFIX
from experiments.ik_lib import fk_pose, solve_ik_multi, world_to_dh, make_tool_pose

IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


def _hold(backend, n=5):
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    for _ in range(n):
        qq, _ = backend.get_joint_state()
        backend.apply_arm_torques(dyn.gravity_vector(qq))
        backend.step()


def _ik_at(z_o):
    azim = np.deg2rad(-14.4)
    _, q_tool = make_tool_pose(np.deg2rad(170), azim)
    p_w = np.array([P.CUBE_POS[0], P.CUBE_POS[1], z_o])
    q, _, _ = solve_ik_multi(world_to_dh(p_w), q_tool, n_init=16, seed=5)
    return q


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()

    cube0, _ = backend.get_cube_pose()

    # ---- Phase 1：下降接触剖面（开指）----
    print("== Phase1 下降接触剖面（width=0.08 开指）==")
    for z_o in np.arange(0.130, 0.049, -0.0025):
        q = _ik_at(z_o)
        if q is None:
            print(f"  z_o={z_o:.4f} IK无解"); continue
        backend.reset_to(q, gripper_width=0.08)
        _hold(backend, 5)
        p_ee, _ = backend.get_ee_pose()
        p_fk = fk_pose(q)[0] + B601_BASE_PREFIX
        cube, _ = backend.get_cube_pose()
        dp = np.linalg.norm(p_ee - p_fk)
        dc = np.linalg.norm(cube - cube0)
        flag = "  <== 接触" if dp > 0.003 else ""
        flag += "  [cube动了!]" if dc > 0.003 else ""
        print(f"  z_o={z_o:.4f}  dp={dp:.4f}  cube_shift={dc:.4f}{flag}")

    # ---- Phase 2：功能抓取扫描 ----
    print("== Phase2 功能抓取扫描（闭合->提升）==")
    for z_o in (0.130, 0.125, 0.120, 0.115, 0.110, 0.105, 0.100):
        q = _ik_at(z_o)
        if q is None:
            print(f"  z_o={z_o:.3f} IK无解"); continue
        # 开指就位
        backend.reset_to(q, gripper_width=0.08)
        _hold(backend, 10)
        # 闭合（drive 位置目标 0.025）
        from config.b601_dynamics import B601NominalDynamics
        dyn = B601NominalDynamics()
        for _ in range(40):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.set_gripper(0.025)
            backend.step()
        width_after = backend.get_gripper_width()
        # 提升 6 cm（teleport 到新 IK 解）
        q_up = _ik_at(z_o + 0.06)
        if q_up is not None:
            backend.articulation.set_joint_positions(
                backend._row(q_up), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(6)), joint_indices=backend.arm_idx)
            for _ in range(30):
                qq, _ = backend.get_joint_state()
                backend.apply_arm_torques(dyn.gravity_vector(qq))
                backend.set_gripper(0.025)
                backend.step()
        cube, _ = backend.get_cube_pose()
        lifted = cube[2] > cube0[2] + 0.02
        print(f"  z_o={z_o:.3f}  闭合后width={width_after:.4f}  "
              f"cube_z={cube[2]:.4f} (初值{cube0[2]:.4f})  "
              f"{'*** LIFTED ***' if lifted else '未夹起'}")
    backend.close()


if __name__ == "__main__":
    main()
