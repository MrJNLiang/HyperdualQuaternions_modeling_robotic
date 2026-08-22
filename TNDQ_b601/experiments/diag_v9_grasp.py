#!/usr/bin/env python3
"""v9 斜向下抓功能验证（正确几何：原点=指尖端，d=0.028，指垫≈方块中心）。

kinematic 关节空间插值逼近（每步硬设，碰撞即表现为方块移位）->
就位检查 -> 重力保持 + 闭合 -> stall 宽度 -> kinematic 提升 -> 方块 z。
扫描 tilt = 115/120/125/130 @ d=0.028。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_v9_grasp.py
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
    backend = IsaacB601Backend(headless=True, solver_pos_iter=32, solver_vel_iter=16)
    backend.setup()
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    phi = np.deg2rad(-14.4)
    cube0, cq0 = backend.get_cube_pose()

    def kin_set(q, s):
        backend.articulation.set_joint_positions(
            backend._row(q), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_positions(
            backend._row(np.full(2, s)), joint_indices=backend.grip_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)

    for tilt in (115, 120, 125, 130):
        R, quat = make_tool_pose(np.deg2rad(tilt), phi)
        xg, zg = R[:, 0], R[:, 2]
        d = 0.028
        p_grasp = P.CUBE_POS + d * xg + 0.005 * zg
        q_grasp, m, _ = solve_ik_multi(world_to_dh(p_grasp), quat,
                                       n_init=64, seed=5)
        if q_grasp is None:
            print(f"tilt={tilt}: IK无解"); continue
        # 退避位：沿接近轴 -x_g 退 8 cm
        p_off = p_grasp - 0.08 * xg
        q_off, _, _ = solve_ik_multi(world_to_dh(p_off), quat,
                                     n_init=64, seed=5, q_warm=q_grasp)
        if q_off is None:
            print(f"tilt={tilt}: 退避位IK无解"); continue
        # 复位方块
        backend.cube.set_world_pose(cube0, cq0)
        backend.cube.set_linear_velocity(np.zeros(3))
        backend.cube.set_angular_velocity(np.zeros(3))
        # [1] kinematic 逼近（关节插值 60 步，开指）
        max_shift = 0.0
        for a in np.linspace(0.0, 1.0, 61):
            q_i = q_off + a * (q_grasp - q_off)
            kin_set(q_i, 0.07)
            backend.step()
            c, _ = backend.get_cube_pose()
            max_shift = max(max_shift, float(np.linalg.norm(c - cube0)))
        cube_at, _ = backend.get_cube_pose()
        shift_at_grasp = float(np.linalg.norm(cube_at - cube0))
        # [2] 重力保持 + 闭合（力矩级）
        qq = q_grasp.copy()
        for _ in range(160):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.set_gripper(0.0)
            backend.step()
        w_stall = backend.get_gripper_width()
        qq_hold, _ = backend.get_joint_state()
        hold_drift = float(np.max(np.abs(qq_hold - q_grasp)))
        # [3] kinematic 提升 8 cm（沿 -x_g 退出 + z 提升）
        p_up = p_grasp + np.array([0.0, 0.0, 0.08]) - 0.02 * xg
        q_up, _, _ = solve_ik_multi(world_to_dh(p_up), quat,
                                    n_init=32, seed=5, q_warm=q_grasp)
        lifted = False
        if q_up is not None:
            for a in np.linspace(0.0, 1.0, 61):
                q_i = qq_hold + a * (q_up - qq_hold)
                kin_set(q_i, 0.0)
                backend.step()
            c, _ = backend.get_cube_pose()
            lifted = c[2] > cube0[2] + 0.03
            cz = c[2]
        else:
            cz = float("nan")
        contact = "接触" if abs(w_stall - P.CUBE_SIZE) < 0.012 else "异常"
        print(f"tilt={tilt} margin={np.rad2deg(m):.0f}deg: "
              f"逼近max方块移位={max_shift:.4f}{'[碰!]' if max_shift > 0.004 else ''}  "
              f"保持漂移={hold_drift:.3f}rad  stall宽={w_stall:.4f}({contact})  "
              f"提升后cube_z={cz:.3f} {'*** 夹起 ***' if lifted else ''}")
    backend.close()


if __name__ == "__main__":
    main()
