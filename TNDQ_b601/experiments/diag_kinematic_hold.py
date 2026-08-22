#!/usr/bin/env python3
"""就位失败归因探针：kinematic teleport（每步硬设关节位置，不涉动力学）
vs 力矩保持。若前者 dp≈0，则就位失败 = 动力学层问题（重力/惯量失配
在深折叠构型发散），而非 PhysX 求解器不收敛。
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from config.params import B601_BASE_PREFIX
from experiments.ik_lib import fk_pose, solve_ik_multi, world_to_dh, make_side_pose
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True, solver_pos_iter=32, solver_vel_iter=16)
    backend.setup()
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    phi = np.deg2rad(-14.4)
    _, quat = make_side_pose(phi, z_up=True)
    cube0, cq0 = backend.get_cube_pose()
    # cube 挪走，纯测就位
    backend.cube.set_world_pose(cube0 + np.array([2.0, 0, 0]), cq0)

    for d in (0.080, 0.075):
        p_w = P.CUBE_POS + d * np.array([np.cos(phi), np.sin(phi), 0.0])
        q, margin, _ = solve_ik_multi(world_to_dh(p_w), quat, n_init=64, seed=5)
        print(f"== d={d:.3f}  q={np.round(q, 3).tolist()} "
              f"margin={np.rad2deg(margin):.1f} deg ==")

        # [A] kinematic：每步硬设位置+速度 0
        backend.reset_to(q, gripper_width=0.14)
        for _ in range(100):
            backend.articulation.set_joint_positions(
                backend._row(q), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(6)), joint_indices=backend.arm_idx)
            backend.step()
        p_ee, _ = backend.get_ee_pose()
        dp = np.linalg.norm(p_ee - (fk_pose(q)[0] + B601_BASE_PREFIX))
        qq, _ = backend.get_joint_state()
        print(f"  [A] kinematic  dp={dp:.4f}  dq={np.round(qq - q, 3).tolist()}")

        # [B] 力矩保持：reset_to 后 tau=g(q)
        backend.reset_to(q, gripper_width=0.14)
        for _ in range(100):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.set_gripper(0.14)
            backend.step()
        p_ee, _ = backend.get_ee_pose()
        dp = np.linalg.norm(p_ee - (fk_pose(q)[0] + B601_BASE_PREFIX))
        qq, _ = backend.get_joint_state()
        print(f"  [B] 力矩保持   dp={dp:.4f}  dq={np.round(qq - q, 3).tolist()}")

        # [C] Isaac 实测重力矩（get_measured_joint_efforts 静止读回）对照
        g_nom = dyn.gravity_vector(q)
        print(f"  [C] g_nom={np.round(g_nom, 2).tolist()}")
    backend.close()


if __name__ == "__main__":
    main()
