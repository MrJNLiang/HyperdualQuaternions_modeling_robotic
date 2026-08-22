#!/usr/bin/env python3
"""顶出归因二选一：(1) 提高求解器迭代；(2) 关 arm 自碰撞。"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from config.params import B601_BASE_PREFIX
from experiments.ik_lib import fk_pose, solve_ik_multi, world_to_dh, make_side_pose
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()

def run(tag, **kw):
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True, **kw)
    backend.setup()
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    phi = np.deg2rad(-14.4)
    R, quat = make_side_pose(phi, z_up=True)
    xg = R[:, 0]
    p_grasp = P.CUBE_POS + 0.075 * xg
    q, _, _ = solve_ik_multi(world_to_dh(p_grasp), quat, n_init=64, seed=5)
    # 立方体挪走，只看臂自身
    cube0, cq = backend.get_cube_pose()
    backend.cube.set_world_pose(cube0 + np.array([2.0, 0, 0]), cq)
    backend.reset_to(q, gripper_width=0.08)
    for _ in range(10):
        qq, _ = backend.get_joint_state()
        backend.apply_arm_torques(dyn.gravity_vector(qq))
        backend.step()
    p_ee, _ = backend.get_ee_pose()
    dp = np.linalg.norm(p_ee - (fk_pose(q)[0] + B601_BASE_PREFIX))
    print(f"RESULT [{tag}] dp={dp:.4f}")
    backend.close()

run("关自碰撞", arm_self_collision=False)
