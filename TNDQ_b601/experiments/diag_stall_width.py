#!/usr/bin/env python3
"""闭合平衡宽度直测：d=0.054（干净就位），手指闭合目标=0（夹到底），
读平衡宽度。有方块时平衡宽度 = 方块宽 + 2*垫偏置；无方块时 ≈ 0。
同时扫 d：0.054/0.06/0.066/0.072。
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik_multi, world_to_dh, make_side_pose
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()

def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True, solver_pos_iter=32, solver_vel_iter=16)
    backend.setup()
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    phi = np.deg2rad(-14.4)
    R, quat = make_side_pose(phi, z_up=True)
    xg = R[:, 0]
    cube0, cq = backend.get_cube_pose()

    def close_eq(d, with_cube):
        if with_cube:
            backend.cube.set_world_pose(cube0, cq)
        else:
            backend.cube.set_world_pose(cube0 + np.array([2.0, 0, 0]), cq)
        backend.cube.set_linear_velocity(np.zeros(3))
        backend.cube.set_angular_velocity(np.zeros(3))
        p = P.CUBE_POS + d * xg
        q, m, _ = solve_ik_multi(world_to_dh(p), quat, n_init=64, seed=5)
        backend.reset_to(q, gripper_width=0.14)
        for _ in range(120):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.set_gripper(0.0)
            backend.step()
        w = backend.get_gripper_width()
        cube, _ = backend.get_cube_pose()
        return w, cube[2]

    for d in (0.054, 0.060, 0.066, 0.072):
        w0, _ = close_eq(d, with_cube=False)
        w1, cz = close_eq(d, with_cube=True)
        print(f"d={d:.3f}  无方块平衡宽={w0:.4f}  有方块平衡宽={w1:.4f}  "
              f"cube_z={cz:.4f}  {'接触!' if w1 > w0 + 0.005 else '未触'}")
    backend.close()

if __name__ == "__main__":
    main()
