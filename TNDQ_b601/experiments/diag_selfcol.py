#!/usr/bin/env python3
"""就位顶出归因：同一抓取位 q，分别（a）立方体在原位、（b）立方体
挪到 2m 外，读 dp。若 (b) 仍顶 -> 自碰撞/求解器；否则 -> 方块接触。
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
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    phi = np.deg2rad(-14.4)
    R, quat = make_side_pose(phi, z_up=True)
    xg = R[:, 0]
    d = 0.075
    p_grasp = P.CUBE_POS + d * xg
    q, m, _ = solve_ik_multi(world_to_dh(p_grasp), quat, n_init=64, seed=5)
    print(f"q_grasp={np.round(q,3).tolist()} margin={np.degrees(m):.1f}deg")
    cube0, cq = backend.get_cube_pose()

    def place(width, tag):
        backend.reset_to(q, gripper_width=width)
        for _ in range(10):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.step()
        p_ee, _ = backend.get_ee_pose()
        dp = np.linalg.norm(p_ee - (fk_pose(q)[0] + B601_BASE_PREFIX))
        print(f"  [{tag}] width={width}  dp={dp:.4f}")

    print("(a) 立方体原位：")
    place(0.08, "a-开0.08")
    print("(b) 立方体挪走：")
    backend.cube.set_world_pose(cube0 + np.array([2.0, 0.0, 0.0]), cq)
    backend.cube.set_linear_velocity(np.zeros(3))
    place(0.08, "b-开0.08")
    place(0.142, "b-开0.142")
    backend.close()

if __name__ == "__main__":
    main()
