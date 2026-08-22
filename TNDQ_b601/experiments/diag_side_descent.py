#!/usr/bin/env python3
"""侧抓下降剖面 v2：d=0.075 固定，每档复位立方体，扫开度 x 高度。

开度小（0.08）：手指内侧穿透立方体 -> 臂被顶出（与深度无关）；
开度大（0.142）：下降中指尖/指板水平扫到立方体侧面 -> 方块被推飞。
本脚本量化不同开度下的干净下降通道与方块扰动。
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
    cube0, cq = backend.get_cube_pose()
    d = 0.075

    for w_open in (0.10, 0.12, 0.142):
        print(f"--- 开度 w={w_open}")
        warm = {"q": None}
        for z_o in np.arange(0.16, 0.065, -0.01):
            backend.cube.set_world_pose(cube0, cq)
            backend.cube.set_linear_velocity(np.zeros(3))
            backend.cube.set_angular_velocity(np.zeros(3))
            p_w = P.CUBE_POS + d * xg + np.array([0, 0, z_o - P.CUBE_POS[2]])
            q = solve_ik_multi(world_to_dh(p_w), quat, n_init=64, seed=5,
                               q_warm=warm["q"])[0]
            if q is None:
                print(f"  z_o={z_o:.3f}: IK无解"); continue
            warm["q"] = q
            backend.reset_to(q, gripper_width=w_open)
            for _ in range(8):
                qq, _ = backend.get_joint_state()
                backend.apply_arm_torques(dyn.gravity_vector(qq))
                backend.step()
            p_ee, _ = backend.get_ee_pose()
            dp = np.linalg.norm(p_ee - (fk_pose(q)[0] + B601_BASE_PREFIX))
            cube, _ = backend.get_cube_pose()
            dc = np.linalg.norm(cube - cube0)
            flag = "  <== 臂被顶" if dp > 0.003 else ""
            flag += "  [cube动]" if dc > 0.004 else ""
            print(f"  z_o={z_o:.3f}  dp={dp:.4f}  cube_shift={dc:.4f}{flag}")
    backend.close()


if __name__ == "__main__":
    main()
