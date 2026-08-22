#!/usr/bin/env python3
"""抓取姿态下手指世界坐标直读：z_o 扫描，读两指原点世界位置，
与立方体(中心 z=0.0825, 顶 0.105)/柱(顶 0.06) 对照，直接确定
手指实际伸展方向与深度（终结 mesh 包围盒代数之争）。
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from config.params import B601_BASE_PREFIX
from experiments.ik_lib import fk_pose, solve_ik_multi, world_to_dh, make_tool_pose
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()

_FINGER_RELS = (
    "/Geometry/base_link/link1/link2/link3/link4/link5/link6/gripper_link/gripper_left",
    "/Geometry/base_link/link1/link2/link3/link4/link5/link6/gripper_link/gripper_right",
)

def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    from pxr import Usd, UsdGeom
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    azim = np.deg2rad(-14.4)
    _, q_tool = make_tool_pose(np.deg2rad(170), azim)
    for z_o in (0.13, 0.11, 0.09, 0.07):
        p_w = np.array([P.CUBE_POS[0], P.CUBE_POS[1], z_o])
        q, _, _ = solve_ik_multi(world_to_dh(p_w), q_tool, n_init=16, seed=5)
        backend.reset_to(q, gripper_width=0.08)
        for _ in range(5):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.step()
        p_ee, _ = backend.get_ee_pose()
        poses = []
        for rel in _FINGER_RELS:
            prim = backend.stage.GetPrimAtPath(P.ISAAC_ROBOT_PRIM + rel)
            xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            poses.append(np.array(xf.ExtractTranslation(), dtype=float))
        print(f"z_o={z_o:.2f}  ee={np.round(p_ee,3).tolist()}")
        print(f"   左指原点={np.round(poses[0],3).tolist()}  右指原点={np.round(poses[1],3).tolist()}")
        print(f"   指原点-ee = {np.round(poses[0]-p_ee,4).tolist()} / {np.round(poses[1]-p_ee,4).tolist()}")
    backend.close()

if __name__ == "__main__":
    main()
