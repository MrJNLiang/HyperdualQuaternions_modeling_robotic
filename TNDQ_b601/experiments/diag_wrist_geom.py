#!/usr/bin/env python3
"""侧抓就位时读 link5/link6/gripper 各 prim 世界位置，定位挡路部件。"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from config.params import B601_BASE_PREFIX
from experiments.ik_lib import fk_pose, solve_ik_multi, world_to_dh, make_side_pose
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()

_RELS = {
    "link5": "/Geometry/base_link/link1/link2/link3/link4/link5",
    "link6": "/Geometry/base_link/link1/link2/link3/link4/link5/link6",
    "gripper": "/Geometry/base_link/link1/link2/link3/link4/link5/link6/gripper_link",
}

def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    from pxr import Usd, UsdGeom
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    phi = np.deg2rad(-14.4)
    R, quat = make_side_pose(phi, z_up=True)
    xg = R[:, 0]
    cube0, _ = backend.get_cube_pose()
    print(f"cube0 = {np.round(cube0,4).tolist()}  (顶面 z={cube0[2]+0.0225:.3f})")
    warm = {"q": None}
    d = 0.075
    for z_o in (0.145, 0.135, 0.125):
        p_w = P.CUBE_POS + d * xg + np.array([0, 0, z_o - P.CUBE_POS[2]])
        q = solve_ik_multi(world_to_dh(p_w), quat, n_init=64, seed=5, q_warm=warm["q"])[0]
        warm["q"] = q
        backend.reset_to(q, gripper_width=0.08)
        for _ in range(8):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.step()
        print(f"--- z_o={z_o:.3f}  q={np.round(q,3).tolist()}")
        for name, rel in _RELS.items():
            prim = backend.stage.GetPrimAtPath(P.ISAAC_ROBOT_PRIM + rel)
            xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            p = np.array(xf.ExtractTranslation(), dtype=float)
            print(f"   {name:8s} = {np.round(p,4).tolist()}   距cube_xy={np.linalg.norm(p[:2]-cube0[:2]):.3f}")
    backend.close()

if __name__ == "__main__":
    main()
