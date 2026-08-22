#!/usr/bin/env python3
"""手指滑移轴在 gripper 系的直测：零位姿态下 stroke=0.04，
读两指原点与 gripper 原点，差值投影到 USD gripper 三轴。
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P

def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    from pxr import Usd, UsdGeom
    from interfaces.isaac_interface import _quat_to_R
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    rels = ("/Geometry/base_link/link1/link2/link3/link4/link5/link6/gripper_link/gripper_left",
            "/Geometry/base_link/link1/link2/link3/link4/link5/link6/gripper_link/gripper_right")
    def prim_pos(rel):
        prim = backend.stage.GetPrimAtPath(P.ISAAC_ROBOT_PRIM + rel)
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return np.array(xf.ExtractTranslation(), dtype=float)
    for s in (0.0, 0.04):
        backend.reset_to(P.Q_INIT, gripper_width=None)
        backend.articulation.set_joint_positions(
            backend._row(np.full(2, s)), joint_indices=backend.grip_idx)
        backend.set_gripper(2 * s)
        for _ in range(8):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.step()
        p_ee, r_ee = backend.get_ee_pose()
        Rg = _quat_to_R(r_ee)
        print(f"--- stroke={s}  USD gripper轴 x={np.round(Rg[:,0],3).tolist()} "
              f"y={np.round(Rg[:,1],3).tolist()} z={np.round(Rg[:,2],3).tolist()}")
        for name, rel in zip(("左指", "右指"), rels):
            dp = prim_pos(rel) - p_ee
            print(f"   {name} 世界差={np.round(dp,4).tolist()}  "
                  f"gripper系分量 (x,y,z)={np.round(Rg.T @ dp,4).tolist()}")
    backend.close()

if __name__ == "__main__":
    main()
