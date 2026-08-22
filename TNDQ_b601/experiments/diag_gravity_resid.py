#!/usr/bin/env python3
"""重力残差直测：kinematic 硬保持目标构型（每步硬设位置+速度0），
读 get_measured_joint_efforts（PhysX 为维持运动学约束实际施加的
关节力 = 真实重力/接触载荷），与名义 gravity_vector 对照。

在三个构型测：Q_INIT（自检基准）、tilt=120 抓取位、退避位。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_gravity_resid.py
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
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    cube0, cq0 = backend.get_cube_pose()
    backend.cube.set_world_pose(cube0 + np.array([2.0, 0, 0]), cq0)

    phi = np.deg2rad(-14.4)
    R, quat = make_tool_pose(np.deg2rad(120), phi)
    xg, zg = R[:, 0], R[:, 2]
    p_grasp = P.CUBE_POS + 0.028 * xg + 0.005 * zg
    q_grasp, _, _ = solve_ik_multi(world_to_dh(p_grasp), quat, n_init=64, seed=5)

    cases = [("Q_INIT", np.asarray(P.Q_INIT, dtype=float)),
             ("抓取位 tilt120", q_grasp)]
    for tag, q in cases:
        for _ in range(50):
            backend.articulation.set_joint_positions(
                backend._row(q), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_positions(
                backend._row(np.full(2, 0.07)), joint_indices=backend.grip_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.step()
        tau_meas = backend.get_measured_joint_efforts()
        g_nom = dyn.gravity_vector(q)
        resid = tau_meas - g_nom
        print(f"[{tag}] q={np.round(q, 3).tolist()}")
        print(f"   实测约束力矩 = {np.round(tau_meas, 3).tolist()}")
        print(f"   名义重力矩   = {np.round(g_nom, 3).tolist()}")
        print(f"   残差(实测-名义) = {np.round(resid, 3).tolist()}  "
              f"|残差|={np.linalg.norm(resid):.3f}")
    backend.close()


if __name__ == "__main__":
    main()
