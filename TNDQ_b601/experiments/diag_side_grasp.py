#!/usr/bin/env python3
"""v8 侧抓功能性验证：tilt=90（x_g 径向外、手指沿 -x_g 径向插入、
开合沿切向）。扫描插入深度 d：teleport 就位（开指）看 dp 是否被顶
（干涉即顶出）-> 闭合 -> 提升 6cm 看 cube 是否被夹起。
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
    cube0, _ = backend.get_cube_pose()

    warm = {"q": None}
    def ik_at(p_w):
        q = solve_ik_multi(world_to_dh(p_w), quat, n_init=64, seed=5,
                           q_warm=warm["q"])[0]
        if q is not None:
            warm["q"] = q
        return q

    def hold(n):
        for _ in range(n):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.step()

    from pxr import Gf
    for d, w_open in ((0.075, 0.10), (0.075, 0.12), (0.075, 0.142), (0.08, 0.142)):
        open_w = w_open
        backend.cube.set_world_pose(cube0, np.array([1.0, 0.0, 0.0, 0.0]))
        backend.cube.set_linear_velocity(np.zeros(3))
        backend.cube.set_angular_velocity(np.zeros(3))
        backend.world.reset()
        p_grasp = P.CUBE_POS + d * xg
        q = ik_at(p_grasp)
        if q is None:
            print(f"d={d:.3f}: IK无解"); continue
        backend.reset_to(q, gripper_width=open_w)
        hold(10)
        p_ee, _ = backend.get_ee_pose()
        p_fk_w = fk_pose(q)[0] + B601_BASE_PREFIX
        dp_place = np.linalg.norm(p_ee - p_fk_w)
        cube_mid, _ = backend.get_cube_pose()
        print(f"  [d={d:.3f}] FK原点={np.round(p_fk_w,4).tolist()} "
              f"USD原点={np.round(p_ee,4).tolist()} cube={np.round(cube_mid,4).tolist()}")
        # 顶出后 USD 实际几何：gripper 三轴
        from interfaces.isaac_interface import _quat_to_R
        _, r_ee = backend.get_ee_pose()
        Ru = _quat_to_R(r_ee)
        print(f"         USD x_g={np.round(Ru[:,0],3).tolist()} "
              f"被顶方向={np.round(p_ee-p_fk_w,4).tolist()}")
        # 闭合
        for _ in range(60):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.set_gripper(0.025)
            backend.step()
        width_after = backend.get_gripper_width()
        # 提升 6cm（同姿态）
        q_up = ik_at(p_grasp + np.array([0.0, 0.0, 0.06]))
        if q_up is not None:
            backend.articulation.set_joint_positions(
                backend._row(q_up), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(6)), joint_indices=backend.arm_idx)
            for _ in range(30):
                qq, _ = backend.get_joint_state()
                backend.apply_arm_torques(dyn.gravity_vector(qq))
                backend.set_gripper(0.025)
                backend.step()
        cube, _ = backend.get_cube_pose()
        lifted = cube[2] > cube0[2] + 0.02
        print(f"d={d:.3f}  就位dp={dp_place:.4f}  闭合后width={width_after:.4f}  "
              f"cube_z={cube[2]:.4f}（初{cube0[2]:.4f}）  "
              f"{'*** LIFTED ***' if lifted else '未夹起'}")
    backend.close()

if __name__ == "__main__":
    main()
