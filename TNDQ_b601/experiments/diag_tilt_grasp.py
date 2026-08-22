#!/usr/bin/env python3
"""斜向下抓功能扫描（v9 候选定型实验）。

手指几何已修正：手指沿 gripper -x_g 伸展。tilt≈110~130 deg 时 x_g
斜向下 -> 手指斜向下探向方块，对应用户指定运动形态（举高->前移->
转腕向下->下降->夹取）。

每档：IK（无方块解做 teleport）-> 重力保持 100 步 -> 闭合到 0（120
步）-> 读 stall 宽度与 cube 位姿。stall 宽 ≈ 方块宽 0.045 => 真实
接触；cube 未掉 => 就位干净。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_tilt_grasp.py
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
    backend = IsaacB601Backend(headless=True, solver_pos_iter=32, solver_vel_iter=16)
    backend.setup()
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    phi = np.deg2rad(-14.4)
    cube0, cq0 = backend.get_cube_pose()
    print(f"cube0={np.round(cube0, 4).tolist()}  方块宽={P.CUBE_SIZE}")

    cases = [(110, 0.12), (120, 0.12), (120, 0.10), (130, 0.11),
             (130, 0.09), (140, 0.10), (150, 0.10)]
    for tilt_deg, d in cases:
        R, quat = make_tool_pose(np.deg2rad(tilt_deg), phi)
        ax = R[:, 0]
        p_w = P.CUBE_POS + d * ax
        q, m, _ = solve_ik_multi(world_to_dh(p_w), quat, n_init=64, seed=5)
        if q is None:
            print(f"tilt={tilt_deg} d={d:.2f}: IK无解"); continue
        # 复位方块
        backend.cube.set_world_pose(cube0, cq0)
        backend.cube.set_linear_velocity(np.zeros(3))
        backend.cube.set_angular_velocity(np.zeros(3))
        backend.reset_to(q, gripper_width=0.14)
        # 就位保持
        for _ in range(100):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.set_gripper(0.14)
            backend.step()
        cube_mid, _ = backend.get_cube_pose()
        shift = np.linalg.norm(cube_mid - cube0)
        # 闭合到底
        for _ in range(120):
            qq, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(qq))
            backend.set_gripper(0.0)
            backend.step()
        w = backend.get_gripper_width()
        cube_end, _ = backend.get_cube_pose()
        fell = cube_end[2] < P.PEDESTAL_H - 0.005
        contact = "接触!" if w > P.CUBE_SIZE - 0.01 else "闭合到底"
        print(f"tilt={tilt_deg} d={d:.2f} margin={np.rad2deg(m):.0f}deg: "
              f"就位cube移位={shift:.3f}{'[掉!]' if shift > 0.01 else ''}  "
              f"stall宽={w:.4f} ({contact})  "
              f"cube_z={cube_end[2]:.3f}{'[掉!]' if fell else ''}")
    backend.close()


if __name__ == "__main__":
    main()
