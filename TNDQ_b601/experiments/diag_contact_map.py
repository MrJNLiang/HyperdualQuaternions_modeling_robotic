#!/usr/bin/env python3
"""侧抓分阶段对位诊断 —— 回答两个问题：

[Q1] 方块何时掉下支柱？就位保持后（闭合前）立即读 cube 位姿，
     与闭合后对比，区分"就位碰落" vs "闭合碰落"。
[Q2] 手指到底在哪？读 gripper_link / gripper_left / gripper_right
     世界位姿（USD xform），把 cube 中心与手指原点变换到 gripper 系，
     输出径向(x_g)/切向(y_g)/竖直(z_g)分量，直接量化垫-方块错位。

每档先有方块、再无方块（cube 挪 2 m）对照 dp，判断就位本身是否被顶。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_contact_map.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from config.params import B601_BASE_PREFIX, ISAAC_ROBOT_PRIM
from experiments.ik_lib import fk_pose, solve_ik_multi, world_to_dh, make_side_pose
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


def _quat_to_R(r):
    w, x, y, z = r
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True, solver_pos_iter=32, solver_vel_iter=16)
    backend.setup()
    from pxr import Usd, UsdGeom

    def link_world_pose(rel):
        prim = backend.stage.GetPrimAtPath(ISAAC_ROBOT_PRIM + rel)
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        gq = xf.ExtractRotation().GetQuaternion()
        q = np.array([gq.GetReal(), *gq.GetImaginary()], dtype=float)
        return np.array(xf.ExtractTranslation(), dtype=float), q / np.linalg.norm(q)

    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    phi = np.deg2rad(-14.4)
    _, quat = make_side_pose(phi, z_up=True)
    cube0, cq0 = backend.get_cube_pose()
    print(f"cube0={np.round(cube0, 4).tolist()}  柱顶={P.PEDESTAL_H:.3f}")

    for d in (0.090, 0.080, 0.075, 0.070):
        p_w = P.CUBE_POS + d * _quat_to_R(quat)[:, 0]
        q, margin, _ = solve_ik_multi(world_to_dh(p_w), quat, n_init=64, seed=5)
        if q is None:
            print(f"d={d:.3f} IK无解"); continue
        print(f"== d={d:.3f}  margin={np.rad2deg(margin):.1f} deg ==")
        for tag, present in (("有方块", True), ("无方块", False)):
            backend.cube.set_world_pose(
                cube0 if present else cube0 + np.array([2.0, 0, 0]), cq0)
            backend.cube.set_linear_velocity(np.zeros(3))
            backend.cube.set_angular_velocity(np.zeros(3))
            backend.reset_to(q, gripper_width=0.14)
            for _ in range(100):
                qq, _ = backend.get_joint_state()
                backend.apply_arm_torques(dyn.gravity_vector(qq))
                backend.set_gripper(0.14)
                backend.step()
            # ---- 就位保持后（闭合前）快照 ----
            cube1, _ = backend.get_cube_pose()
            p_ee, q_ee = backend.get_ee_pose()
            p_fk = fk_pose(q)[0] + B601_BASE_PREFIX
            dp = np.linalg.norm(p_ee - p_fk)
            fell = cube1[2] < P.PEDESTAL_H - 0.005 and present
            print(f"  [{tag}] 就位后 dp={dp:.4f}  cube={np.round(cube1, 4).tolist()}"
                  f"{'  <== 方块已掉!' if fell else ''}")
            if present:
                Rg = _quat_to_R(q_ee)
                rel_c = Rg.T @ (cube1 - p_ee)
                print(f"         cube@gripper系 (x径向外,y开合,z竖直)="
                      f"{np.round(rel_c, 4).tolist()}")
                for name, rel in (("left ", backend._FINGER_RELS[0]),
                                  ("right", backend._FINGER_RELS[1])):
                    pf, _ = link_world_pose(rel)
                    print(f"         finger_{name} origin @gripper系="
                          f"{np.round(Rg.T @ (pf - p_ee), 4).tolist()}  "
                          f"@cube系={np.round(Rg.T @ (pf - cube1), 4).tolist()}")
    backend.close()


if __name__ == "__main__":
    main()
