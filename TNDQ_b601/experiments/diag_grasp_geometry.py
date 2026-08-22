#!/usr/bin/env python3
"""v7 抓取几何对账 —— Isaac 真实位姿 vs DQ FK 预测。

exp1 v7 首跑：下降段在目标前 ~3.5 cm 被阻、夹爪闭合未触立方体
（width 读到 0.040 < 0.045）、cube 未被提升。本脚本在给定 q 下读
Isaac 中 gripper_link / gripper_left / gripper_right / 立方体的世界
位姿，与 fk_pose(q) 对账，定位手指-立方体真实相对几何。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_grasp_geometry.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (B601_BASE_PREFIX, CUBE_POS, Q_INIT)
from experiments.ik_lib import fk_pose, solve_ik_multi, world_to_dh, make_tool_pose
import config.params as P
import experiments.ik_lib as IK

IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()

_FINGER_RELS = (
    "/Geometry/base_link/link1/link2/link3/link4/link5/link6"
    "/gripper_link/gripper_left",
    "/Geometry/base_link/link1/link2/link3/link4/link5/link6"
    "/gripper_link/gripper_right",
)


def _prim_pose(backend, rel):
    from pxr import Usd, UsdGeom
    prim = backend.stage.GetPrimAtPath(P.ISAAC_ROBOT_PRIM + rel)
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    qq = xf.ExtractRotation().GetQuaternion()
    quat = np.array([qq.GetReal(), *qq.GetImaginary()], dtype=float)
    return np.array(xf.ExtractTranslation(), dtype=float), quat / np.linalg.norm(quat)


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()

    azim = np.deg2rad(-14.4)
    _, q_tool = make_tool_pose(np.deg2rad(170), azim)
    grasp = P.SETPOINT_POS
    q_grasp, _, _ = solve_ik_multi(world_to_dh(grasp), q_tool, n_init=32, seed=5)
    # exp1 v7 首跑末态（CSV 读回）
    q_end = np.array([-0.25, -1.875, -1.634, 1.106, 0.0, -0.006])

    for tag, q in [("q_grasp(IK设计)", q_grasp), ("q_end(首跑末态)", q_end)]:
        backend.reset_to(q, gripper_width=0.07)
        for _ in range(10):
            qq, _ = backend.get_joint_state()
            from config.b601_dynamics import B601NominalDynamics
            backend.apply_arm_torques(B601NominalDynamics().gravity_vector(qq))
            backend.step()
        p_fk, r_fk = fk_pose(q)
        p_fk_w = p_fk + B601_BASE_PREFIX
        p_ee, r_ee = backend.get_ee_pose()
        pl, _ = _prim_pose(backend, _FINGER_RELS[0])
        pr, _ = _prim_pose(backend, _FINGER_RELS[1])
        cube_pos, _ = backend.get_cube_pose()
        print(f"\n=== {tag}  q={np.round(q, 3).tolist()} ===")
        print(f"  FK  gripper原点(世界) = {np.round(p_fk_w, 4).tolist()}")
        print(f"  USD gripper原点(世界) = {np.round(p_ee, 4).tolist()}   "
              f"dp={np.linalg.norm(p_ee - p_fk_w):.4f} m")
        print(f"  USD 左指原点 = {np.round(pl, 4).tolist()}")
        print(f"  USD 右指原点 = {np.round(pr, 4).tolist()}")
        print(f"  立方体中心   = {np.round(cube_pos, 4).tolist()}  "
              f"(params CUBE_POS={np.round(CUBE_POS, 4).tolist()})")
        # 手指原点中点 vs 立方体
        mid = 0.5 * (pl + pr)
        print(f"  指根中点-立方体 = {np.round(mid - cube_pos, 4).tolist()}")
        # gripper +x 轴（USD）
        from interfaces.isaac_interface import _quat_to_R
        xg = _quat_to_R(r_ee)[:, 0]
        print(f"  USD gripper +x = {np.round(xg, 3).tolist()}  "
              f"(params TOOL_AXIS={np.round(P.TOOL_AXIS, 3).tolist()})")
    backend.close()


if __name__ == "__main__":
    main()
