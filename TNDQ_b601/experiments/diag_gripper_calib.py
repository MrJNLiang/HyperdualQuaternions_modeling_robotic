#!/usr/bin/env python3
"""夹爪行程/接触经验标定 —— 取代不可靠的 mesh 包围盒代数。

测量 1（开合方向与速率）：固定安全位姿，stroke 取 0/0.02/0.035/0.0715，
读两指 link 原点世界距离 D(s) -> 开/闭方向与每指速率。

测量 2（接触起点）：抓取姿态（tilt 170）下，把 gripper 原点从 z=0.14
向下扫到 0.04（5 mm 步），每档 teleport + 重力保持 5 步，读 USD
gripper 原点与 FK 预测的偏差 dp：dp 突跳处 = 物理接触起点；结合
立方体顶 0.105 / 柱顶 0.06 判定是哪个几何在接触。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_gripper_calib.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config.params as P
import experiments.ik_lib as IK
from config.params import B601_BASE_PREFIX
from experiments.ik_lib import fk_pose, solve_ik_multi, world_to_dh, make_tool_pose

IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()

_FINGER_RELS = (
    "/Geometry/base_link/link1/link2/link3/link4/link5/link6"
    "/gripper_link/gripper_left",
    "/Geometry/base_link/link1/link2/link3/link4/link5/link6"
    "/gripper_link/gripper_right",
)


def _prim_pos(backend, rel):
    from pxr import Usd, UsdGeom
    prim = backend.stage.GetPrimAtPath(P.ISAAC_ROBOT_PRIM + rel)
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    return np.array(xf.ExtractTranslation(), dtype=float)


def _hold(backend, q, n=5):
    from config.b601_dynamics import B601NominalDynamics
    dyn = B601NominalDynamics()
    for _ in range(n):
        qq, _ = backend.get_joint_state()
        backend.apply_arm_torques(dyn.gravity_vector(qq))
        backend.step()


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()

    # ---- 测量 1：指距 vs stroke（零位安全姿态）----
    print("== 测量1 指距-stroke ==")
    for s in (0.0, 0.02, 0.035, 0.0715):
        backend.reset_to(P.Q_INIT, gripper_width=None)
        backend.articulation.set_joint_positions(
            backend._row(np.full(2, s)), joint_indices=backend.grip_idx)
        backend.set_gripper(2 * s)   # 直接按旧口径令 target=stroke 同值
        _hold(backend, P.Q_INIT)
        pl = _prim_pos(backend, _FINGER_RELS[0])
        pr = _prim_pos(backend, _FINGER_RELS[1])
        print(f"  stroke={s:.4f}  D={np.linalg.norm(pl - pr):.4f} m  "
              f"pl={np.round(pl, 3).tolist()} pr={np.round(pr, 3).tolist()}")

    # ---- 测量 2：抓取姿态下扫高度找接触起点 ----
    azim = np.deg2rad(-14.4)
    R_tool, q_tool = make_tool_pose(np.deg2rad(170), azim)
    print("== 测量2 接触起点（stroke=0.035 与 0.0）==")
    for stroke in (0.035, 0.0):
        print(f" --- stroke={stroke}")
        for z_o in np.arange(0.14, 0.039, -0.005):
            p_w = np.array([P.CUBE_POS[0] + 0.005, P.CUBE_POS[1] + 0.001, z_o])
            q, _, _ = solve_ik_multi(world_to_dh(p_w), q_tool, n_init=16, seed=5)
            if q is None:
                print(f"  z={z_o:.3f} IK无解"); continue
            backend.reset_to(q, gripper_width=None)
            backend.articulation.set_joint_positions(
                backend._row(np.full(2, stroke)), joint_indices=backend.grip_idx)
            backend.set_gripper(2 * stroke)
            _hold(backend, q)
            p_ee, _ = backend.get_ee_pose()
            p_fk = fk_pose(q)[0] + B601_BASE_PREFIX
            dp = np.linalg.norm(p_ee - p_fk)
            flag = "  <== 接触" if dp > 0.004 else ""
            print(f"  z_o={z_o:.3f}  dp={dp:.4f}{flag}")
    backend.close()


if __name__ == "__main__":
    main()
