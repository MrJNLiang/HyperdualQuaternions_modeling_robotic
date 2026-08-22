#!/usr/bin/env python3
"""手指运动学 Isaac 直测标定（无方块，纯夹爪驱动）。

每步硬设关节保持臂不动（kinematic），命令 stroke s=0 -> 0.07，
读 gripper_left/right link 世界位姿，在 gripper 系内差分：
  [1] 滑移轴方向（单位向量，gripper 系）：手指开合到底沿哪条轴；
  [2] 原点位置 vs stroke：验证 (-0.042, ∓s, 0)；
  [3] 指尖端判定：mesh 沿伸展向的远端世界位置（用 link 姿态把
      mesh bbox 角点变换到世界，取沿滑移轴最外侧点），给出
      指尖/指垫中心相对 gripper 原点的完整坐标。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_finger_kin.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
from config.params import ISAAC_ROBOT_PRIM


def _quat_to_R(r):
    w, x, y, z = r
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    from pxr import Usd, UsdGeom

    def pose_of(rel):
        prim = backend.stage.GetPrimAtPath(ISAAC_ROBOT_PRIM + rel)
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        gq = xf.ExtractRotation().GetQuaternion()
        q = np.array([gq.GetReal(), *gq.GetImaginary()], dtype=float)
        return np.array(xf.ExtractTranslation(), dtype=float), q / np.linalg.norm(q)

    # 臂保持伸直的良定构型（零位附近，kinematic 硬设）
    q_hold = np.array([0.0, -0.6, -0.6, 0.0, 0.0, 0.0])
    strokes = np.arange(0.0, 0.0701, 0.01)
    data = {name: [] for name in ("left", "right")}
    for s in strokes:
        backend.articulation.set_joint_positions(
            backend._row(q_hold), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_positions(
            backend._row(np.full(2, s)), joint_indices=backend.grip_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)
        backend.step()
        backend.step()
        p_g, q_g = backend.get_ee_pose()
        Rg = _quat_to_R(q_g)
        for name, rel in zip(("left", "right"), backend._FINGER_RELS):
            pf, _ = pose_of(rel)
            data[name].append(Rg.T @ (pf - p_g))   # gripper 系坐标

    for name in ("left", "right"):
        arr = np.array(data[name])
        axis = arr[-1] - arr[0]
        axis_u = axis / np.linalg.norm(axis)
        print(f"[{name}] origin@s=0  = {np.round(arr[0], 4).tolist()}")
        print(f"[{name}] origin@s=0.07 = {np.round(arr[-1], 4).tolist()}")
        print(f"[{name}] 滑移轴(gripper系) = {np.round(axis_u, 4).tolist()}  "
              f"行程={np.linalg.norm(axis):.4f}")

    # 手指 prim extent（局部 bbox）-> 世界，在 gripper 系内给出覆盖范围
    from pxr import UsdGeom as _UG
    p_g, q_g = backend.get_ee_pose()
    Rg = _quat_to_R(q_g)
    for name, rel in zip(("left", "right"), backend._FINGER_RELS):
        prim = backend.stage.GetPrimAtPath(ISAAC_ROBOT_PRIM + rel)
        bnd = _UG.Boundable(prim)
        ext = bnd.ComputeExtentFromPlugins()
        if not ext or len(ext) < 6:
            ext = bnd.GetExtentAttr().Get()
        lo, hi = np.array(ext[:3], dtype=float), np.array(ext[3:6], dtype=float)
        corners = np.array([[x, y, z]
                            for x in (lo[0], hi[0])
                            for y in (lo[1], hi[1])
                            for z in (lo[2], hi[2])])
        xfm = _UG.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        Rm = np.array([[xfm[i][j] for j in range(3)] for i in range(3)],
                      dtype=float)
        tm = np.array(xfm.ExtractTranslation(), dtype=float)
        world_pts = corners @ Rm.T + tm
        local = (world_pts - p_g) @ Rg
        print(f"[{name}] extent(gripper系): "
              f"x[{local[:,0].min():.3f},{local[:,0].max():.3f}] "
              f"y[{local[:,1].min():.3f},{local[:,1].max():.3f}] "
              f"z[{local[:,2].min():.3f},{local[:,2].max():.3f}]")
    # gripper_link 本体的 extent 同法（确认腕部壳体覆盖）
    prim = backend.stage.GetPrimAtPath(ISAAC_ROBOT_PRIM + backend._GRIPPER_LINK_REL)
    bnd = _UG.Boundable(prim)
    ext = bnd.ComputeExtentFromPlugins()
    if not ext or len(ext) < 6:
        ext = bnd.GetExtentAttr().Get()
    print(f"[gripper_link] extent(局部)={np.round(np.array(ext), 4).tolist()}")
    backend.close()


if __name__ == "__main__":
    main()
