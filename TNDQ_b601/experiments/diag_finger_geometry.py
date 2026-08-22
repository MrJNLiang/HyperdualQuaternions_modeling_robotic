#!/usr/bin/env python3
"""手指占据几何直测：USD 碰撞包围盒 -> TCP(gripper_link) 系。

六跑归因：闭合相手指在宽度 ~115 mm 处就把方块推开（指尖/指体先于
指垫接触）。需要精确的手指占据盒：指尖面 x 位置、垫内面 y(stroke)、
z 展布。方法：teleport 抓取位，手指分别置 0.07/0.02 行程，读
gripper_left/right/link 世界变换 + 几何 BoundAPI，变换到 gripper_link
系输出 AABB。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_finger_geometry.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik, make_tool_pose, world_to_dh
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


def _quat_to_R(r):
    w, x, y, z = r
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def grasp_q():
    phi = np.deg2rad(-14.4)
    R, quat = make_tool_pose(np.deg2rad(120), phi)
    xg, zg = R[:, 0], R[:, 2]
    p_grasp = P.CUBE_POS + 0.028 * xg + 0.005 * zg
    q0 = np.array([-0.3, -1.6, -0.6, -0.6, 0.0, 0.0])
    q, res, ok = solve_ik(world_to_dh(p_grasp), quat, q0)
    assert ok, f"IK 失败: {res}"
    return q


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    from pxr import Usd, UsdGeom, Gf, UsdPhysics
    stage = backend.stage
    q_g = grasp_q()

    def xform(path):
        prim = stage.GetPrimAtPath(path)
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        tq = xf.ExtractRotation().GetQuaternion()
        q = np.array([tq.GetReal(), *tq.GetImaginary()], dtype=float)
        return np.array(xf.ExtractTranslation(), dtype=float), q / np.linalg.norm(q)

    def world_bound(path, cache):
        prim = stage.GetPrimAtPath(path)
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        lo, hi = rng.GetMin(), rng.GetMax()
        return np.array(lo), np.array(hi)

    root = str(P.ISAAC_ROBOT_PRIM)
    bcache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy,
         UsdGeom.Tokens.render, UsdGeom.Tokens.guide])
    # 按名字定位 link prim（USD 层级可能有 payload 子树）
    link_paths = {}
    for prim in stage.Traverse():
        nm = prim.GetName()
        if nm in ("gripper_link", "gripper_left", "gripper_right"):
            if UsdPhysics.RigidBodyAPI(prim) or nm not in link_paths:
                link_paths[nm] = str(prim.GetPath())
    print(f"[diag] link prims: {link_paths}")
    cube0, cq0 = backend.get_cube_pose()
    backend.cube.set_world_pose(cube0 + np.array([2.0, 0, 0]), cq0)  # 挪开

    for s, tag in ((0.07, "张开 s=0.07 (w=0.14)"), (0.02, "闭合 s=0.02 (w=0.04)")):
        backend.reset_to(q_g, gripper_width=2 * s)
        backend.articulation.set_joint_positions(
            backend._row(np.full(2, s)), joint_indices=backend.grip_idx)
        for _ in range(20):
            backend.step()
        p_ee, q_ee = backend.get_ee_pose()
        Rw = _quat_to_R(q_ee)           # gripper_link -> world
        # 工具系（控制约定）= gripper_link 系绕 z 转 -B601_TOOL_ANGLE，
        # 使 x_tool = 接近轴（与 ik_lib/R_TOOL_QUAT 同约定）
        c_, s_ = np.cos(-P.B601_TOOL_ANGLE), np.sin(-P.B601_TOOL_ANGLE)
        Rz = np.array([[c_, -s_, 0], [s_, c_, 0], [0, 0, 1]])
        def to_tool(p_w):
            return Rz @ (Rw.T @ (np.asarray(p_w) - p_ee))
        print(f"\n== {tag} ==")
        print(f"  gripper_link 世界位姿: p={np.round(p_ee, 4)}")
        for name in ("gripper_link", "gripper_left", "gripper_right"):
            lo, hi = world_bound(link_paths[name], bcache)
            lo_e, hi_e = to_tool(lo), to_tool(hi)
            lo_e = np.minimum(lo_e, hi_e); hi_e = np.maximum(lo_e, hi_e)
            print(f"  {name:13s} 工具系 AABB: "
                  f"x[{lo_e[0]*1e3:+7.1f},{hi_e[0]*1e3:+7.1f}] "
                  f"y[{lo_e[1]*1e3:+7.1f},{hi_e[1]*1e3:+7.1f}] "
                  f"z[{lo_e[2]*1e3:+7.1f},{hi_e[2]*1e3:+7.1f}] mm")
        # 方块在 gripper 系（设计点核对）
        backend.cube.set_world_pose(cube0, cq0)
        for _ in range(5):
            backend.step()
        c, _ = backend.get_cube_pose()
        print(f"  方块中心 工具系: {np.round(to_tool(c) * 1e3, 1)} mm"
              f"  （半宽 {P.CUBE_SIZE/2*1e3:.1f} mm）")
        backend.cube.set_world_pose(cube0 + np.array([2.0, 0, 0]), cq0)
    backend.close()


if __name__ == "__main__":
    main()
