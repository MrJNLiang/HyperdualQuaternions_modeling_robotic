#!/usr/bin/env python3
"""手指碰撞网格地面真值测绘：PhysX 三角网格 -> gripper_link 系占据。

diag_grasp_verify 细扫裁决：方块在指间却于 w≈93 mm 首触（远大于平行
垫面预期的 70 mm），且 URDF STL 显示手指网格沿接近向有 ±46 mm 伸展。
本脚本直接读 PhysX 碰撞网格顶点（world -> gripper 系），输出手指几何
在 (x=接近, y=开合, z) 三轴的真实占据，裁决"首触面是内面垫还是前伸
叶片"，为 GRASP_POS 深度设计提供地面真值。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_finger_mesh.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik, world_to_dh
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


def _R_from_quat(r):
    w, x, y, z = r
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1 - 2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1 - 2*(x*x+y*y)]])


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    from pxr import Usd, UsdGeom

    q0 = np.array([-0.3, -1.6, -0.6, -0.6, 0.0, 0.0])
    q, res, ok = solve_ik(world_to_dh(P.SETPOINT_POS), P.R_TOOL_QUAT, q0)
    assert ok, f"IK 失败 {res}"
    backend.reset_to(q, gripper_width=P.GRIPPER_OPENING)
    for _ in range(30):
        backend.articulation.set_joint_positions(
            backend._row(q), joint_indices=backend.arm_idx)
        backend.step()

    p_g, q_g = backend.get_ee_pose()
    Rg = _R_from_quat(q_g)

    def to_gripper(p_w):
        return Rg.T @ (np.asarray(p_w) - p_g)

    # 收集 gripper_link 子树所有 Mesh prim 的世界顶点
    stage = backend.stage
    root = stage.GetPrimAtPath(P.ISAAC_ROBOT_PRIM + backend._GRIPPER_LINK_REL)
    all_pts = []
    n_mesh = 0
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            pts = mesh.GetPointsAttr().Get()
            if pts is None or len(pts) == 0:
                continue
            l2w = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            arr = np.array([[float(v[0]), float(v[1]), float(v[2])]
                            for v in pts])
            ones = np.ones((len(arr), 1))
            homo = np.hstack([arr, ones])
            # Gf.Matrix4d：点为行向量右乘
            m = np.array([[float(l2w[r][c]) for c in range(4)]
                          for r in range(4)])
            world = homo @ m.T
            all_pts.append(world[:, :3])
            n_mesh += 1
    pts_w = np.vstack(all_pts)
    pts_g = np.array([to_gripper(p) for p in pts_w]) * 1e3   # mm

    print(f"网格数={n_mesh} 顶点数={len(pts_g)}")
    print("手指+gripper_link 全部碰撞/显示网格在 gripper 系占据(mm):")
    for ax, nm in ((0, "接近x"), (1, "开合y"), (2, "z")):
        print(f"   {nm}: [{pts_g[:, ax].min():+7.1f}, {pts_g[:, ax].max():+7.1f}]")

    # 关键裁决 1：两指内面（开合向最内侧 3 mm 壳层）的 x/z 占据
    # 张开位手指在 y=±70：内面 = 朝中心一侧（左指 +y 侧 / 右指 -y 侧），
    # 这里取 |y| 最小的壳层近似两指内侧面
    print("\n内面壳层（|y|<40mm 且最靠中心的顶点群）:")
    for yband_lo, yband_hi in ((18, 40), (0, 18)):
        sel = pts_g[(np.abs(pts_g[:, 1]) > yband_lo)
                    & (np.abs(pts_g[:, 1]) < yband_hi)]
        if len(sel):
            print(f"   |y|∈({yband_lo},{yband_hi}): n={len(sel)} "
                  f"x[{sel[:, 0].min():+6.1f},{sel[:, 0].max():+6.1f}] "
                  f"z[{sel[:, 2].min():+6.1f},{sel[:, 2].max():+6.1f}]")

    # 关键裁决 2：逐接近向 x 切片看开合向内缘（|y| 最小值）
    # 内缘(x) 即该深度处手指能夹到的最小半宽；首触开度 ≈ 2*内缘
    print("\n逐 x 切片（接近向）的开合向内缘 |y|_min 与 z 范围:")
    for x0 in range(-60, 50, 5):
        sel = pts_g[(pts_g[:, 0] >= x0) & (pts_g[:, 0] < x0 + 5)]
        if len(sel) > 3:
            print(f"   x[{x0:+4d},{x0+5:+4d}]mm: |y|_min={np.abs(sel[:, 1]).min():5.1f} "
                  f"|y|_max={np.abs(sel[:, 1]).max():5.1f} "
                  f"z[{sel[:, 2].min():+6.1f},{sel[:, 2].max():+6.1f}]")

    # 关键裁决 3：方块当前占据 vs 内缘对照
    print("\n方块 gripper 系占据（参考）：x[-38.9,+18.9] y[-22.5,+22.5] "
          "z[-28.8,+28.8] mm（中心 x_ee=-10）")
    backend.close()


if __name__ == "__main__":
    main()
