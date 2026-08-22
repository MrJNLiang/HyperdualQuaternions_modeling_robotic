#!/usr/bin/env python3
"""指垫内面精细测绘（gripper_link 系）：y_inner(s) -> 真实开度 gap(s)。

粗探针网格下限 15 mm 把指垫内面截断，无法反推夹持几何。本脚本加密：
y 从 0 起 2.5 mm 步长，定位每个行程 s 下 +y 指的最小占据 y（指垫内面）。
由对称性 gap(s) = 2*y_inner(s)。输出：
  - 各行程 s 的指垫内面 y_inner（按 x 列，z=0 指垫中面）
  - gap(s) 曲线 -> 夹 45 mm 方块所需行程 s_grasp 与 GRIPPER_GRASP=2*s_grasp
  - 指垫沿接近轴(x)的占据范围 -> GRASP_POS 深度定位依据
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_pad_fine.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik, make_tool_pose, world_to_dh
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


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
    backend = IsaacB601Backend(headless=True, probe_size=0.004)
    backend.setup()
    q_g = grasp_q()
    cube0, cq0 = backend.get_cube_pose()
    backend.cube.set_world_pose(cube0 + np.array([2.0, 0, 0]), cq0)  # 挪开方块

    THR = 0.0018
    xs = np.arange(-0.020, 0.0126, 0.005)   # 接近轴：指垫区
    ys = np.arange(0.0, 0.0476, 0.0025)     # 开合轴：0 -> 45 mm，2.5 mm 步长
    strokes = [0.0715, 0.06, 0.05, 0.04, 0.035, 0.03, 0.025, 0.02]

    def kin_hold(s):
        backend.articulation.set_joint_positions(
            backend._row(q_g), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_positions(
            backend._row(np.full(2, s)), joint_indices=backend.grip_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)
        backend.step()

    print("行程s[mm]  | 各 x 列指垫内面 y_inner[mm] (z=0)")
    gap_rows = []
    for s in strokes:
        kin_hold(s); kin_hold(s)
        p_ee, q_ee = backend.get_ee_pose()
        w, x, y, z = q_ee
        Rw = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])
        row = []
        for xx in xs:
            y_inner = None
            for yy in ys:
                pt = p_ee + Rw @ np.array([xx, yy, 0.0])
                backend.probe.set_world_pose(pt, np.array([1.0, 0, 0, 0]))
                backend.probe.set_linear_velocity(np.zeros(3))
                backend.probe.set_angular_velocity(np.zeros(3))
                for _ in range(5):
                    kin_hold(s)
                pp, _ = backend.get_probe_pose()
                if np.linalg.norm(pp - pt) > THR:
                    y_inner = yy
                    break
            row.append(y_inner)
        # 代表内面：取各 x 列的最小非空 y_inner（最靠中心的垫面）
        valid = [v for v in row if v is not None]
        rep = min(valid) if valid else float("nan")
        gap_rows.append((s, 2 * rep))
        cells = "  ".join(
            (f"{x*1e3:+5.1f}mm:{('%4.1f' % (v*1e3)) if v is not None else '  --'}"
             for x, v in zip(xs, row)))
        print(f" s={s*1e3:5.1f}  | {cells}  | gap~{2*rep*1e3:5.1f}mm")

    print("\n== gap(s) 曲线（gap=2*y_inner，对称近似）==")
    for s, g in gap_rows:
        print(f"  s={s*1e3:5.1f}mm  ->  gap≈{g*1e3:6.1f}mm")
    # 反推夹 45 mm 方块行程：gap(s) 略小于 45 mm 处（垫压紧）
    print("\n方块宽 45 mm：需 gap(s_grasp)≈43~45 mm（过盈压紧）。")
    backend.close()


if __name__ == "__main__":
    main()
