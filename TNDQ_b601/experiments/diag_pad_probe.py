#!/usr/bin/env python3
"""指垫内面位置物理探针（gripper_link 系，零几何假设）。

AABB 分析结论：指尖面（手指网格 +x 端）在原点前 14.7 mm，指体向后
延伸 58.6 mm，方块在抓取位沿 x 深入 50.5 mm——闭合沿 y 扫掠时必有
指体先于指垫接触方块。需要实验确定"闭合扫掠面" y_inner(s)：
抓取位 kinematic 保持，探针微立方体逐点 teleport 到 gripper_link 系
(y 正半轴 15~75 mm, x -55~+20 mm, z ±20 mm)，6 步后位移 > 阈值 =
被手指占据。分别测 s=0.07（全开）与 s=0.02（目标夹持位），得
指垫内面 y 与 x/z 展布 -> 反推 GRASP_POS 与 GRIPPER_GRASP。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_pad_probe.py
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
    backend = IsaacB601Backend(headless=True, probe_size=0.008)
    backend.setup()
    q_g = grasp_q()
    cube0, cq0 = backend.get_cube_pose()
    backend.cube.set_world_pose(cube0 + np.array([2.0, 0, 0]), cq0)

    THR = 0.003
    ys = np.arange(0.015, 0.076, 0.005)
    xs = np.arange(-0.055, 0.021, 0.005)
    zs = [-0.02, -0.01, 0.0, 0.01, 0.02]

    for s, tag in ((0.07, "全开 s=0.07"), (0.02, "夹持 s=0.02")):
        def kin_hold():
            backend.articulation.set_joint_positions(
                backend._row(q_g), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_positions(
                backend._row(np.full(2, s)), joint_indices=backend.grip_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.step()

        kin_hold(); kin_hold()
        p_ee, q_ee = backend.get_ee_pose()
        w, x, y, z = q_ee
        Rw = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])
        occ = np.zeros((len(ys), len(xs), len(zs)), dtype=bool)
        for i, yy in enumerate(ys):
            for j, xx in enumerate(xs):
                for l, zz in enumerate(zs):
                    pt = p_ee + Rw @ np.array([xx, yy, zz])
                    backend.probe.set_world_pose(
                        pt, np.array([1.0, 0, 0, 0]))
                    backend.probe.set_linear_velocity(np.zeros(3))
                    backend.probe.set_angular_velocity(np.zeros(3))
                    for _ in range(6):
                        kin_hold()
                    pp, _ = backend.get_probe_pose()
                    occ[i, j, l] = np.linalg.norm(pp - pt) > THR
        print(f"\n== {tag} ==")
        # 每个 x 列（z 任意）的最小占据 y = 扫掠面
        anyz = occ.any(axis=2)
        for j, xx in enumerate(xs):
            col = anyz[:, j]
            if col.any():
                y_min = ys[np.where(col)[0][0]]
                y_max = ys[np.where(col)[0][-1]]
                print(f"  x={xx*1e3:+6.1f} mm: y占据 [{y_min*1e3:5.1f},"
                      f"{y_max*1e3:5.1f}] mm")
            else:
                print(f"  x={xx*1e3:+6.1f} mm: 无占据")
        # z 展布（y=0.04 附近列）
        j40 = int(np.argmin(np.abs(xs + 0.040)))
        for i, yy in ((len(ys)//3, ys[len(ys)//3]),):
            zcol = occ[i, j40, :]
            if zcol.any():
                print(f"  y={yy*1e3:.0f},x={xs[j40]*1e3:+.0f} mm 处 "
                      f"z占据 [{zs[np.where(zcol)[0][0]]*1e3:+.0f},"
                      f"{zs[np.where(zcol)[0][-1]]*1e3:+.0f}] mm")
    backend.close()


if __name__ == "__main__":
    main()
