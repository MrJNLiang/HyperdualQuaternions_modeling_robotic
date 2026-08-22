#!/usr/bin/env python3
"""v12h 手指内面形状测绘（方块深度处）：判定接触面是否有 x 向斜坡。

v12g 裁决：笛卡尔提升方块仍沿 +x_g 弹飞。假设：接触结构（垫/轨）
内面不是平行于开合向的平面，而是随 x（接近向深度）变化的斜面，
夹紧力分解出 +x_g 分量推飞方块。

方法：抓取位保持（方块挪走），手指 drive 压到 stroke=29 mm，
探针在 gripper 系 (x, z) 网格上沿 y 逐点 teleport，读占据边界
y_inner(x,z)。若 dy_inner/dx != 0 -> 斜坡实锤，给出斜坡方向与斜率。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_face_shape.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik, world_to_dh
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True, probe_size=0.01)
    backend.setup()

    q0 = np.array([-0.3, -1.6, -0.6, -0.6, 0.0, 0.0])
    # 空中构型：同姿态抬高 10 cm（排除支柱，纯测手指形状）
    probe_pose = P.GRASP_POS + np.array([0.0, 0.0, 0.10])
    q, res, ok = solve_ik(world_to_dh(probe_pose), P.R_TOOL_QUAT, q0)
    assert ok, f"IK 失败 {res}"
    # 方块挪走
    cube0, cq0 = backend.get_cube_pose()
    backend.cube.set_world_pose(cube0 + np.array([2.0, 0, 0]), cq0)

    stroke = 0.033
    backend.reset_to(q, gripper_width=2 * stroke)
    for _ in range(100):
        backend.articulation.set_joint_positions(
            backend._row(q), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)
        backend.set_gripper(2 * stroke)
        backend.step()
    p_g, q_g = backend.get_ee_pose()
    w, x, y, z = q_g
    Rg = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    THR = 0.0030

    print(f"ee=({p_g[0]:.4f},{p_g[1]:.4f},{p_g[2]:.4f})  "
          f"实测开度={backend.get_gripper_width()*1e3:.1f}mm")
    # 第二扫：带内高区（斜坡顶面 z≈14 之上）沿 y 找竖直内面
    print(f"{'z\\x':>6} " + "".join(f"{int(x_mm):>7d}"
                                 for x_mm in (-16, -22, -28, -34)))
    for z_mm in (16, 19, 22, 26, 30):
        row = []
        for x_mm in (-16, -22, -28, -34):
            found = None
            for y_mm in range(26, 52, 2):
                p_local = np.array([x_mm * 1e-3, y_mm * 1e-3, z_mm * 1e-3])
                p_w = p_g + Rg @ p_local
                backend.probe.set_world_pose(
                    p_w, np.array([1.0, 0, 0, 0.0]))
                backend.probe.set_linear_velocity(np.zeros(3))
                backend.probe.set_angular_velocity(np.zeros(3))
                for _ in range(5):
                    backend.articulation.set_joint_positions(
                        backend._row(q), joint_indices=backend.arm_idx)
                    backend.articulation.set_joint_velocities(
                        backend._row(np.zeros(8)), joint_indices=None)
                    backend.set_gripper(2 * stroke)
                    backend.step()
                pp, _ = backend.get_probe_pose()
                if np.linalg.norm(pp - p_w) > THR:
                    found = y_mm
                    break
            row.append(f"{found if found is not None else '..':>7}")
        print(f"{z_mm:>5d}mm " + "".join(row))
    # 伪影核查：空点处探针是否真的未动（打印一个代表点的位移）
    p_local = np.array([-0.020, 0.050, 0.010])
    p_w = p_g + Rg @ p_local
    backend.probe.set_world_pose(p_w, np.array([1.0, 0, 0, 0.0]))
    backend.probe.set_linear_velocity(np.zeros(3))
    for _ in range(5):
        backend.articulation.set_joint_positions(
            backend._row(q), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)
        backend.set_gripper(2 * stroke)
        backend.step()
    pp, _ = backend.get_probe_pose()
    print(f"核查点(-20,50,10)mm 位移={(pp - p_w)*1e3} mm")
    backend.close()


if __name__ == "__main__":
    main()
