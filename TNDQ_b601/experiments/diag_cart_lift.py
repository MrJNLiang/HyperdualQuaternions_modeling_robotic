#!/usr/bin/env python3
"""v12g 提升随动复验：修正 diag 提升路径为笛卡尔插值（姿态恒定）。

v12c 夹持裁决的"提升失败"存在 diag 方法缺陷：提升段用关节空间线性
插值 q->ql，中间步 TCP 姿态漂移（非 R_TOOL 恒定），手指框架扭转
把方块撬出。真实 exp1 提升是笛卡尔轨迹（姿态恒为 R_TOOL_QUAT）。

本脚本按真实任务几何复验：v12c GRASP_POS（0.028 x_g - 0.042 z_g）
+ 慢闭到 GRIPPER_GRASP 目标 + 笛卡尔 IK 逐点 teleport 提升 5 cm
（姿态 slerp 恒等 = 保持 R_TOOL），读方块随动。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_cart_lift.py
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
    backend = IsaacB601Backend(headless=True)
    backend.setup()

    q0 = np.array([-0.3, -1.6, -0.6, -0.6, 0.0, 0.0])
    grasp = P.GRASP_POS
    q, res, ok = solve_ik(world_to_dh(grasp), P.R_TOOL_QUAT, q0)
    assert ok, f"IK 失败 {res}"

    # 方块归位 + 硬复位 + 全开保持
    cy = P.CUBE_YAW
    cube_q = np.array([np.cos(0.5 * cy), 0.0, 0.0, np.sin(0.5 * cy)])
    cube_p0 = np.array(P.CUBE_POS, dtype=float)
    cube_p0[2] += 1e-3
    backend.cube.set_world_pose(cube_p0, cube_q)
    backend.cube.set_linear_velocity(np.zeros(3))
    backend.cube.set_angular_velocity(np.zeros(3))
    backend.reset_to(q, gripper_width=P.GRIPPER_OPENING)
    for _ in range(60):
        backend.articulation.set_joint_positions(
            backend._row(q), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)
        backend.step()
    c0, _ = backend.get_cube_pose()

    # 慢闭到夹紧目标，2 s（过盈档由环境变量 SQZ 控制，默认 GRIPPER_GRASP）
    sqz = float(os.environ.get("SQZ", P.GRIPPER_GRASP))
    print(f"夹紧目标开度={sqz*1e3:.0f}mm")
    for _ in range(1000):
        backend.articulation.set_joint_positions(
            backend._row(q), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)
        backend.set_gripper(sqz)
        backend.step()
    c, _ = backend.get_cube_pose()
    d = c - c0
    print(f"闭合后: 开度={backend.get_gripper_width()*1e3:.1f}mm "
          f"扰动=({d[0]*1e3:+.2f},{d[1]*1e3:+.2f},{d[2]*1e3:+.2f})mm")

    # 笛卡尔提升 5 cm：姿态恒定 R_TOOL_QUAT，IK 逐点 warm start
    q_prev = q
    n = 500
    failed = False
    for k in range(n):
        a = k / (n - 1)
        p_i = grasp + np.array([0.0, 0.0, 0.05]) * a
        qi, _, oki = solve_ik(world_to_dh(p_i), P.R_TOOL_QUAT, q_prev)
        if not oki:
            print(f"  提升 IK 失败 @a={a:.2f}")
            failed = True
            break
        q_prev = qi
        backend.articulation.set_joint_positions(
            backend._row(qi), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)
        backend.set_gripper(sqz)
        backend.step()
        if k % 100 == 99:
            ck, _ = backend.get_cube_pose()
            dk = ck - c0
            print(f"  t={2*(k+1):4d}ms 方块位移=({dk[0]*1e3:+6.1f},"
                  f"{dk[1]*1e3:+6.1f},{dk[2]*1e3:+6.1f})mm "
                  f"开度={backend.get_gripper_width()*1e3:4.1f}mm")
    if not failed:
        c, _ = backend.get_cube_pose()
        dz = c[2] - c0[2]
        dh = np.linalg.norm(c[:2] - c0[:2])
        print(f"提升 5 cm 后: dz={dz*1e3:+.1f}mm（随动≈+50 即夹住）"
              f" 水平={dh*1e3:.1f}mm")
    backend.close()


if __name__ == "__main__":
    main()
