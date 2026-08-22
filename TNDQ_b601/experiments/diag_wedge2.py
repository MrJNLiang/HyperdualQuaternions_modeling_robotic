#!/usr/bin/env python3
"""v12k 深位楔形夹持：掌部避撞 + 楔形举升组合验证。

v12j 裁决（STL + 物理探针合并模型）：
  - 真实夹持接触面 = 指内楔形斜面（物理实测：顶面 z 随深度
    x=-16..-34 由 8->14 mm 上升；方块在 v12c 坐支柱上时无处可
    去 -> 沿 +x_g 挤出，全部提升失败根因）；
  - STL 显示指体在 x<-12 处还有 ±12 mm 横轨，轨间净空仅 12 mm，
    45 mm 方块不可能进入 -> 排除"入槽"方案；
  - 浅抓（z_off -8..-16, d_x=28）掌部/腕部直接撞方块（瞬间打飞
    230 mm）-> 掌部避撞需 d_x >= ~65 mm。

方案：d_x ∈ {0.065, 0.080}（掌部退出方块包络）+ z_off ∈ {-0.010,
-0.014}（方块中心降至楔顶高度 c_z=10..14，楔形举升可行）。
慢闭 46 mm 目标观察 dz（期望 +），成功档再做笛卡尔提升随动。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_wedge2.py
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
    q_prev = q0
    x_g, z_g = P.TOOL_AXIS, P._TOOL_Z
    cy = P.CUBE_YAW
    cube_q = np.array([np.cos(0.5 * cy), 0.0, 0.0, np.sin(0.5 * cy)])
    cube_p0 = np.array(P.CUBE_POS, dtype=float)
    cube_p0[2] += 1e-3

    def teleport_arm(qa, n=30):
        for _ in range(n):
            backend.articulation.set_joint_positions(
                backend._row(qa), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.step()

    best = None
    for d_x in (0.065, 0.080):
        for z_off in (-0.010, -0.014):
            grasp = P.CUBE_POS + d_x * x_g + z_off * z_g
            q, res, ok = solve_ik(world_to_dh(grasp), P.R_TOOL_QUAT, q_prev)
            if not ok:
                print(f"d_x={d_x*1e3:.0f} z={z_off*1e3:+.0f}: IK 失败")
                continue
            q_prev = q
            backend.cube.set_world_pose(cube_p0, cube_q)
            backend.cube.set_linear_velocity(np.zeros(3))
            backend.cube.set_angular_velocity(np.zeros(3))
            backend.reset_to(q, gripper_width=P.GRIPPER_OPENING)
            teleport_arm(q, 60)
            c0, _ = backend.get_cube_pose()

            w_target = 0.046
            for k in range(1000):
                backend.articulation.set_joint_positions(
                    backend._row(q), joint_indices=backend.arm_idx)
                backend.articulation.set_joint_velocities(
                    backend._row(np.zeros(8)), joint_indices=None)
                backend.set_gripper(w_target)
                backend.step()
                if k % 250 == 249:
                    ck, _ = backend.get_cube_pose()
                    dk = ck - c0
                    print(f"  d_x={d_x*1e3:3.0f} z={z_off*1e3:+4.0f} "
                          f"t={2*(k+1):4d}ms 位移=({dk[0]*1e3:+6.1f},"
                          f"{dk[1]*1e3:+6.1f},{dk[2]*1e3:+6.1f})mm")
            c, _ = backend.get_cube_pose()
            d = c - c0
            lifted = d[2] > 0.005 and np.linalg.norm(d[:2]) < 0.010
            print(f"d_x={d_x*1e3:.0f} z={z_off*1e3:+.0f}: dz={d[2]*1e3:+6.1f}"
                  f"mm 水平={np.linalg.norm(d[:2])*1e3:5.1f}mm "
                  f"{'★举升成功' if lifted else ''}")
            if lifted and best is None:
                best = (grasp, q.copy(), c.copy())

    if best is None:
        print("\n全部失败")
        backend.close()
        return

    grasp, q, c0 = best
    print("\n== 成功档笛卡尔提升 5 cm 随动验证 ==")
    q_prev = q
    for k in range(500):
        a = k / 499
        p_i = grasp + np.array([0.0, 0.0, 0.05]) * a
        qi, _, oki = solve_ik(world_to_dh(p_i), P.R_TOOL_QUAT, q_prev)
        if not oki:
            print(f"  提升 IK 失败 @a={a:.2f}")
            break
        q_prev = qi
        backend.articulation.set_joint_positions(
            backend._row(qi), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)
        backend.set_gripper(0.046)
        backend.step()
        if k % 100 == 99:
            ck, _ = backend.get_cube_pose()
            dk = ck - c0
            print(f"  t={2*(k+1):4d}ms 方块位移=({dk[0]*1e3:+6.1f},"
                  f"{dk[1]*1e3:+6.1f},{dk[2]*1e3:+6.1f})mm "
                  f"开度={backend.get_gripper_width()*1e3:4.1f}mm")
    c, _ = backend.get_cube_pose()
    print(f"提升后: dz={(c[2]-c0[2])*1e3:+.1f}mm（≈+50 即夹住随动）"
          f" 水平={np.linalg.norm(c[:2]-c0[:2])*1e3:.1f}mm")
    backend.close()


if __name__ == "__main__":
    main()
