#!/usr/bin/env python3
"""v12c 保持/夹持裁决：区分驱动伪影与真实接触，验证夹持可行性。

细扫疑点：stroke≈57（开度 114 mm）处方块即渐移——但垫面探针显示
内面 ≈ stroke±几 mm，该开度下垫面离方块面 25+ mm，不应接触。
假设 A（伪影）：drive 目标切换的瞬态振动经 pedestal 边缘放大；
假设 B（隐藏结构）：存在 USD 独有碰撞几何。

测试：恒开度保持 1 s 观察蠕变（A 成立则保持段零蠕变）；逐级收拢
到接触区观察首触开度；最后夹持+提升 5 cm 验证方块随动。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_hold_grasp.py
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
    q, res, ok = solve_ik(world_to_dh(P.SETPOINT_POS), P.R_TOOL_QUAT, q0)
    assert ok, f"IK 失败 {res}"
    backend.reset_to(q, gripper_width=P.GRIPPER_OPENING)
    for _ in range(60):
        backend.articulation.set_joint_positions(
            backend._row(q), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_positions(
            backend._row(np.full(2, 0.5 * P.GRIPPER_OPENING)),
            joint_indices=backend.grip_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)
        backend.step()
    c0, cq0 = backend.get_cube_pose()

    def hold(stroke, n_steps, tag):
        for _ in range(n_steps):
            backend.articulation.set_joint_positions(
                backend._row(q), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.set_gripper(2.0 * stroke)
            backend.step()
        c, _ = backend.get_cube_pose()
        d = c - c0
        print(f"  {tag}: stroke={stroke*1e3:5.1f}mm 保持{n_steps*2}ms "
              f"方块移位=({d[0]*1e3:+6.2f},{d[1]*1e3:+6.2f},{d[2]*1e3:+6.2f})mm")
        return d

    print("== [1] 恒开度保持（伪影裁决：保持段应零蠕变） ==")
    hold(0.065, 500, "开130保持")
    hold(0.057, 500, "开114保持")          # 细扫"首触"档：伪影则零蠕变
    hold(0.035, 500, "开 70保持")
    hold(0.026, 500, "开 52保持")          # 垫面模型：离方块面 3.5 mm

    print("== [2] 接触区细扫（垫面模型预言 45~65 mm 首触） ==")
    for stroke in (0.0240, 0.0225, 0.0210, 0.0200):
        d = hold(stroke, 500, "细扫")
        if abs(d[1]) > 0.003:
            print("  -> 开合向移位 >3 mm，判定接触/推挤，停止")
            break

    print("== [3] 夹持+提升验证（v12c 实测接触 stroke≈33.2，目标不越过接触点）==")
    squeeze = 0.058                        # 过盈 ~4.4 mm/侧，KP*过盈≈8.8 N/指
    hold(0.5 * squeeze, 500, "夹紧保持")
    w_measured = backend.get_gripper_width()
    print(f"  夹紧后实测开度={w_measured*1e3:.1f}mm（stroke≈其半；"
          f"接触≈33.2，目标 29 -> 驱动压紧）")
    q_lift = q.copy()
    # 提升 5 cm（竖直，关节空间插值；TCP 空间 IK 重解提升位）
    from experiments.ik_lib import solve_ik as sik
    p_lift = P.SETPOINT_POS + np.array([0.0, 0.0, 0.05])
    ql, rl, okl = sik(world_to_dh(p_lift), P.R_TOOL_QUAT, q)
    if not okl:
        print("  提升位 IK 失败")
    else:
        n = 500
        for k in range(n):
            a = k / (n - 1)
            qi = q + a * (ql - q)
            backend.articulation.set_joint_positions(
                backend._row(qi), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.set_gripper(squeeze)
            backend.step()
            if k % 100 == 99:
                c, _ = backend.get_cube_pose()
                pe, _ = backend.get_ee_pose()
                d = c - c0
                print(f"  t={2*(k+1):4d}ms 方块位移=({d[0]*1e3:+6.1f},"
                      f"{d[1]*1e3:+6.1f},{d[2]*1e3:+6.1f})mm "
                      f"ee_z={pe[2]*1e3:.1f}mm")
        c, _ = backend.get_cube_pose()
        dz = c[2] - c0[2]
        dh = np.linalg.norm(c[:2] - c0[:2])
        print(f"  提升 5 cm 后方块: dz={dz*1e3:+.1f}mm（随动≈+50 即夹住）"
              f" 水平移位={dh*1e3:.1f}mm")
    backend.close()


if __name__ == "__main__":
    main()
