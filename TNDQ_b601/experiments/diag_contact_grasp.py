#!/usr/bin/env python3
"""v12b 接触裁决：闭合细扫中读 PhysX 接触对，定位推方块的结构。

diag_grasp_verify 细扫事实：真实开度 ~117 mm 处方块开始被推（臂漂移
0.0 mm，USD/PhysX 一致），STL 内面模型预言 45 mm 才接触——必有 URDF
STL 之外的碰撞几何或映射误差。本脚本在抓取位逐步闭合，每步读
Scene 接触报告，输出与 cube 接触的所有 prim 路径，一锤定音。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_contact_grasp.py
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
    stage = backend.stage

    q0 = np.array([-0.3, -1.6, -0.6, -0.6, 0.0, 0.0])
    q, res, ok = solve_ik(world_to_dh(P.SETPOINT_POS), P.R_TOOL_QUAT, q0)
    assert ok, f"IK 失败 {res}"
    backend.reset_to(q, gripper_width=P.GRIPPER_OPENING)
    for _ in range(30):
        backend.articulation.set_joint_positions(
            backend._row(q), joint_indices=backend.arm_idx)
        backend.step()
    cube0, _ = backend.get_cube_pose()
    base_xy = cube0[:2].copy()

    # PhysX 接触报告：world.get_contacts()（每步后读）
    try:
        _ = backend.world.get_contacts()
        has_contact_api = True
    except Exception as e:
        has_contact_api = False
        print(f"world.get_contacts 不可用: {e}")

    for stroke_cmd in [0.062, 0.060, 0.058, 0.056]:
        for _ in range(300):
            backend.articulation.set_joint_positions(
                backend._row(q), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.set_gripper(2.0 * stroke_cmd)
            backend.step()
        cnow, _ = backend.get_cube_pose()
        dvec = cnow[:2] - base_xy
        print(f"\nstroke={stroke_cmd*1e3:.1f}mm 方块移位="
              f"({dvec[0]*1e3:+.2f},{dvec[1]*1e3:+.2f})mm")
        if not has_contact_api:
            continue
        contacts = backend.world.get_contacts()
        print(f"  接触对数={len(contacts)}")
        seen = set()
        for c in contacts:
            p0 = getattr(c, 'body0', '?')
            p1 = getattr(c, 'body1', '?')
            pair = f"{p0} <-> {p1}"
            if 'ube' in pair.replace('B601', ''):   # 只关心 cube 相关对
                if pair not in seen:
                    seen.add(pair)
                    print(f"  ★ {pair}")
    backend.close()


if __name__ == "__main__":
    main()
