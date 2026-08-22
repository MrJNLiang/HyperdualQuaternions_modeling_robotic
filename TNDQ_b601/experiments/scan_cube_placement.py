#!/usr/bin/env python3
"""立方体外移距离扫描 —— 降低抓取构型伸展度（exp1 Isaac 冻结的缓解路径）。

自碰撞排除后的结论（diag_selfcollide3 --mode off）：关自碰撞后伸展
构型仍完全冻结（dq 逐位相同），根因 = PhysX 求解器在极限伸展构型
（j3 贴下限位、arm 全伸）的构型锁。缓解方向：立方体沿当前径向外移，
让抓取构型回到臂的"舒适区"（肘部更弯、远离全伸奇异面）。

顶抓已排除（probe_topdown_reach.py）：tilt 180/170/160/150 全 FAIL，
外移 0.25 m 仍 FAIL——该臂结构不支持低空顶抓，只能水平侧抓。

扫描 r_h ∈ {0.50, 0.52, 0.54, 0.56}（立方体中心水平距离）：
    [1] grasp/hover/init/lift 路标 IK（限位余量 + sigma_min）；
    [2] INIT -> hover 下降链 20 点；
    [3] hover -> grasp 进插链 20 点 + grasp -> lift 提升链 10 点；
    [4] 带载圆周整圈 72 点（圆心随立方体外移）；
全链 PASS 且构型伸展度明显下降的最小 r_h 即回填值。

用法（纯 python）：
    python3 experiments/scan_cube_placement.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    CUBE_POS, CUBE_SIZE, GRASP_TCP_Z_OFFSET, L_TCP, TOOL_AXIS, GRIPPER_Y,
    SETPOINT_HOVER_CLEARANCE, INIT_CLEARANCE, GRASP_LIFT_HEIGHT,
    CIRCLE_RADIUS,
)
from experiments.ik_lib import (
    solve_ik, solve_ik_multi, fk_pose, quat_angle, joint_margin, sigma_min,
    world_to_dh, cube_yaw_for_gripper,
)
from config.params import R_TOOL_QUAT

RQ = R_TOOL_QUAT / np.linalg.norm(R_TOOL_QUAT)

# 当前径向单位向量（立方体外移方向 = 保持方位角不变）
RAD0 = np.array([CUBE_POS[0], CUBE_POS[1], 0.0])
RAD0 = RAD0 / np.linalg.norm(RAD0)


def derive(r_h):
    """给定立方体中心水平距离，推导全部任务几何。"""
    cube = np.array([r_h * RAD0[0], r_h * RAD0[1], CUBE_SIZE / 2])
    grasp_tcp = cube + np.array([0.0, 0.0, GRASP_TCP_Z_OFFSET])
    grasp = grasp_tcp + L_TCP * TOOL_AXIS
    hover = grasp + SETPOINT_HOVER_CLEARANCE * TOOL_AXIS
    init = hover + np.array([0.0, 0.0, INIT_CLEARANCE])
    lift = grasp + np.array([0.0, 0.0, GRASP_LIFT_HEIGHT])
    yaw = cube_yaw_for_gripper(np.array([GRIPPER_Y[0], GRIPPER_Y[1], 0.0]))
    return cube, grasp_tcp, grasp, hover, init, lift, yaw


def _metrics(q, p_w):
    p, _ = fk_pose(q)
    return (float(np.linalg.norm(p - world_to_dh(p_w))),
            joint_margin(q), sigma_min(q))


def scan(r_h):
    notes = []
    cube, grasp_tcp, grasp, hover, init, lift, yaw = derive(r_h)

    # [1] 路标 IK
    qs = {}
    for tag, p_w in [("grasp", grasp), ("hover", hover),
                     ("init", init), ("lift", lift)]:
        q, m, _ = solve_ik_multi(world_to_dh(p_w), RQ, n_init=16, seed=7)
        if q is None:
            return False, None, f"{tag} 不可达"
        dp, _, smin = _metrics(q, p_w)
        if dp > 1e-6 or m < np.deg2rad(2.0) or smin < 0.05:
            return False, q, (f"{tag}: dp={dp:.1e} margin={np.rad2deg(m):.1f}deg"
                              f" smin={smin:.3f} [不足]")
        qs[tag] = q
        notes.append(f"{tag}: margin {np.rad2deg(m):.1f}deg smin {smin:.3f}")

    # [2] INIT -> hover 下降链
    q_prev, worst = qs["init"], [0.0, np.inf]
    for s in np.linspace(0.0, 1.0, 20)[1:]:
        p_w = init + s * (hover - init)
        q, _, _ = solve_ik(world_to_dh(p_w), RQ, q_prev)
        dp, _, smin = _metrics(q, p_w)
        worst = [max(worst[0], dp), min(worst[1], smin)]
        q_prev = q
    if worst[0] > 1e-6 or worst[1] < 0.05:
        return False, qs["init"], "; ".join(notes) + " [下降链失败]"
    notes.append(f"下降链 smin {worst[1]:.3f}")

    # [3] 进插 + 提升链
    q_prev, worst = qs["hover"], [0.0, np.inf]
    for s in np.linspace(0.0, 1.0, 20)[1:]:
        p_w = hover + s * (grasp - hover)
        q, _, _ = solve_ik(world_to_dh(p_w), RQ, q_prev)
        dp, _, smin = _metrics(q, p_w)
        worst = [max(worst[0], dp), min(worst[1], smin)]
        q_prev = q
    for s in np.linspace(0.0, 1.0, 10)[1:]:
        p_w = grasp + s * (lift - grasp)
        q, _, _ = solve_ik(world_to_dh(p_w), RQ, q_prev)
        dp, _, smin = _metrics(q, p_w)
        worst = [max(worst[0], dp), min(worst[1], smin)]
        q_prev = q
    if worst[0] > 1e-6 or worst[1] < 0.05:
        return False, qs["init"], "; ".join(notes) + " [进插/提升链失败]"
    notes.append(f"进插链 smin {worst[1]:.3f}")

    # [4] 带载圆周整圈
    rad = np.array([cube[0], cube[1], 0.0])
    rad = rad / np.linalg.norm(rad)
    tang = np.array([-rad[1], rad[0], 0.0])
    center = grasp_tcp + np.array([0.0, 0.0, GRASP_LIFT_HEIGHT]) \
        + CIRCLE_RADIUS * rad
    q_prev, worst = qs["lift"], [0.0, np.inf, np.inf]
    for k in range(73):
        th = 2.0 * np.pi * k / 72.0
        p_tcp = center - CIRCLE_RADIUS * (np.cos(th) * rad
                                          + np.sin(th) * tang)
        p_w = p_tcp + L_TCP * TOOL_AXIS
        q, _, _ = solve_ik(world_to_dh(p_w), RQ, q_prev)
        dp, m, smin = _metrics(q, p_w)
        worst = [max(worst[0], dp), min(worst[1], m), min(worst[2], smin)]
        q_prev = q
    if (worst[0] > 1e-6 or worst[1] < np.deg2rad(3.0) or worst[2] < 0.05):
        return False, qs["init"], "; ".join(notes) + " [圆周失败]"
    notes.append(f"圆周 margin {np.rad2deg(worst[1]):.1f}deg smin {worst[2]:.3f}")

    return True, qs["init"], "; ".join(notes), cube, yaw


def main():
    print(f"当前径向 = {np.round(RAD0, 4).tolist()}（方位 "
          f"{np.rad2deg(np.arctan2(RAD0[1], RAD0[0])):.1f} deg），"
          f"当前立方体水平距离 {np.hypot(*CUBE_POS[:2]):.4f} m\n")
    best = None
    for r_h in (0.50, 0.52, 0.54, 0.56):
        out = scan(r_h)
        if len(out) == 5:
            ok, q_init, note, cube, yaw = out
        else:
            ok, q_init, note = out
            cube, yaw = None, None
        print(f"r_h={r_h:.2f}: {'PASS' if ok else 'FAIL'}  {note}")
        if ok and best is None:
            best = (r_h, q_init, cube, yaw)
    if best is None:
        print("结论: FAIL - 无可行外移距离")
        return 1
    r_h, q_init, cube, yaw = best
    print(f"\n结论: PASS - 建议回填 CUBE_POS（水平距离 {r_h:.2f} m）：")
    print(f"CUBE_POS = np.array([{cube[0]:.4f}, {cube[1]:.4f}, "
          f"CUBE_SIZE / 2])")
    print(f"CUBE_YAW = np.deg2rad({np.rad2deg(yaw):.1f})")
    print("Q_INIT = np.array([" + ", ".join(f"{v:+.6f}" for v in q_init)
          + "])")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
