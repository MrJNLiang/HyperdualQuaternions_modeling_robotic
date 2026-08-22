"""
B601 任务几何回归验证（结论已回填 config/params.py，本脚本作为验证保留）。

验证链（姿态恒定 R_TOOL；目标为世界系 gripper 原点位置，进 IK 前经
world_to_dh 转 DH 基座系）：
  [1] 锚点几何自检：任务姿态水平性 / TCP 高度 / 立方体对准角；
  [2] 路标多初值 IK：anchor / hover / Q_INIT / lift（限位余量最大解）；
  [3] 实验一路径：Q_INIT -> hover 直线插值 20 点热启动 IK；
  [4] 实验二路径：hover -> grasp 沿 TOOL_AXIS 水平进插 20 点 +
      grasp -> lift 竖直提升 10 点（热启动）；
  [5] 带载圆周整圈 72 点热启动 IK（theta=0 = lift 位，theta=2pi 闭合）；
  [6] Q_INIT 回填建议。

IK 基础设施与全部已验证结论（DQ twist 语义 / 残差-雅可比配套 /
TRF 信赖域）见 experiments/ik_lib.py。

用法（TNDQ_b601 目录下）：
    python3 experiments/design_task_geometry.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    CUBE_POS, CUBE_SIZE, CUBE_YAW, GRASP_TCP, GRASP_TCP_Z_OFFSET,
    GRASP_POS, SETPOINT_POS, INIT_POS, LIFT_POS, L_TCP,
    SETPOINT_HOVER_CLEARANCE, GRASP_LIFT_HEIGHT,
    CIRCLE_CENTER, CIRCLE_RADIUS, TOOL_AXIS, GRIPPER_Y, R_TOOL_QUAT,
    JOINT_LOWER, JOINT_UPPER,
)
from experiments.ik_lib import (
    solve_ik, solve_ik_multi, report, joint_margin,
    world_to_dh, cube_yaw_for_gripper,
)

# IK 内部限位同 ik_lib（留 5 度安全边距）
RQ = R_TOOL_QUAT / np.linalg.norm(R_TOOL_QUAT)


def main():
    ok_all = True

    # ---- [1] 锚点几何自检 ------------------------------------------------
    tilt = np.degrees(np.arccos(np.clip(TOOL_AXIS[2], -1.0, 1.0)))
    grasp_tilt = np.degrees(np.arcsin(np.clip(abs(GRIPPER_Y[2]), -1.0, 1.0)))
    print("[1] 任务姿态自检（水平侧抓方案，search_anchor 候选 1）:")
    print(f"    工具轴 tilt = {tilt:.1f} deg（90 = 水平）")
    print(f"    手指开合竖直分量 |GRIPPER_Y.z| = {abs(GRIPPER_Y[2]):.3f}"
          f" -> 夹持倾斜 {grasp_tilt:.1f} deg（摩擦 0.6 可吸收）")
    print(f"    TCP 高度 = {GRASP_TCP[2]:.4f} m"
          f"（立方体中心 {CUBE_POS[2]:.4f} + 偏置 {GRASP_TCP_Z_OFFSET:.3f} m）")
    yaw_ref = np.degrees(cube_yaw_for_gripper(
        np.array([GRIPPER_Y[0], GRIPPER_Y[1], 0.0])))
    yaw_err = abs((yaw_ref - np.degrees(CUBE_YAW) + 180.0) % 360.0 - 180.0)
    print(f"    立方体对准角：CUBE_YAW={np.degrees(CUBE_YAW):+.1f} deg vs "
          f"GRIPPER_Y 水平投影 {yaw_ref:+.1f} deg（偏差 {yaw_err:.1f} deg）")
    print(f"    手指行程 0.0715 m vs 立方体边长 {CUBE_SIZE:.3f} m：可夹持")

    # ---- [2] 路标 IK ------------------------------------------------------
    print("[2] 路标 IK（多初值 -> 最大限位余量解，世界系目标）:")
    results = {}
    for tag, p_w in [("anchor(抓取位)", GRASP_POS),
                     ("hover(实验一目标)", SETPOINT_POS),
                     ("init(Q_INIT位)", INIT_POS),
                     ("lift(圆周起始)", LIFT_POS)]:
        q, m, res = solve_ik_multi(world_to_dh(p_w), RQ, n_init=16, seed=7)
        if q is None:
            print(f"    {tag:<18s} FAIL：无收敛解")
            ok_all = False
            continue
        report(tag, q, world_to_dh(p_w), RQ)
        results[tag] = q
    if len(results) < 4:
        print("结论: FAIL - 存在不可达路标")
        return 1
    q_anchor, q_hover, q_init, q_lift = (results[k] for k in
                                         ("anchor(抓取位)", "hover(实验一目标)",
                                          "init(Q_INIT位)", "lift(圆周起始)"))

    # ---- [3] 实验一路径（Q_INIT -> hover 直线插值）-----------------------
    print("[3] 实验一路径（Q_INIT -> hover 直线插值 20 点，热启动）:")
    worst = [0.0, 0.0, np.inf, np.inf]
    q_prev = q_init
    for s in np.linspace(0.0, 1.0, 20)[1:]:
        p_w = INIT_POS + s * (SETPOINT_POS - INIT_POS)
        q, res, ok = solve_ik(world_to_dh(p_w), RQ, q_prev)
        dp, dth, m, smin = report(f"s={s:.2f}", q, world_to_dh(p_w), RQ)
        worst = [max(worst[0], dp), max(worst[1], dth),
                 min(worst[2], m), min(worst[3], smin)]
        q_prev = q
    print(f"    -> 最差: dp={worst[0]:.2e} m  dtheta={worst[1]:.2e} rad  "
          f"限位余量={np.rad2deg(worst[2]):.2f} deg  sigma_min={worst[3]:.4f}")
    if worst[0] > 1e-6 or worst[1] > 1e-6 or worst[2] < np.deg2rad(2.0):
        ok_all = False

    # ---- [4] 实验二路径（hover -> grasp 进插 + grasp -> lift 提升）------
    print("[4] 实验二路径（hover -> grasp 水平进插 20 点 + 竖直提升 10 点）:")
    worst4 = [0.0, 0.0, np.inf, np.inf]
    q_prev = q_hover
    for s in np.linspace(0.0, 1.0, 20)[1:]:
        p_w = SETPOINT_POS + s * (GRASP_POS - SETPOINT_POS)
        q, res, ok = solve_ik(world_to_dh(p_w), RQ, q_prev)
        dp, dth, m, smin = report(f"插值 s={s:.2f}", q, world_to_dh(p_w), RQ)
        worst4 = [max(worst4[0], dp), max(worst4[1], dth),
                  min(worst4[2], m), min(worst4[3], smin)]
        q_prev = q
    for s in np.linspace(0.0, 1.0, 10)[1:]:
        p_w = GRASP_POS + s * (LIFT_POS - GRASP_POS)
        q, res, ok = solve_ik(world_to_dh(p_w), RQ, q_prev)
        dp, dth, m, smin = report(f"提升 s={s:.2f}", q, world_to_dh(p_w), RQ)
        worst4 = [max(worst4[0], dp), max(worst4[1], dth),
                  min(worst4[2], m), min(worst4[3], smin)]
        q_prev = q
    print(f"    -> 最差: dp={worst4[0]:.2e} m  dtheta={worst4[1]:.2e} rad  "
          f"限位余量={np.rad2deg(worst4[2]):.2f} deg  sigma_min={worst4[3]:.4f}")
    if worst4[0] > 1e-6 or worst4[1] > 1e-6 or worst4[2] < np.deg2rad(2.0):
        ok_all = False

    # ---- [5] 带载圆周整圈（热启动）---------------------------------------
    print(f"[5] 带载圆周整圈（72 点热启动 IK，圆心 {CIRCLE_CENTER} "
          f"R={CIRCLE_RADIUS}）:")
    rad = np.array([CUBE_POS[0], CUBE_POS[1], 0.0])
    rad = rad / np.linalg.norm(rad)
    tang = np.array([-rad[1], rad[0], 0.0])
    worst5 = [0.0, 0.0, np.inf, np.inf]
    q_prev = q_lift
    for k in range(73):                     # 含 theta=2pi 闭合检验
        th = 2.0 * np.pi * k / 72.0
        p_tcp = CIRCLE_CENTER - CIRCLE_RADIUS * (np.cos(th) * rad
                                                 + np.sin(th) * tang)
        p_w = p_tcp + L_TCP * TOOL_AXIS      # gripper = TCP + L_TCP * 轴
        q, res, ok = solve_ik(world_to_dh(p_w), RQ, q_prev)
        dp, dth, m, smin = report(f"theta={np.rad2deg(th):5.0f} deg", q,
                                  world_to_dh(p_w), RQ)
        worst5 = [max(worst5[0], dp), max(worst5[1], dth),
                  min(worst5[2], m), min(worst5[3], smin)]
        q_prev = q
    print(f"    -> 最差: dp={worst5[0]:.2e} m  dtheta={worst5[1]:.2e} rad  "
          f"限位余量={np.rad2deg(worst5[2]):.2f} deg  sigma_min={worst5[3]:.4f}")
    if (worst5[0] > 1e-6 or worst5[1] > 1e-6
            or worst5[2] < np.deg2rad(3.0) or worst5[3] < 0.05):
        ok_all = False

    # ---- [6] Q_INIT 回填建议 ---------------------------------------------
    margin_init = joint_margin(q_init)
    print("[6] 建议 Q_INIT（回填 config/params.py）：")
    print("Q_INIT = np.array([" + ", ".join(f"{v:+.6f}" for v in q_init) + "])"
          + f"   # 最小限位余量 {np.rad2deg(margin_init):.1f} deg")

    print(f"结论: {'PASS - 任务几何全部可达' if ok_all else 'REVIEW - 余量偏紧或存在失败点'}")
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
