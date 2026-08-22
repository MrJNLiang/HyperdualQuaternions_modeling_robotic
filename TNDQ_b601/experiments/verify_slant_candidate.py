"""斜抓候选复核 —— 首选/备选候选的全链几何验证。

search_slant_grasp.py 输出 top5 后的第二道关卡：
  1. 限位余量 / sigma_min 真实过滤（表格层未过滤）；
  2. 抓取位手指下缘离地间隙（防穿地；手指装于 gripper x=-0.0421、
     沿 y 开合，行程 0.0715）；
  3. 新 phi 下的立方体对准角（cube_yaw_for）；
  4. SETPOINT / INIT 的 IK（Q_INIT 重算）与下降/进插链可行性；
  5. 与 Q_F 冻结构型的距离对照。

用法（纯 python）：python experiments/verify_slant_candidate.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    CUBE_SIZE, GRASP_TCP_Z_OFFSET, L_TCP, SETPOINT_HOVER_CLEARANCE,
    INIT_CLEARANCE, JOINT_LOWER, JOINT_UPPER,
)
from experiments.ik_lib import (
    solve_ik_multi, make_tool_pose, world_to_dh, quat_to_R,
    joint_margin, sigma_min, cube_yaw_for, dh_frames_world, fk_pose,
)

# 手指几何（URDF gripper_link：手指装于 x=-0.0421，沿 y 开合）
FINGER_X = -0.0421
FINGER_HALF_OPEN = 0.0715 / 2

# search_slant_grasp top 候选（r, tilt）
CANDIDATES = [(0.550, 85), (0.525, 85), (0.575, 85), (0.550, 80)]

# Q_F 冻结构型（probe_combo_scan 锁死点），对照用
Q_FREEZE = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])

PHI_CUBE = -0.252


def finger_clearance(p_grip, R_tool):
    """抓取位 gripper 框内手指角点的最低 z（世界系）。"""
    corners = []
    for fx in (FINGER_X, 0.0):                 # 手指根 -> 指尖
        for fy in (-FINGER_HALF_OPEN, FINGER_HALF_OPEN):
            corners.append(p_grip + R_tool @ np.array([fx, fy, 0.0]))
    return min(c[2] for c in corners)


def check(tag, r, tilt_deg):
    tilt = np.deg2rad(tilt_deg)
    cube = np.array([r * np.cos(PHI_CUBE), r * np.sin(PHI_CUBE),
                     CUBE_SIZE / 2])
    p_tcp = cube + np.array([0.0, 0.0, GRASP_TCP_Z_OFFSET])
    R_tool, q_tool = make_tool_pose(tilt, PHI_CUBE)
    p_w = world_to_dh(p_tcp) + L_TCP * R_tool[:, 0]

    print(f"\n=== 候选 r={r:.3f} m, tilt={tilt_deg} deg ===")
    q_g, margin, res = solve_ik_multi(p_w, q_tool, n_init=40, seed=17)
    if q_g is None or res > 1e-6:
        print(f"  抓取位 IK 失败")
        return None
    smin = sigma_min(q_g)
    mdeg = np.rad2deg(margin)
    print(f"  抓取位: margin={mdeg:.1f} deg  sigma_min={smin:.3f}  "
          f"res={res:.2e}")
    print(f"  q_grasp = {np.round(q_g, 4).tolist()}")
    print(f"  |q-q_F| = {np.round(np.abs(q_g - Q_FREEZE), 3).tolist()}  "
          f"(欧氏 {np.linalg.norm(q_g - Q_FREEZE):.3f} rad)")

    # 穿地检查：抓取位手指角点 + 各 DH 帧
    # gripper 原点（世界系）= TCP + L_TCP*axis
    grip_w = p_tcp + L_TCP * R_tool[:, 0]
    clr = finger_clearance(grip_w, R_tool)
    frames = dh_frames_world(q_g)
    print(f"  抓取位手指最低点 z = {clr*1000:.1f} mm   "
          f"DH 帧最低 z = {frames[:, 2].min()*1000:.1f} mm")

    # SETPOINT：沿 +x_g 退开
    setpoint = grip_w + SETPOINT_HOVER_CLEARANCE * R_tool[:, 0]
    q_s, m_s, res_s = solve_ik_multi(world_to_dh(setpoint), q_tool,
                                     n_init=40, seed=23, q_warm=q_g)
    ok_s = q_s is not None and res_s < 1e-6 and m_s > np.deg2rad(2)
    print(f"  SETPOINT = {np.round(setpoint, 4).tolist()}")
    print(f"  SETPOINT IK: {'PASS' if ok_s else 'FAIL'}  "
          f"margin={np.rad2deg(m_s):.1f} deg  sigma={sigma_min(q_s):.3f}")

    # INIT：SETPOINT 上方
    init_p = setpoint + np.array([0.0, 0.0, INIT_CLEARANCE])
    q_i, m_i, res_i = solve_ik_multi(world_to_dh(init_p), q_tool,
                                     n_init=40, seed=29, q_warm=q_s)
    ok_i = q_i is not None and res_i < 1e-6 and m_i > np.deg2rad(2)
    print(f"  INIT = {np.round(init_p, 4).tolist()}")
    print(f"  INIT IK: {'PASS' if ok_i else 'FAIL'}  "
          f"margin={np.rad2deg(m_i):.1f} deg  sigma={sigma_min(q_i):.3f}")
    if ok_i:
        print(f"  Q_INIT = {np.round(q_i, 6).tolist()}")

    # 限位全程检查（INIT -> SETPOINT -> GRASP 直线插值）
    if ok_i and ok_s:
        traj = np.vstack([np.linspace(q_i, q_s, 11),
                          np.linspace(q_s, q_g, 11)])
        lo = (traj - JOINT_LOWER).min()
        hi = (JOINT_UPPER - traj).min()
        print(f"  下降链最小限位余量 = {np.rad2deg(min(lo, hi)):.1f} deg")

    # 对准角
    yaw = cube_yaw_for(PHI_CUBE)
    print(f"  立方体对准 yaw = {np.rad2deg(yaw):.1f} deg")
    print(f"  R_TOOL_QUAT(wxyz) = {np.round(q_tool, 6).tolist()}")
    print(f"  CUBE_POS = {np.round(cube, 4).tolist()}")

    if ok_s and ok_i and clr > 0.003 and mdeg > 5:
        return {"r": r, "tilt": tilt_deg, "margin": mdeg, "clr": clr,
                "q_grasp": q_g, "q_init": q_i}
    return None


def main():
    results = []
    for r, t in CANDIDATES:
        out = check("cand", r, t)
        if out:
            results.append(out)
    print("\n--- 汇总 ---")
    if not results:
        print("无候选通过全链复核")
        return
    results.sort(key=lambda d: (-d["clr"], -d["margin"]))
    for d in results:
        print(f"  r={d['r']:.3f} tilt={d['tilt']} deg  余量 {d['margin']:.1f} deg"
              f"  手指离地 {d['clr']*1000:.1f} mm")
    best = results[0]
    print(f"\n推荐候选: r={best['r']:.3f} m, tilt={best['tilt']} deg")


if __name__ == "__main__":
    main()
