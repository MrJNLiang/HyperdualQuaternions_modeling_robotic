"""
顶抓可达性定向核查（用户提出"爪子朝下从上往下抓"方案的数值验证）。

背景：search_anchor.py 120k 采样统计结论为"近竖直域（tilt<30 deg，
TCP z<=0.05）样本 0，顶抓不可达；侧抓域 75~105 deg 是唯一方式"。
本脚本针对当前 CUBE_POS 做定向 IK 验证（统计可能有洞，先实锤再定方案）：

  [A] 当前立方体位置纯顶抓：tilt = 0/10/20/30 deg（gripper +x 竖直
      向下，方位取 PHI+pi = 指向基座，经 make_tool_pose(tilt, phi) 构造），
      TCP = 立方体中心 +5 mm，多初值 IK；
  [B] 顶抓外推：tilt=0/15 deg，沿立方体径向（远离底座）步进 0.05 m，
      找到顶抓可行所需的最小外移距离（验证"放远一点就能顶抓"）；
  [C] 当前位置陡倾极限：tilt = 25~75 deg 步进 5，找当前距离下最陡
      可行接近角（折中方案候选）；
  [S] 健全性自检：当前水平侧抓姿态（params TOOL_AXIS 同款）必须 OK，
      否则探针本身有 bug。

坐标系约定（两版探针 sign bug 教训）：make_tool_pose(tilt, phi) 的
gripper +x 轴 = [sin(tilt)cos(phi), sin(tilt)sin(phi), cos(tilt)]——
tilt=0 竖直向上、tilt=90 水平、tilt=180 竖直向下；顶抓接近向 = 工具轴
近竖直向下且水平投影朝基座 = make_tool_pose(180-eps, PHI+pi)。

用法（TNDQ_b601 目录下）：
    python3 experiments/probe_topdown_reach.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    CUBE_POS, GRASP_TCP_Z_OFFSET, L_TCP, TOOL_AXIS,
)
from experiments.ik_lib import (
    solve_ik_multi, make_tool_pose, world_to_dh, dh_to_world,
    joint_margin, sigma_min,
)

# 立方体水平方位（从基座指向立方体）与 TCP 目标高度
RAD = np.array([CUBE_POS[0], CUBE_POS[1], 0.0])
RAD = RAD / np.linalg.norm(RAD)
PHI = float(np.arctan2(RAD[1], RAD[0]))          # 指向立方体的方位角
TCP_Z = CUBE_POS[2] + GRASP_TCP_Z_OFFSET         # TCP 目标高度（世界系）

OK_RES = 1e-6          # IK 收敛判据（位置+姿态残差范数，m/rad 同阶）
OK_MARGIN = np.deg2rad(2.0)   # 限位余量下限
OK_SIGMA = 0.05        # sigma_min 下限


def _gripper_pos(p_tcp_dh, tilt, phi):
    """TCP（指间中心）DH 系位置 -> gripper 原点 DH 系位置。"""
    _, q_tool = make_tool_pose(np.deg2rad(tilt), phi)
    from experiments.ik_lib import quat_to_R
    axis = quat_to_R(q_tool)[:, 0]               # gripper +x（世界系）
    return p_tcp_dh + L_TCP * axis, q_tool, axis


def _try(tilt, phi, p_tcp_world, n_init=16):
    """单点 IK：返回 (ok, 信息 dict)。"""
    p_tcp_dh = world_to_dh(p_tcp_world)
    p_w, q_tool, axis = _gripper_pos(p_tcp_dh, tilt, phi)
    q, margin, res = solve_ik_multi(p_w, q_tool, n_init=n_init, seed=11)
    if q is None:
        return False, {"res": np.inf, "margin": 0.0, "smin": 0.0, "q": None}
    ok = (res < OK_RES and margin > OK_MARGIN)
    info = {"res": res, "margin": margin,
            "smin": sigma_min(q) if ok else 0.0, "q": q}
    ok = ok and info["smin"] > OK_SIGMA
    return ok, info


def main():
    print(f"立方体中心 {CUBE_POS}，水平距离 {np.hypot(*CUBE_POS[:2]):.4f} m，"
          f"方位 {np.rad2deg(PHI):.1f} deg")
    print(f"TCP 目标高度 = {TCP_Z:.4f} m（中心 + {GRASP_TCP_Z_OFFSET} m）\n")

    # ---- [S] 健全性自检：水平侧抓（已知可行）必须 PASS -------------------
    print("[S] 自检（当前水平侧抓姿态，应 OK）:")
    phi_side = float(np.arctan2(TOOL_AXIS[1], TOOL_AXIS[0]))
    p_tcp = CUBE_POS + np.array([0.0, 0.0, GRASP_TCP_Z_OFFSET])
    ok, info = _try(90.0, phi_side, p_tcp)
    print(f"    tilt=90 deg phi={np.rad2deg(phi_side):.1f} deg: "
          f"{'OK ' if ok else 'FAIL!!'}  res={info['res']:.2e}  "
          f"限位余量={np.rad2deg(info['margin']):.1f} deg  "
          f"sigma_min={info['smin']:.3f}")
    if not ok:
        print("    探针自检失败，结果不可信！")
        return

    # ---- [A] 当前位置纯顶抓 ---------------------------------------------
    # 接近向竖直向下：gripper +x 指向下（tilt=180-eps）、水平投影朝基座
    phi_down = PHI + np.pi
    print(f"\n[A] 当前位置纯顶抓（接近向竖直向下，phi={np.rad2deg(phi_down):.1f} deg）:")
    for tilt in [180.0, 170.0, 160.0, 150.0]:
        ok, info = _try(tilt, phi_down, p_tcp)
        print(f"    tilt={tilt:5.1f} deg（离竖直 {180.0 - tilt:.0f} deg）: "
              f"{'OK ' if ok else 'FAIL'}  res={info['res']:.2e}  "
              f"限位余量={np.rad2deg(info['margin']):.1f} deg  "
              f"sigma_min={info['smin']:.3f}")

    # ---- [B] 顶抓沿径向外推 ---------------------------------------------
    print("\n[B] 顶抓沿径向外推（多远才能顶抓？）:")
    for tilt in [180.0, 165.0]:
        row = []
        for d in np.arange(0.0, 0.251, 0.05):
            p_tcp_d = CUBE_POS + np.array([0.0, 0.0, GRASP_TCP_Z_OFFSET]) \
                + d * RAD
            ok, info = _try(tilt, phi_down, p_tcp_d, n_init=20)
            row.append(("OK" if ok else "FAIL"))
        dists = ", ".join(f"{d:.2f}:{r}" for d, r in
                          zip(np.arange(0.0, 0.251, 0.05), row))
        print(f"    tilt={tilt:5.1f} deg  外移(m):可行 -> {dists}")

    # ---- [C] 当前位置陡倾极限（从顶抓域向侧抓域扫描）---------------------
    print("\n[C] 当前位置陡倾极限（最陡能到多少？接近向朝基座）:")
    best_tilt = None
    for tilt in range(155, 100, -5):
        ok, info = _try(tilt, phi_down, p_tcp, n_init=20)
        print(f"    tilt={tilt} deg（离竖直 {180 - tilt} deg）: "
              f"{'OK ' if ok else 'FAIL'}  "
              f"res={info['res']:.2e}  "
              f"限位余量={np.rad2deg(info['margin']):.1f} deg  "
              f"sigma_min={info['smin']:.3f}")
        if ok:
            best_tilt = tilt

    # ---- 结论 -------------------------------------------------------------
    print("\n结论汇总:")
    print(f"  - 当前位置纯顶抓（离竖直<=30 deg）可行性见 [A]")
    print(f"  - 顶抓所需最小外移见 [B]（全 FAIL 则该臂结构不支持低空顶抓）")
    print(f"  - 当前位置最陡可行接近角 tilt = "
          f"{best_tilt if best_tilt else '无（仅水平侧抓）'} deg"
          f"（离竖直 {180 - best_tilt if best_tilt else 'NA'} deg）")
    print(f"  - 参考：当前水平侧抓工具轴 = {TOOL_AXIS}（tilt "
          f"{np.degrees(np.arccos(np.clip(TOOL_AXIS[2], -1, 1))):.1f} deg）")


if __name__ == "__main__":
    main()
