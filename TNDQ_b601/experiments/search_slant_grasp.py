"""斜向抓取联合搜索 —— 立方体径向距离 × 工具倾角网格 IK 可行性。

用户方案：不要求垂直顶抓，斜着夹即可；立方体位置与轨迹可同时改。

坐标系约定（与 params 一致）：接近向 = gripper -x；立方体位于
TCP（指间中心），gripper 原点 = TCP + L_TCP * x̂；make_tool_pose(tilt, phi)
的 +x 轴 = [sin(tilt)cos(phi), sin(tilt)sin(phi), cos(tilt)]，tilt 从
竖直向上起量：tilt=0 接近向竖直向下（纯顶抓），tilt=90 接近向水平
（侧抓，当前 96.5）。斜抓域 = tilt ∈ [25, 85]，phi = 立方体方位
（+x 朝径向外上方，接近向 -x 朝基座斜下方）。

搜索空间：
    r    ∈ [0.30, 0.60] m（立方体中心水平距，步进 0.025）
    tilt ∈ [25, 85] deg
    TCP  = 立方体中心 + 5 mm（立方体在地面，中心高 0.0225 m）
判据：IK 残差 < 1e-6、限位余量 > 2 deg、sigma_min > 0.05。
输出：可行域表 + 按限位余量排序的 top5 候选（含 q 解），供回填 params。

用法（纯 python，无需 Isaac）：
    python experiments/search_slant_grasp.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import CUBE_SIZE, GRASP_TCP_Z_OFFSET, L_TCP
from experiments.ik_lib import (
    solve_ik_multi, make_tool_pose, world_to_dh, quat_to_R,
    joint_margin, sigma_min,
)

OK_RES = 1e-6
OK_MARGIN = np.deg2rad(2.0)
OK_SIGMA = 0.05

PHI_CUBE = -0.252          # 当前立方体方位角（rad，≈-14.4 deg），轴对称故不影响
TCP_Z = CUBE_SIZE / 2 + GRASP_TCP_Z_OFFSET


def try_point(r, tilt_deg, n_init=20):
    tilt = np.deg2rad(tilt_deg)
    phi = PHI_CUBE                       # +x 水平投影 = 径向朝外
    cube = np.array([r * np.cos(PHI_CUBE), r * np.sin(PHI_CUBE),
                     CUBE_SIZE / 2])
    p_tcp = cube + np.array([0.0, 0.0, GRASP_TCP_Z_OFFSET])
    _, q_tool = make_tool_pose(tilt, phi)
    axis = quat_to_R(q_tool)[:, 0]
    p_w = world_to_dh(p_tcp) + L_TCP * axis      # gripper 原点（DH 系）
    q, margin, res = solve_ik_multi(p_w, q_tool, n_init=n_init, seed=17)
    if q is None:
        return None
    ok = res < OK_RES and margin > OK_MARGIN
    smin = sigma_min(q) if ok else 0.0
    ok = ok and smin > OK_SIGMA
    return {"ok": ok, "res": res, "margin": margin, "smin": smin, "q": q,
            "axis": axis, "q_tool": q_tool, "cube": cube}


def main():
    rs = np.arange(0.30, 0.601, 0.025)
    tilts = list(range(25, 86, 5))
    print(f"TCP 高度 {TCP_Z:.4f} m；+x 朝径向外上方（phi={np.rad2deg(PHI_CUBE):.1f} deg），"
          f"接近向 -x 朝基座斜下方")
    print("可行域（O=可行，x=不可行；tilt 列，从竖直起量）:")
    print("  r\\tilt " + " ".join(f"{t:4d}" for t in tilts))
    table = {}
    for r in rs:
        row = []
        for t in tilts:
            res = try_point(r, t)
            ok = bool(res and res["ok"])
            row.append("O" if ok else "x")
            if ok:
                table[(round(r, 3), t)] = res
        print(f"  {r:5.3f}   " + "    ".join(row))
    print(f"\n可行点数: {len(table)} / {len(rs) * len(tilts)}")
    if not table:
        print("全域不可行！")
        return
    top5 = sorted(table.items(), key=lambda kv: -kv[1]["margin"])[:5]
    print("\nTop5 候选（按限位余量）:")
    for (r_b, t_b), info in top5:
        print(f"  r={r_b:.3f} m, tilt={t_b} deg: 余量 "
              f"{np.rad2deg(info['margin']):.1f} deg, "
              f"sigma_min={info['smin']:.3f}, q={np.round(info['q'], 3).tolist()}")
    (r_b, t_b), info = top5[0]
    axis, q_tool, cube = info["axis"], info["q_tool"], info["cube"]
    print(f"\n首选候选几何（世界系）:")
    print(f"  立方体中心 = {np.round(cube, 4).tolist()}")
    print(f"  工具轴 +x = {np.round(axis, 4).tolist()}  "
          f"q_tool={np.round(q_tool, 4).tolist()}")
    tcp = cube + np.array([0, 0, GRASP_TCP_Z_OFFSET])
    grip0 = tcp + L_TCP * axis
    setpoint = grip0 + 0.08 * axis
    print(f"  抓取 TCP = {np.round(tcp, 4).tolist()}")
    print(f"  gripper 原点（抓取位）= {np.round(grip0, 4).tolist()}")
    print(f"  SETPOINT(+x 退 8cm) = {np.round(setpoint, 4).tolist()}")
    print(f"  INIT(SETPOINT 上方 8cm) = {np.round(setpoint + np.array([0, 0, 0.08]), 4).tolist()}")


if __name__ == "__main__":
    main()
