"""
B601 抓取锚点搜索 v3：样本驱动水平侧抓（结论回填 config/params.py）。

三轮数据驱动的方案演进：
  v1 竖直顶抓失败：世界系（含 B601_BASE_PREFIX）下"工具倾斜 < 25 deg +
     防穿地"域内 TCP 最低 0.216 m，无法触及地面物体（j2/j3 负限位 +
     偏置腕）。
  v2 精确水平姿态失败：侧抓域（tilt 75~105 deg）样本丰富（3076/60000），
     但"tilt = 90 deg + 手指开合严格水平"是测度零的苛刻约束——规范化
     姿态在同位置可达率仅 ~47%（诊断 [4]），90 候选全灭。
  v3 样本驱动：从可行样本中选姿态本身接近规范的种子（|tilt-90|<=18
     deg、|y_g.z|<=0.20，夹持倾斜 ~11 deg 内由摩擦吸收），任务姿态 =
     种子实际姿态（恒定），立方体绕 z 对准 y_g 的水平投影。

种子筛选（限位内随机采样 + 防穿地 + TCP 低空 + 半径域）-> 方位分桶
分散化 -> 每种子验证任务链（目标世界系，进 IK 前 world_to_dh）：
  [anchor] 抓取位：TCP = 立方体中心上方 GRASP_TCP_Z_OFFSET，
           gripper 原点 = TCP + L_TCP * x_g；
  [hover]  抓取位沿 -x_g 水平退开 HOVER_CLR（实验一定点目标），
           其上方 INIT_CLR 为 Q_INIT；
  [lift]   抓取位竖直提升 LIFT_H（圆周起始）；
  [circle] 提升后水平圆周 16 点热启动 IK：圆心沿径向外偏 CIRC_R，
           theta=0 恰为提升位。

排序：整圈最小限位余量降序（圆周持续跟踪的安全储备）。

用法（TNDQ_b601 目录下）：
    python3 experiments/search_anchor.py
"""

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import CUBE_SIZE, L_TCP, JOINT_LOWER, JOINT_UPPER
from experiments.ik_lib import (
    solve_ik, joint_margin, sigma_min, world_to_dh, dh_to_world,
    fk_pose, quat_to_R, dh_frames_world, cube_yaw_for_gripper,
)

# --- 搜索域任务常量（与 params.py 目标语义一致，选定后回填） -------------
GRASP_TCP_Z_OFFSET = 0.005     # TCP 高于立方体中心 [m]（手指下缘离地余量）
LIFT_H = 0.07                   # 抓取后竖直提升 [m]（= GRASP_LIFT_HEIGHT）
CIRC_R = 0.06                   # 圆周半径 [m]（= CIRCLE_RADIUS）
HOVER_CLR = 0.08                # 抓取位沿 -x_g 水平退开 [m]
INIT_CLR = 0.08                 # hover 上方竖直抬高 [m]（Q_INIT）
TCP_Z = CUBE_SIZE / 2 + GRASP_TCP_Z_OFFSET

# --- 种子筛选参数 ----------------------------------------------------------
N_SAMPLE = 80000                # 随机采样数
TILT_BAND = np.deg2rad([72.0, 108.0])   # 工具轴与竖直夹角域（90=水平）
YG_Z_MAX = 0.20                 # 手指开合方向竖直分量上限（夹持倾斜）
TCP_Z_BAND = (0.018, 0.040)     # 种子 TCP 世界高度域（立方体中心附近）
R_BAND = (0.25, 0.50)           # TCP 半径域
FRAME_CLEARANCE = 0.015         # DH 帧离地下限 [m]
N_BUCKETS = 24                  # 方位分桶数（分散化）
PER_BUCKET = 3                  # 每桶保留种子数


def sample_seeds():
    """限位内随机采样 -> 侧抓可行种子（姿态 + 位置 + 关节角）。"""
    rng = np.random.default_rng(11)
    seeds = []
    for _ in range(N_SAMPLE):
        q = rng.uniform(JOINT_LOWER, JOINT_UPPER)
        p, r = fk_pose(q)                     # DH 系
        R = quat_to_R(r)
        xg, yg = R[:, 0], R[:, 1]
        tilt = np.arccos(np.clip(xg[2], -1.0, 1.0))
        if not (TILT_BAND[0] <= tilt <= TILT_BAND[1]):
            continue
        if abs(yg[2]) > YG_Z_MAX:
            continue
        if dh_frames_world(q)[:, 2].min() < FRAME_CLEARANCE:
            continue
        tcpw = dh_to_world(p) - L_TCP * xg    # TCP 世界坐标
        if not (TCP_Z_BAND[0] <= tcpw[2] <= TCP_Z_BAND[1]):
            continue
        rr = np.hypot(tcpw[0], tcpw[1])
        if not (R_BAND[0] <= rr <= R_BAND[1]):
            continue
        seeds.append({
            "q": q, "rq": r, "xg": xg, "yg": yg,
            "tcpw": tcpw, "tilt": tilt, "margin": joint_margin(q),
        })
    return seeds


def pick_diverse(seeds):
    """按 TCP 方位分桶分散化，每桶取姿态最规范（y 水平、tilt 近 90、
    限位余量大）的前 PER_BUCKET 个种子。"""
    buckets = {}
    for sd in seeds:
        az = np.arctan2(sd["tcpw"][1], sd["tcpw"][0])
        b = int((az + np.pi) / (2.0 * np.pi) * N_BUCKETS) % N_BUCKETS
        buckets.setdefault(b, []).append(sd)
    picked = []
    for b in sorted(buckets):
        lst = sorted(buckets[b], key=lambda s: (
            abs(s["yg"][2]),
            abs(np.rad2deg(s["tilt"]) - 90.0),
            -s["margin"],
        ))
        picked.extend(lst[:PER_BUCKET])
    return picked


def eval_seed(sd, stats=None):
    """验证一个种子锚点的完整任务链，通过返回统计 dict，否则 None。"""
    def _fail(stage):
        if stats is not None:
            stats[stage] = stats.get(stage, 0) + 1
        return None

    xg, yg, rq = sd["xg"], sd["yg"], sd["rq"]
    tcp_xy = sd["tcpw"][:2]
    tcp_g = np.array([tcp_xy[0], tcp_xy[1], TCP_Z])       # 世界系
    pg = tcp_g + L_TCP * xg                               # 抓取位 gripper 原点
    rad = np.array([tcp_xy[0], tcp_xy[1], 0.0]) / np.linalg.norm(tcp_xy)
    tang = np.array([-rad[1], rad[0], 0.0])

    # [anchor] 抓取位（热启动 = 种子关节角）
    q_a, _, ok = solve_ik(world_to_dh(pg), rq, sd["q"], max_nfev=250)
    if not ok:
        return _fail("anchor_ik")
    m_a = joint_margin(q_a)
    if m_a < np.deg2rad(4.0):
        return _fail("anchor_margin")

    # [hover] 水平退开（热启动），再竖直抬高 INIT_CLR 为 Q_INIT
    p_h = pg - HOVER_CLR * xg
    q_h, _, ok_h = solve_ik(world_to_dh(p_h), rq, q_a, max_nfev=200)
    if not ok_h:
        return _fail("hover_ik")
    p_i = p_h + np.array([0.0, 0.0, INIT_CLR])
    q_i, _, ok_i = solve_ik(world_to_dh(p_i), rq, q_h, max_nfev=200)
    if not ok_i:
        return _fail("init_ik")

    # [lift] 竖直提升（热启动）
    p_l = pg + np.array([0.0, 0.0, LIFT_H])
    q_l, _, ok = solve_ik(world_to_dh(p_l), rq, q_a, max_nfev=200)
    if not ok:
        return _fail("lift_ik")

    # [circle] 提升后水平圆周 16 点（圆心沿径向外偏，theta=0 = 提升位）
    center = tcp_g + np.array([0.0, 0.0, LIFT_H]) + CIRC_R * rad
    mmin, smin_min = np.inf, np.inf
    q_prev = q_l
    for k in range(1, 17):
        th = 2.0 * np.pi * k / 16.0
        p_tcp = center - CIRC_R * (np.cos(th) * rad + np.sin(th) * tang)
        q, _, ok = solve_ik(world_to_dh(p_tcp + L_TCP * xg), rq,
                            q_prev, max_nfev=200)
        if not ok:
            return _fail("circle_ik")
        mmin = min(mmin, joint_margin(q))
        smin_min = min(smin_min, sigma_min(q))
        q_prev = q
    if mmin < np.deg2rad(3.0) or smin_min < 0.05:
        return _fail("circle_margin")

    yg_h = np.array([yg[0], yg[1], 0.0])
    return {
        "q_anchor": q_a, "q_lift": q_l, "q_hover": q_h, "q_init": q_i,
        "rq": rq, "xg": xg, "yg": yg,
        "tcp": tcp_g, "m_anchor": m_a, "m_circle": mmin,
        "smin_circle": smin_min,
        "tilt_deg": float(np.rad2deg(sd["tilt"])),
        "yg_z": float(abs(yg[2])),
        "cube_yaw": cube_yaw_for_gripper(yg_h),
    }


def main():
    t0 = time.time()
    print(f"[1] 采样 {N_SAMPLE} 个限位内构型，筛选侧抓种子...")
    seeds = sample_seeds()
    print(f"    种子数：{len(seeds)}")
    if not seeds:
        print("FAIL：无种子，需放宽筛选")
        return 1
    picked = pick_diverse(seeds)
    print(f"[2] 方位分桶分散化后验证 {len(picked)} 个种子（每桶 "
          f"{PER_BUCKET} 个，共 {N_BUCKETS} 桶）...")

    results, stats = [], {}
    for sd in picked:
        res = eval_seed(sd, stats)
        if res is not None:
            results.append(res)
    print(f"[3] 通过全部关卡：{len(results)} / {len(picked)}"
          f"（{time.time() - t0:.1f} s）")
    print("    各关失败统计：")
    for stage in sorted(stats, key=lambda k: -stats[k]):
        print(f"        {stage:<15s} {stats[stage]:4d}")
    if not results:
        print("FAIL：无候选通过")
        return 1

    # 排序：整圈最小限位余量降序，并列者 sigma_min 大者优先
    results.sort(key=lambda d: (d["m_circle"], d["smin_circle"]), reverse=True)

    print("\n[top 候选]（按整圈最小限位余量降序）")
    for i, d in enumerate(results[:8]):
        print(f"\n--- 候选 {i + 1} ---")
        print(f"  TCP 世界 = ({d['tcp'][0]:+.4f}, {d['tcp'][1]:+.4f}, "
              f"{d['tcp'][2]:.4f})  半径 {np.hypot(d['tcp'][0], d['tcp'][1]):.3f} m")
        print(f"  tilt={d['tilt_deg']:.1f} deg（90=水平）  "
              f"|y_g.z|={d['yg_z']:.3f}  "
              f"CUBE_YAW={np.rad2deg(d['cube_yaw']):+.1f} deg")
        print(f"  锚点限位余量={np.rad2deg(d['m_anchor']):.1f} deg  "
              f"整圈最小余量={np.rad2deg(d['m_circle']):.1f} deg  "
              f"整圈 sigma_min={d['smin_circle']:.3f}")
        print(f"  q_anchor = np.array([{', '.join(f'{v:+.4f}' for v in d['q_anchor'])}])")
        print(f"  q_lift   = np.array([{', '.join(f'{v:+.4f}' for v in d['q_lift'])}])")
        print(f"  q_init   = np.array([{', '.join(f'{v:+.4f}' for v in d['q_init'])}])")
        print(f"  R_TOOL_QUAT = np.array([{', '.join(f'{v:+.6f}' for v in d['rq'])}])"
              f"   # [w,x,y,z]")
        print(f"  TOOL_AXIS = np.array([{', '.join(f'{v:+.6f}' for v in d['xg'])}])")
        print(f"  GRIPPER_Y  = np.array([{', '.join(f'{v:+.6f}' for v in d['yg'])}])")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
