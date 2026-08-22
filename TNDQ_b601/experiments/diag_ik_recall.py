"""
诊断：世界系（含 B601_BASE_PREFIX）下重扫可行抓取域 + IK 召回率。

背景：search_anchor.py 144 候选全部 anchor_ik 失败。怀疑坐标系语义
错位——fk_pose 输出为 DH 基座系（z 比世界 / base_link 系低 0.08465 m，
prefix 为纯平移、姿态恒等），若目标位置按世界系定义而直接送入 IK，
相当于把目标整体抬高 8.5 cm。

输出：
  [1] 世界系可行抓取域扫描（工具倾斜 < 25 deg、DH 帧离地 > 2 cm），
      打印 gripper / TCP 世界高度与半径分布；
  [2] 地面抓取存在性：TCP 世界 z <= 0.04 的样本统计；
  [3] 可行样本 IK 召回率（自身位姿 + 扰动初值，热启动）；
  [4] 同位置规范化姿态（y 轴水平）可达率（IK 从样本 q 出发）。

用法：python3 experiments/diag_ik_recall.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    B601_DH_TABLE, B601_BASE_PREFIX, JOINT_LOWER, JOINT_UPPER, L_TCP,
)
from experiments.ik_lib import fk_pose, make_tool_pose, solve_ik, quat_to_R


def dh_mat(a, alpha, d, th):
    ct, st = np.cos(th), np.sin(th)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a],
        [st, ct * ca, -ct * sa, 0.0],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0]])


def dh_frames_world(q):
    """各 DH 帧原点的世界坐标（base_link 系，机器人立于地面时=世界系）。"""
    T = np.eye(4)
    pts = []
    for i, row in enumerate(B601_DH_TABLE):
        a, alpha, d, th0, _ = row
        T = T @ dh_mat(a, alpha, d, th0 + q[i])
        pts.append(T[:3, 3] + B601_BASE_PREFIX)
    return np.array(pts)


def main():
    rng = np.random.default_rng(5)
    N = 60000
    feas = []
    for _ in range(N):
        q = rng.uniform(JOINT_LOWER, JOINT_UPPER)
        p, r = fk_pose(q)                     # DH 基座系
        R = quat_to_R(r)
        xg = R[:, 0]
        tilt = np.arccos(np.clip(xg[2], -1.0, 1.0))
        if tilt > np.deg2rad(25.0):
            continue
        frames = dh_frames_world(q)
        if frames[:, 2].min() < 0.02:         # 连杆防穿地（世界系）
            continue
        pw = p + B601_BASE_PREFIX             # gripper 原点世界坐标
        feas.append((q, p, r, tilt, pw, xg))

    print(f"[1] 可行样本（倾斜<25deg + 防穿地）：{len(feas)} / {N}")
    if not feas:
        print("FAIL：可行域为空")
        return 1
    Pw = np.array([f[4] for f in feas])
    rxy = np.hypot(Pw[:, 0], Pw[:, 1])
    tcpw = np.array([f[4] - L_TCP * f[5] for f in feas])
    print(f"    gripper 世界 z: [{Pw[:, 2].min():.3f}, {Pw[:, 2].max():.3f}]  "
          f"半径: [{rxy.min():.3f}, {rxy.max():.3f}]")
    print(f"    TCP     世界 z: [{tcpw[:, 2].min():.3f}, {tcpw[:, 2].max():.3f}]")

    # [2] 地面抓取存在性
    low = [(f, tw) for f, tw in zip(feas, tcpw) if tw[2] <= 0.04]
    print(f"[2] TCP 世界 z <= 0.04 的样本：{len(low)}")
    if low:
        lw = np.array([tw for _, tw in low])
        lr = np.hypot(lw[:, 0], lw[:, 1])
        print(f"    TCP z: [{lw[:, 2].min():.3f}, {lw[:, 2].max():.3f}]  "
              f"半径: [{lr.min():.3f}, {lr.max():.3f}]")
        tilts = np.rad2deg([f[3] for f, _ in low])
        print(f"    倾斜: [{tilts.min():.1f}, {tilts.max():.1f}] deg")

    # [3] IK 召回率（自身位姿 + 扰动初值）
    n_test = min(30, len(feas))
    idx = rng.choice(len(feas), n_test, replace=False)
    n_rec = 0
    for i in idx:
        q, p, r, tilt, pw, xg = feas[i]
        q0 = q + rng.normal(0.0, 0.05, 6)
        _, res, ok = solve_ik(p, r, q0, max_nfev=200)
        n_rec += ok
    print(f"[3] IK 召回率（扰动 0.05 rad 初值）：{n_rec}/{n_test}")

    # [4] 同位置规范化姿态（y 水平）可达率
    n_norm_ok = 0
    errs = []
    for i in idx:
        q, p, r, tilt, pw, xg = feas[i]
        phi = np.arctan2(xg[1], xg[0])
        _, rq = make_tool_pose(tilt, phi)
        q_sol, res, ok = solve_ik(p, rq, q, max_nfev=200)
        n_norm_ok += ok
        if ok:
            errs.append(res)
    print(f"[4] 规范化姿态（同位置、y 水平）可达率：{n_norm_ok}/{n_test}")
    if errs:
        print(f"    收敛残差范数 max = {max(errs):.2e}")

    # [5] 低空 TCP 全景（不限倾斜）：侧抓可行性探索 -------------------------
    # 近竖直工具（tilt<25deg）下 TCP 最低 0.216 m，无法抓地面物体；
    # 检查低空 TCP 样本的工具方向分布，验证侧向抓取（工具轴水平）。
    rng2 = np.random.default_rng(23)
    low_all = []
    for _ in range(N):
        q = rng2.uniform(JOINT_LOWER, JOINT_UPPER)
        p, r = fk_pose(q)
        R = quat_to_R(r)
        xg = R[:, 0]
        frames = dh_frames_world(q)
        if frames[:, 2].min() < 0.01:       # 防穿地（贴地 1 cm）
            continue
        tcpw = p + B601_BASE_PREFIX - L_TCP * xg
        if tcpw[2] <= 0.05:
            tilt = np.rad2deg(np.arccos(np.clip(xg[2], -1.0, 1.0)))
            low_all.append((q, tilt, tcpw))
    print(f"[5] 低空全景（TCP 世界 z<=0.05 + 防穿地 1cm）：{len(low_all)} / {N}")
    if low_all:
        tilts = np.array([t for _, t, _ in low_all])
        tcps = np.array([t for _, _, t in low_all])
        rr = np.hypot(tcps[:, 0], tcps[:, 1])
        print(f"    TCP z: [{tcps[:, 2].min():.3f}, {tcps[:, 2].max():.3f}]  "
              f"半径: [{rr.min():.3f}, {rr.max():.3f}]")
        # 倾斜直方（15 deg 一档，90 deg = 工具轴水平=侧抓）
        bins = np.arange(0.0, 181.0, 15.0)
        hist, _ = np.histogram(tilts, bins=bins)
        print("    工具倾斜分布（90deg=水平侧抓）：")
        for i in range(len(hist)):
            if hist[i] > 0:
                bar = "#" * min(60, int(hist[i] / max(1, hist.max()) * 50))
                print(f"      [{bins[i]:5.0f},{bins[i + 1]:5.0f}) deg: {hist[i]:5d} {bar}")
        n_side = int(np.sum((tilts >= 75.0) & (tilts <= 105.0)))
        n_near = int(np.sum(tilts < 30.0))
        print(f"    侧抓域（75~105 deg）：{n_side}   近竖直域（<30 deg）：{n_near}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
