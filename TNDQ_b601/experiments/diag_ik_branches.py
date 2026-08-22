#!/usr/bin/env python3
"""枚举 v9 抓取位的全部 IK 分支（纯 numpy），寻找腕关节远离零位的构型。

v9 四跑结论：抓取位 q5≈q6≈0 附近深折叠构型重力残差慢漂 ~1 cm/s，
闭合相 yg 漂 4 cm 抓空。若存在 q5 远离 0 的等价姿态分支（腕弯折），
重力残差签名不同，可能消除慢漂。
运行：python3 TNDQ_b601/experiments/diag_ik_branches.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik, make_tool_pose, world_to_dh, joint_margin


def main():
    phi = np.deg2rad(-14.4)
    R, quat = make_tool_pose(np.deg2rad(120), phi)
    xg, zg = R[:, 0], R[:, 2]
    p_grasp = P.CUBE_POS + 0.028 * xg + 0.005 * zg
    rng = np.random.default_rng(7)
    sols = []
    for _ in range(600):
        q0 = rng.uniform(IK.IK_LO, IK.IK_HI, 6)
        q, res, ok = solve_ik(world_to_dh(p_grasp), quat, q0)
        if not ok:
            continue
        m = joint_margin(q)
        if m < np.deg2rad(8):
            continue
        if any(np.max(np.abs(s[0] - q)) < 0.05 for s in sols):
            continue
        sols.append((q, m))
    print(f"{len(sols)} 个分支（margin >= 8 deg）:")
    for q, m in sorted(sols, key=lambda s: -s[1]):
        print(f"  margin={np.rad2deg(m):5.1f}deg  q5={q[4]:+6.3f} "
              f"q6={q[5]:+6.3f}  q={np.round(q, 2)}")


if __name__ == "__main__":
    main()
