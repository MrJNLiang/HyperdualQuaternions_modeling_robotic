#!/usr/bin/env python3
"""离线 IK 可行性扫描：不同 tilt 下在方块附近的可达姿态与限位余量。

用户要求夹爪更明显朝下。B601 j2/j3 负限位约束下竖直下探不可达，
本扫描定量给出最陡可行 tilt（工具轴与竖直夹角）及关节限位余量，
供 exp1 抓取姿态选型（无需 Isaac，纯运动学）。
运行：python3 TNDQ_b601/experiments/diag_tilt_scan.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik_multi, make_tool_pose, world_to_dh, fk_pose
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()

LO, HI = P.JOINT_LOWER, P.JOINT_UPPER


def margin(q):
    return float(min(np.minimum(q - LO, HI - q)))


def main():
    phi = np.deg2rad(-14.4)
    cube = P.CUBE_POS
    print("方块 CUBE_POS =", np.round(cube, 4))
    print(f"{'tilt°':>6} {'ok':>3} {'pos_err_mm':>10} {'ori_err_deg':>10} "
          f"{'margin_deg':>10}  工具轴z(向下=-1)")
    for tilt_deg in [100, 110, 120, 130, 140, 150, 155, 160, 165, 170]:
        tilt = np.deg2rad(tilt_deg)
        R, quat = make_tool_pose(tilt, phi)
        xg = R[:, 0]
        # 抓取 TCP 目标 = 方块中心沿接近轴后退一点（指垫中心对方块中心）
        p_tcp = cube + 0.010 * xg
        q, m, res = None, None, None
        try:
            q, m, res = solve_ik_multi(world_to_dh(p_tcp), quat,
                                       n_init=24, seed=3)
        except Exception as e:
            q = None
        ok = q is not None
        if ok:
            pe, re_ = fk_pose(q)
            perr = np.linalg.norm(pe - world_to_dh(p_tcp)) * 1e3
            from experiments.ik_lib import quat_mul, quat_conj
            dq = quat_mul(re_, quat_conj(quat))
            oerr = 2 * np.arccos(np.clip(abs(dq[0]), 0, 1))
            print(f"{tilt_deg:6.0f} {'Y':>3} "
                  f"{perr:10.2f} {np.rad2deg(oerr):10.2f} "
                  f"{np.rad2deg(m):10.1f}  {xg[2]:+.3f}")
        else:
            print(f"{tilt_deg:6.0f}   n  {'IK 失败/不收敛':>30}")


if __name__ == "__main__":
    main()
