#!/usr/bin/env python3
"""B601TCPChain 工具约定统一验证（exp1 发散根因修复的冒烟测试）。

修复内容（见 run_lib.B601TCPChain docstring）：主循环 FK 层与轨迹起点
补上 DH 链 -> gripper_link 的尾变换 E = Rz(B601_TOOL_ANGLE)。

根因回顾：Q_INIT 为 ik_lib（含 E 的 TCP 约定）IK 解，而主循环曾用裸
TNDQSerialChain（无 E）——两者姿态恒差 90 deg，goto 轨迹含幽灵旋转
（|xi_d| 峰值 0.75，旋转分量占 0.71），跟踪需大幅腕部运动而发散。

本脚本纯 python 验证四项性质：
  [1] TCP 姿态对账：B601TCPChain.fkm(Q_INIT) 姿态 = R_TOOL_QUAT，
      且与 ik_lib.fk_pose 逐位一致（设计期与控制期约定统一）；
  [2] twist 不变性：右乘常值酉元不改 xi / J / Jdot_qdot / (3.8) 残差
      （与裸 TNDQSerialChain 逐项差 < 1e-12）；
  [3] goto 轨迹净化：从 TCP 起点出发的 goto 无旋转分量，
      平移峰值速度 = 余弦 ramp 理论值 (pi/2)*0.08/T_MOVE；
  [4] 误差层零初始化：t=0 期望 = 实测（同 TCP 约定），e_z = 0
      （修复前裸链约定下初始姿态误差恒 90 deg）。

运行：
    /home/liang/miniconda3/envs/dq_hinf/bin/python TNDQ_b601/experiments/verify_tcp_chain.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config.params import (
    B601_DH_TABLE, Q_INIT, R_TOOL_QUAT, SETPOINT_POS,
)
from core.kinematics import TNDQSerialChain
from core.dq_algebra import dq_rotation, dq_translation
from control.error_system import full_error_state
from run_lib import B601TCPChain, w2dh
from simdata.trajectory_generator import goto_trajectory

T_MOVE = 3.0


def main():
    ok = True
    bare = TNDQSerialChain(B601_DH_TABLE)
    tcp = B601TCPChain(B601_DH_TABLE)

    # ---- [1] TCP 姿态对账 ----------------------------------------------
    # （R_TOOL_QUAT 先归一化：params 存的 6 位小数模长偏 3e-7，arccos 在
    #   1 附近的平方根放大余会把拉成 0.09 deg 伪影差角）
    x_tcp = tcp.fkm(Q_INIT)
    r_tcp = dq_rotation(x_tcp)
    r_ref = R_TOOL_QUAT / np.linalg.norm(R_TOOL_QUAT)
    dth = 2.0 * np.arccos(min(1.0, abs(float(r_tcp @ r_ref))))
    from experiments.ik_lib import fk_pose
    p_ik, r_ik = fk_pose(Q_INIT)
    dp_ik = float(np.linalg.norm(
        np.asarray(dq_translation(x_tcp)) - p_ik))
    dr_ik = 2.0 * np.arccos(min(1.0, abs(float(r_tcp @ r_ik))))
    print(f"[1] TCP 姿态对账: vs R_TOOL_QUAT 差角 = {np.rad2deg(dth):.2e} deg"
          f"  vs ik_lib.fk_pose: dp = {dp_ik:.2e} m  差角 = {np.rad2deg(dr_ik):.2e} deg")
    # dth 阈值 1e-4 rad：Q_INIT 回填 6 位小数的舍入残差 ~1.2e-6 rad，
    # 本质判据是与 ik_lib.fk_pose（设计工具）逐位一致
    if dth > 1e-4 or dp_ik > 1e-12 or dr_ik > 1e-12:
        ok = False

    # ---- [2] twist / 雅可比 / 残差不变性 ---------------------------------
    rng = np.random.default_rng(11)
    qd = rng.uniform(-0.6, 0.6, 6)
    fk_b = bare.fk_outputs(Q_INIT, qd, None, True)
    fk_t = tcp.fk_outputs(Q_INIT, qd, None, True)
    diffs = {key: float(np.max(np.abs(
        np.asarray(fk_b[key]) - np.asarray(fk_t[key]))))
        for key in ("xi", "xi_dot", "Jdot_qdot", "J", "c0", "c1", "c2")}
    worst = max(diffs.values())
    print("[2] twist 不变性（裸链 vs TCP 链逐项 max|diff|）：")
    for key, val in diffs.items():
        print(f"      {key:<10s} = {val:.2e}")
    if worst > 1e-12:
        ok = False

    # ---- [3] goto 轨迹净化 -----------------------------------------------
    # 残余旋转仪 1.2e-6 rad（Q_INIT 回填 6 位小数的舍入），
    # omega_d 峰值 = angle * s'_peak，平移峰值 = 余弦 ramp 理论值
    traj = goto_trajectory(x_tcp, w2dh(SETPOINT_POS), R_TOOL_QUAT, T_MOVE)
    om_max, v_max = 0.0, 0.0
    for t in np.linspace(0.0, T_MOVE, 61):
        xi_d = traj.evaluate(float(t))["xi_d"]
        om_max = max(om_max, float(np.linalg.norm(xi_d[:3])))
        v_max = max(v_max, float(np.linalg.norm(xi_d[3:])))
    v_theory = (np.pi / 2.0) * 0.08 / T_MOVE    # 余弦 ramp 峰值速度
    print(f"[3] goto 净化: max|omega_d| = {om_max:.2e} rad/s（应 ~1e-6）"
          f"  max|v_d| = {v_max:.5f} m/s（理论 {v_theory:.5f}）")
    if om_max > 1e-5 or abs(v_max - v_theory) > 1e-3:
        ok = False

    # ---- [4] 误差层零初始化（静止状态：qd = 0）----------------------------
    fk0 = tcp.fk_outputs(Q_INIT, np.zeros(6), None, True)
    des0 = traj.evaluate(0.0)
    err0 = full_error_state(fk0["x_breve"], des0["x_breve_d"])
    e_z0 = float(np.linalg.norm(err0["e_z"]))
    e_xi0 = float(np.linalg.norm(err0["e_xi"]))
    print(f"[4] 误差层零初始化: |e_z(t=0)| = {e_z0:.2e}（应 ~0）"
          f"  |e_xi(t=0)| = {e_xi0:.2e}")
    if e_z0 > 1e-9 or e_xi0 > 1e-9:
        ok = False

    print("=== VERIFY TCP CHAIN {} ===".format("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
