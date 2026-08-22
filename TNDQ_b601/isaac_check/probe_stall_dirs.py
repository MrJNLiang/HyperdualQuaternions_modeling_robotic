#!/usr/bin/env python3
"""多关节协同方向探针 —— 验证组合锁内仅协同方向能降组合误差。

根因链（exp1 dither8）：全位姿 IK 解 q*=[-0.252,-2.8455,-1.7184,
-1.2144,0,0] 位于深折叠组合锁区内（q2=-2.845）；稳态卡点
q_stall=[-0.253,-2.641,-1.549,-1.175,0.015,-0.002]（pos_err=0.056）。
单关节探针在锁区内无法降 e_comb（v3.3 位置卡死），因单关节运动
被锁/方向错。本探针在 q_stall 处测试方向集合：
  单关节 j2..j6（±）、双组合（j2j3/j2j4/j3j4/j2j3j4 等），
每方向施 0.1 s 单位幅值力矩（重力补偿基线，逐方向 reset），
测 e_comb 变化。若存在协同方向 delta<0 且单关节全 >=0，
则 v3.4 探针应扩到协同方向。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_stall_dirs.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.dq_algebra import dq_from_r_p, dq_rotation, dq_translation

Q_STALL = np.array([-0.253, -2.6408, -1.5493, -1.1751, 0.0146, -0.002])
PROBE_STEPS = 50      # 0.1 s

DIRS = {
    "j2+": {1: +1.0}, "j2-": {1: -1.0},
    "j3+": {2: +1.0}, "j3-": {2: -1.0},
    "j4+": {3: +1.0}, "j4-": {3: -1.0},
    "j5+": {4: +1.0}, "j5-": {4: -1.0},
    "j6+": {5: +1.0}, "j6-": {5: -1.0},
    "j2+j3+": {1: +1.0, 2: +1.0}, "j2-j3-": {1: -1.0, 2: -1.0},
    "j2+j4+": {1: +1.0, 3: +1.0}, "j2-j4-": {1: -1.0, 3: -1.0},
    "j3+j4+": {2: +1.0, 3: +1.0}, "j3-j4-": {2: -1.0, 3: -1.0},
    "j2+j3+j4+": {1: +1.0, 2: +1.0, 3: +1.0},
    "j2-j3-j4-": {1: -1.0, 2: -1.0, 3: -1.0},
    "IK方向": {1: -0.0, 2: -1.0, 3: -1.0, 4: -0.0},   # q_stall->q* 主方向
}


def comb_err(chain, fk_x, des_x):
    dp = dq_translation(des_x) - dq_translation(fk_x)
    r, r_d = dq_rotation(fk_x), dq_rotation(des_x)
    ori = 2.0 * np.arccos(min(1.0, abs(float(r @ r_d))))
    return float(np.linalg.norm(dp)), ori


def main():
    print(">>> main enter", flush=True)
    from config.b601_dynamics import B601NominalDynamics
    from config.params import B601_DH_TABLE, R_TOOL_QUAT, SETPOINT_POS
    from interfaces.isaac_interface import IsaacB601Backend
    from experiments.run_lib import B601TCPChain, w2dh

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    print(">>> backend ready", flush=True)
    dyn = B601NominalDynamics()
    chain = B601TCPChain(B601_DH_TABLE)
    # 期望位姿（DH 基座系 + TCP 工具系）
    qd = R_TOOL_QUAT / np.linalg.norm(R_TOOL_QUAT)
    x_d = dq_from_r_p(qd, w2dh(SETPOINT_POS))   # DQ 数组（comb_err 口径）
    print(">>> x_d built", flush=True)
    try:
        x0 = chain.fk_tndq(Q_STALL).ch[0]
        print(">>> fk done", flush=True)
        p0, o0 = comb_err(chain, x0, x_d)
        print(f">>> comb_err done", flush=True)
        print(f"q_stall 处 e_comb: pos={p0:.4f} m  ori={o0:.4f} rad",
              flush=True)
        results = []
        for name, comp in DIRS.items():
            backend.reset_to(Q_STALL)
            extra = np.zeros(6)
            for jj, v in comp.items():
                extra[jj] = v
            for _ in range(PROBE_STEPS):
                q_now, _ = backend.get_joint_state()
                backend.apply_arm_torques(
                    dyn.gravity_vector(q_now) + extra)
                backend.step()
            q1, _ = backend.get_joint_state()
            x1 = chain.fk_tndq(q1).ch[0]
            p1, o1 = comb_err(chain, x1, x_d)
            de = (p1 + 0.05 * o1) - (p0 + 0.05 * o0)
            dq = np.max(np.abs(q1 - Q_STALL))
            results.append((name, de, p1, dq))
            print(f"    {name:12s}: Δe_comb={de:+.4f}  "
                  f"pos_after={p1:.4f}  |Δq|max={dq:.4f}", flush=True)
        results.sort(key=lambda r: r[1])
        print("最优方向: " + "  ".join(
            f"{r[0]}({r[1]:+.4f})" for r in results[:3]), flush=True)
    finally:
        backend.close()
    print("=== STALL DIRS DONE ===")


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        import traceback
        traceback.print_exc()
        print("=== STALL DIRS FAILED ===", flush=True)
