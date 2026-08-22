#!/usr/bin/env python3
"""有偏交变 dither 穿越锁定区验证 —— v3 方案开环预检。

背景（exp1_slant_v4_dither3 分析）：零均值正弦 dither（v2，3 N·m）
使臂在锁点附近振荡（pos_err 恒 0.06~0.08 m、q2/q3/q4 周期摆动），
无法穿越组合锁区——因为闭环净反馈本身被锁，交变均值 0 等于没加
推力。probe_breakaway [2] 证明"突破后组合力矩持续驱动"（0.83 ->
1.57 rad），说明只要关节保持同向速度穿过锁区即可。

本探针：Q_S 处施加 TAU_NET + 有偏交变
    extra[j*] = A * (1 + sin(2 pi f t))        （均值 +A，恒 >= 0）
对比零均值版本
    extra[j*] = A * sin(2 pi f t)
j* 取闭环净反馈主分量（probe_combo_v4：j2，幅值 1.61 最大）。

测试矩阵：j* = j2；(A, f) in {(1.5, 5), (3.0, 5), (3.0, 3)}，
有偏 vs 零均值各 3 s，报每 0.5 s 的 |dq|max 与总位移。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_dither_bias.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_S = np.array([-0.2519, -2.6989, -1.6336, -1.2405, 0.0001, -0.0020])
TAU_NET = np.array([0.02, -1.61, 0.72, 0.27, 0.0, 0.0])
J_STAR = 1          # j2（闭环净反馈主分量）
DT = 1.0 / 500


def run_case(backend, dyn, amp, freq, biased, n_steps=1500):
    backend.reset_to(Q_S)
    g = dyn.gravity_vector(Q_S)
    q0, _ = backend.get_joint_state()
    marks = []
    for k in range(n_steps):
        t = k * DT
        s = np.sin(2.0 * np.pi * freq * t)
        extra = np.zeros(6)
        # 偏置与反馈同向（TAU_NET[J_STAR] 的符号）
        sgn = np.sign(TAU_NET[J_STAR])
        extra[J_STAR] = sgn * amp * ((1.0 + s) if biased else s)
        backend.apply_arm_torques(g + TAU_NET + extra)
        backend.step()
        if (k + 1) % 250 == 0:
            q, qd = backend.get_joint_state()
            marks.append((t + DT, float(np.max(np.abs(q - q0))),
                          float(np.max(np.abs(qd)))))
    return marks


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        for amp, freq in ((1.5, 5.0), (3.0, 5.0), (3.0, 3.0)):
            for biased in (True, False):
                tag = "有偏" if biased else "零均值"
                print(f"=== {tag} A={amp} f={freq} Hz (j{J_STAR + 1}) ===",
                      flush=True)
                marks = run_case(backend, dyn, amp, freq, biased)
                print("  " + "  ".join(
                    f"t={t:.1f}:|dq|={d:.3f},|qd|={v:.2f}"
                    for t, d, v in marks), flush=True)
    finally:
        backend.close()
    print("=== DITHER BIAS DONE ===")


if __name__ == "__main__":
    main()
