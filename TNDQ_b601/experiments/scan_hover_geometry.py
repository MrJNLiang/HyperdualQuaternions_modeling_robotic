#!/usr/bin/env python3
"""hover 几何修复扫描 —— SETPOINT_POS sign bug 的修复量验证。

bug（exp1 稳态 2cm 误差根因）：SETPOINT_POS = GRASP_POS - C*T 落在
grasp 的 -T 侧 = 物体侧，C=0.08 < L_TCP=0.082 时指间中心几乎正落
在立方体中心上方 5mm——夹爪从正上方压在立方体顶面上（CSV 实测
TCP 冻结于 z=0.0472 = 立方体顶 +2.2mm，力矩未饱和，接触力平衡）。

修复方向：SETPOINT_POS = GRASP_POS + C*T（+T 侧，物体外），进插
沿 -T 从 hover 到 grasp（手指接近向 = gripper -x，与开合对准）。

本脚本扫描 C ∈ {0.08, 0.06, 0.05, 0.03}：
    [1] hover IK 可达性 + 限位余量 + sigma_min；
    [2] INIT（hover 上方 INIT_CLEARANCE）IK -> Q_INIT 候选；
    [3] 实验一路径 INIT -> hover 直线 20 点（热启动）；
    [4] 实验二进插 hover -> grasp 沿 TOOL_AXIS 20 点（热启动）；
全部 PASS 的最大 C 即回填值。

用法（纯 python）：
    python experiments/scan_hover_geometry.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    GRASP_POS, INIT_CLEARANCE, LIFT_POS, R_TOOL_QUAT, TOOL_AXIS,
)
from experiments.ik_lib import (
    solve_ik, solve_ik_multi, fk_pose, quat_angle, joint_margin, sigma_min,
    world_to_dh,
)

RQ = R_TOOL_QUAT / np.linalg.norm(R_TOOL_QUAT)


def _metrics(q, p_w):
    """(dp, margin, sigma_min)——report() 的静默版。"""
    p, _ = fk_pose(q)
    return (float(np.linalg.norm(p - world_to_dh(p_w))),
            joint_margin(q), sigma_min(q))


def scan(c):
    """一个 clearance 候选的全链验证，返回 (ok, q_init_best, 备注)。"""
    hover = GRASP_POS + c * TOOL_AXIS
    init = hover + np.array([0.0, 0.0, INIT_CLEARANCE])
    notes = []

    # [1] hover 可达性
    q_hover, m_h, _ = solve_ik_multi(world_to_dh(hover), RQ, n_init=16, seed=7)
    if q_hover is None:
        return False, None, "hover 不可达"
    dp, _, smin = _metrics(q_hover, hover)
    notes.append(f"hover dp {dp:.1e} margin {np.rad2deg(m_h):.1f}deg "
                 f"smin {smin:.3f}")
    if m_h < np.deg2rad(2.0) or smin < 0.05:
        return False, None, "; ".join(notes) + " [余量不足]"

    # [2] Q_INIT（限位余量最大解）
    q_init, m_i, _ = solve_ik_multi(world_to_dh(init), RQ, n_init=16, seed=7)
    if q_init is None:
        return False, None, "; ".join(notes) + " INIT 不可达"
    notes.append(f"INIT margin {np.rad2deg(m_i):.1f}deg")

    # [3] INIT -> hover 路径
    q_prev, worst = q_init, [0.0, np.inf]
    for s in np.linspace(0.0, 1.0, 20)[1:]:
        p_w = init + s * (hover - init)
        q, _, _ = solve_ik(world_to_dh(p_w), RQ, q_prev)
        if q is None:
            return False, q_init, "; ".join(notes) + f" 路径断于 s={s:.2f}"
        dp, _, smin = _metrics(q, p_w)
        worst = [max(worst[0], dp), min(worst[1], smin)]
        q_prev = q
    notes.append(f"下降链 dp {worst[0]:.1e} smin {worst[1]:.3f}")
    if worst[0] > 1e-6 or worst[1] < 0.05:
        return False, q_init, "; ".join(notes) + " [下降链失败]"

    # [4] hover -> grasp 进插（沿 -T）
    q_prev, worst = q_hover, [0.0, np.inf]
    for s in np.linspace(0.0, 1.0, 20)[1:]:
        p_w = hover + s * (GRASP_POS - hover)
        q, _, _ = solve_ik(world_to_dh(p_w), RQ, q_prev)
        if q is None:
            return False, q_init, "; ".join(notes) + f" 进插断于 s={s:.2f}"
        dp, _, smin = _metrics(q, p_w)
        worst = [max(worst[0], dp), min(worst[1], smin)]
        q_prev = q
    notes.append(f"进插链 dp {worst[0]:.1e} smin {worst[1]:.3f}")
    if worst[0] > 1e-6 or worst[1] < 0.05:
        return False, q_init, "; ".join(notes) + " [进插链失败]"

    return True, q_init, "; ".join(notes)


def main():
    best = None
    for c in (0.08, 0.06, 0.05, 0.03):
        ok, q_init, note = scan(c)
        print(f"C={c:.2f}: {'PASS' if ok else 'FAIL'}  {note}")
        if ok and best is None:
            best = (c, q_init)
    if best is None:
        print("结论: FAIL - 无可行 clearance（需调整任务姿态或立方体位置）")
        return 1
    c, q_init = best
    print(f"结论: PASS - 建议回填 SETPOINT_HOVER_CLEARANCE = {c}（+T 侧外退开）")
    print(f"Q_INIT = np.array(["
          + ", ".join(f"{v:+.6f}" for v in q_init) + "])")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
