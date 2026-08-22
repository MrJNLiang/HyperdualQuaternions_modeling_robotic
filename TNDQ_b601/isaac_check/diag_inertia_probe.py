#!/usr/bin/env python3
"""惯量探针诊断 —— exp1 发散根因定位（t=1.98s j5 超限位）。

现象（exp1_setpoint.csv）：
    q5 自 t=0 起以 ~0.42 rad/s 单调漂移（-1.02 -> -1.49），末端误差仅
    3 mm（沿 Jacobian 小奇异值方向漂移），sigma_min 0.143 -> 0.04，
    t=1.0s 后 J^+ 放大正反馈指数发散。
静态自检（isaac_interface [4]）只验证了重力通道 g(q)；本探针针对
**惯性/速度通道**：computed torque 的 M̂ q̈_ref 项若高估实际惯量，
反馈力矩会在轻关节上产生超预期加速度 -> 奇异方向漂移。

方法（恒力矩阶跃激励，初始瞬时法）：
    对每个关节 i：reset_to(Q_INIT) -> 重力补偿悬挂 0.3 s ->
    施加 tau = g(q) + dt_i*e_i 记录前 25 步 (q, qd) ->
    qdd_i = qd[2:14] 对 t 线性拟合斜率（跳过 solver 首步瞬态）
    -> M_ii_eff = dt_i / qdd_i（忽略耦合的等效对角惯量），
       同时记录其余关节 qdd（M^{-1} 列信息）。
    与 B601NominalDynamics.mass_matrix(Q_INIT) 对角对比。
顺带检验 qd 传感符号（正 dt_i -> 正 qdd_i）。

运行方式：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/diag_inertia_probe.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import Q_INIT
from config.b601_dynamics import B601NominalDynamics

HOLD_STEPS = 150       # 悬挂（纯重力补偿）时长 [步] = 0.3 s
PROBE_STEPS = 25       # 激励记录窗口 [步] = 50 ms
FIT_SKIP = 2           # 拟合跳过 solver 首步瞬态
FIT_LEN = 12           # 拟合窗口长度 [步] = 24 ms（位移 << 0.01 rad，M 视为常值）

DTAU = np.array([0.3, 0.3, 0.3, 0.05, 0.05, 0.05])   # 各关节激励力矩 [N*m]


def main():
    from interfaces.isaac_interface import IsaacB601Backend

    dyn = B601NominalDynamics()
    M_model = dyn.mass_matrix(Q_INIT)
    Minv_model = np.linalg.inv(M_model)

    backend = IsaacB601Backend(headless=True)
    backend.setup()

    lines = []
    lines.append("[惯量探针] M̂(Q_INIT) 对角   = " +
                 np.array2string(np.diag(M_model), precision=5))
    lines.append("[惯量探针] M̂⁻¹ 对角        = " +
                 np.array2string(np.diag(Minv_model), precision=3))

    Minv_cols = np.zeros((6, 6))
    for i in range(6):
        backend.reset_to(Q_INIT)
        # 悬挂：纯重力补偿，等瞬态衰减
        for _ in range(HOLD_STEPS):
            q, qd = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(q))
            backend.step()

        q0, qd0 = backend.get_joint_state()
        # 阶跃激励 tau = g(q) + dtau_i * e_i（重力每步重算，保持准静态）
        ts, qs, qds = [], [], []
        for k in range(PROBE_STEPS):
            q, qd = backend.get_joint_state()
            tau = dyn.gravity_vector(q)
            tau[i] += DTAU[i]
            backend.apply_arm_torques(tau)
            backend.step()
            ts.append((k + 1) * 0.002)
            qs.append(q.copy())
            qds.append(qd.copy())

        ts = np.array(ts)
        qds = np.array(qds)
        qs = np.array(qs)
        # qdd 拟合（各关节分别拟合 qd(t) 斜率）
        sl = slice(FIT_SKIP, FIT_SKIP + FIT_LEN)
        A = np.vstack([ts[sl], np.ones(sl.stop - sl.start)]).T
        qdd = np.linalg.lstsq(A, qds[sl], rcond=None)[0][0]
        Minv_cols[:, i] = qdd / DTAU[i]
        lines.append(
            f"[关节{i + 1}] dt={DTAU[i]:+.2f} N·m  qdd={qdd[i]:+9.3f} rad/s²  "
            f"M_ii 实测={DTAU[i] / qdd[i] if qdd[i] != 0 else float('inf'):9.5f}  "
            f"M̂_ii={M_model[i, i]:9.5f}  "
            f"比值={M_model[i, i] * qdd[i] / DTAU[i]:6.2f}  "
            f"漂移q0={q0[i]:+.3f}→{qs[-1][i]:+.3f}")

    # 符号检验 + M 实测（6 列齐全 -> Minv -> M）
    M_meas = np.linalg.inv(Minv_cols) if np.linalg.det(Minv_cols) != 0 else None
    lines.append("")
    lines.append("[惯量探针] M̂⁻¹ 实测（列=激励关节）=")
    for r in range(6):
        lines.append("    " + np.array2string(Minv_cols[r], precision=3) +
                     "   | 模型 " + np.array2string(Minv_model[r], precision=3))
    if M_meas is not None:
        lines.append("[惯量探针] M 实测对角 = " +
                     np.array2string(np.diag(M_meas), precision=5))
        ratio = np.diag(M_model) / np.diag(M_meas)
        lines.append("[惯量探针] M̂/M 实测比值（>1 = 模型高估惯量 -> "
                     "computed torque 反馈过强）= " +
                     np.array2string(ratio, precision=2))

    for ln in lines:
        print(ln, flush=True)
    backend.close()


if __name__ == "__main__":
    main()
