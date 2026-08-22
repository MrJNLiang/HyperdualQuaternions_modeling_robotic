#!/usr/bin/env python3
"""最小回路变体矩阵 —— 定位控制栈发散的破坏组件。

diag_loop_sim [A]（完美模型）也发散 => 问题在控制栈自身（非 Isaac）。
且控制器与被控对象共用同一 FK/J，DH 表错误被排除。

变体矩阵（全部纯调节：期望恒 = fkm(Q_INIT)，初始即目标）：
    V1  base 增益，无治理/限幅/阻尼   —— 最裸控制律 (5.2)
    V2  V1 + tuned 增益               —— 增益敏感性
    V3  V1 + goto 轨迹                —— 运动引入
    V4  V1 + 治理器                   —— 治理器相互作用
    V5  V1 + 关节阻尼                 —— 阻尼作用
    V6  V1 + qdd 限幅                 —— 限幅作用
判据：漂移/收敛（末段 pos_err、|qd|）。

运行：
    /home/liang/miniconda3/envs/dq_hinf/bin/python TNDQ_b601/experiments/diag_min_loop.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    B601_DH_TABLE, Q_INIT, SETPOINT_POS, R_TOOL_QUAT, DT, GAIN_SETS,
    PINV_DAMPING, QDDOT_MAX, JOINT_DAMPING,
)
from config.b601_dynamics import B601NominalDynamics, clip_torque
from control.control_law import geometric_computed_torque_law
from control.error_system import full_error_state
from core.kinematics import TNDQSerialChain
from core.dq_algebra import dq_translation, dq_rotation

from run_lib import joint_safety_governor, w2dh, B601TCPChain
from simdata.trajectory_generator import (
    goto_trajectory, SetpointTrajectoryTNDQ)


def run_variant(name, duration=8.0, gain="base", traj_kind="setpoint",
                governor=False, damping_inject=False, qddot_clip=False,
                tau_clip=False, verbose_every=2.0):
    chain = B601TCPChain(B601_DH_TABLE)
    dyn = B601NominalDynamics()
    gains = GAIN_SETS[gain]
    K_d, k_p = gains["K_d"], gains["k_p"]

    x_init = chain.fkm(Q_INIT)
    if traj_kind == "setpoint":
        # 纯调节：期望恒为初始位姿（p 用 FK 输出，自洽）
        from core.dq_algebra import dq_translation as _dt
        traj = SetpointTrajectoryTNDQ(_dt(x_init), dq_rotation(x_init))
    else:
        traj = goto_trajectory(x_init, w2dh(SETPOINT_POS), R_TOOL_QUAT, 3.0)

    q, qd = Q_INIT.copy(), np.zeros(6)
    n_steps = int(round(duration / DT))
    hist = []
    for k in range(n_steps):
        t = k * DT
        fk = chain.fk_outputs(q, qd, None, True)
        des = traj.evaluate(t)
        err = full_error_state(fk["x_breve"], des["x_breve_d"])
        sig_min = float(np.linalg.svd(fk["J"], compute_uv=False)[-1])
        qddot_ref, _ = geometric_computed_torque_law(
            err, des["xi_d"], des["xi_dot_d"],
            fk["J"], fk["Jdot_qdot"], K_d, k_p, damping=PINV_DAMPING)
        if qddot_clip:
            qn = float(np.linalg.norm(qddot_ref))
            if qn > QDDOT_MAX:
                qddot_ref = qddot_ref * (QDDOT_MAX / qn)
        if governor:
            qddot_ref, _ = joint_safety_governor(q, qd, qddot_ref)
        tau = dyn.computed_torque(q, qd, qddot_ref)
        if damping_inject:
            tau = tau - JOINT_DAMPING * qd
        if tau_clip:
            tau, _ = clip_torque(tau)
        # 半隐式欧拉完美被控对象
        qdd = dyn.forward_dynamics(q, qd, tau)
        qd = qd + qdd * DT
        q = q + qd * DT

        if k % int(verbose_every / DT) == 0:
            p = dq_translation(fk["x"])
            pd = dq_translation(des["x_d"])
            r, rd = dq_rotation(fk["x"]), dq_rotation(des["x_d"])
            pe = float(np.linalg.norm(p - pd))
            oe = float(2.0 * np.arccos(min(1.0, abs(float(r @ rd)))))
            hist.append((t, pe, oe))
            print(f"    t={t:4.1f}  pos={pe:.5f}  ori={oe:.5f}  "
                  f"|qd|={np.abs(qd).max():.4f}  sig={sig_min:.3f}  "
                  f"|qdd|={np.linalg.norm(qddot_ref):7.2f}", flush=True)
    print(f"[{name}] 结束 q={np.round(q, 3)} |qd|={np.abs(qd).max():.4f}\n",
          flush=True)


if __name__ == "__main__":
    print("== V1 base 增益纯调节，全裸 ==")
    run_variant("V1", gain="base")
    print("== V2 tuned 增益纯调节 ==")
    run_variant("V2", gain="tuned")
    print("== V3 base + goto 轨迹 ==")
    run_variant("V3", gain="base", traj_kind="goto")
    print("== V4 base + 治理器 ==")
    run_variant("V4", gain="base", governor=True)
    print("== V5 base + 关节阻尼 ==")
    run_variant("V5", gain="base", damping_inject=True)
    print("== V6 base + qdd 限幅 ==")
    run_variant("V6", gain="base", qddot_clip=True)
