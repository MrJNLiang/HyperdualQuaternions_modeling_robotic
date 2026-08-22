#!/usr/bin/env python3
"""冻结态闭环重放 —— 从 Q_FREEZE 起跑真实闭环，高频记录全通道。

已确认的事实链：
  - 纯仿真闭环（含惯量 +7% / 重力残差）全部收敛（diag_loop_sim）；
  - 重力矩精确对账（measured == g_hat，逐位）；
  - 有效惯量比 0.8~1.3（j1-j4），模型失配不足以解释发散；
  - 冻结态下 CSV 记录 tau≈4.4 N·m 开环施加臂能动（diag_openloop_freeze），
    但闭环中臂冻结且 qddot_ref 饱和 30 —— 闭环特有的差异只剩：
    (a) tau_cmd 与实测 effort 是否一致（drive/maxForce 改写？）；
    (b) qd 测量是否干净（-K_d e_xi 对噪声极敏感，K_d=24）；
    (c) 臂是否真的对闭环力矩无响应。

本诊断：teleport Q_FREEZE -> 完整控制律（与 run_lib 逐行一致）->
每步记录 qd_meas / tau_cmd / measured_effort，2 s：
  [1] tau_cmd vs measured_effort 逐步差；
  [2] qd 测量统计（均值/标准差/峰值）；
  [3] q 漂移（闭环下臂动不动）。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/diag_freeze_closedloop.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    B601_DH_TABLE, DEFAULT_GAIN_SET, DT, GAIN_SETS, PINV_DAMPING, Q_INIT,
    QDDOT_MAX, R_TOOL_QUAT, SETPOINT_POS, SETPOINT_HOLD_TIME,
    SINGULARITY_DAMPING, SINGULARITY_TOL, JOINT_DAMPING,
)
from config.b601_dynamics import B601NominalDynamics, clip_torque
from control.control_law import geometric_computed_torque_law
from control.error_system import full_error_state
from simdata.trajectory_generator import goto_trajectory

from experiments.run_lib import B601TCPChain, joint_safety_governor, w2dh

Q_FREEZE = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])
T_MOVE = 3.0


def main():
    from interfaces.isaac_interface import IsaacB601Backend

    chain = B601TCPChain(B601_DH_TABLE)
    dyn = B601NominalDynamics()
    gains = GAIN_SETS[DEFAULT_GAIN_SET]
    K_d, k_p = gains["K_d"], gains["k_p"]
    traj = goto_trajectory(chain.fkm(Q_INIT), w2dh(SETPOINT_POS),
                           R_TOOL_QUAT, T_MOVE)
    duration = T_MOVE + SETPOINT_HOLD_TIME

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    try:
        backend.reset_to(Q_FREEZE)
        t_start = 7.0          # 冻结发生在 t>3s 的定点段；直接从定点段起跑
        n_steps = int(round(2.0 / DT))
        qd_all, diff_all = [], []
        q0, _ = backend.get_joint_state()
        for k in range(n_steps):
            t = t_start + k * DT
            q, qd = backend.get_joint_state()
            fk = chain.fk_outputs(q, qd, q_ddot=None, with_jacobian=True)
            des = traj.evaluate(t)
            err = full_error_state(fk["x_breve"], des["x_breve_d"])
            sig_min = float(np.linalg.svd(fk["J"], compute_uv=False)[-1])
            damping = (PINV_DAMPING if sig_min >= SINGULARITY_TOL
                       else SINGULARITY_DAMPING)
            qddot_ref, _ = geometric_computed_torque_law(
                err, des["xi_d"], des["xi_dot_d"], fk["J"], fk["Jdot_qdot"],
                K_d, k_p, damping=damping)
            qn = float(np.linalg.norm(qddot_ref))
            if qn > QDDOT_MAX:
                qddot_ref = qddot_ref * (QDDOT_MAX / qn)
            qddot_ref, _ = joint_safety_governor(q, qd, qddot_ref)
            tau = dyn.computed_torque(q, qd, qddot_ref) - JOINT_DAMPING * qd
            tau, _ = clip_torque(tau)
            backend.apply_arm_torques(tau)
            backend.step()

            qd_all.append(qd.copy())
            if k % 100 == 0:
                m = backend.get_measured_joint_efforts()
                diff_all.append(m - tau)
                print(f"t={t:6.3f} |qd|max={np.max(np.abs(qd)):.4f} "
                      f"|qdd_ref|={np.linalg.norm(qddot_ref):.1f} "
                      f"tau={np.round(tau, 2).tolist()}", flush=True)
                print(f"         measured={np.round(m, 2).tolist()}",
                      flush=True)
        qd_all = np.array(qd_all)
        diff_all = np.array(diff_all)
        q1, _ = backend.get_joint_state()
        print(f"\n[结果] qd 测量: mean={np.round(qd_all.mean(0), 4).tolist()}")
        print(f"            std ={np.round(qd_all.std(0), 4).tolist()}")
        print(f"            max ={np.round(np.abs(qd_all).max(0), 4).tolist()}")
        print(f"[结果] measured - cmd: max|diff|="
              f"{np.abs(diff_all).max():.4f} N·m")
        print(f"[结果] 闭环 2s 后 dq = {np.round(q1 - q0, 4).tolist()} "
              f"max={np.max(np.abs(q1 - q0)):.4f} rad")
    finally:
        backend.close()
    print("=== DIAG FREEZE CLOSEDLOOP DONE ===")


if __name__ == "__main__":
    main()
