#!/usr/bin/env python3
"""qdd_ref 方向有效惯量测量 —— 1 物理步初始加速度对账。

probe_freeze_impulse 发现：单关节扰动持续加速（物理健康），但闭环净
反馈力矩 tau_fb = M_hat @ qdd_ref（qdd_ref 90% 在 j4，沿 M 最小特征
方向，λ_min=2.6e-4）先动后灭。假设：物理臂沿 qdd_ref 方向的有效惯量
远大于名义值 -> 实际任务加速度 ≪ 指令甚至反向 -> 闭环正反馈发散。

测量方法（超短时窗，构型近似不变，重力残差二阶小）：
  teleport Q_F -> tau = g_hat + alpha * dir（dir 单位化）-> 1 步 ->
  读 qd -> qdd_meas = qd / dt。alpha 小到 5 步内位移 < 1e-4 rad。

方向：
  [B] dir = qdd_ref/|qdd_ref|（闭环反馈方向，冻结点计算）
  [C] dir = e_j2（对照，模型预言可靠）
  [D] dir = e_j4（对照）

判据：方向有效惯量 m_dir = alpha / (dir · qdd_meas)；与名义瑞利商
m_hat = dir^T M_hat dir 对比。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_dir_inertia.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    B601_DH_TABLE, DEFAULT_GAIN_SET, GAIN_SETS, PINV_DAMPING, Q_INIT,
    R_TOOL_QUAT, SETPOINT_POS,
)
from control.control_law import geometric_computed_torque_law
from control.error_system import full_error_state
from simdata.trajectory_generator import goto_trajectory

from experiments.run_lib import B601TCPChain, w2dh

Q_F = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])


def measure(backend, dyn, direction, alpha, n_steps=5):
    """tau = g_hat + alpha*dir 施加 n_steps 步，返回逐步 qd。"""
    backend.reset_to(Q_F)
    g = dyn.gravity_vector(Q_F)
    tau = g + alpha * direction
    qds = []
    for _ in range(n_steps):
        backend.apply_arm_torques(tau)
        backend.step()
        _, qd = backend.get_joint_state()
        qds.append(qd.copy())
    return np.array(qds)


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    chain = B601TCPChain(B601_DH_TABLE)
    dyn = B601NominalDynamics()
    traj = goto_trajectory(chain.fkm(Q_INIT), w2dh(SETPOINT_POS),
                           R_TOOL_QUAT, 3.0)
    g = GAIN_SETS[DEFAULT_GAIN_SET]
    fk = chain.fk_outputs(Q_F, np.zeros(6), None, True)
    des = traj.evaluate(7.0)
    err = full_error_state(fk["x_breve"], des["x_breve_d"])
    qdd_raw, u_task = geometric_computed_torque_law(
        err, des["xi_d"], des["xi_dot_d"], fk["J"], fk["Jdot_qdot"],
        g["K_d"], g["k_p"], damping=PINV_DAMPING)
    dir_b = qdd_raw / np.linalg.norm(qdd_raw)

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    M = dyn.mass_matrix(Q_F)
    try:
        cases = [
            ("B qdd_ref方向", dir_b, 0.30),
            ("C e_j2     ", np.eye(6)[1], 0.30),
            ("D e_j4     ", np.eye(6)[3], 0.10),
        ]
        for tag, d, alpha in cases:
            qds = measure(backend, dyn, d, alpha)
            qdd_1 = qds[0] / backend.physics_dt          # 第 1 步加速度
            qdd_5 = qds[4] / (5 * backend.physics_dt)    # 5 步均值
            m_hat = float(d @ M @ d)                     # 名义瑞利商
            proj = float(d @ qdd_1)
            m_eff = alpha / proj if proj > 1e-9 else float("inf")
            print(f"[{tag}] alpha={alpha}")
            print(f"    qdd(1步) ={np.round(qdd_1, 2).tolist()}")
            print(f"    qdd(5步均)={np.round(qdd_5, 2).tolist()}")
            print(f"    名义 M_hat 预言: qdd = M^-1 (alpha d) = "
                  f"{np.round(np.linalg.solve(M, alpha * d), 2).tolist()}")
            print(f"    方向投影加速度 dir·qdd = {proj:.2f} rad/s2  "
                  f"-> 有效惯量 {m_eff:.5f} kg m2 (名义瑞利商 {m_hat:.5f},"
                  f" 比值 {m_eff / m_hat:.2f})")
    finally:
        backend.close()
    print("=== PROBE DIR INERTIA DONE ===")


if __name__ == "__main__":
    main()
