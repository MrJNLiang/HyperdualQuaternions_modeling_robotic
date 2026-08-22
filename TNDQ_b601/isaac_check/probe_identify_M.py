#!/usr/bin/env python3
"""物理质量矩阵辨识 —— 2 步短脉冲逐列构建 M_phys，对比 M_hat。

probe_joint_grid 发现：单关节力矩在 Q_F 全部有效（物理健康），但
闭环反馈组合 TAU_FB 锁死；probe_dir_inertia 显示第 1 步沿 dir_b 的
响应大于模型预言（物理惯量小于名义）——M_hat 与物理 M 不一致嫌疑。

方法（超短窗，构型/重力近似不变）：
  teleport q0 -> tau = g_hat(q0) + F0 * e_i -> 2 物理步 ->
  M_phys^{-1}[:, i] ≈ (qd[1] - qd[0]) / (F0 * dt)
  （逐步差分消去第 1 步求解器 artifacts）-> M_phys = inv(...)

输出：两构型（Q_F / Q_INIT）各自 M_phys 特征值 vs M_hat 特征值；
dir_b（冻结点 qdd_ref 方向）瑞利商对比。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_identify_M.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import B601_DH_TABLE, Q_INIT

Q_F = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])
TAU_FB = np.array([0.033, -1.347, 0.625, 0.281, -0.004, 0.003])
F0 = 1.0   # 脉冲幅值 N·m


def identify(backend, dyn, q0):
    dt = backend.physics_dt
    g = dyn.gravity_vector(q0)
    Minv = np.zeros((6, 6))
    for i in range(6):
        backend.reset_to(q0)
        tau = g.copy()
        tau[i] += F0
        qds = []
        for k in range(3):
            backend.apply_arm_torques(tau)
            backend.step()
            _, qd = backend.get_joint_state()
            qds.append(qd.copy())
        # 差分 1->2 与 2->3，取一致性更好的前者（构型漂移最小）
        col_a = (qds[1] - qds[0]) / (F0 * dt)
        col_b = (qds[2] - qds[1]) / (F0 * dt)
        Minv[:, i] = col_a
        print(f"    col j{i + 1}: d1={np.round(col_a, 2).tolist()}  "
              f"d2={np.round(col_b, 2).tolist()}")
    M = np.linalg.inv(Minv)
    return 0.5 * (M + M.T)


def report(tag, M_hat, M_phys, dir_b=None):
    ev_h = np.sort(np.linalg.eigvalsh(M_hat))
    ev_p = np.sort(np.linalg.eigvalsh(M_phys))
    print(f"[{tag}]")
    print(f"    M_hat  特征值: {np.round(ev_h, 6).tolist()}")
    print(f"    M_phys 特征值: {np.round(ev_p, 6).tolist()}")
    print(f"    特征比值 p/h : {np.round(ev_p / np.maximum(ev_h, 1e-12), 2).tolist()}")
    print(f"    max|M_phys-M_hat| = {np.max(np.abs(M_phys - M_hat)):.5f}")
    if dir_b is not None:
        rh = float(dir_b @ M_hat @ dir_b)
        rp = float(dir_b @ M_phys @ dir_b)
        print(f"    dir_b 瑞利商: 名义 {rh:.6f} vs 物理 {rp:.6f}"
              f"（比值 {rp / rh:.2f}）")


def main():
    from config.b601_dynamics import B601NominalDynamics
    from control.control_law import geometric_computed_torque_law
    from control.error_system import full_error_state
    from interfaces.isaac_interface import IsaacB601Backend
    from simdata.trajectory_generator import goto_trajectory
    from config.params import (
        DEFAULT_GAIN_SET, GAIN_SETS, PINV_DAMPING, R_TOOL_QUAT, SETPOINT_POS,
    )
    from experiments.run_lib import B601TCPChain, w2dh

    chain = B601TCPChain(B601_DH_TABLE)
    dyn = B601NominalDynamics()
    traj = goto_trajectory(chain.fkm(Q_INIT), w2dh(SETPOINT_POS),
                           R_TOOL_QUAT, 3.0)
    g = GAIN_SETS[DEFAULT_GAIN_SET]
    fk = chain.fk_outputs(Q_F, np.zeros(6), None, True)
    des = traj.evaluate(7.0)
    err = full_error_state(fk["x_breve"], des["x_breve_d"])
    qdd_raw, _ = geometric_computed_torque_law(
        err, des["xi_d"], des["xi_dot_d"], fk["J"], fk["Jdot_qdot"],
        g["K_d"], g["k_p"], damping=PINV_DAMPING)
    dir_b = qdd_raw / np.linalg.norm(qdd_raw)

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    try:
        print("=== 辨识 @ Q_F ===")
        M_p_F = identify(backend, dyn, Q_F)
        print("=== 辨识 @ Q_INIT ===")
        M_p_I = identify(backend, dyn, np.asarray(Q_INIT, float))
        report("Q_F", dyn.mass_matrix(Q_F), M_p_F, dir_b)
        report("Q_INIT", dyn.mass_matrix(np.asarray(Q_INIT, float)), M_p_I)
        np.savez(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "M_phys.npz"), M_F=M_p_F, M_I=M_p_I)
    finally:
        backend.close()
    print("=== IDENTIFY M DONE ===")


if __name__ == "__main__":
    main()
