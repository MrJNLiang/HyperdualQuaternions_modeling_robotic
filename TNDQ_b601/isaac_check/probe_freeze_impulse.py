#!/usr/bin/env python3
"""冻结构型短时窗力矩响应探针 —— 逐物理步解析速度响应曲线。

diag_freeze_closedloop 显示闭环 2 s 净位移 ≈ 0（但 |qd| 抖动到 0.25），
与模型预言（净反馈力矩 1.5 N·m -> j4 +25 rad/s²）矛盾。2 s 聚合会掩盖
短时响应（可能先动后被闭环抵消/求解器锁死）。本探针用**开环常值力矩**
（与闭环在冻结点施加的净力矩逐项相同）逐物理步记录 qd：

  [A] tau = g_hat(Q_F)                    （纯重力补偿基线，应几乎不动）
  [B] tau = g_hat(Q_F) + tau_fb           （闭环净反馈力矩原样开环施加）
  [C] tau = g_hat(Q_F) + 0.5*e_j2         （单关节标定，对照模型 M^-1）
  [D] tau = g_hat(Q_F) + 0.5*e_j4

输出每步 max|qd| 与关键关节速度，前 50 步（0.1 s）逐步打印 -> 若前几步
有加速度随后衰减归零，即"先动后锁"（求解器/接触渐进锁死）；若从第一步
就无响应，即构型级力矩传递失效。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_freeze_impulse.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_F = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])
TAU_FB = np.array([0.033, -1.347, 0.625, 0.281, -0.004, 0.003])


def run_case(backend, dyn, tag, tau_extra, n_steps=50):
    backend.reset_to(Q_F)
    g = dyn.gravity_vector(Q_F)
    tau = g + tau_extra
    qd_hist = []
    for k in range(n_steps):
        backend.apply_arm_torques(tau)
        backend.step()
        _, qd = backend.get_joint_state()
        qd_hist.append(qd.copy())
    qd_hist = np.array(qd_hist)
    print(f"[{tag}] extra={np.round(tau_extra, 3).tolist()}")
    for k in [0, 1, 2, 4, 9, 19, 29, 49]:
        print(f"    step {k + 1:2d} (t={(k + 1) * 0.002:.3f}s): "
              f"qd={np.round(qd_hist[k], 4).tolist()}")
    print(f"    末步 max|qd|={np.max(np.abs(qd_hist[-1])):.4f}  "
          f"全程 max|qd|={np.max(np.abs(qd_hist)):.4f}")
    return qd_hist


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    M = dyn.mass_matrix(Q_F)
    Minv = np.linalg.inv(M)
    try:
        run_case(backend, dyn, "A 纯重力补偿", np.zeros(6))
        run_case(backend, dyn, "B +闭环净反馈", TAU_FB)
        run_case(backend, dyn, "C +0.5 e_j2", 0.5 * np.eye(6)[1])
        run_case(backend, dyn, "D +0.5 e_j4", 0.5 * np.eye(6)[3])
        print("\n模型预言（M_hat^-1 @ extra）：")
        print(f"  B: {np.round(Minv @ TAU_FB, 2).tolist()}")
        print(f"  C: {np.round(Minv @ (0.5 * np.eye(6)[1]), 2).tolist()}")
        print(f"  D: {np.round(Minv @ (0.5 * np.eye(6)[3]), 2).tolist()}")
    finally:
        backend.close()
    print("=== PROBE FREEZE IMPULSE DONE ===")


if __name__ == "__main__":
    main()
