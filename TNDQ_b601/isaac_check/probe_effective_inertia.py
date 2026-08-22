#!/usr/bin/env python3
"""有效惯量探针 —— Isaac 物理臂 vs RNEA 名义模型的 M(q)/g(q) 对账。

exp1 闭环发散的关键矛盾：冻结点后 qddot_ref 饱和 30 rad/s²、tau≈4.4 N·m
不饱和，但实际关节加速度 ≈ 0（符号混乱）；而冻结构型开环施加同一力矩
臂却能大幅运动（diag_openloop_freeze）。若名义 M̂ 与物理有效 M 严重失配，
计算力矩 tau = M̂ qdd_ref + ... 在物理端产生的实际加速度 = M⁻¹ M̂ qdd_ref
偏离指令，高增益（K_d=24）下即等效正反馈 -> 发散。

本探针（teleport 静止测量，无闭环干扰）：
  [1] 读 articulation 全部 link 质量，与 URDF 名义总质量对账；
  [2] 三个构型（Q_INIT / 冻结终态 / 冻结态 q4 归零）：tau = g_hat(q) 保持，
      读 measured_joint_efforts（= PhysX 实际重力矩）vs 名义 g_hat(q)；
  [3] 冻结终态：tau = g_hat + e_i * 1 N·m（逐关节 j1..j6），测初始加速度
      qdd_meas，与 M_hat(q)⁻¹ e_i 对比 -> 有效惯量方向增益比。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_effective_inertia.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import Q_INIT

Q_FREEZE = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        # ---- [1] link 质量对账 ----
        from config.b601_dynamics import B601_LINK_MASS
        try:
            masses = np.asarray(
                backend.articulation.get_body_masses(), float).reshape(-1)
            names = list(getattr(backend.articulation, "body_names",
                                 [f"body{i}" for i in range(len(masses))]))
            print(f"[1] Isaac link 质量（共 {len(names)} body，"
                  f"总 {masses.sum():.4f} kg）:")
            for n, m in zip(names, masses):
                print(f"      {n:20s} {m:8.4f}")
        except Exception as exc:  # noqa: BLE001
            print(f"[1] get_body_masses 不可用: {exc}")
            masses = None
        print(f"    URDF 名义 arm 总质量 = {float(np.sum(B601_LINK_MASS)):.4f} kg"
              f"（links: {np.round(B601_LINK_MASS, 3).tolist()}）")
        if masses is not None:
            print(f"    Isaac/URDF 总质量比 = "
                  f"{masses.sum() / float(np.sum(B601_LINK_MASS)):.3f}"
                  "（含夹爪/基座 body，比值略大于 1 属正常；显著偏离即失配）")

        # ---- [2] 重力矩对账（静止保持）----
        for tag, q in [("Q_INIT", np.asarray(Q_INIT, float)),
                       ("Q_FREEZE", Q_FREEZE)]:
            backend.reset_to(q)
            g_hat = dyn.gravity_vector(q)
            for _ in range(30):
                backend.apply_arm_torques(g_hat)
                backend.step()
            tau_m = backend.get_measured_joint_efforts()
            q_now, qd_now = backend.get_joint_state()
            print(f"[2] {tag}: g_hat   ={np.round(g_hat, 3).tolist()}")
            print(f"          measured={np.round(tau_m, 3).tolist()}")
            print(f"          diff    ={np.round(tau_m - g_hat, 3).tolist()}"
                  f"  |dq|={np.max(np.abs(qd_now)):.4f}")

        # ---- [3] 单位扰动力矩 -> 初始加速度 vs M_hat^-1 ----
        q = Q_FREEZE
        g_hat = dyn.gravity_vector(q)
        M = dyn.mass_matrix(q)
        Minv = np.linalg.inv(M)
        print("[3] 冻结终态逐关节 +1 N·m 扰动（tau = g_hat + e_i）：")
        print("    j  qdd_meas(0.10s)           M_hat^-1 e_i              ratio")
        for j in range(6):
            backend.reset_to(q)
            tau = g_hat.copy()
            tau[j] += 1.0
            n_warm = 5
            for _ in range(n_warm):
                backend.apply_arm_torques(tau)
                backend.step()
            q0, _ = backend.get_joint_state()
            n_meas = 50          # 0.10 s
            for _ in range(n_meas):
                backend.apply_arm_torques(tau)
                backend.step()
            q1, qd1 = backend.get_joint_state()
            dt = n_meas * backend.physics_dt
            qdd = (qd1 - 0.0) / dt       # 从静止起步
            qdd_fd = 2.0 * (q1 - q0) / (dt * dt)
            pred = Minv[:, j]
            ratio = qdd_fd[j] / pred[j] if abs(pred[j]) > 1e-6 else float("nan")
            print(f"    {j + 1}  {np.round(qdd_fd, 2).tolist()}")
            print(f"       pred={np.round(pred, 2).tolist()}  "
                  f"diag ratio={ratio:.2f}")
    finally:
        backend.close()
    print("=== PROBE EFFECTIVE INERTIA DONE ===")


if __name__ == "__main__":
    main()
