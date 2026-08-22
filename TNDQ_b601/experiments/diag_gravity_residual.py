#!/usr/bin/env python3
"""纯重力补偿自由漂移探针：量化抓取位名义重力模型残差。

惯量对账已证伪失配（m/com/主轴全 1.000），本探针直接测量：
传送到抓取位 q_g，仅施加 tau = g_hat(q)（名义重力补偿，无 PD 反馈），
若名义模型与 PhysX 真值一致，臂应静止；实测 q(t) 偏离 -> 残差
r = M(q) qdd_meas 即为真实重力力矩误差（含 drive 残留阻尼等）。

对照点：再加一组 tau = g_hat(q) + k_p 反馈（模拟 exp1），观察是否
出现"饱和型慢漂"，以区分"模型残差"与"反馈-残差平衡"。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_gravity_residual.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.params import GRIPPER_OPENING
from config.b601_dynamics import B601NominalDynamics, clip_torque
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik, make_tool_pose, world_to_dh
import config.params as P


def grasp_q():
    phi = np.deg2rad(-14.4)
    R, quat = make_tool_pose(np.deg2rad(120), phi)
    xg, zg = R[:, 0], R[:, 2]
    p_grasp = P.CUBE_POS + 0.028 * xg + 0.005 * zg
    q0 = np.array([-0.3, -1.6, -0.6, -0.6, 0.0, 0.0])
    q, res, ok = solve_ik(world_to_dh(p_grasp), quat, q0)
    assert ok, f"IK 失败: {res}"
    return q


def run_hold(backend, dyn, q_g, mode, duration=3.0, dt_ctrl=0.002):
    """mode='gravity': tau=g_hat(q); mode='feedback': tau=g_hat+kp误差."""
    from experiments.run_lib import B601TCPChain
    chain = B601TCPChain(P.B601_DH_TABLE)
    backend.reset_to(q_g, gripper_width=GRIPPER_OPENING)
    n_sub = int(dt_ctrl / backend.physics_dt)
    n_ctrl = int(duration / dt_ctrl)
    x_d = chain.fkm(q_g)
    hist = []
    for k in range(n_ctrl):
        q, qd = backend.get_joint_state()
        q, qd = q[:6], qd[:6]
        tau = dyn.gravity_vector(q)
        if mode == "feedback":
            e = chain.jac(q) if False else None
            # 任务空间弱反馈（模拟 exp1 small_arm 增益量级）
            fk_x = chain.fk_tndq(q, qd)
            err = chain.combined_error(fk_x, x_d) if hasattr(
                chain, "combined_error") else None
            if err is None:
                # 退化为关节空间弱 PD
                tau = tau + 40.0 * (q_g - q) - 4.0 * qd
            else:
                tau = tau  # 占位
        tau, _ = clip_torque(tau)
        backend.apply_arm_torques(tau)
        for _ in range(n_sub):
            backend.step()
        if k % 50 == 0:
            p = chain.fk_tndq(q).translation if hasattr(
                chain.fk_tndq(q), "translation") else None
            hist.append((k * dt_ctrl, q.copy(), qd.copy()))
    hist = np.array([(t, *(qq - q_g), *np.zeros(0)) for t, qq, _ in hist])
    return hist


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    q_g = grasp_q()
    print(f"抓取位 q_g = {np.round(q_g, 4)}")
    dyn = B601NominalDynamics()
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    try:
        for mode in ("gravity",):
            hist = run_hold(backend, dyn, q_g, mode, duration=3.0)
            t = hist[:, 0]
            dq = hist[:, 1:7]
            print(f"\n[{mode}] 纯重力补偿自由漂移（相对 q_g）：")
            for i in range(6):
                d = dq[:, i]
                # 稳态速度估计（后 1s 线性拟合）
                m = t >= t[-1] - 1.0
                v = np.polyfit(t[m], d[m], 1)[0] if m.sum() > 2 else 0.0
                print(f"  q{i+1}: 终点偏差 {d[-1]*1e3:+7.1f} mrad   "
                      f"末段速度 {v:+.4f} rad/s")
            # TCP 级位移
            from experiments.run_lib import B601TCPChain
            chain = B601TCPChain(P.B601_DH_TABLE)
            p0 = chain.fkm(q_g)[:3, 3]
            p1 = chain.fkm(q_g + dq[-1])[:3, 3]
            print(f"  TCP 位移: {np.round((p1 - p0) * 1e3, 1)} mm  "
                  f"|{np.linalg.norm(p1 - p0) * 1e3:.1f}| mm")
            # 残差力矩估计：M @ qdd（中心差分）
            acc = np.gradient(np.gradient(dq, t, axis=0), t, axis=0)
            r = dyn.mass_matrix(q_g) @ acc[len(acc) // 4]
            print(f"  残差力矩估计 r = M qdd ≈ {np.round(r, 3)} N·m")
    finally:
        backend.close()


if __name__ == "__main__":
    main()
