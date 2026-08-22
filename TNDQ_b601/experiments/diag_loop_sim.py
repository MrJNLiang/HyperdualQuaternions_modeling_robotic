#!/usr/bin/env python3
"""控制栈闭环纯仿真二分测试 —— Isaac 发散根因定位（第二轮）。

exp1 在 Isaac 中两次发散（阻尼注入后仍发散，j4 漂移速度 0.8 rad/s
远超悬挂重力残差预测的 0.09 rad/s -> 闭环存在正反馈嫌疑）。

二分法：用 B601NominalDynamics.forward_dynamics 做被控对象（与控制器
名义模型完全一致的完美世界），跑与 run_lib.run_tndq_experiment 完全
相同的控制栈（误差层/控制律/限幅/治理/阻尼注入/clip）。

    [A] 完美模型（mismatch=1.0）  稳定 -> 问题在 Isaac 物理失配
                                    发散 -> 控制栈移植 bug / B601 参数不适配
    [B] 失配模型（惯量+7%、重力微扰）稳定 -> 失配不是主因
                                      发散 -> 复现 Isaac 行为，定量对齐

运行（纯 numpy，无需 Isaac）：
    /home/liang/miniconda3/envs/dq_hinf/bin/python TNDQ_b601/experiments/diag_loop_sim.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    B601_DH_TABLE, Q_INIT, DT, GAIN_SETS,
    DEFAULT_GAIN_SET, PINV_DAMPING, QDDOT_MAX, JOINT_DAMPING,
    SINGULARITY_TOL, SINGULARITY_DAMPING, GRIPPER_OPENING,
)
from config.b601_dynamics import B601NominalDynamics, clip_torque
from control.control_law import geometric_computed_torque_law
from control.error_system import full_error_state
from core.kinematics import TNDQSerialChain
from core.dq_algebra import dq_translation, dq_rotation

from run_lib import (joint_safety_governor, w2dh, B601TCPChain,
                     build_setpoint_goto_trajectory)


class FakeBackend:
    """forward_dynamics 半隐式欧拉被控对象；plant_dyn 可与控制器模型失配。"""

    def __init__(self, plant_dyn, tau_bias=None):
        self.plant = plant_dyn
        self.tau_bias = tau_bias if tau_bias is not None else np.zeros(6)
        self.q = Q_INIT.copy()
        self.qd = np.zeros(6)
        self.tau = np.zeros(6)

    def reset_to(self, q_init, gripper_width=None):
        self.q = np.asarray(q_init, dtype=float).copy()
        self.qd = np.zeros(6)

    def get_joint_state(self):
        return self.q.copy(), self.qd.copy()

    def apply_arm_torques(self, tau):
        self.tau = np.asarray(tau, dtype=float).copy()

    def set_gripper(self, width_m):
        pass

    def get_gripper_width(self):
        return GRIPPER_OPENING

    def get_cube_pose(self):
        return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])

    def step(self, render=None):
        qdd = self.plant.forward_dynamics(self.q, self.qd,
                                          self.tau + self.tau_bias)
        self.qd = self.qd + qdd * DT
        self.q = self.q + self.qd * DT


def run_case(name, plant_dyn, tau_bias=None, duration=None):
    chain = B601TCPChain(B601_DH_TABLE)
    ctrl_dyn = B601NominalDynamics()
    gains = GAIN_SETS[DEFAULT_GAIN_SET]
    K_d, k_p = gains["K_d"], gains["k_p"]

    traj, t_move, _t_close = build_setpoint_goto_trajectory()
    if duration is None:
        duration = t_move + 4.0          # 运动段 + 4 s 定点保持

    backend = FakeBackend(plant_dyn, tau_bias)
    backend.reset_to(Q_INIT)
    n_steps = int(round(duration / DT))

    log_t, log_pe, log_oe, gov_n, sat_n = [], [], [], 0, 0
    for k in range(n_steps):
        t = k * DT
        q, qd = backend.get_joint_state()
        fk = chain.fk_outputs(q, qd, None, True)
        des = traj.evaluate(t)
        err = full_error_state(fk["x_breve"], des["x_breve_d"])
        sig_min = float(np.linalg.svd(fk["J"], compute_uv=False)[-1])
        damping = PINV_DAMPING if sig_min >= SINGULARITY_TOL \
            else SINGULARITY_DAMPING
        qddot_ref, _ = geometric_computed_torque_law(
            err, des["xi_d"], des["xi_dot_d"],
            fk["J"], fk["Jdot_qdot"], K_d, k_p, damping=damping,
            M=ctrl_dyn.mass_matrix(q))
        qn = float(np.linalg.norm(qddot_ref))
        if qn > QDDOT_MAX:
            qddot_ref = qddot_ref * (QDDOT_MAX / qn)
        qddot_ref, gov = joint_safety_governor(q, qd, qddot_ref)
        gov_n += int(gov)
        tau = ctrl_dyn.computed_torque(q, qd, qddot_ref)
        tau = tau - JOINT_DAMPING * qd
        tau, sat = clip_torque(tau)
        sat_n += int(sat)
        backend.apply_arm_torques(tau)
        backend.step()

        if k % 5 == 0:
            p_w = dq_translation(fk["x"])
            pd_w = dq_translation(des["x_d"])
            r, r_d = dq_rotation(fk["x"]), dq_rotation(des["x_d"])
            log_t.append(t)
            log_pe.append(float(np.linalg.norm(p_w - pd_w)))
            log_oe.append(float(2.0 * np.arccos(
                min(1.0, abs(float(r @ r_d))))))

    log_t, log_pe, log_oe = map(np.array, (log_t, log_pe, log_oe))
    tail = log_t >= duration - 1.0
    q_end, qd_end = backend.get_joint_state()
    print(f"[{name}] 末段 pos_err={log_pe[tail].mean():.5f} m  "
          f"ori_err={log_oe[tail].mean():.5f} rad  "
          f"峰值 pos={log_pe.max():.4f} m  sigma_min_end={sig_min:.3f}\n"
          f"         治理 {gov_n} 步 / 饱和 {sat_n} 步 / "
          f"|qd|_end={np.abs(qd_end).max():.3f} rad/s / "
          f"q_end={np.round(q_end, 3)}", flush=True)
    return log_pe[tail].mean()


def main():
    # [A] 完美模型
    run_case("A 完美模型", B601NominalDynamics())

    # [B] 惯量 +7% 失配（探针实测上界）
    dyn_b = B601NominalDynamics(mismatch_scale=1.07)
    run_case("B 惯量+7%", dyn_b)

    # [C] 惯量 +7% 且重力通道加 mN·m 级偏差（复现 Isaac 残差量级）
    dyn_c = B601NominalDynamics(mismatch_scale=1.07)
    bias = np.array([0.001, -0.002, 0.003, -0.004, 0.002, -0.0015])
    run_case("C 惯量+7%+重力残差", dyn_c, tau_bias=bias)


if __name__ == "__main__":
    main()
