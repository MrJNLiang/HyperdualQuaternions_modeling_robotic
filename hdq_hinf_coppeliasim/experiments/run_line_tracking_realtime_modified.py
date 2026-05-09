import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

from sim.joint_names import JOINT_PATHS_7R, BASE_PATH, TIP_PATH
from sim.coppelia_client import CoppeliaJointClient

from configs.kuka_like_7r import DH_TABLE
from core.robot_dh import SerialDHRobot
from core.trajectory_circle import SmoothCircleTrajectory
# from core.trajectory_line import LineTrajectory
from core.disturbances import joint_velocity_disturbance, measurement_noise
from core.dq_compute import compute_dq_outputs
from core.hdq_compute import compute_hdq_outputs
from core.errors import pose_error, pose_error_norm
from core.controllers import hinf_tracking_control


def _norm3_or_6(x):
    return float(np.linalg.norm(np.asarray(x, dtype=float)))


def main():
    # ========== 实验参数 ==========
    total_time = 12.0
    control_dt = 0.02        # 50 Hz

    # 控制器仍然需要 J(q)，这里默认用 DQ/geometric Jacobian
    jacobian_method = "geometric"

    # True: 反馈误差用 CoppeliaSim 真实末端 tip 位姿 x_sim
    # False: 反馈误差用 Python 模型 FK 位姿 x_model
    use_sim_tip_feedback = False

    # 是否同时计算 HDQ，用于比较 xi_DQ 与 xi_HDQ；关闭后可测试计算延迟影响
    compute_hdq_for_log = True

    gamma_O = 1.0
    gamma_T = 1.0
    damping = 1e-3

    qdot_limit = 0.8

    disturbance_scale = 0.0      # 先设0，跑通后改 0.3 / 0.5
    pos_noise_scale = 0.0
    vel_noise_scale = 0.0

    rng = np.random.default_rng(0)

    # ========== 初始化 ==========
    sim_robot = CoppeliaJointClient(JOINT_PATHS_7R)
    sim_robot.set_tip_and_base(
        tip_path=TIP_PATH,
        base_path=BASE_PATH
    )

    model_robot = SerialDHRobot(DH_TABLE)

    sim_robot.start()

    # 避开竖直伸直的奇异位形
    q_safe = np.array([0.1, -0.6, 0.4, -0.9, 0.5, 0.6, 0.2], dtype=float)

    # 先清零速度，避免上一轮残留速度继续推动关节
    sim_robot.set_joint_target_velocity(np.zeros(len(sim_robot.joints)))
    time.sleep(0.1)

    for h, qi in zip(sim_robot.joints, q_safe):
        sim_robot.sim.setJointPosition(h, float(qi))

    sim_robot.set_joint_target_velocity(np.zeros(len(sim_robot.joints)))
    time.sleep(0.2)

    q0, qdot0 = sim_robot.read_state()
    qdot0 = np.nan_to_num(qdot0, nan=0.0)

    print("q_safe =", np.round(q_safe, 4))
    print("q0 read =", np.round(q0, 4))
    print("||q0 - q_safe|| =", np.linalg.norm(q0 - q_safe))

    # Python FK 位姿，以及 CoppeliaSim 真实 tip 位姿
    x0_model = model_robot.fkm(q0)
    x0_sim = sim_robot.read_tip_pose_dq()

    O_model_sim_0, T_model_sim_0, _ = pose_error(x0_model, x0_sim)
    print("\n===== initial model-vs-sim tip check =====")
    print("||O(model, sim)|| =", _norm3_or_6(O_model_sim_0))
    print("||T(model, sim)|| =", _norm3_or_6(T_model_sim_0))

    # 轨迹起点使用 CoppeliaSim 真实末端位姿
    x0 = x0_sim

    # ========== 末端轨迹 ==========
    traj = SmoothCircleTrajectory(
        x_start=x0,
        radius=0.04,
        period=8.0,
        ramp_time=2.0
    )

    # 检查轨迹 t=0 是否确实从当前真实 tip 位姿开始
    xd0, xi_d0 = traj.evaluate(0.0)
    O_traj0, T_traj0, _ = pose_error(x0_sim, xd0)
    print("\n===== trajectory initialization check =====")
    print("||O(x0_sim, xd(0))|| =", _norm3_or_6(O_traj0))
    print("||T(x0_sim, xd(0))|| =", _norm3_or_6(T_traj0))
    print("xi_d(0) =", np.round(xi_d0, 6))

    # ========== 日志 ==========
    t_log = []

    # 真实 CoppeliaSim tip 相对于目标轨迹的误差
    e_sim_log = []
    eO_sim_log = []
    eT_sim_log = []

    # Python FK 模型相对于目标轨迹的误差
    e_model_log = []
    eO_model_log = []
    eT_model_log = []

    # Python FK 模型与 CoppeliaSim tip 之间的差异
    e_model_sim_log = []
    eO_model_sim_log = []
    eT_model_sim_log = []

    qdot_cmd_log = []
    qdot_send_log = []
    qdot_meas_log = []

    xi_dq_log = []
    xi_hdq_log = []
    xi_diff_log = []

    runtime_dq_ms_log = []
    runtime_hdq_ms_log = []
    runtime_total_ms_log = []

    try:
        t_start = time.time()
        next_time = t_start

        while True:
            # 控制频率控制：先睡眠，再读取当前时间，避免 t 使用 sleep 前的旧值
            wall_now = time.time()
            if wall_now < next_time:
                time.sleep(next_time - wall_now)
            next_time += control_dt

            t = time.time() - t_start
            if t >= total_time:
                break

            loop_t0 = time.perf_counter_ns()

            # ========== 1. 实时读取 q 和 q_dot ==========
            q_raw, qdot_raw = sim_robot.read_state()
            qdot_raw = np.nan_to_num(qdot_raw, nan=0.0)

            # ========== 2. 加测量噪声，可选 ==========
            noise_q, noise_qdot = measurement_noise(
                n=len(q_raw),
                pos_scale=pos_noise_scale,
                vel_scale=vel_noise_scale,
                rng=rng
            )
            q_meas = q_raw + noise_q
            qdot_meas = qdot_raw + noise_qdot

            # ========== 3. DQ 计算：x_model, J, xi_DQ ==========
            t_dq0 = time.perf_counter_ns()
            dq_out = compute_dq_outputs(
                model_robot,
                q_meas,
                qdot_meas,
                jacobian_method=jacobian_method
            )
            t_dq1 = time.perf_counter_ns()
            runtime_dq_ms = (t_dq1 - t_dq0) / 1e6

            x_model = dq_out["x"]
            J = dq_out["J"]
            xi_dq = dq_out["xi"]

            # ========== 4. HDQ 计算：x_hdq, xi_HDQ；只用于对比 ==========
            if compute_hdq_for_log:
                t_hdq0 = time.perf_counter_ns()
                hdq_out = compute_hdq_outputs(
                    model_robot,
                    q_meas,
                    qdot_meas
                )
                t_hdq1 = time.perf_counter_ns()
                runtime_hdq_ms = (t_hdq1 - t_hdq0) / 1e6
                xi_hdq = hdq_out["xi"]
                xi_diff = np.linalg.norm(xi_dq - xi_hdq)
            else:
                runtime_hdq_ms = 0.0
                xi_hdq = np.zeros(6)
                xi_diff = 0.0

            # ========== 5. 从 CoppeliaSim 读取真实末端 tip 位姿 ==========
            x_sim = sim_robot.read_tip_pose_dq()

            # ========== 6. 生成时变末端轨迹 ==========
            xd, xi_d = traj.evaluate(t)

            # ========== 7. 三种误差 ==========
            # 真实跟踪误差：CoppeliaSim tip vs 目标轨迹
            O_sim, T_sim, x_tilde_sim = pose_error(x_sim, xd)

            # 模型跟踪误差：Python FK vs 目标轨迹
            O_model, T_model, x_tilde_model = pose_error(x_model, xd)

            # 模型-真实末端差异：Python FK vs CoppeliaSim tip
            O_model_sim, T_model_sim, _ = pose_error(x_model, x_sim)

            # ========== 8. 控制器反馈位姿选择 ==========
            if use_sim_tip_feedback:
                # 更接近真实机器人：误差反馈来自仿真器真实末端位姿；J仍来自名义模型
                O_ctrl, T_ctrl, x_tilde_ctrl = O_sim, T_sim, x_tilde_sim
            else:
                # 纯模型反馈：误差完全由 Python FK 估计
                O_ctrl, T_ctrl, x_tilde_ctrl = O_model, T_model, x_tilde_model

            # ========== 9. H∞ tracking 控制律 ==========
            qdot_cmd, task_velocity, kO, kT = hinf_tracking_control(
                J=J,
                O=O_ctrl,
                T=T_ctrl,
                x_tilde=x_tilde_ctrl,
                xi_d=xi_d,
                gamma_O=gamma_O,
                gamma_T=gamma_T,
                damping=damping
            )

            qdot_cmd = np.clip(qdot_cmd, -qdot_limit, qdot_limit)

            # ========== 10. 加关节速度扰动 ==========
            d_joint = joint_velocity_disturbance(
                t=t,
                n=len(qdot_cmd),
                scale=disturbance_scale
            )
            qdot_send = qdot_cmd + d_joint
            qdot_send = np.clip(qdot_send, -qdot_limit, qdot_limit)

            # ========== 11. 发送给CoppeliaSim ==========
            sim_robot.set_joint_target_velocity(qdot_send)

            loop_t1 = time.perf_counter_ns()
            runtime_total_ms = (loop_t1 - loop_t0) / 1e6

            # ========== 12. 记录 ==========
            t_log.append(t)

            eO_sim_log.append(_norm3_or_6(O_sim))
            eT_sim_log.append(_norm3_or_6(T_sim))
            e_sim_log.append(pose_error_norm(O_sim, T_sim))

            eO_model_log.append(_norm3_or_6(O_model))
            eT_model_log.append(_norm3_or_6(T_model))
            e_model_log.append(pose_error_norm(O_model, T_model))

            eO_model_sim_log.append(_norm3_or_6(O_model_sim))
            eT_model_sim_log.append(_norm3_or_6(T_model_sim))
            e_model_sim_log.append(pose_error_norm(O_model_sim, T_model_sim))

            qdot_cmd_log.append(qdot_cmd.copy())
            qdot_send_log.append(qdot_send.copy())
            qdot_meas_log.append(qdot_meas.copy())

            xi_dq_log.append(xi_dq.copy())
            xi_hdq_log.append(xi_hdq.copy())
            xi_diff_log.append(xi_diff)

            runtime_dq_ms_log.append(runtime_dq_ms)
            runtime_hdq_ms_log.append(runtime_hdq_ms)
            runtime_total_ms_log.append(runtime_total_ms)

            if len(t_log) % 20 == 0:
                print(
                    f"t={t:6.3f} | "
                    f"eT_sim={eT_sim_log[-1]:.5f} | "
                    f"eT_model={eT_model_log[-1]:.5f} | "
                    f"model-sim={eT_model_sim_log[-1]:.5f} | "
                    f"xi_diff={xi_diff:.3e} | "
                    f"DQ={runtime_dq_ms:.3f}ms | "
                    f"HDQ={runtime_hdq_ms:.3f}ms | "
                    f"loop={runtime_total_ms:.3f}ms"
                )

    finally:
        sim_robot.stop()

    # ========== 保存结果 ==========
    os.makedirs("results/data", exist_ok=True)
    os.makedirs("results/plots", exist_ok=True)

    t_log = np.array(t_log)

    np.savez(
        "results/data/line_tracking_realtime_result.npz",
        time=t_log,

        eO_sim=np.array(eO_sim_log),
        eT_sim=np.array(eT_sim_log),
        e_sim=np.array(e_sim_log),

        eO_model=np.array(eO_model_log),
        eT_model=np.array(eT_model_log),
        e_model=np.array(e_model_log),

        eO_model_sim=np.array(eO_model_sim_log),
        eT_model_sim=np.array(eT_model_sim_log),
        e_model_sim=np.array(e_model_sim_log),

        qdot_cmd=np.array(qdot_cmd_log),
        qdot_send=np.array(qdot_send_log),
        qdot_meas=np.array(qdot_meas_log),

        xi_dq=np.array(xi_dq_log),
        xi_hdq=np.array(xi_hdq_log),
        xi_diff=np.array(xi_diff_log),

        runtime_dq_ms=np.array(runtime_dq_ms_log),
        runtime_hdq_ms=np.array(runtime_hdq_ms_log),
        runtime_total_ms=np.array(runtime_total_ms_log),

        use_sim_tip_feedback=use_sim_tip_feedback,
        compute_hdq_for_log=compute_hdq_for_log,
        gamma_O=gamma_O,
        gamma_T=gamma_T,
        damping=damping,
        qdot_limit=qdot_limit,
        disturbance_scale=disturbance_scale
    )

    # ========== 画图：真实误差 vs 模型误差 ==========
    plt.figure()
    plt.plot(t_log, eO_sim_log, label="true orientation error: sim tip vs target ||O||")
    plt.plot(t_log, eT_sim_log, label="true translation error: sim tip vs target ||T||")
    plt.plot(t_log, e_sim_log, label="true combined error")
    plt.xlabel("time [s]")
    plt.ylabel("true tracking error")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/line_tracking_true_sim_error.png", dpi=200)

    plt.figure()
    plt.plot(t_log, eT_sim_log, label="true translation error: sim tip vs target")
    plt.plot(t_log, eT_model_log, label="model translation error: Python FK vs target")
    plt.plot(t_log, eT_model_sim_log, label="model-sim translation difference: Python FK vs sim tip")
    plt.xlabel("time [s]")
    plt.ylabel("translation error [m]")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/translation_error_comparison.png", dpi=200)

    plt.figure()
    plt.plot(t_log, eO_model_sim_log, label="orientation difference: Python FK vs sim tip")
    plt.plot(t_log, eT_model_sim_log, label="translation difference: Python FK vs sim tip")
    plt.xlabel("time [s]")
    plt.ylabel("model-sim difference")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/model_vs_sim_tip_error.png", dpi=200)

    # ========== 画图：DQ/HDQ twist 差异 ==========
    plt.figure()
    plt.semilogy(t_log, np.array(xi_diff_log) + 1e-20, label="||xi_DQ - xi_HDQ||")
    plt.xlabel("time [s]")
    plt.ylabel("twist difference, log scale")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/dq_hdq_twist_difference_log.png", dpi=200)

    # ========== 画图：计算耗时 ==========
    plt.figure()
    plt.plot(t_log, runtime_dq_ms_log, label="DQ output time")
    plt.plot(t_log, runtime_hdq_ms_log, label="HDQ output time")
    plt.plot(t_log, runtime_total_ms_log, label="total loop calculation time")
    plt.xlabel("time [s]")
    plt.ylabel("runtime [ms]")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/runtime_dq_hdq.png", dpi=200)

    plt.show()

    print("\nFinished.")
    print("use_sim_tip_feedback =", use_sim_tip_feedback)
    print("compute_hdq_for_log =", compute_hdq_for_log)
    print("Mean true eT_sim [m] =", np.mean(eT_sim_log))
    print("Mean model eT_model [m] =", np.mean(eT_model_log))
    print("Mean model-sim eT [m] =", np.mean(eT_model_sim_log))
    print("Mean DQ time  [ms] =", np.mean(runtime_dq_ms_log))
    print("Mean HDQ time [ms] =", np.mean(runtime_hdq_ms_log))
    print("Mean xi diff       =", np.mean(xi_diff_log))
    print("Max xi diff        =", np.max(xi_diff_log))


if __name__ == "__main__":
    main()
