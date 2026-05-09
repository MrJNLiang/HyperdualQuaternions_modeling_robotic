import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

from sim.joint_names import JOINT_PATHS_7R
from sim.coppelia_client import CoppeliaJointClient

from configs.kuka_like_7r import DH_TABLE
from core.robot_dh import SerialDHRobot
from core.trajectory_circle import SmoothCircleTrajectory
# from core.trajectory_line import LineTrajectory
from core.disturbances import joint_velocity_disturbance, measurement_noise
from core.fk_backend import compute_fk_outputs
from core.dq_compute import compute_dq_outputs
from core.hdq_compute import compute_hdq_outputs
from core.errors import pose_error, pose_error_norm
from core.controllers import hinf_tracking_control


def main():
    # ========== 实验参数 ==========
    total_time = 12.0
    control_dt = 0.02        # 50 Hz，先不要太快
    backend_for_control = "dq"   # "dq" 或 "hdq"
    jacobian_method = "geometric"

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
    model_robot = SerialDHRobot(DH_TABLE)

    sim_robot.start()

    q_safe = np.array([0.1, -0.6, 0.4, -0.9, 0.5, 0.6, 0.2])

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

    x0 = model_robot.fkm(q0)

    # 末端轨迹
    traj = SmoothCircleTrajectory(
        x_start=x0,
        radius=0.04,
        period=8.0,
        ramp_time=2.0
    )
    '''traj = LineTrajectory(
        x_start=x0,
        delta_p=np.array([0.12, 0.00, 0.08]),
        duration=8.0
    )'''

    # ========== 日志 ==========
    t_log = []
    e_log = []
    eO_log = []
    eT_log = []

    qdot_cmd_log = []
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
            wall_now = time.time()
            t = wall_now - t_start

            if t >= total_time:
                break

            loop_t0 = time.perf_counter_ns()

            # 控制频率控制
            if wall_now < next_time:
                time.sleep(next_time - wall_now)

            next_time += control_dt

            # ========== 1. 实时读取 q 和 q_dot ==========
            q_raw, qdot_raw = sim_robot.read_state()

            # 如果某些速度读不到，用0替代；后面也可以改成差分兜底
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

            # ========== 3. DQ和HDQ都计算一遍，用于比较 ==========
            t_dq0 = time.perf_counter_ns()
            dq_out = compute_dq_outputs(
                model_robot,
                q_meas,
                qdot_meas,
                jacobian_method=jacobian_method
            )
            t_dq1 = time.perf_counter_ns()

            t_hdq0 = time.perf_counter_ns()
            hdq_out = compute_hdq_outputs(
                model_robot,
                q_meas,
                qdot_meas
            )
            t_hdq1 = time.perf_counter_ns()

            runtime_dq_ms = (t_dq1 - t_dq0) / 1e6
            runtime_hdq_ms = (t_hdq1 - t_hdq0) / 1e6

            xi_dq = dq_out["xi"]
            xi_hdq = hdq_out["xi"]
            xi_diff = np.linalg.norm(xi_dq - xi_hdq)

            # ========== 4. 选择控制用的FK输出 ==========
            if backend_for_control == "dq":
                x_current = dq_out["x"]
                J = dq_out["J"]
            elif backend_for_control == "hdq":
                # HDQ能给x和xi，但逆运动学控制仍然需要J
                x_current = hdq_out["x"]
                J = model_robot.pose_jacobian_geometric(q_meas)
            else:
                raise ValueError(f"Unknown backend_for_control: {backend_for_control}")

            # ========== 5. 生成时变末端直线轨迹 ==========
            xd, xi_d = traj.evaluate(t)

            # ========== 6. 位姿误差 ==========
            O, T, x_tilde = pose_error(x_current, xd)

            # ========== 7. H∞ tracking 控制律 ==========
            qdot_cmd, task_velocity, kO, kT = hinf_tracking_control(
                J=J,
                O=O,
                T=T,
                x_tilde=x_tilde,
                xi_d=xi_d,
                gamma_O=gamma_O,
                gamma_T=gamma_T,
                damping=damping
            )

            qdot_cmd = np.clip(qdot_cmd, -qdot_limit, qdot_limit)

            # ========== 8. 加关节速度扰动 ==========
            d_joint = joint_velocity_disturbance(
                t=t,
                n=len(qdot_cmd),
                scale=disturbance_scale
            )

            qdot_send = qdot_cmd + d_joint
            qdot_send = np.clip(qdot_send, -qdot_limit, qdot_limit)

            # ========== 9. 发送给CoppeliaSim ==========
            sim_robot.set_joint_target_velocity(qdot_send)

            loop_t1 = time.perf_counter_ns()
            runtime_total_ms = (loop_t1 - loop_t0) / 1e6

            # ========== 10. 记录 ==========
            t_log.append(t)
            eO_log.append(np.linalg.norm(O))
            eT_log.append(np.linalg.norm(T))
            e_log.append(pose_error_norm(O, T))

            qdot_cmd_log.append(qdot_cmd.copy())
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
                    f"eO={eO_log[-1]:.5f} | "
                    f"eT={eT_log[-1]:.5f} | "
                    f"xi_diff={xi_diff:.3e} | "
                    f"DQ={runtime_dq_ms:.3f}ms | "
                    f"HDQ={runtime_hdq_ms:.3f}ms"
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
        eO=np.array(eO_log),
        eT=np.array(eT_log),
        e=np.array(e_log),
        qdot_cmd=np.array(qdot_cmd_log),
        qdot_meas=np.array(qdot_meas_log),
        xi_dq=np.array(xi_dq_log),
        xi_hdq=np.array(xi_hdq_log),
        xi_diff=np.array(xi_diff_log),
        runtime_dq_ms=np.array(runtime_dq_ms_log),
        runtime_hdq_ms=np.array(runtime_hdq_ms_log),
        runtime_total_ms=np.array(runtime_total_ms_log)
    )

    # ========== 画图 ==========
    plt.figure()
    plt.plot(t_log, eO_log, label="orientation error ||O||")
    plt.plot(t_log, eT_log, label="translation error ||T||")
    plt.plot(t_log, e_log, label="combined error")
    plt.xlabel("time [s]")
    plt.ylabel("error")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/line_tracking_error.png", dpi=200)

    plt.figure()
    plt.plot(t_log, xi_diff_log, label="||xi_DQ - xi_HDQ||")
    plt.xlabel("time [s]")
    plt.ylabel("twist difference")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/dq_hdq_twist_difference.png", dpi=200)

    plt.figure()
    plt.plot(t_log, runtime_dq_ms_log, label="DQ output time")
    plt.plot(t_log, runtime_hdq_ms_log, label="HDQ output time")
    plt.plot(t_log, runtime_total_ms_log, label="total loop time")
    plt.xlabel("time [s]")
    plt.ylabel("runtime [ms]")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/runtime_dq_hdq.png", dpi=200)

    plt.show()

    print("\nFinished.")
    print("Mean DQ time  [ms] =", np.mean(runtime_dq_ms_log))
    print("Mean HDQ time [ms] =", np.mean(runtime_hdq_ms_log))
    print("Mean xi diff       =", np.mean(xi_diff_log))


if __name__ == "__main__":
    main()