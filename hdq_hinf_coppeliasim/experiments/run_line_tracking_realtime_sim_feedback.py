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
from core.disturbances import joint_velocity_disturbance, measurement_noise
from core.dq_compute import compute_dq_outputs
from core.hdq_compute import compute_hdq_outputs
from core.errors import pose_error, pose_error_norm
from core.controllers import hinf_tracking_control


def mat12_to_T(m):
    """Convert CoppeliaSim 3x4 object matrix list to a 4x4 matrix."""
    m = np.array(m, dtype=float).reshape(3, 4)
    T = np.eye(4)
    T[:3, :3] = m[:, :3]
    T[:3, 3] = m[:, 3]
    return T


def sim_pose_jacobian_dq(sim_robot):
    """
    Build the DQ-convention spatial Jacobian directly from CoppeliaSim joint frames.

    Convention used by the H∞ controller:
        x_dot = 1/2 * xi * x
        xi = omega + eps * (p_dot + p x omega)

    For a revolute joint with axis z and origin o, both expressed in the same base frame:
        omega_i = z_i
        p_dot_i = z_i x (p_tip - o_i)
        dual_i = p_dot_i + p_tip x omega_i = o_i x z_i

    Therefore column i is:
        J_i = [z_i ; o_i x z_i]

    This avoids using the possibly mismatched Python DH parameters for the CoppeliaSim feedback loop.
    """
    sim = sim_robot.sim
    base = sim_robot.base
    J = np.zeros((6, len(sim_robot.joints)))

    for i, h in enumerate(sim_robot.joints):
        Tj = mat12_to_T(sim.getObjectMatrix(h, base))
        o = Tj[:3, 3]
        z = Tj[:3, 2]
        z_norm = np.linalg.norm(z)
        if z_norm < 1e-12:
            raise RuntimeError(f"Joint {i+1} axis norm is too small")
        z = z / z_norm
        J[:, i] = np.r_[z, np.cross(o, z)]

    return J


def rate_limit_qdot(qdot_cmd, qdot_prev, dt, max_acc=1.0):
    """Limit per-joint acceleration of the velocity command."""
    delta = qdot_cmd - qdot_prev
    delta = np.clip(delta, -max_acc * dt, max_acc * dt)
    return qdot_prev + delta


def main():
    # ========== Experiment parameters ==========
    total_time = 12.0
    control_dt = 0.02          # 50 Hz outer loop

    # Use CoppeliaSim real tip pose + CoppeliaSim-derived Jacobian for the actual feedback.
    # Python DQ/HDQ is computed only for logging/comparison.
    gamma_O = 2.0
    gamma_T = 2.0
    damping = 1e-2
    qdot_limit = 0.45
    max_acc = 0.8

    disturbance_scale = 0.0
    pos_noise_scale = 0.0
    vel_noise_scale = 0.0
    rng = np.random.default_rng(0)

    # ========== Init ==========
    sim_robot = CoppeliaJointClient(JOINT_PATHS_7R)
    sim_robot.set_tip_and_base(tip_path=TIP_PATH, base_path=BASE_PATH)

    # Nominal Python model is kept only for DQ/HDQ timing and consistency comparison.
    model_robot = SerialDHRobot(DH_TABLE)

    sim_robot.start()

    q_safe = np.array([0.1, -0.6, 0.4, -0.9, 0.5, 0.6, 0.2], dtype=float)

    sim_robot.set_joint_target_velocity(np.zeros(len(sim_robot.joints)))
    time.sleep(0.1)

    for h, qi in zip(sim_robot.joints, q_safe):
        sim_robot.sim.setJointPosition(h, float(qi))

    sim_robot.set_joint_target_velocity(np.zeros(len(sim_robot.joints)))
    time.sleep(0.2)

    q0, qdot0 = sim_robot.read_state()
    qdot0 = np.nan_to_num(qdot0, nan=0.0)

    x0_sim = sim_robot.read_tip_pose_dq()
    x0_model = model_robot.fkm(q0)
    O0_ms, T0_ms, _ = pose_error(x0_model, x0_sim)

    print("q_safe =", np.round(q_safe, 4))
    print("q0 read =", np.round(q0, 4))
    print("||q0 - q_safe|| =", np.linalg.norm(q0 - q_safe))
    print("initial model-sim eT =", np.linalg.norm(T0_ms))
    print("NOTE: actual control uses CoppeliaSim tip pose and CoppeliaSim-derived Jacobian, not Python DH Jacobian.")

    # Trajectory starts from the real CoppeliaSim tip pose.
    traj = SmoothCircleTrajectory(
        x_start=x0_sim,
        radius=0.04,
        period=8.0,
        ramp_time=2.0,
    )

    # Check trajectory initialization.
    xd0, xi0 = traj.evaluate(0.0)
    O_init, T_init, _ = pose_error(x0_sim, xd0)
    print("trajectory init ||O|| =", np.linalg.norm(O_init))
    print("trajectory init ||T|| =", np.linalg.norm(T_init))
    print("trajectory init xi_d =", np.round(xi0, 6))

    # ========== Logs ==========
    t_log = []
    eO_ctrl_log = []
    eT_ctrl_log = []
    e_ctrl_log = []

    eT_model_log = []
    eT_model_sim_log = []

    qdot_cmd_log = []
    qdot_meas_log = []

    xi_dq_log = []
    xi_hdq_log = []
    xi_diff_log = []

    runtime_dq_ms_log = []
    runtime_hdq_ms_log = []
    runtime_total_ms_log = []

    qdot_cmd_prev = np.zeros(len(sim_robot.joints))

    try:
        t_start = time.time()
        next_time = t_start

        while True:
            wall_now = time.time()
            t = wall_now - t_start
            if t >= total_time:
                break

            if wall_now < next_time:
                time.sleep(next_time - wall_now)
            next_time += control_dt

            loop_t0 = time.perf_counter_ns()

            # 1. Read actual joint state.
            q_raw, qdot_raw = sim_robot.read_state()
            qdot_raw = np.nan_to_num(qdot_raw, nan=0.0)

            noise_q, noise_qdot = measurement_noise(
                n=len(q_raw),
                pos_scale=pos_noise_scale,
                vel_scale=vel_noise_scale,
                rng=rng,
            )
            q_meas = q_raw + noise_q
            qdot_meas = qdot_raw + noise_qdot

            # 2. Read real CoppeliaSim tip pose.
            x_sim = sim_robot.read_tip_pose_dq()

            # 3. Actual control Jacobian from CoppeliaSim joint frames.
            J_sim = sim_pose_jacobian_dq(sim_robot)

            # 4. Nominal DQ/HDQ calculations only for timing and consistency logs.
            t_dq0 = time.perf_counter_ns()
            dq_out = compute_dq_outputs(
                model_robot,
                q_meas,
                qdot_meas,
                jacobian_method="geometric",
            )
            t_dq1 = time.perf_counter_ns()

            t_hdq0 = time.perf_counter_ns()
            hdq_out = compute_hdq_outputs(model_robot, q_meas, qdot_meas)
            t_hdq1 = time.perf_counter_ns()

            runtime_dq_ms = (t_dq1 - t_dq0) / 1e6
            runtime_hdq_ms = (t_hdq1 - t_hdq0) / 1e6

            xi_dq = dq_out["xi"]
            xi_hdq = hdq_out["xi"]
            xi_diff = np.linalg.norm(xi_dq - xi_hdq)

            # 5. Desired time-varying tip trajectory.
            xd, xi_d = traj.evaluate(t)

            # 6. Feedback error uses real CoppeliaSim tip pose.
            O, T, x_tilde = pose_error(x_sim, xd)

            # 7. H∞ tracking control with sim-derived Jacobian.
            qdot_cmd, task_velocity, kO, kT = hinf_tracking_control(
                J=J_sim,
                O=O,
                T=T,
                x_tilde=x_tilde,
                xi_d=xi_d,
                gamma_O=gamma_O,
                gamma_T=gamma_T,
                damping=damping,
            )

            qdot_cmd = np.clip(qdot_cmd, -qdot_limit, qdot_limit)
            qdot_cmd = rate_limit_qdot(qdot_cmd, qdot_cmd_prev, control_dt, max_acc=max_acc)
            qdot_cmd_prev = qdot_cmd.copy()

            # 8. Add optional joint velocity disturbance.
            d_joint = joint_velocity_disturbance(t=t, n=len(qdot_cmd), scale=disturbance_scale)
            qdot_send = np.clip(qdot_cmd + d_joint, -qdot_limit, qdot_limit)

            # 9. Send velocity command.
            sim_robot.set_joint_target_velocity(qdot_send)

            loop_t1 = time.perf_counter_ns()
            runtime_total_ms = (loop_t1 - loop_t0) / 1e6

            # 10. Diagnostic model errors.
            x_model = dq_out["x"]
            O_model, T_model, _ = pose_error(x_model, xd)
            O_model_sim, T_model_sim, _ = pose_error(x_model, x_sim)

            # Logs.
            t_log.append(t)
            eO_ctrl_log.append(np.linalg.norm(O))
            eT_ctrl_log.append(np.linalg.norm(T))
            e_ctrl_log.append(pose_error_norm(O, T))
            eT_model_log.append(np.linalg.norm(T_model))
            eT_model_sim_log.append(np.linalg.norm(T_model_sim))

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
                    f"eT_ctrl={eT_ctrl_log[-1]:.5f} | "
                    f"eT_model={eT_model_log[-1]:.5f} | "
                    f"model-sim={eT_model_sim_log[-1]:.5f} | "
                    f"max_qdot={np.max(np.abs(qdot_send)):.3f} | "
                    f"xi_diff={xi_diff:.3e} | "
                    f"DQ={runtime_dq_ms:.3f}ms | "
                    f"HDQ={runtime_hdq_ms:.3f}ms | "
                    f"loop={runtime_total_ms:.3f}ms"
                )

    finally:
        sim_robot.stop()

    # ========== Save ==========
    os.makedirs("results/data", exist_ok=True)
    os.makedirs("results/plots", exist_ok=True)

    t_log = np.array(t_log)

    np.savez(
        "results/data/line_tracking_realtime_sim_feedback_result.npz",
        time=t_log,
        eO_ctrl=np.array(eO_ctrl_log),
        eT_ctrl=np.array(eT_ctrl_log),
        e_ctrl=np.array(e_ctrl_log),
        eT_model=np.array(eT_model_log),
        eT_model_sim=np.array(eT_model_sim_log),
        qdot_cmd=np.array(qdot_cmd_log),
        qdot_meas=np.array(qdot_meas_log),
        xi_dq=np.array(xi_dq_log),
        xi_hdq=np.array(xi_hdq_log),
        xi_diff=np.array(xi_diff_log),
        runtime_dq_ms=np.array(runtime_dq_ms_log),
        runtime_hdq_ms=np.array(runtime_hdq_ms_log),
        runtime_total_ms=np.array(runtime_total_ms_log),
    )

    # ========== Plots ==========
    plt.figure()
    plt.plot(t_log, eT_ctrl_log, label="CoppeliaSim tip tracking eT")
    plt.plot(t_log, eT_model_log, label="Python FK tracking eT")
    plt.plot(t_log, eT_model_sim_log, label="Python FK vs CoppeliaSim tip eT")
    plt.xlabel("time [s]")
    plt.ylabel("translation error [m]")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/sim_feedback_translation_errors.png", dpi=200)

    plt.figure()
    plt.semilogy(t_log, np.array(xi_diff_log) + 1e-20, label="||xi_DQ - xi_HDQ||")
    plt.xlabel("time [s]")
    plt.ylabel("twist difference, log scale")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/sim_feedback_dq_hdq_twist_difference_log.png", dpi=200)

    plt.figure()
    plt.plot(t_log, runtime_dq_ms_log, label="DQ output time")
    plt.plot(t_log, runtime_hdq_ms_log, label="HDQ output time")
    plt.plot(t_log, runtime_total_ms_log, label="total loop computation time")
    plt.xlabel("time [s]")
    plt.ylabel("runtime [ms]")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/sim_feedback_runtime_dq_hdq.png", dpi=200)

    plt.show()

    print("\nFinished.")
    print("Mean DQ time  [ms] =", np.mean(runtime_dq_ms_log))
    print("Mean HDQ time [ms] =", np.mean(runtime_hdq_ms_log))
    print("Mean xi diff       =", np.mean(xi_diff_log))
    print("Mean eT_ctrl       =", np.mean(eT_ctrl_log))
    print("Mean eT_model      =", np.mean(eT_model_log))
    print("Mean model-sim eT  =", np.mean(eT_model_sim_log))


if __name__ == "__main__":
    main()
