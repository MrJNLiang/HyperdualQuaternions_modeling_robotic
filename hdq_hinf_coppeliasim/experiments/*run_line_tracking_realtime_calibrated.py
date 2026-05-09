"""
Consistent real-time task-space tracking experiment for CoppeliaSim.

Key design choice:
    - The controller uses the Python nominal model consistently:
        x_model = FK_model(q)
        J_model = J_model(q)
        xd_model trajectory starts from x0_model
    - CoppeliaSim tip pose is read only for evaluation/logging:
        x_sim = read_tip_pose_dq()
        model-sim error = pose_error(x_model, x_sim)

This avoids the unstable mixed-frame case:
    trajectory starts from x0_sim, but controller feedback uses x_model.

Requirements:
    sim/joint_names.py should define:
        JOINT_PATHS_7R, BASE_PATH, TIP_PATH
    sim/coppelia_client.py should provide CoppeliaJointClient with:
        set_tip_and_base(...), read_tip_pose_dq(), read_state(),
        set_joint_target_velocity(...), start(), stop()
"""

import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

from sim.joint_names import JOINT_PATHS_7R, BASE_PATH, TIP_PATH
from sim.coppelia_client import CoppeliaJointClient

# from configs.kuka_like_7r import DH_TABLE
# from core.robot_dh import SerialDHRobot
from core.coppelia_poe_model import CoppeliaPOEModel
from core.trajectory_circle import SmoothCircleTrajectory
from core.disturbances import joint_velocity_disturbance, measurement_noise
from core.dq_compute import compute_dq_outputs
from core.hdq_compute import compute_hdq_outputs
from core.errors import pose_error, pose_error_norm
from core.controllers import hinf_tracking_control
from core.dq_math import dq_mul, dq_conj


def rate_limit_qdot(qdot_cmd, qdot_prev, dt, max_acc=1.2):
    """Limit joint velocity command rate of change."""
    delta = qdot_cmd - qdot_prev
    delta = np.clip(delta, -max_acc * dt, max_acc * dt)
    return qdot_prev + delta


def safe_pose_error(x_a, x_b):
    """Return norms of pose_error(x_a, x_b)."""
    O, T, x_tilde = pose_error(x_a, x_b)
    return np.linalg.norm(O), np.linalg.norm(T), O, T, x_tilde


def main():
    # ========== Experiment parameters ==========
    total_time = 12.0
    control_dt = 0.02          # 50 Hz

    # Very important:
    # "model" means the controller uses x_model, J_model, and a model-frame trajectory.
    # Keep this as "model" until model-sim error is small and frames are verified.
    control_feedback_source = "sim_aligned"   # "model" or "sim_aligned"
    # Use "model" first. If model-sim-aligned is small and stable, try "sim_aligned".
    backend_for_control = "dq"          # "dq" or "hdq"
    jacobian_method = "geometric"

    # Start conservatively. After stable operation, you can reduce gamma and damping.
    gamma_O = 1.0
    gamma_T = 1.0
    damping = 1e-3

    qdot_limit = 0.8
    use_rate_limit = True
    max_qdot_acc = 1.2         # rad/s^2, increase if the response is too slow

    disturbance_scale = 0.0
    pos_noise_scale = 0.0
    vel_noise_scale = 0.0

    # Trajectory parameters. Radius is small by default.
    circle_radius = 0.04
    circle_period = 8.0
    circle_ramp_time = 2.0

    rng = np.random.default_rng(0)

    # ========== Initialization ==========
    '''sim_robot = CoppeliaJointClient(JOINT_PATHS_7R)
    sim_robot.set_tip_and_base(tip_path=TIP_PATH, base_path=BASE_PATH)
    model_robot = SerialDHRobot(DH_TABLE)

    sim_robot.start()

    q_safe = np.array([0.1, -0.6, 0.4, -0.9, 0.5, 0.6, 0.2], dtype=float)'''
    sim_robot = CoppeliaJointClient(JOINT_PATHS_7R)
    sim_robot.set_tip_and_base(
        tip_path=TIP_PATH,
        base_path=BASE_PATH
    )

    sim_robot.start()

    q_safe = np.array([0.1, -0.6, 0.4, -0.9, 0.5, 0.6, 0.2])

    sim_robot.set_joint_target_velocity(np.zeros(len(sim_robot.joints)))
    time.sleep(0.1)

    for h, qi in zip(sim_robot.joints, q_safe):
        sim_robot.sim.setJointPosition(h, float(qi))

    sim_robot.set_joint_target_velocity(np.zeros(len(sim_robot.joints)))
    time.sleep(0.2)

    q0, qdot0 = sim_robot.read_state()
    qdot0 = np.nan_to_num(qdot0, nan=0.0)

    model_robot = CoppeliaPOEModel.from_coppelia(
        sim=sim_robot.sim,
        joint_handles=sim_robot.joints,
        tip_handle=sim_robot.tip,
        base_handle=sim_robot.base,
        q_home=q0,
        joint_paths=JOINT_PATHS_7R,
        tip_path=TIP_PATH,
        base_label=BASE_PATH
    )  

    model_robot.print_summary()

    # Clear residual velocity before setting the initial pose.
    sim_robot.set_joint_target_velocity(np.zeros(len(sim_robot.joints)))
    time.sleep(0.1)

    for h, qi in zip(sim_robot.joints, q_safe):
        sim_robot.sim.setJointPosition(h, float(qi))

    sim_robot.set_joint_target_velocity(np.zeros(len(sim_robot.joints)))
    time.sleep(0.2)

    q0, qdot0 = sim_robot.read_state()
    qdot0 = np.nan_to_num(qdot0, nan=0.0)

    x0_model = model_robot.fkm(q0)
    x0_sim = sim_robot.read_tip_pose_dq()

    # ===== CoppeliaSim tip pose alignment calibration =====
    # Raw CoppeliaSim tip pose often differs from the Python FK pose because
    # the selected tip object/base object may include a fixed tool-frame offset.
    # We compute a constant right-side offset so that:
    #     x0_sim_aligned = x0_sim_raw * sim_to_model_right_offset = x0_model
    # Then each loop uses:
    #     x_sim_aligned = x_sim_raw * sim_to_model_right_offset
    sim_to_model_right_offset = dq_mul(dq_conj(x0_sim), x0_model)
    x0_sim_aligned = dq_mul(x0_sim, sim_to_model_right_offset)

    O_model_sim_raw0, T_model_sim_raw0, _ = pose_error(x0_model, x0_sim)
    O_model_sim_aligned0, T_model_sim_aligned0, _ = pose_error(x0_model, x0_sim_aligned)

    # Build trajectory in the Python model frame.
    # If control_feedback_source="sim_aligned", the aligned sim pose is calibrated into this frame.
    x0_for_traj = x0_model

    traj = SmoothCircleTrajectory(
        x_start=x0_for_traj,
        radius=circle_radius,
        period=circle_period,
        ramp_time=circle_ramp_time,
    )

    # Debug: trajectory must start exactly from the selected initial pose.
    xd0, xid0 = traj.evaluate(0.0)
    if control_feedback_source == "model":
        O0, T0, _ = pose_error(x0_model, xd0)
    else:
        O0, T0, _ = pose_error(x0_sim, xd0)

    print("\n========== Initialization check ==========")
    print("control_feedback_source =", control_feedback_source)
    print("backend_for_control     =", backend_for_control)
    print("q_safe                  =", np.round(q_safe, 4))
    print("q0 read                 =", np.round(q0, 4))
    print("||q0 - q_safe||         =", np.linalg.norm(q0 - q_safe))
    print("||O(init feedback,traj)|| =", np.linalg.norm(O0))
    print("||T(init feedback,traj)|| =", np.linalg.norm(T0))
    print("raw ||O(model,sim)|| at init      =", np.linalg.norm(O_model_sim_raw0))
    print("raw ||T(model,sim)|| at init      =", np.linalg.norm(T_model_sim_raw0))
    print("aligned ||O(model,sim)|| at init  =", np.linalg.norm(O_model_sim_aligned0))
    print("aligned ||T(model,sim)|| at init  =", np.linalg.norm(T_model_sim_aligned0))
    print("xi_d(0) =", np.round(xid0, 8))
    print("==========================================\n")

    # ========== Logs ==========
    t_log = []

    # Controller feedback error: the error actually used by controller.
    eO_ctrl_log = []
    eT_ctrl_log = []
    e_ctrl_log = []

    # Evaluation errors.
    eO_model_log = []
    eT_model_log = []
    e_model_log = []

    eO_sim_log = []
    eT_sim_log = []
    e_sim_log = []

    eO_model_sim_raw_log = []
    eT_model_sim_raw_log = []
    eO_model_sim_aligned_log = []
    eT_model_sim_aligned_log = []

    qdot_cmd_log = []
    qdot_send_log = []
    qdot_meas_log = []

    xi_dq_log = []
    xi_hdq_log = []
    xi_diff_log = []

    runtime_dq_ms_log = []
    runtime_hdq_ms_log = []
    runtime_total_ms_log = []

    qdot_cmd_prev = np.zeros(len(JOINT_PATHS_7R), dtype=float)

    try:
        t_start = time.time()
        next_time = t_start

        while True:
            wall_now = time.time()
            if wall_now < next_time:
                time.sleep(next_time - wall_now)
                wall_now = time.time()

            t = wall_now - t_start
            if t >= total_time:
                break

            loop_t0 = time.perf_counter_ns()
            next_time += control_dt

            # 1. Read joint state.
            q_raw, qdot_raw = sim_robot.read_state()
            qdot_raw = np.nan_to_num(qdot_raw, nan=0.0)

            # 2. Optional measurement noise.
            noise_q, noise_qdot = measurement_noise(
                n=len(q_raw),
                pos_scale=pos_noise_scale,
                vel_scale=vel_noise_scale,
                rng=rng,
            )
            q_meas = q_raw + noise_q
            qdot_meas = qdot_raw + noise_qdot

            # 3. DQ and HDQ outputs.
            t_dq0 = time.perf_counter_ns()
            dq_out = compute_dq_outputs(
                model_robot,
                q_meas,
                qdot_meas,
                jacobian_method=jacobian_method,
            )
            t_dq1 = time.perf_counter_ns()

            t_hdq0 = time.perf_counter_ns()
            hdq_out = compute_hdq_outputs(model_robot, q_meas, qdot_meas)
            t_hdq1 = time.perf_counter_ns()

            runtime_dq_ms = (t_dq1 - t_dq0) / 1e6
            runtime_hdq_ms = (t_hdq1 - t_hdq0) / 1e6

            x_model = dq_out["x"]
            J_model = dq_out["J"]
            xi_dq = dq_out["xi"]
            xi_hdq = hdq_out["xi"]
            xi_diff = np.linalg.norm(xi_dq - xi_hdq)

            # 4. Read CoppeliaSim tip pose for evaluation and optionally feedback.
            # Raw pose is in the CoppeliaSim selected tip/base convention.
            x_sim_raw = sim_robot.read_tip_pose_dq()
            # Aligned pose is mapped into the Python model frame using the initial calibration.
            x_sim_aligned = dq_mul(x_sim_raw, sim_to_model_right_offset)

            # 5. Desired trajectory.
            xd, xi_d = traj.evaluate(t)

            # 6. Compute model/sim evaluation errors.
            O_model, T_model, x_tilde_model = pose_error(x_model, xd)
            O_sim_raw, T_sim_raw, x_tilde_sim_raw = pose_error(x_sim_raw, xd)
            O_sim_aligned, T_sim_aligned, x_tilde_sim_aligned = pose_error(x_sim_aligned, xd)
            O_model_sim_raw, T_model_sim_raw, _ = pose_error(x_model, x_sim_raw)
            O_model_sim_aligned, T_model_sim_aligned, _ = pose_error(x_model, x_sim_aligned)

            # 7. Select controller feedback pose, but keep J_model.
            if control_feedback_source == "model":
                if backend_for_control == "dq":
                    x_current = x_model
                    O_ctrl, T_ctrl, x_tilde_ctrl = O_model, T_model, x_tilde_model
                    J = J_model
                elif backend_for_control == "hdq":
                    # HDQ pose should equal model pose; use HDQ x for feedback but J_model for inverse kinematics.
                    x_current = hdq_out["x"]
                    O_ctrl, T_ctrl, x_tilde_ctrl = pose_error(x_current, xd)
                    J = model_robot.pose_jacobian_geometric(q_meas)
                else:
                    raise ValueError(f"Unknown backend_for_control: {backend_for_control}")
            elif control_feedback_source == "sim_aligned":
                # Use only after model-sim-aligned error is small and stable.
                x_current = x_sim_aligned
                O_ctrl, T_ctrl, x_tilde_ctrl = O_sim_aligned, T_sim_aligned, x_tilde_sim_aligned
                J = J_model
            else:
                raise ValueError(f"Unknown control_feedback_source: {control_feedback_source}")

            # 8. H-infinity tracking control.
            qdot_cmd, task_velocity, kO, kT = hinf_tracking_control(
                J=J,
                O=O_ctrl,
                T=T_ctrl,
                x_tilde=x_tilde_ctrl,
                xi_d=xi_d,
                gamma_O=gamma_O,
                gamma_T=gamma_T,
                damping=damping,
            )

            qdot_cmd = np.clip(qdot_cmd, -qdot_limit, qdot_limit)

            if use_rate_limit:
                qdot_cmd = rate_limit_qdot(
                    qdot_cmd,
                    qdot_cmd_prev,
                    control_dt,
                    max_acc=max_qdot_acc,
                )
                qdot_cmd_prev = qdot_cmd.copy()

            # 9. Optional joint velocity disturbance.
            d_joint = joint_velocity_disturbance(
                t=t,
                n=len(qdot_cmd),
                scale=disturbance_scale,
            )
            qdot_send = np.clip(qdot_cmd + d_joint, -qdot_limit, qdot_limit)

            # 10. Send command to CoppeliaSim.
            sim_robot.set_joint_target_velocity(qdot_send)

            loop_t1 = time.perf_counter_ns()
            runtime_total_ms = (loop_t1 - loop_t0) / 1e6

            # 11. Logs.
            t_log.append(t)

            eO_ctrl_log.append(np.linalg.norm(O_ctrl))
            eT_ctrl_log.append(np.linalg.norm(T_ctrl))
            e_ctrl_log.append(pose_error_norm(O_ctrl, T_ctrl))

            eO_model_log.append(np.linalg.norm(O_model))
            eT_model_log.append(np.linalg.norm(T_model))
            e_model_log.append(pose_error_norm(O_model, T_model))

            # For sim evaluation, store the aligned sim tracking error as the main sim error.
            eO_sim_log.append(np.linalg.norm(O_sim_aligned))
            eT_sim_log.append(np.linalg.norm(T_sim_aligned))
            e_sim_log.append(pose_error_norm(O_sim_aligned, T_sim_aligned))

            eO_model_sim_raw_log.append(np.linalg.norm(O_model_sim_raw))
            eT_model_sim_raw_log.append(np.linalg.norm(T_model_sim_raw))
            eO_model_sim_aligned_log.append(np.linalg.norm(O_model_sim_aligned))
            eT_model_sim_aligned_log.append(np.linalg.norm(T_model_sim_aligned))

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
                    f"eT_ctrl={eT_ctrl_log[-1]:.5f} | "
                    f"eT_model={eT_model_log[-1]:.5f} | "
                    f"eT_sim_aligned={eT_sim_log[-1]:.5f} | "
                    f"model-sim-raw={eT_model_sim_raw_log[-1]:.5f} | "
                    f"model-sim-aligned={eT_model_sim_aligned_log[-1]:.5f} | "
                    f"max_qdot={np.max(np.abs(qdot_send)):.3f} | "
                    f"xi_diff={xi_diff:.3e} | "
                    f"DQ={runtime_dq_ms:.3f}ms | "
                    f"HDQ={runtime_hdq_ms:.3f}ms | "
                    f"loop={runtime_total_ms:.3f}ms"
                )

    finally:
        sim_robot.stop()

    # ========== Save results ==========
    os.makedirs("results/data", exist_ok=True)
    os.makedirs("results/plots", exist_ok=True)

    t_log = np.array(t_log)

    np.savez(
        "results/data/line_tracking_realtime_consistent_result.npz",
        time=t_log,
        eO_ctrl=np.array(eO_ctrl_log),
        eT_ctrl=np.array(eT_ctrl_log),
        e_ctrl=np.array(e_ctrl_log),
        eO_model=np.array(eO_model_log),
        eT_model=np.array(eT_model_log),
        e_model=np.array(e_model_log),
        eO_sim=np.array(eO_sim_log),
        eT_sim=np.array(eT_sim_log),
        e_sim=np.array(e_sim_log),
        eO_model_sim_raw=np.array(eO_model_sim_raw_log),
        eT_model_sim_raw=np.array(eT_model_sim_raw_log),
        eO_model_sim_aligned=np.array(eO_model_sim_aligned_log),
        eT_model_sim_aligned=np.array(eT_model_sim_aligned_log),
        qdot_cmd=np.array(qdot_cmd_log),
        qdot_send=np.array(qdot_send_log),
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
    plt.plot(t_log, eT_ctrl_log, label="controller translation error")
    plt.plot(t_log, eT_model_log, label="model tip vs target")
    plt.plot(t_log, eT_sim_log, label="aligned CoppeliaSim tip vs target")
    plt.plot(t_log, eT_model_sim_raw_log, label="model tip vs raw sim tip")
    plt.plot(t_log, eT_model_sim_aligned_log, label="model tip vs aligned sim tip")
    plt.xlabel("time [s]")
    plt.ylabel("translation error [m]")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/translation_errors_consistent.png", dpi=200)

    plt.figure()
    plt.plot(t_log, eO_ctrl_log, label="controller orientation error")
    plt.plot(t_log, eO_model_log, label="model orientation error")
    plt.plot(t_log, eO_sim_log, label="CoppeliaSim orientation error")
    plt.xlabel("time [s]")
    plt.ylabel("orientation error")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/orientation_errors_consistent.png", dpi=200)

    plt.figure()
    plt.semilogy(t_log, np.array(xi_diff_log) + 1e-20, label="||xi_DQ - xi_HDQ||")
    plt.xlabel("time [s]")
    plt.ylabel("twist difference, log scale")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/dq_hdq_twist_difference_log_consistent.png", dpi=200)

    plt.figure()
    plt.plot(t_log, runtime_dq_ms_log, label="DQ output time")
    plt.plot(t_log, runtime_hdq_ms_log, label="HDQ output time")
    plt.plot(t_log, runtime_total_ms_log, label="total loop time")
    plt.xlabel("time [s]")
    plt.ylabel("runtime [ms]")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/runtime_dq_hdq_consistent.png", dpi=200)

    plt.figure()
    plt.plot(t_log, [np.max(np.abs(v)) for v in qdot_send_log], label="max |qdot_send|")
    plt.xlabel("time [s]")
    plt.ylabel("rad/s")
    plt.grid(True)
    plt.legend()
    plt.savefig("results/plots/qdot_command_consistent.png", dpi=200)

    plt.show()

    print("\nFinished.")
    print("Mean DQ time  [ms] =", np.mean(runtime_dq_ms_log))
    print("Mean HDQ time [ms] =", np.mean(runtime_hdq_ms_log))
    print("Mean xi diff       =", np.mean(xi_diff_log))
    print("Mean eT_ctrl       =", np.mean(eT_ctrl_log))
    print("Mean eT_model      =", np.mean(eT_model_log))
    print("Mean eT_sim_aligned        =", np.mean(eT_sim_log))
    print("Mean model-sim raw eT      =", np.mean(eT_model_sim_raw_log))
    print("Mean model-sim aligned eT  =", np.mean(eT_model_sim_aligned_log))


if __name__ == "__main__":
    main()
