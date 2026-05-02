import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.robot_dh import SerialDHRobot
from src.errors import pose_error, pose_error_norm
from src.controllers import hinf_setpoint_control
from src.dq_math import integrate_pose_left_twist
from src.disturbances import disturbance_signals
from configs.kuka_like_7r import DH_TABLE


def run_one_case(
    gamma_O=1.0,
    gamma_T=1.0,
    disturbance_scale=1.0,
    dt=0.005,
    total_time=8.0,
    damping=1e-3,
    jacobian_method="numeric",
    save_prefix="disturbance"
):
    robot = SerialDHRobot(DH_TABLE)

    steps = int(total_time / dt)

    q = np.array([0.2, -0.4, 0.3, -0.7, 0.5, 0.3, -0.2], dtype=float)
    q_goal = np.array([0.7, -0.2, 0.6, -0.4, 0.3, 0.6, 0.1], dtype=float)

    xd = robot.fkm(q_goal)

    # Important:
    # x_nom is the nominal pose from FK(q)
    # x_real is the disturbed real end-effector pose.
    x_real = robot.fkm(q)

    time_log = []
    eO_log = []
    eT_log = []
    e_log = []
    qdot_norm_log = []

    # H∞ energy logs
    O_energy_log = []
    T_energy_log = []
    dO_energy_log = []
    dT_energy_log = []

    O_energy = 0.0
    T_energy = 0.0
    dO_energy = 0.0
    dT_energy = 0.0

    # Runtime logs, unit: milliseconds
    time_fk_log = []
    time_jac_log = []
    time_err_log = []
    time_ctrl_log = []
    time_int_log = []
    time_total_log = []

    for k in range(steps):
        t = k * dt
        step_t0 = time.perf_counter_ns()

        # 1. FK for nominal pose.
        t0 = time.perf_counter_ns()
        x_nom = robot.fkm(q)
        t1 = time.perf_counter_ns()

        # 2. Jacobian from nominal q.
        if jacobian_method == "numeric":
            J = robot.pose_jacobian_numeric(q)
        elif jacobian_method == "geometric":
            J = robot.pose_jacobian_geometric(q)
        elif jacobian_method == "hdq":
            J = robot.pose_jacobian_hdq(q)
        elif jacobian_method == "hdq_fast":
            J = robot.pose_jacobian_hdq_fast(q)
        else:
            raise ValueError(f"Unknown jacobian_method: {jacobian_method}")
    
        t2 = time.perf_counter_ns()

        # 3. Error uses disturbed real pose.
        O, T, x_tilde = pose_error(x_real, xd)
        t3 = time.perf_counter_ns()

        # 4. H∞ controller.
        q_dot, task_velocity, kO, kT = hinf_setpoint_control(
            J, O, T,
            gamma_O=gamma_O,
            gamma_T=gamma_T,
            damping=damping
        )
        t4 = time.perf_counter_ns()

        # 5. Disturbances.
        vw, vc = disturbance_signals(t, scale=disturbance_scale)

        # Nominal commanded spatial twist.
        xi_cmd = J @ q_dot

        # Actual disturbed spatial twist.
        xi_real = xi_cmd + vw + vc

        # 6. Integrate nominal joint state and real disturbed pose.
        q = q + q_dot * dt
        x_real = integrate_pose_left_twist(x_real, xi_real, dt)
        t5 = time.perf_counter_ns()

        # 7. H∞ energy accumulation.
        # Orientation output energy.
        O_energy += float(np.dot(O, O)) * dt

        # Translation output energy.
        T_energy += float(np.dot(T, T)) * dt

        # Orientation disturbance energy:
        # first 3 components of vw and vc.
        dO_energy += (
            float(np.dot(vw[:3], vw[:3])) +
            float(np.dot(vc[:3], vc[:3]))
        ) * dt

        # Translation disturbance energy:
        # last 3 components of vw and vc.
        dT_energy += (
            float(np.dot(vw[3:], vw[3:])) +
            float(np.dot(vc[3:], vc[3:]))
        ) * dt

        # 8. Logs.
        time_log.append(t)
        eO_log.append(np.linalg.norm(O))
        eT_log.append(np.linalg.norm(T))
        e_log.append(pose_error_norm(O, T))
        qdot_norm_log.append(np.linalg.norm(q_dot))

        O_energy_log.append(O_energy)
        T_energy_log.append(T_energy)
        dO_energy_log.append(dO_energy)
        dT_energy_log.append(dT_energy)

        step_t1 = time.perf_counter_ns()

        time_fk_log.append((t1 - t0) / 1e6)
        time_jac_log.append((t2 - t1) / 1e6)
        time_err_log.append((t3 - t2) / 1e6)
        time_ctrl_log.append((t4 - t3) / 1e6)
        time_int_log.append((t5 - t4) / 1e6)
        time_total_log.append((step_t1 - step_t0) / 1e6)

    # H∞ simulated attenuation.
    eps = 1e-12
    gamma_O_sim = np.sqrt(O_energy / (dO_energy + eps))
    gamma_T_sim = np.sqrt(T_energy / (dT_energy + eps))

    result = {
        "time": np.array(time_log),
        "eO": np.array(eO_log),
        "eT": np.array(eT_log),
        "e": np.array(e_log),
        "qdot_norm": np.array(qdot_norm_log),

        "O_energy": np.array(O_energy_log),
        "T_energy": np.array(T_energy_log),
        "dO_energy": np.array(dO_energy_log),
        "dT_energy": np.array(dT_energy_log),

        "gamma_O_sim": gamma_O_sim,
        "gamma_T_sim": gamma_T_sim,

        "time_fk_ms": np.array(time_fk_log),
        "time_jac_ms": np.array(time_jac_log),
        "time_err_ms": np.array(time_err_log),
        "time_ctrl_ms": np.array(time_ctrl_log),
        "time_int_ms": np.array(time_int_log),
        "time_total_ms": np.array(time_total_log),

        "kO": kO,
        "kT": kT,
        "gamma_O": gamma_O,
        "gamma_T": gamma_T,
        "disturbance_scale": disturbance_scale,
        "jacobian_method": jacobian_method
    }

    os.makedirs("results/plots", exist_ok=True)
    os.makedirs("results/data", exist_ok=True)

    np.savez(f"results/data/{save_prefix}.npz", **result)

    return result


def summarize_runtime(result):
    def stats(x):
        return np.mean(x), np.median(x), np.std(x), np.max(x)

    names = [
        ("FK", result["time_fk_ms"]),
        ("Jacobian", result["time_jac_ms"]),
        ("Error", result["time_err_ms"]),
        ("Control", result["time_ctrl_ms"]),
        ("Integrate", result["time_int_ms"]),
        ("Total", result["time_total_ms"]),
    ]

    print("\nRuntime per step [ms]")
    print("-----------------------------------------------")
    print(f"{'Block':<12} {'Mean':>10} {'Median':>10} {'Std':>10} {'Max':>10}")
    for name, arr in names:
        mean, median, std, maxv = stats(arr)
        print(f"{name:<12} {mean:10.4f} {median:10.4f} {std:10.4f} {maxv:10.4f}")


def plot_result(result, save_prefix="disturbance"):
    t = result["time"]

    plt.figure()
    plt.plot(t, result["eO"], label="orientation error ||O||")
    plt.plot(t, result["eT"], label="translation error ||T||")
    plt.plot(t, result["e"], label="combined error")
    plt.xlabel("time [s]")
    plt.ylabel("error")
    plt.legend()
    plt.grid(True)
    plt.savefig(f"results/plots/{save_prefix}_error.png", dpi=200)

    plt.figure()
    plt.plot(t, result["qdot_norm"], label="||q_dot||")
    plt.xlabel("time [s]")
    plt.ylabel("control input norm")
    plt.legend()
    plt.grid(True)
    plt.savefig(f"results/plots/{save_prefix}_control_norm.png", dpi=200)

    plt.figure()
    plt.plot(t, result["time_total_ms"], label="total runtime per step")
    plt.plot(t, result["time_jac_ms"], label="Jacobian runtime")
    plt.plot(t, result["time_ctrl_ms"], label="controller runtime")
    plt.xlabel("time [s]")
    plt.ylabel("runtime [ms]")
    plt.legend()
    plt.grid(True)
    plt.savefig(f"results/plots/{save_prefix}_runtime.png", dpi=200)

    plt.show()


def main():
    # You can change gamma here.
    gamma_O = 0.5
    gamma_T = 0.5

    result = run_one_case(
        gamma_O=gamma_O,
        gamma_T=gamma_T,
        disturbance_scale=1.0,
        dt=0.005,
        total_time=8.0,
        damping=1e-3,
        save_prefix="disturbance_gamma_1"
    )

    print("\nFinished disturbed H∞ simulation.")
    print("-----------------------------------------------")
    print(f"gamma_O theoretical = {result['gamma_O']:.4f}")
    print(f"gamma_T theoretical = {result['gamma_T']:.4f}")
    print(f"kO = {result['kO']:.4f}")
    print(f"kT = {result['kT']:.4f}")
    print(f"gamma_O_sim = {result['gamma_O_sim']:.4f}")
    print(f"gamma_T_sim = {result['gamma_T_sim']:.4f}")
    print(f"Final orientation error = {result['eO'][-1]:.6f}")
    print(f"Final translation error = {result['eT'][-1]:.6f}")

    summarize_runtime(result)
    plot_result(result, save_prefix="disturbance_gamma_1")


if __name__ == "__main__":
    main()