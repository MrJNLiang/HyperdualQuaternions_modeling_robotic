import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# Allow importing src from project root
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.robot_dh import SerialDHRobot
from src.errors import pose_error, pose_error_norm
from src.controllers import hinf_setpoint_control
from configs.kuka_like_7r import DH_TABLE


def main():
    robot = SerialDHRobot(DH_TABLE)

    dt = 0.005
    total_time = 8.0
    steps = int(total_time / dt)

    # Initial joint configuration
    q = np.array([0.2, -0.4, 0.3, -0.7, 0.5, 0.3, -0.2], dtype=float)

    # Goal joint configuration.
    # We use its FK as desired pose xd.
    q_goal = np.array([0.7, -0.2, 0.1, -0.4, 0.3, 0.6, 0.1], dtype=float)
    xd = robot.fkm(q_goal)

    gamma_O = 1.0
    gamma_T = 1.0
    damping = 1e-3

    time_log = []
    eO_log = []
    eT_log = []
    e_log = []
    qdot_norm_log = []
    q_log = []

    for k in range(steps):
        t = k * dt

        x = robot.fkm(q)
        J = robot.pose_jacobian_numeric(q)

        O, T, x_tilde = pose_error(x, xd)

        q_dot, task_velocity, kO, kT = hinf_setpoint_control(
            J, O, T,
            gamma_O=gamma_O,
            gamma_T=gamma_T,
            damping=damping
        )

        # Simple Euler integration
        q = q + q_dot * dt

        time_log.append(t)
        eO_log.append(np.linalg.norm(O))
        eT_log.append(np.linalg.norm(T))
        e_log.append(pose_error_norm(O, T))
        qdot_norm_log.append(np.linalg.norm(q_dot))
        q_log.append(q.copy())

    os.makedirs("results/plots", exist_ok=True)
    os.makedirs("results/data", exist_ok=True)

    time_log = np.array(time_log)
    eO_log = np.array(eO_log)
    eT_log = np.array(eT_log)
    e_log = np.array(e_log)
    qdot_norm_log = np.array(qdot_norm_log)

    np.savez(
        "results/data/setpoint_result.npz",
        time=time_log,
        eO=eO_log,
        eT=eT_log,
        e=e_log,
        qdot_norm=qdot_norm_log,
        q=np.array(q_log)
    )

    plt.figure()
    plt.plot(time_log, eO_log, label="orientation error ||O||")
    plt.plot(time_log, eT_log, label="translation error ||T||")
    plt.plot(time_log, e_log, label="combined error")
    plt.xlabel("time [s]")
    plt.ylabel("error")
    plt.legend()
    plt.grid(True)
    plt.savefig("results/plots/setpoint_error.png", dpi=200)

    plt.figure()
    plt.plot(time_log, qdot_norm_log, label="||q_dot||")
    plt.xlabel("time [s]")
    plt.ylabel("control input norm")
    plt.legend()
    plt.grid(True)
    plt.savefig("results/plots/setpoint_control_norm.png", dpi=200)

    plt.show()

    print("Finished.")
    print(f"kO={kO:.4f}, kT={kT:.4f}")
    print(f"Final orientation error: {eO_log[-1]:.6f}")
    print(f"Final translation error: {eT_log[-1]:.6f}")
    print(f"Final combined error: {e_log[-1]:.6f}")


if __name__ == "__main__":
    main()