import os
import sys
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from sim.joint_names import JOINT_PATHS_7R


def main():
    client = RemoteAPIClient()
    sim = client.require("sim")

    joints = [sim.getObject(path) for path in JOINT_PATHS_7R]

    dt = 0.01
    total_time = 10.0

    # 关节空间PD控制参数
    kp = 2.0
    max_qdot = 0.8

    sim.setStepping(True)
    sim.setSimulationTimeStep(dt)
    sim.startSimulation()

    try:
        while sim.getSimulationTime() < total_time:
            t = sim.getSimulationTime()

            q = np.array([sim.getJointPosition(h) for h in joints], dtype=float)

            q_des = np.array([
                0.40 * np.sin(0.5 * t),
                0.35 * np.sin(0.6 * t),
                0.30 * np.sin(0.7 * t),
                0.25 * np.sin(0.8 * t),
                0.20 * np.sin(0.9 * t),
                0.18 * np.sin(1.0 * t),
                0.15 * np.sin(1.1 * t),
            ])

            # 简单速度控制：q_dot_cmd = kp * (q_des - q)
            q_dot_cmd = kp * (q_des - q)
            q_dot_cmd = np.clip(q_dot_cmd, -max_qdot, max_qdot)

            for h, vi in zip(joints, q_dot_cmd):
                sim.setJointTargetVelocity(h, float(vi))

            sim.step()

            if int(t / dt) % 100 == 0:
                try:
                    qdot = np.array([sim.getJointVelocity(h) for h in joints], dtype=float)
                except Exception:
                    qdot = np.zeros(7)

                print("t =", round(t, 3))
                print("q =", np.round(q, 3))
                print("q_des =", np.round(q_des, 3))
                print("q_dot_cmd =", np.round(q_dot_cmd, 3))
                print("q_dot_meas =", np.round(qdot, 3))
                print()

    finally:
        for h in joints:
            sim.setJointTargetVelocity(h, 0.0)
        sim.stopSimulation()


if __name__ == "__main__":
    main()