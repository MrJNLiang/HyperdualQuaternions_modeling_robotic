import os
import sys
import time
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from sim.joint_names import JOINT_PATHS_7R


def main():
    client = RemoteAPIClient()
    sim = client.require("sim")

    # 如果之前仿真还没停，先停掉
    try:
        sim.stopSimulation()
        time.sleep(0.5)
    except Exception:
        pass

    joints = [sim.getObject(path) for path in JOINT_PATHS_7R]

    total_time = 8.0

    sim.setStepping(True)
    sim.startSimulation()

    try:
        while sim.getSimulationTime() < total_time:
            t = sim.getSimulationTime()

            q_des = np.array([
                0.40 * np.sin(0.5 * t),
                0.35 * np.sin(0.6 * t),
                0.30 * np.sin(0.7 * t),
                0.25 * np.sin(0.8 * t),
                0.20 * np.sin(0.9 * t),
                0.18 * np.sin(1.0 * t),
                0.15 * np.sin(1.1 * t),
            ])

            for h, qi in zip(joints, q_des):
                sim.setJointPosition(h, float(qi))

            sim.step()

            # 关键：让图形界面有时间刷新
            time.sleep(0.02)

        print("finished")

    finally:
        sim.stopSimulation()
        try:
            sim.setStepping(False)
        except Exception:
            pass


if __name__ == "__main__":
    main()