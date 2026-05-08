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

    try:
        sim.stopSimulation()
        time.sleep(0.5)
    except Exception:
        pass

    h = sim.getObject(JOINT_PATHS_7R[0])

    print("joint =", sim.getObjectAlias(h, 2))

    for angle in [0.0, 0.3, -0.3, 0.6, -0.6, 0.0]:
        sim.setJointPosition(h, angle)
        time.sleep(1.0)
        q = sim.getJointPosition(h)
        print("set =", angle, "read =", q)

    print("finished")


if __name__ == "__main__":
    main()