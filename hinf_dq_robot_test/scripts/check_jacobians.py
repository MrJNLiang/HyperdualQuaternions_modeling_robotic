import os
import sys
import time
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.robot_dh import SerialDHRobot
from configs.kuka_like_7r import DH_TABLE


def time_call(func, repeat=100):
    times = []
    out = None

    for _ in range(repeat):
        t0 = time.perf_counter_ns()
        out = func()
        t1 = time.perf_counter_ns()
        times.append((t1 - t0) / 1e6)

    return out, np.array(times)


def main():
    robot = SerialDHRobot(DH_TABLE)

    q = np.array([0.2, -0.4, 0.3, -0.7, 0.5, 0.3, -0.2], dtype=float)

    J_num, t_num = time_call(lambda: robot.pose_jacobian_numeric(q), repeat=200)
    J_geo, t_geo = time_call(lambda: robot.pose_jacobian_geometric(q), repeat=200)
    J_hdq, t_hdq = time_call(lambda: robot.pose_jacobian_hdq(q), repeat=200)

    print("\nJacobian comparison")
    print("-----------------------------------------")
    print("||J_num - J_geo|| =", np.linalg.norm(J_num - J_geo))
    print("||J_num - J_hdq|| =", np.linalg.norm(J_num - J_hdq))
    print("||J_geo - J_hdq|| =", np.linalg.norm(J_geo - J_hdq))

    print("\nMax absolute difference")
    print("-----------------------------------------")
    print("max |J_num - J_geo| =", np.max(np.abs(J_num - J_geo)))
    print("max |J_num - J_hdq| =", np.max(np.abs(J_num - J_hdq)))
    print("max |J_geo - J_hdq| =", np.max(np.abs(J_geo - J_hdq)))

    print("\nRuntime [ms]")
    print("-----------------------------------------")
    print(f"{'Method':<12} {'Mean':>10} {'Median':>10} {'Std':>10} {'Max':>10}")
    for name, arr in [
        ("numeric", t_num),
        ("geometric", t_geo),
        ("HDQ", t_hdq),
    ]:
        print(f"{name:<12} {np.mean(arr):10.5f} {np.median(arr):10.5f} {np.std(arr):10.5f} {np.max(arr):10.5f}")

    print("\nJ_numeric =")
    print(J_num)

    print("\nJ_geometric =")
    print(J_geo)

    print("\nJ_hdq =")
    print(J_hdq)


if __name__ == "__main__":
    main()