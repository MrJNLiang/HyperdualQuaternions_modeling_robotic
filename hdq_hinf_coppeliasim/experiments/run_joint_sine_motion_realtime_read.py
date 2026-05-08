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

    # 先停止仿真，避免上一次程序残留
    try:
        sim.stopSimulation()
        time.sleep(0.5)
    except Exception:
        pass

    joints = [sim.getObject(path) for path in JOINT_PATHS_7R]

    print("Joint paths:")
    for path, h in zip(JOINT_PATHS_7R, joints):
        print(path, "->", h, sim.getObjectAlias(h, 2))

    total_time = 12.0

    # 这里先不用 setStepping，避免界面像卡住
    sim.startSimulation()

    q_prev = None
    t_prev = None

    try:
        t_start = time.time()

        while time.time() - t_start < total_time:
            # 用真实墙钟时间生成轨迹，更容易看见运动
            t = time.time() - t_start

            # 增大幅度，单位是 rad
            q_des = np.array([
                0.90 * np.sin(0.8 * t),
                0.70 * np.sin(0.7 * t),
                0.70 * np.sin(0.6 * t),
                0.60 * np.sin(0.9 * t),
                0.50 * np.sin(1.0 * t),
                0.45 * np.sin(1.1 * t),
                0.40 * np.sin(1.2 * t),
            ])

            # 直接设置关节角：这是运动学测试方式
            for h, qi in zip(joints, q_des):
                sim.setJointPosition(h, float(qi))

            # 实时读取关节角
            q_read = np.array([sim.getJointPosition(h) for h in joints], dtype=float)

            # 用差分估计关节速度，更稳，不依赖 getJointVelocity
            if q_prev is not None:
                dt = t - t_prev
                qdot_est = (q_read - q_prev) / max(dt, 1e-9)
            else:
                qdot_est = np.zeros_like(q_read)

            q_prev = q_read.copy()
            t_prev = t

            # 每0.2秒打印一次
            if int(t * 5) != int((t - 0.03) * 5):
                print("\n t =", round(t, 3))
                print(" q_des  =", np.round(q_des, 3))
                print(" q_read =", np.round(q_read, 3))
                print(" qdot_est =", np.round(qdot_est, 3))
                print(" error  =", np.round(q_des - q_read, 4))

            time.sleep(0.03)

    finally:
        sim.stopSimulation()


if __name__ == "__main__":
    main()