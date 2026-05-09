import os
import sys
import time
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from sim.joint_names import JOINT_PATHS_7R, TIP_PATH
from configs.kuka_like_7r import DH_TABLE
from core.robot_dh import SerialDHRobot


def mat12_to_T(m):
    """
    CoppeliaSim getObjectMatrix usually returns 12 numbers:
        [r11 r12 r13 x,
         r21 r22 r23 y,
         r31 r32 r33 z]
    """
    m = np.array(m, dtype=float).reshape(3, 4)
    T = np.eye(4)
    T[:3, :3] = m[:, :3]
    T[:3, 3] = m[:, 3]
    return T


def get_T(sim, obj, rel=-1):
    return mat12_to_T(sim.getObjectMatrix(obj, rel))


def print_T(name, T):
    print(f"\n{name}")
    print("p =", np.round(T[:3, 3], 6))
    print("R =")
    print(np.round(T[:3, :3], 6))


def main():
    client = RemoteAPIClient()
    sim = client.require("sim")

    robot = SerialDHRobot(DH_TABLE)

    try:
        sim.stopSimulation()
        time.sleep(0.3)
    except Exception:
        pass

    joints = [sim.getObject(p) for p in JOINT_PATHS_7R]
    tip = sim.getObject(TIP_PATH)

    # 你现在稳定的非奇异初始姿态
    q_safe = np.array([0.1, -0.6, 0.4, -0.9, 0.5, 0.6, 0.2], dtype=float)

    for h, qi in zip(joints, q_safe):
        sim.setJointPosition(h, float(qi))

    time.sleep(0.3)

    q_read = np.array([sim.getJointPosition(h) for h in joints], dtype=float)

    print("\n===== Joint paths =====")
    for i, (path, h) in enumerate(zip(JOINT_PATHS_7R, joints), start=1):
        print(i, path, "full =", sim.getObjectAlias(h, 2), "handle =", h)

    print("\nTIP_PATH =", TIP_PATH)
    print("TIP full =", sim.getObjectAlias(tip, 2))

    print("\nq_safe =", np.round(q_safe, 6))
    print("q_read =", np.round(q_read, 6))
    print("||q_read - q_safe|| =", np.linalg.norm(q_read - q_safe))

    # Python FK
    T_py = robot.fk_transform(q_read)
    p_py = T_py[:3, 3]

    print_T("Python FK T_py(base->tip)", T_py)

    # CoppeliaSim world transforms
    T_tip_world = get_T(sim, tip, -1)
    print_T("CoppeliaSim T_world_tip", T_tip_world)

    print("\n===== Joint world frames =====")
    for i, h in enumerate(joints, start=1):
        Tj = get_T(sim, h, -1)
        z_axis_world = Tj[:3, 2]
        print(f"\nJoint {i}: {sim.getObjectAlias(h, 2)}")
        print("origin world =", np.round(Tj[:3, 3], 6))
        print("z-axis world =", np.round(z_axis_world, 6))

    # 尝试几个候选 base：world、模型root、joint1、joint1 parent
    candidate_base_paths = [
        None,
        "/LBR4p",
        "/lbr4p_joint_1",
    ]

    # 如果 joint1 有 parent，也加入
    try:
        parent1 = sim.getObjectParent(joints[0])
        if parent1 != -1:
            parent1_path = sim.getObjectAlias(parent1, 2)
            candidate_base_paths.append(parent1_path)
    except Exception:
        pass

    print("\n===== Candidate base comparison =====")
    print("Python FK position p_py =", np.round(p_py, 6))

    for base_path in candidate_base_paths:
        try:
            if base_path is None:
                base_handle = -1
                label = "world(-1)"
            else:
                base_handle = sim.getObject(base_path)
                label = base_path

            T_tip_base = get_T(sim, tip, base_handle)
            p_sim = T_tip_base[:3, 3]

            print("\nbase =", label)
            print("p_sim_tip_in_base =", np.round(p_sim, 6))
            print("||p_py - p_sim|| =", np.linalg.norm(p_py - p_sim))

        except Exception as e:
            print("\nbase =", base_path, "failed:", e)

    print("\n===== Important judgement =====")
    print("如果所有候选base下 ||p_py-p_sim|| 都很大，说明 TIP_PATH 或 DH_TABLE/关节方向/零位不一致。")
    print("如果某个base下明显变小，说明 BASE_PATH 应该改成那个base。")


if __name__ == "__main__":
    main()