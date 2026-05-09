import time
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from core.dq_math import q_mul


class CoppeliaJointClient:
    def __init__(self, joint_paths):
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")

        self.joint_paths = joint_paths
        self.joints = [self.sim.getObject(path) for path in joint_paths]
        self.n = len(self.joints)

        print("Connected joints:")
        for path, h in zip(joint_paths, self.joints):
            print(path, "->", h, self.sim.getObjectAlias(h, 2))

    def start(self):
        try:
            self.sim.stopSimulation()
            time.sleep(0.5)
        except Exception:
            pass

        self.sim.startSimulation()

    def stop(self):
        self.set_joint_target_velocity(np.zeros(self.n))
        time.sleep(0.1)
        self.sim.stopSimulation()

    def read_q(self):
        return np.array(
            [self.sim.getJointPosition(h) for h in self.joints],
            dtype=float
        )

    def read_qdot(self):
        """
        读取仿真器返回的实际关节速度。
        这不是 q_dot_cmd，而是 CoppeliaSim 当前关节速度。
        """
        qdot = []

        for h in self.joints:
            try:
                qdot.append(self.sim.getJointVelocity(h))
            except Exception:
                qdot.append(np.nan)

        return np.array(qdot, dtype=float)

    def read_state(self):
        q = self.read_q()
        qdot = self.read_qdot()
        return q, qdot

    def set_joint_target_velocity(self, qdot_cmd):
        qdot_cmd = np.asarray(qdot_cmd, dtype=float).reshape(self.n)

        for h, v in zip(self.joints, qdot_cmd):
            self.sim.setJointTargetVelocity(h, float(v))

    def set_tip_and_base(self, tip_path, base_path=None):
        """
        tip_path: 末端对象路径，比如 /LBR4p/connection 或 /lbr4p_tip
        base_path: 基座对象路径。若为 None，则读取世界坐标系下的末端位姿。
        """
        self.tip = self.sim.getObject(tip_path)

        if base_path is None:
            self.base = -1
        else:
            self.base = self.sim.getObject(base_path)

        print("Tip object:", tip_path, "->", self.sim.getObjectAlias(self.tip, 2))
        if base_path is not None:
            print("Base object:", base_path, "->", self.sim.getObjectAlias(self.base, 2))
        else:
            print("Base object: world frame")

    def read_tip_position_quaternion(self):
        """
        返回 CoppeliaSim 中末端相对于 base 的真实位姿。
        position: [x, y, z]
        quaternion returned by CoppeliaSim is usually [x, y, z, w].
        """
        p = self.sim.getObjectPosition(self.tip, self.base)
        q_xyzw = self.sim.getObjectQuaternion(self.tip, self.base)

        p = np.array(p, dtype=float)
        q_xyzw = np.array(q_xyzw, dtype=float)

        return p, q_xyzw

    def read_tip_pose_dq(self):
        """
        把 CoppeliaSim 真实末端位姿转换为我们自己的 DQ 格式：
            x = r + eps * 1/2 * p * r

        我们内部 quaternion 顺序为 [w, x, y, z]。
        CoppeliaSim 常见返回为 [x, y, z, w]，所以要转换。
        """
        p, q_xyzw = self.read_tip_position_quaternion()

        # CoppeliaSim: [x, y, z, w]
        # internal:    [w, x, y, z]
        r = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=float)

        # 归一化，防止数值漂移
        r = r / np.linalg.norm(r)

        p_quat = np.r_[0.0, p]
        qd = 0.5 * q_mul(p_quat, r)

        x = np.r_[r, qd]
        return x