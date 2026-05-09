import time
import numpy as np

from core.dq_math import dq_from_transform, dq_mul, vec6_to_pure_dq
from core.hdq_math import HDQ


def skew(v):
    v = np.asarray(v, dtype=float).reshape(3)
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ], dtype=float)


def mat12_to_T(m):
    """
    CoppeliaSim getObjectMatrix returns 12 numbers:
        [r11 r12 r13 x,
         r21 r22 r23 y,
         r31 r32 r33 z]
    """
    m = np.asarray(m, dtype=float).reshape(3, 4)
    T = np.eye(4)
    T[:3, :3] = m[:, :3]
    T[:3, 3] = m[:, 3]
    return T


def get_T(sim, obj, rel=-1):
    return mat12_to_T(sim.getObjectMatrix(obj, rel))


def se3_exp(S, theta):
    """
    Spatial screw exponential.

    S = [omega_x, omega_y, omega_z, v_x, v_y, v_z]
    T(theta) = exp([S] theta)
    """
    S = np.asarray(S, dtype=float).reshape(6)
    w = S[:3]
    v = S[3:]

    T = np.eye(4)
    wn = np.linalg.norm(w)

    if wn < 1e-12:
        # prismatic joint
        T[:3, 3] = v * theta
        return T

    # ensure unit axis
    w = w / wn
    v = v / wn
    theta = theta * wn

    wx = skew(w)
    R = np.eye(3) + np.sin(theta) * wx + (1.0 - np.cos(theta)) * (wx @ wx)
    V = (
        np.eye(3) * theta
        + (1.0 - np.cos(theta)) * wx
        + (theta - np.sin(theta)) * (wx @ wx)
    )
    p = V @ v

    T[:3, :3] = R
    T[:3, 3] = p
    return T


def adjoint(T):
    """
    Spatial adjoint for twist order [omega; v].
    If V_b = [omega; v], then V_s = Ad_T V_b.
    """
    R = T[:3, :3]
    p = T[:3, 3]
    Ad = np.zeros((6, 6))
    Ad[:3, :3] = R
    Ad[3:, :3] = skew(p) @ R
    Ad[3:, 3:] = R
    return Ad


class CoppeliaPOEModel:
    """
    Kinematic model extracted from a CoppeliaSim serial manipulator.

    It uses Product of Exponentials (POE):
        T(q) = exp(S1*(q1-q_home1)) ... exp(Sn*(qn-q_homen)) M

    S_i is extracted from each CoppeliaSim joint frame at q_home:
        revolute spatial screw S_i = [z_i; -z_i x o_i] = [z_i; o_i x z_i]
    where o_i and z_i are expressed in the chosen base frame.

    The 6D velocity convention matches the DQ/H∞ convention:
        xi = [omega; p_dot + p x omega]
    which is the same as the spatial screw vector bottom part v.
    """

    def __init__(self, S_list, M, q_home, joint_paths=None, tip_path=None, base_label=None):
        self.S = np.asarray(S_list, dtype=float).T  # 6 x n
        self.M = np.asarray(M, dtype=float).reshape(4, 4)
        self.q_home = np.asarray(q_home, dtype=float).reshape(-1)
        self.n = self.q_home.size
        self.joint_paths = joint_paths or []
        self.tip_path = tip_path
        self.base_label = base_label

    @classmethod
    def from_coppelia(cls, sim, joint_handles, tip_handle, base_handle=-1,
                      q_home=None, settle_time=0.2, joint_paths=None,
                      tip_path=None, base_label=None):
        n = len(joint_handles)

        if q_home is not None:
            q_home = np.asarray(q_home, dtype=float).reshape(n)
            for h, qi in zip(joint_handles, q_home):
                sim.setJointPosition(h, float(qi))
            time.sleep(settle_time)

        q_actual = np.array([sim.getJointPosition(h) for h in joint_handles], dtype=float)

        S_list = []
        for h in joint_handles:
            Tj = get_T(sim, h, base_handle)
            o = Tj[:3, 3]
            z = Tj[:3, 2]
            z_norm = np.linalg.norm(z)
            if z_norm < 1e-12:
                raise ValueError("Joint axis norm is too small. Check joint frame.")
            z = z / z_norm

            # revolute joint spatial screw, order [omega; v]
            # v = -omega x o = o x omega
            v = np.cross(o, z)
            S_list.append(np.r_[z, v])

        M = get_T(sim, tip_handle, base_handle)

        return cls(
            S_list=S_list,
            M=M,
            q_home=q_actual,
            joint_paths=joint_paths,
            tip_path=tip_path,
            base_label=base_label,
        )

    def fk_transform(self, q):
        q = np.asarray(q, dtype=float).reshape(self.n)
        T = np.eye(4)
        for i in range(self.n):
            theta = q[i] - self.q_home[i]
            T = T @ se3_exp(self.S[:, i], theta)
        return T @ self.M

    def fkm(self, q):
        return dq_from_transform(self.fk_transform(q))

    def pose_jacobian_geometric(self, q):
        """
        Spatial Jacobian in DQ/H∞ convention, 6 x n.
        """
        q = np.asarray(q, dtype=float).reshape(self.n)
        J = np.zeros((6, self.n))
        T = np.eye(4)
        for i in range(self.n):
            J[:, i] = adjoint(T) @ self.S[:, i]
            theta = q[i] - self.q_home[i]
            T = T @ se3_exp(self.S[:, i], theta)
        return J

    def pose_jacobian_numeric(self, q, eps=1e-6):
        """
        Optional fallback numerical Jacobian based on POE FK.
        """
        from core.dq_math import dq_conj, dq_vec6

        q = np.asarray(q, dtype=float).reshape(self.n)
        x = self.fkm(q)
        x_conj = dq_conj(x)
        J = np.zeros((6, self.n))
        for i in range(self.n):
            dq_step = np.zeros(self.n)
            dq_step[i] = eps
            x_plus = self.fkm(q + dq_step)
            x_minus = self.fkm(q - dq_step)
            dx_dqi = (x_plus - x_minus) / (2.0 * eps)
            xi_i = 2.0 * dq_mul(dx_dqi, x_conj)
            J[:, i] = dq_vec6(xi_i)
        return J

    def hdq_fkm_with_qdot(self, q, qdot):
        """
        HDQ-compatible output:
            X = x + eps_star * x_dot

        This uses xi = J(q) qdot and x_dot = 1/2 xi x.
        It is a consistent velocity propagation for the extracted POE model.
        """
        q = np.asarray(q, dtype=float).reshape(self.n)
        qdot = np.asarray(qdot, dtype=float).reshape(self.n)
        x = self.fkm(q)
        J = self.pose_jacobian_geometric(q)
        xi_vec = J @ qdot
        xi_dq = vec6_to_pure_dq(xi_vec)
        x_dot = 0.5 * dq_mul(xi_dq, x)
        return HDQ(x, x_dot)

    def print_summary(self):
        print("\n===== CoppeliaPOEModel summary =====")
        print("n =", self.n)
        print("q_home =", np.round(self.q_home, 6))
        print("M position =", np.round(self.M[:3, 3], 6))
        print("S screws [omega; v] columns:")
        print(np.round(self.S, 6))
