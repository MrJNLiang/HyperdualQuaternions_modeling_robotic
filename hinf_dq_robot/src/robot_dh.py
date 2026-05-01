import numpy as np
from src.dq_math import dq_from_transform, dq_mul, dq_conj, dq_vec6


def dh_transform(a, alpha, d, theta):
    """
    Standard DH transform.
    """
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    ct = np.cos(theta)
    st = np.sin(theta)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,      ca,      d],
        [0.0,    0.0,     0.0,    1.0]
    ], dtype=float)


class SerialDHRobot:
    """
    Simple serial manipulator using standard DH parameters.

    Each row:
    [a, alpha, d, theta_offset, joint_type]

    joint_type:
    0 = revolute
    1 = prismatic
    """

    def __init__(self, dh_table):
        self.dh_table = dh_table
        self.n = len(dh_table)

    def fk_transform(self, q):
        T = np.eye(4)

        for i, row in enumerate(self.dh_table):
            a, alpha, d, theta_offset, joint_type = row

            if int(joint_type) == 0:
                theta = theta_offset + q[i]
                di = d
            else:
                theta = theta_offset
                di = d + q[i]

            T = T @ dh_transform(a, alpha, di, theta)

        return T

    def fkm(self, q):
        """
        Forward kinematics as unit dual quaternion.
        """
        T = self.fk_transform(q)
        return dq_from_transform(T)

    def pose_jacobian_numeric(self, q, eps=1e-6):
        """
        Numerical analytical Jacobian in DQ form.

        Since the paper uses:
            x_dot = 1/2 * xi * x
        then:
            xi_i = 2 * (dx/dq_i) * x^*
        """
        q = np.asarray(q, dtype=float)
        x = self.fkm(q)
        x_conj = dq_conj(x)

        J = np.zeros((6, self.n))

        for i in range(self.n):
            dq_step = np.zeros(self.n)
            dq_step[i] = eps

            x_plus = self.fkm(q + dq_step)
            x_minus = self.fkm(q - dq_step)

            dx_dqi = (x_plus - x_minus) / (2.0 * eps)

            # xi_i = 2 * dx_dqi * x^*
            xi_i = 2.0 * dq_mul(dx_dqi, x_conj)

            J[:, i] = dq_vec6(xi_i)

        return J