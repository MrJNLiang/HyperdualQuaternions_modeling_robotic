import numpy as np
from src.dq_math import dq_from_transform, dq_mul, dq_conj, dq_vec6
from src.hdq_math import hdq_identity, hdq_from_standard_dh, spatial_twist_from_hdq
from src.hdq_math import dq_identity, dq_standard_dh_and_derivative

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
    
    def fk_all_transforms(self, q):
        """
        Return all frame transforms from base to each frame.

        T_list[0] = base frame transform I
        T_list[i] = transform from base to frame i
        """
        q = np.asarray(q, dtype=float)
        T = np.eye(4)
        T_list = [T.copy()]

        for i, row in enumerate(self.dh_table):
            a, alpha, d, theta_offset, joint_type = row

            if int(joint_type) == 0:
                theta = theta_offset + q[i]
                di = d
            else:
                theta = theta_offset
                di = d + q[i]

            T = T @ dh_transform(a, alpha, di, theta)
            T_list.append(T.copy())

        return T_list

    def pose_jacobian_geometric(self, q):
        """
        Analytical/geometric Jacobian in the DQ twist convention used by the paper:

            xi = omega + eps * (p_dot + p x omega)

        For standard DH:
            revolute joint:
                omega = z_{i-1}
                dual_part = o_{i-1} x z_{i-1}

            prismatic joint:
                omega = 0
                dual_part = z_{i-1}
        """
        q = np.asarray(q, dtype=float)
        T_list = self.fk_all_transforms(q)

        J = np.zeros((6, self.n))

        for i, row in enumerate(self.dh_table):
            joint_type = int(row[4])

            T_prev = T_list[i]
            o = T_prev[:3, 3]
            z = T_prev[:3, 2]

            if joint_type == 0:
                omega = z
                dual_part = np.cross(o, z)
            else:
                omega = np.zeros(3)
                dual_part = z

            J[:, i] = np.r_[omega, dual_part]

        return J

    def hdq_fkm_with_qdot(self, q, q_dot):
        """
        HDQ forward kinematics.

        Returns:
            X = x + eps_star * x_dot
        """
        q = np.asarray(q, dtype=float)
        q_dot = np.asarray(q_dot, dtype=float)

        X = hdq_identity()

        for i, row in enumerate(self.dh_table):
            a, alpha, d, theta_offset, joint_type = row

            if int(joint_type) == 0:
                theta = theta_offset + q[i]
                theta_dot = q_dot[i]
                di = d
                d_dot = 0.0
            else:
                theta = theta_offset
                theta_dot = 0.0
                di = d + q[i]
                d_dot = q_dot[i]

            Xi = hdq_from_standard_dh(
                a=a,
                alpha=alpha,
                d=di,
                theta=theta,
                theta_dot=theta_dot,
                d_dot=d_dot
            )

            X = X * Xi

        return X

    def pose_jacobian_hdq(self, q):
        """
        HDQ automatic-differentiation Jacobian.

        For each column i:
            set q_dot = e_i
            compute X = x + eps_star * x_dot
            J_i = vec6(2 * x_dot * x^*)
        """
        q = np.asarray(q, dtype=float)
        J = np.zeros((6, self.n))

        for i in range(self.n):
            q_dot_seed = np.zeros(self.n)
            q_dot_seed[i] = 1.0

            X = self.hdq_fkm_with_qdot(q, q_dot_seed)
            J[:, i] = spatial_twist_from_hdq(X)

        return J
    
    def pose_jacobian_hdq_fast(self, q):
        """
        Fast HDQ/automatic-differentiation Jacobian.

        Instead of running one full HDQ-FK for each column,
        use the chain rule:

            x = X1 X2 ... Xn

            dx/dq_i = X1...X_{i-1} * dXi/dq_i * X_{i+1}...Xn

            J_i = vec6(2 * dx/dq_i * x^*)

        This is not finite difference.
        It is the expanded HDQ chain-rule derivative.
        """
        from src.dq_math import dq_mul, dq_conj, dq_vec6

        q = np.asarray(q, dtype=float)

        X_links = []
        dX_links = []

        for i, row in enumerate(self.dh_table):
            a, alpha, d, theta_offset, joint_type = row

            if int(joint_type) == 0:
                theta = theta_offset + q[i]
                di = d
            else:
                theta = theta_offset
                di = d + q[i]

            Xi, dXi = dq_standard_dh_and_derivative(
                a=a,
                alpha=alpha,
                d=di,
                theta=theta,
                joint_type=int(joint_type)
            )

            X_links.append(Xi)
            dX_links.append(dXi)

        # prefix[i] = X1 * X2 * ... * Xi
        # prefix[0] = identity before first link
        prefix = [dq_identity()]
        for i in range(self.n):
            prefix.append(dq_mul(prefix[-1], X_links[i]))

        # suffix[i] = X_i * X_{i+1} * ... * X_n
        # suffix[n] = identity after last link
        suffix = [None] * (self.n + 1)
        suffix[self.n] = dq_identity()
        for i in reversed(range(self.n)):
            suffix[i] = dq_mul(X_links[i], suffix[i + 1])

        x = prefix[self.n]
        x_conj = dq_conj(x)

        J = np.zeros((6, self.n))

        for i in range(self.n):
            dx_dqi = dq_mul(dq_mul(prefix[i], dX_links[i]), suffix[i + 1])

            xi_i = 2.0 * dq_mul(dx_dqi, x_conj)

            J[:, i] = dq_vec6(xi_i)

        return J