import numpy as np
from core.dq_math import q_mul, dq_mul, dq_conj, dq_vec6


def dq_zero():
    return np.zeros(8, dtype=float)


def dq_identity():
    return np.array([1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0], dtype=float)


def dq_trans_x(a):
    return np.array([1.0, 0.0, 0.0, 0.0,
                     0.0, a / 2.0, 0.0, 0.0], dtype=float)


def dq_trans_z(d):
    return np.array([1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, d / 2.0], dtype=float)


def dq_trans_z_dot(d_dot):
    """
    Time derivative of Tz(d).
    Only dual z component changes.
    """
    return np.array([0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, d_dot / 2.0], dtype=float)


def dq_rot_x(alpha):
    c = np.cos(alpha / 2.0)
    s = np.sin(alpha / 2.0)
    return np.array([c, s, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0], dtype=float)


def dq_rot_z(theta):
    c = np.cos(theta / 2.0)
    s = np.sin(theta / 2.0)
    return np.array([c, 0.0, 0.0, s,
                     0.0, 0.0, 0.0, 0.0], dtype=float)


def dq_rot_z_dot(theta, theta_dot):
    """
    Time derivative of Rz(theta).
    r = [cos(theta/2), 0, 0, sin(theta/2)]
    """
    c = np.cos(theta / 2.0)
    s = np.sin(theta / 2.0)

    return np.array([
        -0.5 * s * theta_dot,
         0.0,
         0.0,
         0.5 * c * theta_dot,
         0.0,
         0.0,
         0.0,
         0.0
    ], dtype=float)


class HDQ:
    """
    Minimal hyper-dual quaternion object:

        X = dq + eps_star * hd

    where:
        dq = ordinary dual quaternion pose
        hd = time derivative of dq
    """

    def __init__(self, dq, hd=None):
        self.dq = np.asarray(dq, dtype=float).reshape(8)

        if hd is None:
            self.hd = np.zeros(8, dtype=float)
        else:
            self.hd = np.asarray(hd, dtype=float).reshape(8)

    def __mul__(self, other):
        return HDQ(
            dq_mul(self.dq, other.dq),
            dq_mul(self.dq, other.hd) + dq_mul(self.hd, other.dq)
        )

    def conj(self):
        return HDQ(dq_conj(self.dq), dq_conj(self.hd))


def hdq_identity():
    return HDQ(dq_identity(), dq_zero())


def hdq_from_standard_dh(a, alpha, d, theta, theta_dot=0.0, d_dot=0.0):
    """
    Standard DH:
        A_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)

    HDQ version:
        X_i = A_i + eps_star * A_i_dot

    Only theta and d can have derivatives.
    """

    Rz = HDQ(dq_rot_z(theta), dq_rot_z_dot(theta, theta_dot))
    Tz = HDQ(dq_trans_z(d), dq_trans_z_dot(d_dot))
    Tx = HDQ(dq_trans_x(a), dq_zero())
    Rx = HDQ(dq_rot_x(alpha), dq_zero())

    return Rz * Tz * Tx * Rx


def spatial_twist_from_hdq(X):
    """
    Given:
        X = x + eps_star * x_dot

    For the H∞ paper convention:
        x_dot = 1/2 * xi * x

    Therefore:
        xi = 2 * x_dot * x^*
    """
    xi = 2.0 * dq_mul(X.hd, dq_conj(X.dq))
    return dq_vec6(xi)

def dq_standard_dh_and_derivative(a, alpha, d, theta, joint_type):
    """
    Return:
        Xi      : DQ transform of one standard-DH link
        dXi_dq  : derivative of Xi w.r.t. its joint variable

    Standard DH:
        Xi = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)

    joint_type:
        0 = revolute, derivative w.r.t. theta
        1 = prismatic, derivative w.r.t. d
    """
    Rz = dq_rot_z(theta)
    Tz = dq_trans_z(d)
    Tx = dq_trans_x(a)
    Rx = dq_rot_x(alpha)

    Xi = dq_mul(dq_mul(dq_mul(Rz, Tz), Tx), Rx)

    if int(joint_type) == 0:
        dRz = dq_rot_z_dot(theta, 1.0)
        dXi = dq_mul(dq_mul(dq_mul(dRz, Tz), Tx), Rx)
    else:
        dTz = dq_trans_z_dot(1.0)
        dXi = dq_mul(dq_mul(dq_mul(Rz, dTz), Tx), Rx)

    return Xi, dXi