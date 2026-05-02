import numpy as np
from src.dq_math import dq_mul, dq_conj, dq_translation


def pose_error(x, xd):
    """
    Right-invariant spatial error:
        x_tilde = x * xd^*

    z_tilde = 1 - x_tilde.

    For first version, assume no unwinding problem:
        O(z) = Im(z_real) = -vec(r_tilde)
        T(z) = p_tilde

    Returns:
        O: 3D orientation error
        T: 3D translation error
        x_tilde: dual quaternion error
    """
    x_tilde = dq_mul(x, dq_conj(xd))

    r_tilde = x_tilde[:4]
    p_tilde = dq_translation(x_tilde)

    # z = 1 - x_tilde
    # real part of z is 1 - r_tilde
    # imaginary part O(z) = -vector part of r_tilde
    O = -r_tilde[1:4]

    # From paper: T(z) = p_tilde
    T = p_tilde

    return O, T, x_tilde


def pose_error_norm(O, T):
    return np.sqrt(np.dot(O, O) + np.dot(T, T))

