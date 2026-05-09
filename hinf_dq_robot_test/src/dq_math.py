import numpy as np


def q_mul(a, b):
    """
    Quaternion multiplication.
    q = [w, x, y, z]
    """
    aw, ax, ay, az = a
    bw, bx, by, bz = b

    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw
    ], dtype=float)


def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def q_normalize(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("Quaternion norm too small")
    return q / n


def q_from_rotm(R):
    """
    Rotation matrix to unit quaternion [w, x, y, z].
    """
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)

    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

    return q_normalize(np.array([w, x, y, z], dtype=float))


def dq_mul(a, b):
    """
    Dual quaternion multiplication.
    dq = [qr(4), qd(4)]
    """
    ar = a[:4]
    ad = a[4:]
    br = b[:4]
    bd = b[4:]

    real = q_mul(ar, br)
    dual = q_mul(ar, bd) + q_mul(ad, br)
    return np.r_[real, dual]


def dq_conj(x):
    """
    Dual quaternion conjugate used for rigid pose inverse
    when x is unit dual quaternion.
    """
    return np.r_[q_conj(x[:4]), q_conj(x[4:])]


def dq_from_rt(R, p):
    """
    Build pose dual quaternion:
    x = r + eps * 1/2 * p * r
    where p is pure quaternion [0, px, py, pz].
    This follows the convention used in the H∞ paper.
    """
    r = q_from_rotm(R)
    p_quat = np.r_[0.0, np.asarray(p, dtype=float)]
    qd = 0.5 * q_mul(p_quat, r)
    return np.r_[r, qd]


def dq_from_transform(T):
    R = T[:3, :3]
    p = T[:3, 3]
    return dq_from_rt(R, p)


def dq_translation(x):
    """
    For x = r + eps * 1/2 * p * r,
    p = 2 * qd * r_conj.
    """
    r = x[:4]
    qd = x[4:]
    p_quat = 2.0 * q_mul(qd, q_conj(r))
    return p_quat[1:4]


def dq_normalize_pose(x):
    """
    Normalize real quaternion part and keep pose structure approximately.
    """
    r = q_normalize(x[:4])
    return np.r_[r, x[4:]]


def dq_vec6(xi):
    """
    Pure dual quaternion twist xi = omega + eps * v.
    Return [omega_x, omega_y, omega_z, v_x, v_y, v_z].
    """
    return np.r_[xi[1:4], xi[5:8]]


def vec6_to_pure_dq(v):
    """
    Convert 6D twist vector to pure dual quaternion.

    v = [wx, wy, wz, vx, vy, vz]

    xi = omega + eps * velocity
       = [0, wx, wy, wz] + eps [0, vx, vy, vz]
    """
    v = np.asarray(v, dtype=float).reshape(6)
    return np.array([
        0.0, v[0], v[1], v[2],
        0.0, v[3], v[4], v[5]
    ], dtype=float)


def dq_pose_normalize(x):
    """
    Normalize a pose dual quaternion.

    x = r + eps * 1/2 * p * r

    After Euler integration, numerical drift appears.
    We normalize r and reconstruct the dual part from the current translation.
    """
    r = q_normalize(x[:4])
    qd = x[4:]

    p_quat = 2.0 * q_mul(qd, q_conj(r))
    p = p_quat[1:4]

    qd_new = 0.5 * q_mul(np.r_[0.0, p], r)

    return np.r_[r, qd_new]


def integrate_pose_left_twist(x, xi_vec6, dt):
    """
    Integrate:
        x_dot = 1/2 * xi * x

    where xi is a pure dual quaternion represented by vec6:
        xi_vec6 = [omega, velocity]

    This matches the spatial/inertial twist convention used in the H∞ paper.
    """
    xi = vec6_to_pure_dq(xi_vec6)
    x_dot = 0.5 * dq_mul(xi, x)
    x_next = x + dt * x_dot
    return dq_pose_normalize(x_next)
