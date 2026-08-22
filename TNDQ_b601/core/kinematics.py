"""
TNDQ serial-chain forward kinematics -- paper Sec. 3.2 and Appendix B.

Chain product law (Proposition 1 / formula (3.4)):

    x_bar = prod_i x_bar_i(q_i, q_i_dot, q_i_ddot)

One O(n) chain product simultaneously outputs (x_hat, x_hat_dot, x_hat_ddot)
in the three TNDQ channels; derived quantities xi, xi_dot, Jdot*qdot follow
from formula (3.5) without ever forming Jdot explicitly.

Robot description: standard DH table rows [a, alpha, d, theta_offset, type]
(type 0 = revolute), KUKA LBR4+-like 7R (config/params.py).
"""

import numpy as np

from core.dq_algebra import (
    dq_mul, dq_conj, dq_vec6, dq_Ad,
    dq_rot_z, dq_rot_x, dq_trans_z, dq_trans_x, dq_identity,
)
from core.tndq_algebra import (
    TNDQ, twist_from_tndq, twist_dot_from_tndq, unit_constraint_residuals,
)

# Local screw axis of a revolute DH joint: rotation about the local z axis.
# As a pure DQ:  s = k (unit quaternion imaginary k), zero dual part.
_S_LOCAL_Z = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0], dtype=float)


def tndq_joint_factor_dh(a, alpha, d, theta, theta_dot, theta_ddot):
    """
    TNDQ representation of one revolute standard-DH joint factor
    (Appendix B.1).  The factor is A_i = Rz(theta) Tz(d) Tx(a) Rx(alpha);
    only Rz depends on the joint variable, with local screw axis s = k:

        d/dq Rz = (1/2) s Rz
    hence along the time curve (Appendix B.1 closed forms):
        Rz_dot  = (1/2) q_dot  s Rz
        Rz_ddot = (1/2) q_ddot s Rz + (1/4) q_dot^2 s^2 Rz

    The constant tail Tz Tx Rx enters as a TNDQ with zero derivative
    channels; the product follows formula (3.2).
    """
    Rz = dq_rot_z(theta)
    s_Rz = dq_mul(_S_LOCAL_Z, Rz)
    ss_Rz = dq_mul(_S_LOCAL_Z, s_Rz)

    Rz_dot = 0.5 * theta_dot * s_Rz
    Rz_ddot = 0.5 * theta_ddot * s_Rz + 0.25 * (theta_dot ** 2) * ss_Rz

    rot_bar = TNDQ(Rz, Rz_dot, Rz_ddot)

    # constant link transform Tz(d) Tx(a) Rx(alpha)
    tail = dq_mul(dq_mul(dq_trans_z(d), dq_trans_x(a)), dq_rot_x(alpha))
    return rot_bar * TNDQ.from_constant(tail)


class TNDQSerialChain:
    """
    7R serial manipulator (KUKA LBR4+-like) modeled as a TNDQ chain.

    dh_table rows: [a, alpha, d, theta_offset, joint_type], type 0 = revolute.
    """

    def __init__(self, dh_table):
        self.dh_table = np.asarray(dh_table, dtype=float)
        self.n = len(self.dh_table)
        for row in self.dh_table:
            if int(row[4]) != 0:
                raise NotImplementedError("Only revolute joints (7R chain) are supported.")

    # -- TNDQ chain FK  (formula (3.4)) --------------------------------------

    def fk_tndq(self, q, q_dot=None, q_ddot=None):
        """
        Implementation of formula (3.4):  x_bar = prod_i x_bar_i.
        Returns the full TNDQ (channels: x, x_dot, x_ddot).
        Setting q_ddot = 0 makes the vec6 of xi_dot equal Jdot*qdot alone
        (formula (3.5), remark below it).
        """
        q = np.asarray(q, dtype=float).reshape(self.n)
        q_dot = np.zeros(self.n) if q_dot is None else np.asarray(q_dot, dtype=float).reshape(self.n)
        q_ddot = np.zeros(self.n) if q_ddot is None else np.asarray(q_ddot, dtype=float).reshape(self.n)

        x_bar = TNDQ.identity()
        for i, row in enumerate(self.dh_table):
            a, alpha, d, theta_offset, _ = row
            x_bar_i = tndq_joint_factor_dh(
                a=a, alpha=alpha, d=d,
                theta=theta_offset + q[i],
                theta_dot=q_dot[i],
                theta_ddot=q_ddot[i],
            )
            x_bar = x_bar * x_bar_i        # TNDQ product (3.2)
        return x_bar

    def fkm(self, q):
        """Pose-only FK: sigma^0 channel of the chain (paper Sec. 2.2)."""
        return self.fk_tndq(q).to_dq()

    # -- geometric Jacobian ---------------------------------------------------

    def jacobian(self, q):
        """
        Geometric Jacobian J in the DQ twist convention (paper Sec. 2.2):
            vec6 xi = J(q) q_dot.
        Column i is the joint screw transported to the base frame by the
        adjoint of the pose prefix:  J_i = vec6( Ad_{x_1...x_{i-1} Rz_i^-} s_i ).
        Since s = k commutes with Rz, the prefix *before* joint i suffices.
        All operations stay inside DQ algebra ([P1] prefix structure, Sec. 6.2).
        """
        q = np.asarray(q, dtype=float).reshape(self.n)
        J = np.zeros((6, self.n))
        prefix = dq_identity()
        for i, row in enumerate(self.dh_table):
            a, alpha, d, theta_offset, _ = row
            # screw axis of joint i transported by the chain prefix
            J[:, i] = dq_vec6(dq_Ad(prefix, _S_LOCAL_Z))
            Rz = dq_rot_z(theta_offset + q[i])
            tail = dq_mul(dq_mul(dq_trans_z(d), dq_trans_x(a)), dq_rot_x(alpha))
            prefix = dq_mul(prefix, dq_mul(Rz, tail))
        return J

    # -- full FK output bundle ------------------------------------------------

    def fk_outputs(self, q, q_dot, q_ddot=None, with_jacobian=True):
        """
        One-pass kinematic pipeline (paper Sec. 6.2, forward-kinematics layer):

            measured chain:  x_bar = prod x_bar_i(q, q_dot, q_ddot=0)
                             -> x, x_dot, xi = 2 x_dot x*        (3.5)
                             -> Jdot*qdot = vec6(xi_dot)|_{qddot=0}  (3.5)
            constraint residuals c0, c1, c2 monitored per (3.8).

        If q_ddot is given, xi_dot contains the full Jdot qdot + J qddot.
        """
        q_dot = np.asarray(q_dot, dtype=float).reshape(self.n)

        # chain with qddot = 0: sigma^2 channel yields Jdot*qdot directly (3.5)
        x_bar0 = self.fk_tndq(q, q_dot, None)
        Jdot_qdot = dq_vec6(twist_dot_from_tndq(x_bar0))

        if q_ddot is None:
            x_bar = x_bar0
            xi_dot_vec = Jdot_qdot
        else:
            x_bar = self.fk_tndq(q, q_dot, q_ddot)
            xi_dot_vec = dq_vec6(twist_dot_from_tndq(x_bar))

        xi = twist_from_tndq(x_bar)
        c0, c1, c2 = unit_constraint_residuals(x_bar)

        out = {
            "x_bar": x_bar,                       # TNDQ representation (3.3a)
            "x_breve": x_bar.to_hdq(),            # HDQ truncation (3.6), lossless (Prop. 2)
            "x": x_bar.to_dq(),                   # pose DQ (sigma^0 channel)
            "x_dot": np.array(x_bar.ch[1]),
            "xi": dq_vec6(xi),                    # vec6 twist (3.5)
            "xi_dot": xi_dot_vec,                 # vec6(xi_dot) = Jdot qdot + J qddot (3.5)
            "Jdot_qdot": Jdot_qdot,               # read out free of explicit Jdot (3.5)
            "c0": c0, "c1": c1, "c2": c2,         # constraint residuals (3.8)
        }
        if with_jacobian:
            out["J"] = self.jacobian(q)
        return out
