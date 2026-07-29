"""
Unit tests for the key mathematical properties of the TNDQ framework.

Coverage (paper reference in brackets):
    T1  TNDQ ring axioms: sigma^3 = 0 nilpotency, associativity   [Def. 1, (3.2)]
    T2  Proposition 1: chain product = derivatives of the product [(3.4)]
    T3  Proposition 2: lossless truncation TNDQ -> HDQ -> DQ      [(3.6)/(3.7)]
    T4  Twist / twist-rate identities and Jdot*qdot readout       [(3.5)]
    T5  Unit-constraint residual family at machine precision      [(3.8)]
    T6  Theorem 1: xi_tilde purity, sign invariance, (4.4)
    T7  Theorem 2: output error kinematics e_z_dot = A e_xi       [(4.5)]
    T8  Lemma 1: transport rule d/dt(Ad_{x_tilde} xi_d)           [(5.3)/(5.4)]
    T9  Reprojection restores the constraint family               [Sec. 3.4]

Numerical differentiation only appears on the *reference* side of each
comparison (central differences); the TNDQ side is purely algebraic.

Run from the TNDQ_sim root:
    python -m tests.test_math_properties        # standalone
    python -m pytest tests/ -q                  # or with pytest
"""

import numpy as np

from core.dq_algebra import (
    dq_mul, dq_conj, dq_identity, dq_vec6, dq_Ad, dq_ad, vec6_to_pure_dq,
)
from core.tndq_algebra import (
    TNDQ, twist_from_tndq, twist_dot_from_tndq,
    unit_constraint_residuals, reproject_tndq,
)
from core.kinematics import TNDQSerialChain
from control.error_system import (
    hdq_error, twist_error_from_hdq, output_error, A_matrix, full_error_state,
)
from config.params import KUKA_LBR4_DH, N_JOINTS
from simdata.input_simulation import default_joint_sine_7r
from simdata.trajectory_generator import LineTrajectoryTNDQ

RNG = np.random.default_rng(42)
CHAIN = TNDQSerialChain(KUKA_LBR4_DH)
SIGNAL = default_joint_sine_7r()


def _random_tndq():
    """Random (non-unit) TNDQ element for pure ring-axiom tests."""
    return TNDQ(*RNG.standard_normal((3, 8)))


# ---------------------------------------------------------------------------
# T1 -- ring axioms of A2 = H_hat[sigma]/(sigma^3)   (Definition 1, (3.2))
# ---------------------------------------------------------------------------

def test_sigma_nilpotency():
    """sigma * sigma * sigma = 0 in A2  (sigma^3 = 0)."""
    zero = np.zeros(8)
    sigma = TNDQ(zero, dq_identity(), zero)      # element "sigma"
    s3 = sigma * sigma * sigma
    assert np.max(np.abs(s3.ch)) == 0.0
    # sigma^2 = 2 * (1/2 sigma^2) survives: channel 2 = 2 * identity
    s2 = sigma * sigma
    assert np.allclose(s2.ch[2], 2.0 * dq_identity())
    assert np.max(np.abs(s2.ch[:2])) == 0.0


def test_tndq_associativity():
    """(a b) c = a (b c) -- A2 is an associative algebra (Sec. 3.1)."""
    a, b, c = _random_tndq(), _random_tndq(), _random_tndq()
    lhs = (a * b) * c
    rhs = a * (b * c)
    assert np.allclose(lhs.ch, rhs.ch, atol=1e-12)


# ---------------------------------------------------------------------------
# T2 -- Proposition 1: chain product law (3.4)
# ---------------------------------------------------------------------------

def test_proposition1_chain_vs_numeric_derivatives():
    """
    The TNDQ chain product (3.4) must equal the time derivatives of the
    pose-only product x(t) = prod_i x_i(q_i(t)) obtained by central
    differences.  This validates that one O(n) product propagates
    (x, x_dot, x_ddot) exactly.
    """
    t0, h = 1.234, 1e-5

    def pose(t):
        q, _, _ = SIGNAL(t)
        return CHAIN.fkm(q)

    q, q_dot, q_ddot = SIGNAL(t0)
    x_bar = CHAIN.fk_tndq(q, q_dot, q_ddot)

    x_num = pose(t0)
    x_dot_num = (pose(t0 + h) - pose(t0 - h)) / (2 * h)
    x_ddot_num = (pose(t0 + h) - 2 * pose(t0) + pose(t0 - h)) / h ** 2

    assert np.allclose(x_bar.ch[0], x_num, atol=1e-12)
    assert np.allclose(x_bar.ch[1], x_dot_num, atol=1e-7)
    assert np.allclose(x_bar.ch[2], x_ddot_num, atol=1e-4)


# ---------------------------------------------------------------------------
# T3 -- Proposition 2: lossless truncation (3.6)/(3.7), Table 1
# ---------------------------------------------------------------------------

def test_proposition2_truncation_commutes_with_product():
    """(a_bar b_bar)|_HDQ = a_bar|_HDQ * b_bar|_HDQ, entrywise equality."""
    for _ in range(10):
        a, b = _random_tndq(), _random_tndq()
        lhs = (a * b).to_hdq()                    # truncate after multiplying
        rhs = a.to_hdq() * b.to_hdq()             # multiply after truncating
        assert np.array_equal(lhs.ch, rhs.ch)     # exact (same float ops)


def test_dq_truncation_commutes():
    """(a_bar b_bar)|_DQ = a|_DQ b|_DQ  (Table 1, bottom of the tower)."""
    a, b = _random_tndq(), _random_tndq()
    assert np.allclose((a * b).to_dq(), dq_mul(a.to_dq(), b.to_dq()), atol=1e-12)


# ---------------------------------------------------------------------------
# T4 -- formula (3.5): xi, xi_dot, and construction-free Jdot*qdot
# ---------------------------------------------------------------------------

def test_twist_identities_3_5():
    """
    (3.5): xi = 2 x_dot x*;  vec6(xi) = J q_dot;
           vec6(xi_dot) = Jdot q_dot + J q_ddot, with the Jdot q_dot part
           read from the q_ddot = 0 chain (no explicit Jdot ever built).
    """
    q, q_dot, q_ddot = SIGNAL(0.777)
    out = CHAIN.fk_outputs(q, q_dot, q_ddot)
    J = out["J"]

    # vec6(xi) = J q_dot   (twist consistency, Sec. 2.2)
    assert np.allclose(out["xi"], J @ q_dot, atol=1e-10)

    # vec6(xi_dot) - Jdot*qdot = J q_ddot   (linearity of (3.5) in q_ddot)
    assert np.allclose(out["xi_dot"] - out["Jdot_qdot"], J @ q_ddot, atol=1e-10)

    # Jdot*qdot from the chain vs numerical differentiation of J(q(t)) q_dot(t)
    h = 1e-6

    def J_qdot(t):
        qt, qdt, _ = SIGNAL(t)
        return CHAIN.jacobian(qt) @ qdt

    # d/dt (J qdot) = Jdot qdot + J qddot
    dJq_num = (J_qdot(0.777 + h) - J_qdot(0.777 - h)) / (2 * h)
    assert np.allclose(out["xi_dot"], dJq_num, atol=1e-6)


# ---------------------------------------------------------------------------
# T5 -- unit constraint family (3.8) along the chain
# ---------------------------------------------------------------------------

def test_unit_constraints_machine_precision():
    """c0, c1, c2 of (3.8) vanish to machine precision on exact chain data."""
    for t in np.linspace(0.0, 5.0, 7):
        q, q_dot, q_ddot = SIGNAL(t)
        x_bar = CHAIN.fk_tndq(q, q_dot, q_ddot)
        c0, c1, c2 = unit_constraint_residuals(x_bar)
        assert c0 < 1e-12 and c1 < 1e-12 and c2 < 1e-11, (c0, c1, c2)


# ---------------------------------------------------------------------------
# T6 -- Theorem 1: HDQ error element (4.1)-(4.4)
# ---------------------------------------------------------------------------

def _measured_and_desired(t):
    q, q_dot, q_ddot = SIGNAL(t)
    x_breve = CHAIN.fk_tndq(q, q_dot, q_ddot).to_hdq()
    traj = LineTrajectoryTNDQ(CHAIN.fkm(np.zeros(N_JOINTS)),
                              delta_p=[0.1, 0.05, -0.1], duration=4.0,
                              rot_axis=[0, 0, 1], rot_angle=0.4)
    des = traj.evaluate(t)
    return x_breve, des


def test_theorem1_twist_error():
    """
    Theorem 1: xi_tilde = 2 x_tilde_dot x_tilde* is pure, invariant under
    the sign flip x_breve_tilde -> -x_breve_tilde, and satisfies (4.4):
        xi_tilde = xi - Ad_{x_tilde} xi_d.
    """
    t = 0.9
    x_breve, des = _measured_and_desired(t)
    x_breve_tilde = hdq_error(x_breve, des["x_breve_d"])
    e_xi, xi_tilde = twist_error_from_hdq(x_breve_tilde)

    # purity: scalar and dual-scalar parts vanish (checked pre-projection)
    raw = 2.0 * dq_mul(x_breve_tilde.ch[1], dq_conj(x_breve_tilde.ch[0]))
    assert abs(raw[0]) < 1e-12 and abs(raw[4]) < 1e-12

    # sign-flip invariance (Theorem 1(i))
    from core.tndq_algebra import HDQ
    flipped = HDQ(-x_breve_tilde.ch[0], -x_breve_tilde.ch[1])
    e_xi_f, _ = twist_error_from_hdq(flipped)
    assert np.allclose(e_xi, e_xi_f, atol=1e-12)

    # formula (4.4): xi_tilde = xi - Ad_{x_tilde} xi_d
    xi = 2.0 * dq_mul(x_breve.ch[1], dq_conj(x_breve.ch[0]))
    xi_d = vec6_to_pure_dq(des["xi_d"])
    rhs = xi - dq_Ad(x_breve_tilde.to_dq(), xi_d)
    assert np.allclose(dq_vec6(xi_tilde), dq_vec6(rhs), atol=1e-10)


# ---------------------------------------------------------------------------
# T7 -- Theorem 2: e_z_dot = A(x_tilde) e_xi   (4.5)
# ---------------------------------------------------------------------------

def test_theorem2_output_error_kinematics():
    """Central-difference d/dt e_z must match A(x_tilde) e_xi."""
    t0, h = 1.1, 1e-6

    def ez_of(t):
        x_breve, des = _measured_and_desired(t)
        err = full_error_state(x_breve, des["x_breve_d"])
        return err["e_z"]

    x_breve, des = _measured_and_desired(t0)
    err = full_error_state(x_breve, des["x_breve_d"])
    ez_dot_num = (ez_of(t0 + h) - ez_of(t0 - h)) / (2 * h)
    assert np.allclose(ez_dot_num, err["A"] @ err["e_xi"], atol=1e-6)


# ---------------------------------------------------------------------------
# T8 -- Lemma 1: transport rule (5.3)/(5.4)
# ---------------------------------------------------------------------------

def test_lemma1_transport_rule():
    """
    Lemma 1: d/dt ( Ad_{x_tilde} xi_d ) = Ad xi_dot_d + ad_{xi_tilde} Ad xi_d.
    Left side by central differences of the transported desired twist.
    """
    t0, h = 0.6, 1e-6

    def transported(t):
        x_breve, des = _measured_and_desired(t)
        x_breve_tilde = hdq_error(x_breve, des["x_breve_d"])
        return dq_Ad(x_breve_tilde.to_dq(), vec6_to_pure_dq(des["xi_d"]))

    x_breve, des = _measured_and_desired(t0)
    x_breve_tilde = hdq_error(x_breve, des["x_breve_d"])
    x_tilde = x_breve_tilde.to_dq()
    _, xi_tilde = twist_error_from_hdq(x_breve_tilde)

    lhs = (transported(t0 + h) - transported(t0 - h)) / (2 * h)
    rhs = (dq_Ad(x_tilde, vec6_to_pure_dq(des["xi_dot_d"]))
           + dq_ad(xi_tilde, dq_Ad(x_tilde, vec6_to_pure_dq(des["xi_d"]))))
    assert np.allclose(dq_vec6(lhs), dq_vec6(rhs), atol=1e-5)


# ---------------------------------------------------------------------------
# T9 -- reprojection (Sec. 3.4)
# ---------------------------------------------------------------------------

def test_reprojection_restores_constraints():
    """After perturbing a valid TNDQ, reprojection drives (3.8) back down."""
    q, q_dot, q_ddot = SIGNAL(0.3)
    x_bar = CHAIN.fk_tndq(q, q_dot, q_ddot)
    x_bar.ch += 1e-4 * RNG.standard_normal((3, 8))    # inject drift

    c_before = unit_constraint_residuals(x_bar)
    x_proj = reproject_tndq(x_bar)
    c_after = unit_constraint_residuals(x_proj)

    assert c_after[0] < 1e-12                          # exact unit pose
    assert c_after[1] < 1e-12 and c_after[2] < 1e-12   # pure twist / rate
    assert all(a <= b + 1e-15 for a, b in zip(c_after, c_before))


# ---------------------------------------------------------------------------
# standalone runner
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    tests = [(name, fn) for name, fn in sorted(globals().items())
             if name.startswith("test_") and callable(fn)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"[PASS] {name}")
        except AssertionError as exc:
            failed += 1
            print(f"[FAIL] {name}: {exc}")
    print("-" * 50)
    print(f"{len(tests) - failed}/{len(tests)} tests passed")
    raise SystemExit(1 if failed else 0)
