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
    T10 Screw-log map vec6(2 ln x): identity limit, exact screws  [Sec. 6.4 C2]
    T11 Screw-log derivative d/dt vec6(2 ln x_tilde) = e_xi       [Sec. 6.4 C2]
    T12 Faithful [Ch20] law oracle: cancellation + convergence    [Sec. 6.4 C2]

Numerical differentiation only appears on the *reference* side of each
comparison (central differences); the TNDQ side is purely algebraic.

Run from the TNDQ_sim root:
    python -m tests.test_math_properties        # standalone
    python -m pytest tests/ -q                  # or with pytest
"""

import numpy as np

from core.dq_algebra import (
    dq_mul, dq_conj, dq_identity, dq_vec6, dq_Ad, dq_ad, vec6_to_pure_dq,
    dq_log2_vec6,
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
# T10 -- screw-log map vec6(2 ln x)  (baseline C2 pose feedback, Sec. 6.4)
# ---------------------------------------------------------------------------

def _quat_axis_angle(n, phi):
    n = np.asarray(n, dtype=float) / np.linalg.norm(n)
    return np.array([np.cos(phi / 2), *(np.sin(phi / 2) * n)])


def _pose_dq(r, p):
    """Unit pose DQ from rotation quaternion r and translation p, (2.1):
    x = (1 + eps p/2) r  as a DQ product."""
    r = np.asarray(r, dtype=float)
    t_dq = np.array([1.0, 0, 0, 0, 0.0, *(np.asarray(p, dtype=float) / 2)])
    return dq_mul(t_dq, np.r_[r, np.zeros(4)])


def _normalize_udq(x):
    """Renormalize to a unit DQ (|r| = 1, <r, qd> = 0)."""
    r = x[:4] / np.linalg.norm(x[:4])
    qd = x[4:] - np.dot(x[4:], r) * r
    return np.r_[r, qd]


def test_dq_log2_identity_limit_and_screws():
    """
    vec6(2 ln x) = [phi n; d n + phi m]:
      (a) near identity  -> [-2 O; T] with O = -Im(r), T = p;
      (b) pure translation -> [0; p];
      (c) exact screw roundtrip: (phi, n) from axis-angle, pitch d = n.p,
          axis line c = n x m reproduces p = (I - R) c + d n.
    """
    # (a) near-identity limit (correction is O(phi |p|): the exact dual
    # part d n + phi m approaches p only linearly in phi)
    n = np.array([0.2, -0.9, 0.4])
    n /= np.linalg.norm(n)
    phi, p = 1e-3, np.array([0.01, -0.02, 0.03])
    x = _pose_dq(_quat_axis_angle(n, phi), p)
    ell = dq_log2_vec6(x)
    O, T = -x[1:4], p
    assert np.allclose(ell, np.r_[-2.0 * O, T], atol=2e-5)

    # (b) pure translation
    x_t = _pose_dq(np.array([1.0, 0, 0, 0]), p)
    assert np.allclose(dq_log2_vec6(x_t), np.r_[np.zeros(3), p], atol=1e-12)

    # (c) exact screw roundtrip on random poses away from phi = pi
    for _ in range(10):
        n = RNG.standard_normal(3)
        n /= np.linalg.norm(n)
        phi = float(RNG.uniform(0.2, 2.6))
        p = RNG.standard_normal(3) * 0.2
        x = _pose_dq(_quat_axis_angle(n, phi), p)
        ell = dq_log2_vec6(x)
        # rotation part: phi n
        assert np.allclose(ell[:3], phi * n, atol=1e-10)
        # pitch: projection of the dual part on the axis = n . p
        d = float(n @ p)
        assert abs(ell[3:] @ n - d) < 1e-10
        # moment: axis line c = n x m reproduces p = (I - R) c + d n
        m = (ell[3:] - d * n) / phi
        c = np.cross(n, m)
        ctheta, stheta = np.cos(phi), np.sin(phi)
        R = (ctheta * np.eye(3) + (1 - ctheta) * np.outer(n, n)
             + stheta * np.array([[0, -n[2], n[1]],
                                  [n[2], 0, -n[0]],
                                  [-n[1], n[0], 0]]))
        assert np.allclose(p, (np.eye(3) - R) @ c + d * n, atol=1e-10)


# ---------------------------------------------------------------------------
# T11 -- log-map derivative: d/dt vec6(2 ln x_tilde) = e_xi near identity
# ---------------------------------------------------------------------------

def test_dq_log2_derivative_equals_twist_error():
    """
    Near identity the screw-log coordinates differentiate to the right-
    invariant twist error: ell_dot = e_xi + O(||ell||^2).  This is the
    convention fact that fixes the sign of C2's pose feedback ([Ch20]'s
    +2 K_P ln(x_e) becomes -K_P vec6(2 ln x_tilde) in our conventions).
    """
    n = np.array([0.3, 0.8, -0.5])
    n /= np.linalg.norm(n)
    amp_phi, amp_p = 1e-3, 2e-3

    def x_tilde_of(t):
        phi = amp_phi * np.sin(0.7 * t)
        p = amp_p * np.array([np.sin(1.1 * t), np.cos(0.9 * t),
                              np.sin(0.5 * t)])
        return _pose_dq(_quat_axis_angle(n, phi), p)

    t0, h = 1.3, 1e-6
    x0 = x_tilde_of(t0)
    ell_dot_num = (dq_log2_vec6(x_tilde_of(t0 + h))
                   - dq_log2_vec6(x_tilde_of(t0 - h))) / (2 * h)
    # e_xi = vec6(2 x_dot x*)  (numerical x_dot, exact twist extraction)
    x_dot = (x_tilde_of(t0 + h) - x_tilde_of(t0 - h)) / (2 * h)
    e_xi = dq_vec6(2.0 * dq_mul(x_dot, dq_conj(x0)))
    assert np.allclose(ell_dot_num, e_xi, atol=5e-6)


# ---------------------------------------------------------------------------
# T12 -- faithful [Ch20] law oracle: composition + closed-loop convergence
# ---------------------------------------------------------------------------

def test_chandra20_law_oracle():
    """
    Oracle for control/control_law.py::dq_chandra2020_law:
      (a) composition: with a perfect model (xi_dot = a_cmd), substitution
          cancels the feedforward, d(e_xi)/dt = a_cmd - u_ff
          = -K_v e_xi - K_P vec6(2 ln x_tilde)  (sign convention locked);
      (b) closed loop: integrating xi_dot = a_cmd on the true error
          kinematics x_dot = (1/2) xi x with xi_d = 0 drives both
          vec6(2 ln x_tilde) and e_xi to zero (asymptotic stability).
    """
    from control.control_law import dq_chandra2020_law, feedforward_term
    from config.params import CH20_K_V, CH20_K_P

    # (a) composition on real trajectory error data
    x_breve, des = _measured_and_desired(0.9)
    err = full_error_state(x_breve, des["x_breve_d"])
    zero6 = np.zeros(6)
    qdd_ref, u_task = dq_chandra2020_law(
        err, des["xi_d"], des["xi_dot_d"], np.eye(6), zero6,
        CH20_K_V, CH20_K_P, damping=0.0)
    u_ff = feedforward_term(err["x_tilde"], err["xi_tilde"],
                            des["xi_d"], des["xi_dot_d"])
    ell = dq_log2_vec6(err["x_tilde"])
    expected = u_ff - CH20_K_V @ err["e_xi"] - CH20_K_P @ ell
    assert np.allclose(u_task, expected, atol=1e-12)
    assert np.allclose(qdd_ref, u_task, atol=1e-12)     # J = I6, no damping

    # (b) closed-loop convergence with a stationary target (xi_d = 0)
    n = np.array([1.0, 2.0, -1.0])
    n /= np.linalg.norm(n)
    x = _pose_dq(_quat_axis_angle(n, 0.5), np.array([0.05, -0.03, 0.04]))
    e = np.zeros(6)
    dt, t_end = 5e-4, 4.0

    def deriv(x, e):
        err_s = {"x_tilde": x, "xi_tilde": vec6_to_pure_dq(e), "e_xi": e}
        _, a_cmd = dq_chandra2020_law(err_s, zero6, zero6, np.eye(6), zero6,
                                      CH20_K_V, CH20_K_P, damping=0.0)
        xi = vec6_to_pure_dq(e)               # xi_d = 0 -> xi = e_xi
        x_dot = 0.5 * dq_mul(xi, x)
        return x_dot, a_cmd

    for _ in range(int(round(t_end / dt))):
        dx1, de1 = deriv(x, e)
        dx2, de2 = deriv(_normalize_udq(x + 0.5 * dt * dx1), e + 0.5 * dt * de1)
        dx3, de3 = deriv(_normalize_udq(x + 0.5 * dt * dx2), e + 0.5 * dt * de2)
        dx4, de4 = deriv(_normalize_udq(x + dt * dx3), e + dt * de3)
        x = _normalize_udq(x + dt / 6 * (dx1 + 2 * dx2 + 2 * dx3 + dx4))
        e = e + dt / 6 * (de1 + 2 * de2 + 2 * de3 + de4)

    assert np.linalg.norm(dq_log2_vec6(x)) < 1e-4
    assert np.linalg.norm(e) < 1e-4


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
