"""Numerical checks for the direct quaternion storage and Schur synthesis."""

import numpy as np


def skew(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])


def direction(rng):
    x = rng.normal(size=3)
    return x / np.linalg.norm(x)


def gains(gw, gz, gv, gt, eta0, wbar):
    bw, bv = 2 / gw, 2 / gv
    po = gw**2 / (eta0 * gz**2) + (1 + 1 / (2 - eta0)) / (eta0 * gw**2)
    pt = gv**2 / gt**2 + 1 / (2 * gv**2) + wbar**2 / 4
    return bw, bv, po, pt


def derivatives(eta, o, w, t, v, dw, dv, bw, bv, po, pt):
    odot = -(eta * w + np.cross(o, w)) / 2
    etadot = o @ w / 2
    wdot = -bw * w + po * eta * o / 2 + dw
    tdot = v - np.cross(t, w)
    vdot = -bv * v - pt * t + dv
    direct_w = bw * (
        (w - bw * o / 2) @ wdot
        + (po * o - bw * w / 2) @ odot
        - bw**2 * etadot
    )
    direct_v = bv * (
        (v + bv * t / 4) @ vdot
        + ((pt + bv**2 / 4) * t + bv * v / 4) @ tdot
    )
    closed_w = (
        -bw**2 * (1 - eta / 4) * (w @ w)
        - bw**2 * po * eta * (o @ o) / 4
        + bw * (w - bw * o / 2) @ dw
    )
    closed_v = (
        -3 * bv**2 * (v @ v) / 4
        - bv**2 * pt * (t @ t) / 4
        - bv**2 * t @ skew(w) @ v / 4
        + bv * (v + bv * t / 4) @ dv
    )
    return direct_w, direct_v, closed_w, closed_v


def main():
    rng = np.random.default_rng(20260926)
    max_relative_error = 0.0
    worst_normalized_supply = -np.inf
    worst_schur_eigenvalue = np.inf
    worst_spectral_residual = np.inf
    worst_joint_storage_residual = np.inf
    for _ in range(5000):
        gw, gz, gv, gt = 10 ** rng.uniform(-1, 0.7, size=4)
        eta0 = rng.uniform(0.2, 0.95)
        wbar = rng.uniform(0.1, 3)
        bw, bv, po, pt = gains(gw, gz, gv, gt, eta0, wbar)
        eta = rng.uniform(eta0, 1)
        o = np.sqrt(1 - eta**2) * direction(rng)
        w = rng.uniform(0, wbar) * direction(rng)
        t = 10 ** rng.uniform(-2, 1) * direction(rng)
        v = rng.normal(size=3)
        # These inputs maximize the instantaneous supply residual.
        dw = bw * (w - bw * o / 2) / 2
        dv = bv * (v + bv * t / 4) / 2
        a, b, c, d = derivatives(eta, o, w, t, v, dw, dv, bw, bv, po, pt)
        err = max(abs(a - c) / (1 + abs(c)), abs(b - d) / (1 + abs(d)))
        max_relative_error = max(max_relative_error, err)
        assert err < 1e-9

        rw = c + (o @ o) / gz**2 + (w @ w) / gw**2 - dw @ dw
        rv = d + (t @ t) / gt**2 + (v @ v) / gv**2 - dv @ dv
        normalized = max(rw / (1 + abs(c) + dw @ dw), rv / (1 + abs(d) + dv @ dv))
        worst_normalized_supply = max(worst_normalized_supply, normalized)
        assert normalized <= 1e-10

        pw = np.array([[po + bw**2, -bw / 2], [-bw / 2, 1]])
        pv = np.array([[pt + bv**2 / 4, bv / 4], [bv / 4, 1]])
        assert np.linalg.eigvalsh(pw)[0] > 0
        assert np.linalg.eigvalsh(pv)[0] > 0
        ww = bw * (w @ w / 2 + po * (o @ o) / 2 + bw**2 * (1 - eta) - bw * o @ w / 2)
        attitude_lower = bw * (po + 3 * bw**2 / 4) * (o @ o) / 2
        velocity_lower = bw * (1 - bw**2 / (4 * (po + bw**2))) * (w @ w) / 2
        assert ww >= max(attitude_lower, velocity_lower) - 1e-8 * max(1, abs(ww))

        iw = np.eye(3)
        nw = np.kron(
            [
                [po * eta / gw**2 - 1 / gz**2, 0, 1 / gw**2],
                [0, (3 - eta) / gw**2, -1 / gw],
                [1 / gw**2, -1 / gw, 1],
            ],
            iw,
        )
        nv = np.block(
            [
                [(pt / gv**2 - 1 / gt**2) * iw, skew(w) / (2 * gv**2), -iw / (2 * gv**2)],
                [-skew(w) / (2 * gv**2), 2 * iw / gv**2, -iw / gv],
                [-iw / (2 * gv**2), -iw / gv, iw],
            ]
        )
        yw, yv = np.r_[o, w, dw], np.r_[t, v, dv]
        assert abs(rw + yw @ nw @ yw) < 1e-8 * (1 + abs(rw) + np.linalg.norm(yw)**2 * np.linalg.norm(nw))
        assert abs(rv + yv @ nv @ yv) < 1e-8 * (1 + abs(rv) + np.linalg.norm(yv)**2 * np.linalg.norm(nv))

        sw = np.array(
            [
                [po * eta / gw**2 - 1 / gz**2 - 1 / gw**4, 1 / gw**3],
                [1 / gw**3, (2 - eta) / gw**2],
            ]
        )
        cross = skew(w) / (2 * gv**2) - np.eye(3) / (2 * gv**3)
        sv = np.block(
            [
                [(pt / gv**2 - 1 / gt**2 - 1 / (4 * gv**4)) * np.eye(3), cross],
                [cross.T, np.eye(3) / gv**2],
            ]
        )
        assert np.allclose(nw[:6, :6] - nw[:6, 6:] @ nw[6:, :6], np.kron(sw, iw))
        assert np.allclose(nv[:6, :6] - nv[:6, 6:] @ nv[6:, :6], sv)
        ew = np.linalg.eigvalsh(sw)[0] / max(1, np.linalg.norm(sw, 2))
        ev = np.linalg.eigvalsh(sv)[0] / max(1, np.linalg.norm(sv, 2))
        worst_schur_eigenvalue = min(worst_schur_eigenvalue, ew, ev)
        assert min(ew, ev) >= -1e-10

        # Check the generalized-eigenvalue constants used for the L_infinity bound.
        qw = np.diag([1 / gz**2, 1 / gw**2])
        hw = np.array([[po + 2 * bw**2 / (1 + eta0), -bw / 2], [-bw / 2, 1]])
        qv = np.diag([1 / gt**2, 1 / gv**2])
        hv = np.array([[pt + bv**2 / 4, bv / 4], [bv / 4, 1]])
        cw = 2 / (bw * np.linalg.eigvalsh(np.diag([gz, gw]) @ hw @ np.diag([gz, gw]))[-1])
        cv = 2 / (bv * np.linalg.eigvalsh(np.diag([gt, gv]) @ hv @ np.diag([gt, gv]))[-1])
        xw = rng.normal(size=2)
        xv = rng.normal(size=2)
        spectral_w = xw @ qw @ xw - cw * bw / 2 * xw @ hw @ xw
        spectral_v = xv @ qv @ xv - cv * bv / 2 * xv @ hv @ xv
        worst_spectral_residual = min(worst_spectral_residual, spectral_w, spectral_v)
        assert min(spectral_w, spectral_v) >= -1e-10

        joint_w = min(bw * np.linalg.eigvalsh(pw)[0] / 2,
                      bv * np.linalg.eigvalsh(pv)[0] / 2)
        storage_v = bv * ((v @ v + (pt + bv**2 / 4) * (t @ t)
                           + bv * (t @ v) / 2) / 2)
        joint_residual = ww + storage_v - joint_w * (
            o @ o + w @ w + t @ t + v @ v)
        worst_joint_storage_residual = min(worst_joint_storage_residual, joint_residual)
        assert joint_residual >= -1e-8 * (1 + ww + storage_v)

        a_matrix = np.block([
            [-(eta * iw + skew(o)) / 2, np.zeros((3, 3))],
            [-skew(t), iw],
        ])
        fb = np.r_[po * eta * o / 2 - bw * w, -pt * t - bv * v]
        l_state = np.sqrt((max(bw, bv)**2
                           + np.linalg.norm(a_matrix, 2)**2 * max(po, pt)**2) / joint_w)
        assert np.linalg.norm(fb) <= l_state * np.sqrt(ww + storage_v) + 1e-8

    # Check each Schur boundary independently. The rotation certificate
    # has no speed bound; its null vector need not lie in the combined domain.
    gw, gz, gv, gt, eta0, wbar = 0.2, 0.05, 0.3, 0.1, 0.8, 1.0
    bw, bv, po, pt = gains(gw, gz, gv, gt, eta0, wbar)
    o = np.array([np.sqrt(1 - eta0**2), 0, 0])
    w = -o / (gw * (2 - eta0))
    dw = bw * (w - bw * o / 2) / 2
    _, _, cw, _ = derivatives(eta0, o, w, np.zeros(3), np.zeros(3), dw, np.zeros(3), bw, bv, po, pt)
    assert abs(cw + o @ o / gz**2 + w @ w / gw**2 - dw @ dw) < 1e-8
    w = np.array([0, 0, wbar])
    t = np.array([1, 0, 0])
    v = np.cross(w, t) / 2 + t / (2 * gv)
    dv = bv * (v + bv * t / 4) / 2
    _, _, _, cv = derivatives(1, np.zeros(3), w, t, v, np.zeros(3), dv, bw, bv, po, pt)
    assert abs(cv + t @ t / gt**2 + v @ v / gv**2 - dv @ dv) < 1e-8

    print("5000 nonlinear derivative, worst-input supply and Schur checks: PASS")
    print(f"Maximum relative derivative error: {max_relative_error:.3e}")
    print(f"Largest normalized supply residual: {worst_normalized_supply:.3e}")
    print(f"Smallest normalized Schur eigenvalue: {worst_schur_eigenvalue:.3e}")
    print(f"Smallest L_infinity spectral residual: {worst_spectral_residual:.3e}")
    print(f"Smallest joint storage residual: {worst_joint_storage_residual:.3e}")
    print("Rotation and translation boundary equality checks: PASS")
    print("Storage positivity, joint feedback bound, full supply matrix checks: PASS")
    print(f"Example gains: b_omega={bw:.6f}, b_v={bv:.6f}, p_O={po:.6f}, p_T={pt:.6f}")


if __name__ == "__main__":
    main()
