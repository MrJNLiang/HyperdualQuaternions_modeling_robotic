"""
Input simulation: joint-space test signals and exogenous disturbances.

Two roles (paper Sec. 6.3 experiment design):

1.  Joint test signals with closed-form (q, q_dot, q_ddot) for the algebraic
    validation experiments E0 (Propositions 1/2, formulas (3.4)/(3.7)) and
    E2 (error kinematics, Theorems 1/2).  Closed forms keep the TNDQ chain
    inputs exact, so any residual is attributable to the algebra, not to
    numerical differentiation.

2.  Acceleration-level exogenous disturbances w_dyn for the closed loop
        q_ddot = q_ddot_ref + w_dyn            (formula (5.1))
    -  L2-type (finite energy): exponentially decaying sinusoid, used to
       measure the achieved H-infinity gain against Theorem 3(c) (E4).
    -  L-infinity bias type (bounded, persistent): constant offset plus a
       bounded sinusoid, used to verify the ISS ultimate ball (5.7),
       Theorem 3(d) (E5).

The robot model itself is the KUKA LBR4+ 7R chain defined by the DH table
in config/params.py; signals here are purely joint-space and model-free.
"""

import numpy as np


# ---------------------------------------------------------------------------
# 1. Joint-space test signals (closed-form q, q_dot, q_ddot) -- E0/E2
# ---------------------------------------------------------------------------

class JointSineSignal:
    """
    Per-joint sinusoid with closed-form derivatives:

        q_i(t)      = q0_i + A_i sin(w_i t + phi_i)
        q_dot_i(t)  =        A_i w_i   cos(w_i t + phi_i)
        q_ddot_i(t) =       -A_i w_i^2 sin(w_i t + phi_i)

    Exact (q, q_dot, q_ddot) triples feed the TNDQ joint factors (Appendix
    B.1) so that Proposition 1 (3.4) and formula (3.5) can be checked to
    machine precision (experiment E0).
    """

    def __init__(self, q0, amplitude, omega, phase=None):
        self.q0 = np.asarray(q0, dtype=float)
        n = self.q0.shape[0]
        self.A = np.broadcast_to(np.asarray(amplitude, dtype=float), (n,)).copy()
        self.w = np.broadcast_to(np.asarray(omega, dtype=float), (n,)).copy()
        self.phi = (np.zeros(n) if phase is None
                    else np.broadcast_to(np.asarray(phase, dtype=float), (n,)).copy())

    def __call__(self, t):
        """Returns (q, q_dot, q_ddot) at time t (all shape (n,))."""
        arg = self.w * t + self.phi
        q = self.q0 + self.A * np.sin(arg)
        q_dot = self.A * self.w * np.cos(arg)
        q_ddot = -self.A * self.w ** 2 * np.sin(arg)
        return q, q_dot, q_ddot


def default_joint_sine_7r(q0=None):
    """
    Default 7R test signal: distinct frequencies/phases per joint so all
    channels of the TNDQ chain (sigma^0, sigma^1, sigma^2) are excited
    simultaneously and no cross-term of (3.2) stays silent.
    """
    if q0 is None:
        q0 = np.array([0.3, 0.4, -0.2, 0.8, 0.1, 0.6, -0.3])
    amplitude = np.array([0.40, 0.35, 0.30, 0.45, 0.25, 0.35, 0.30])
    omega = np.array([1.0, 1.3, 0.7, 1.7, 2.1, 0.9, 1.5])
    phase = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
    return JointSineSignal(q0, amplitude, omega, phase)


# ---------------------------------------------------------------------------
# 2. Acceleration-level disturbances w_dyn of (5.1) -- E4/E5
# ---------------------------------------------------------------------------

class L2Disturbance:
    """
    Finite-energy (L2) joint-acceleration disturbance for Theorem 3(c)/E4:

        w_i(t) = A_i exp(-lambda t) sin(w_i t + phi_i),   t >= t_on

    ||w||_L2 < inf because of the exponential envelope, so the H-infinity
    inequality (5.6) applies:  kappa^-1 int||e_xi||^2 <= gamma_a^2 int||d||^2 + 2V(0).
    """

    def __init__(self, n_joints, amplitude=1.0, decay=0.5, omega=3.0, t_on=1.0,
                 seed=0):
        rng = np.random.default_rng(seed)
        self.A = amplitude * (0.5 + rng.random(n_joints))
        self.lam = float(decay)
        self.w = omega * (0.5 + rng.random(n_joints))
        self.phi = 2.0 * np.pi * rng.random(n_joints)
        self.t_on = float(t_on)

    def __call__(self, t):
        """Returns w_dyn(t), shape (n,)."""
        if t < self.t_on:
            return np.zeros_like(self.A)
        tau = t - self.t_on
        return self.A * np.exp(-self.lam * tau) * np.sin(self.w * tau + self.phi)

    def energy(self, t_end, dt=1e-3):
        """Numerical int_0^{t_end} ||w||^2 dt (RHS budget of (5.6))."""
        ts = np.arange(0.0, t_end, dt)
        return sum(float(w @ w) for w in map(self, ts)) * dt


class BiasDisturbance:
    """
    Bounded persistent (L-infinity) disturbance for Theorem 3(d)/E5:

        w_i(t) = b_i + A_i sin(w_i t + phi_i),   t >= t_on

    ||w||_inf <= |b| + |A| stays bounded but does not vanish, so the loop
    converges only to the ISS ultimate ball (5.7):
        limsup ||e_xi|| <= ||d_b||_inf / lambda_min(K_d).
    """

    def __init__(self, n_joints, bias=0.5, amplitude=0.2, omega=2.0, t_on=1.0,
                 seed=1):
        rng = np.random.default_rng(seed)
        sign = np.where(rng.random(n_joints) < 0.5, -1.0, 1.0)
        self.b = bias * sign
        self.A = amplitude * rng.random(n_joints)
        self.w = omega * (0.5 + rng.random(n_joints))
        self.phi = 2.0 * np.pi * rng.random(n_joints)
        self.t_on = float(t_on)

    def __call__(self, t):
        """Returns w_dyn(t), shape (n,)."""
        if t < self.t_on:
            return np.zeros_like(self.b)
        return self.b + self.A * np.sin(self.w * (t - self.t_on) + self.phi)

    def sup_norm(self):
        """Conservative ||w||_inf bound |b| + |A| (input to (5.7))."""
        return float(np.linalg.norm(self.b) + np.linalg.norm(self.A))


class ZeroDisturbance:
    """Nominal case w_dyn = 0 (Theorem 3(b): exponential-type convergence)."""

    def __init__(self, n_joints):
        self.n = int(n_joints)

    def __call__(self, t):
        return np.zeros(self.n)


# ---------------------------------------------------------------------------
# 3. Optional measurement noise (kept off in the paper's core experiments)
# ---------------------------------------------------------------------------

class MeasurementNoise:
    """
    Additive zero-mean Gaussian noise on (q, q_dot).  Not part of the
    theorems' assumptions -- it enters d(t) of Theorem 3 through the FK map;
    provided only for robustness spot checks.
    """

    def __init__(self, n_joints, sigma_q=0.0, sigma_qdot=0.0, seed=2):
        self.rng = np.random.default_rng(seed)
        self.n = int(n_joints)
        self.sigma_q = float(sigma_q)
        self.sigma_qdot = float(sigma_qdot)

    def __call__(self, q, q_dot):
        qn = q + self.sigma_q * self.rng.standard_normal(self.n)
        qdn = q_dot + self.sigma_qdot * self.rng.standard_normal(self.n)
        return qn, qdn
