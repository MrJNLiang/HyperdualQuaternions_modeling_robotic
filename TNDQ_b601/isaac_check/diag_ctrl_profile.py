"""控制链逐层剖面（实时化优化依据，纯计算不含 PhysX）。

运行：~/isaacsim/python.sh TNDQ_b601/isaac_check/diag_ctrl_profile.py
输出各层单步耗时（us），热点排序供优化决策。
"""
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_B601 = os.path.dirname(_HERE)
for _p in (_B601, os.path.join(_B601, "experiments")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from config.params import B601_DH_TABLE, DEFAULT_GAIN_SET, GAIN_SETS, Q_INIT
from config.b601_dynamics import B601NominalDynamics
from control.control_law import geometric_computed_torque_law
from control.error_system import full_error_state
from run_lib import B601TCPChain, build_setpoint_goto_trajectory_pinocchio


def bench(name, fn, n=300):
    for _ in range(20):
        fn()                      # warmup（缓存/分支稳定）
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    us = (time.perf_counter() - t0) / n * 1e6
    print(f"  {name:<34s} {us:9.1f} us", flush=True)
    return us


def main():
    chain = B601TCPChain(B601_DH_TABLE)
    dyn = B601NominalDynamics()
    gains = GAIN_SETS[DEFAULT_GAIN_SET]
    K_d, k_p = gains["K_d"], gains["k_p"]
    traj, _, _ = build_setpoint_goto_trajectory_pinocchio()

    rng = np.random.default_rng(0)
    sums = []
    for tag, q in (("Q_INIT", Q_INIT.copy()),
                   ("q_mid(t=7s)", traj.spline.evaluate(7.0)[0]),
                   ("q_grasp(t=14s)", traj.spline.evaluate(14.0)[0])):
        qd = 0.1 * rng.standard_normal(6)
        fk = chain.fk_outputs(q, qd, with_jacobian=True)
        des = traj.evaluate(7.0)
        err = full_error_state(fk["x_breve"], des["x_breve_d"])
        print(f"--- 构型 {tag} ---")
        total = 0.0
        total += bench("fk_outputs(+J)", lambda: chain.fk_outputs(
            q, qd, with_jacobian=True))
        total += bench("traj.evaluate", lambda: traj.evaluate(7.0))
        total += bench("full_error_state", lambda: full_error_state(
            fk["x_breve"], des["x_breve_d"]))
        total += bench("svd(J)", lambda: np.linalg.svd(
            fk["J"], compute_uv=False))
        total += bench("dyn.mass_matrix", lambda: dyn.mass_matrix(q))
        total += bench("dyn.computed_torque", lambda: dyn.computed_torque(
            q, qd, np.zeros(6)))

        def _law():
            geometric_computed_torque_law(
                err, des["xi_d"], des["xi_dot_d"],
                fk["J"], fk["Jdot_qdot"], K_d, k_p,
                damping=1e-6, M=dyn.mass_matrix(q))
        total += bench("law(+M)", _law)
        sums.append(total)
        print(f"  {'SUM(控制层合计)':<34s} {total:9.1f} us\n", flush=True)
    print(f"三构型控制层合计均值: {np.mean(sums) / 1000:.2f} ms/步")


if __name__ == "__main__":
    main()
