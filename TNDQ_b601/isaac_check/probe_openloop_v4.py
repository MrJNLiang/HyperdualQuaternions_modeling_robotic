#!/usr/bin/env python3
"""v4 斜抓构型开环物理响应探针 —— exp1_slant 首跑"慢爬+反向"根因。

现象（exp1 slant v4，Isaac）：
  - t<1s goto 跟踪正常（pos_err ~2 mm）；
  - t=2s 起臂与期望背道而驰（pz 0.104 -> 0.110 上升，期望下降），
    之后 9 s 几乎不动（q2 仅漂 0.06 rad），tau2 ~4-5.7 N·m、
    qddot_ref 14.9 rad/s^2 但无加速度响应；
  - 纯仿真（diag_loop_sim）同参数全收敛。

开环排除控制栈，逐项检查：
  [A] Q_INIT 重力补偿静置（应不动，漂多少=重力残差）；
  [B] 重力补偿 + j2 单分量 ±1.35 N·m（应动）；
  [C] 复现 exp1 t=0.5s 闭环力矩（CSV 回放）：动否；
  [D] 读 USD joint1-6 authored maxJointVelocity / 阻尼属性
      （速度封顶嫌疑：11 s 走 0.1 rad -> ~0.01 rad/s 有效封顶）。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_openloop_v4.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_INIT = np.array([-0.251958, -2.607660, -1.617296,
                   -1.077630, 0.000041, -0.000004])
# exp1 CSV t=0.5s 的闭环力矩（回放）
TAU_LOOP_05 = None   # 运行时从 CSV 读取


def run_hold(backend, tau, n_steps, label):
    q_start, _ = backend.get_joint_state()
    for _ in range(n_steps):
        backend.apply_arm_torques(tau)
        backend.step()
    q_end, qd_end = backend.get_joint_state()
    dq = q_end - q_start
    print(f"  {label}: dq={np.round(dq, 4).tolist()}  "
          f"|dq|max={np.max(np.abs(dq)):.5f} rad  "
          f"qd_end={np.round(qd_end, 4).tolist()}")
    return dq


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        g = dyn.gravity_vector(Q_INIT)
        print(f"g(Q_INIT) = {np.round(g, 3).tolist()}")

        print("=== [A] 重力补偿静置 1 s ===")
        backend.reset_to(Q_INIT)
        run_hold(backend, g, 500, "grav-comp")

        print("=== [B] 重力补偿 + j2 分量 0.5 s ===")
        for sgn in (+1.0, -1.0):
            backend.reset_to(Q_INIT)
            t = g.copy()
            t[1] += sgn * 1.35
            run_hold(backend, t, 250, f"j2 {sgn:+.0f}*1.35")

        print("=== [C] exp1 闭环力矩回放 ===", flush=True)
        try:
            csv = os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), "results", "exp1_setpoint.csv")
            with open(csv) as f:
                header = f.readline().strip().split(",")
            data = np.loadtxt(csv, delimiter=",", skiprows=1)
            icol = {c: i for i, c in enumerate(header)}
            for tt in (0.5, 2.0, 5.0):
                row = data[np.abs(data[:, icol["t"]] - tt) < 1e-9]
                if len(row) == 0:
                    continue
                row = row[0]
                tau = np.array([row[icol[f"tau{i}"]] for i in range(1, 7)])
                q0 = np.array([row[icol[f"q{i}"]] for i in range(1, 7)])
                backend.reset_to(q0)
                print(f"  -- t={tt}s tau={np.round(tau, 2).tolist()}",
                      flush=True)
                run_hold(backend, tau, 250, f"replay t={tt}s")
        except Exception:
            import traceback
            traceback.print_exc()

        print("=== [D] USD authored 关节属性 ===")
        stage = backend.stage
        for prim in stage.Traverse():
            if prim.GetTypeName() != "PhysicsRevoluteJoint":
                continue
            out = [str(prim.GetPath())]
            for attr in prim.GetAttributes():
                n = attr.GetName()
                if any(k in n.lower() for k in
                       ("velocity", "damping", "stiffness", "maxforce",
                        "friction", "armature")):
                    out.append(f"{n}={attr.Get()}")
            print("  " + "  ".join(out))
    finally:
        backend.close()
    print("=== OPENLOOP V4 DONE ===")


if __name__ == "__main__":
    main()
