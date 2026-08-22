#!/usr/bin/env python3
"""PhysX 接触报告 —— Q_F 处 B 力矩驱动过程的接触事件抓取。

假设：B 组合力矩"先动后灭" = 微动后触发接触约束（小间隙）。
get_physx_simulation_interface().get_contact_report() 逐步抓取。

  [A] 纯重力补偿 10 步（基线：应无新接触或恒定接触）
  [B] g+TAU_FB 50 步：逐步打印接触对（出现新对 = 锁死来源实锤）

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_contact_report.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_F = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])
TAU_FB = np.array([0.033, -1.347, 0.625, 0.281, -0.004, 0.003])


def fmt_report(rep, verbose=False):
    headers, data = rep
    n = len(headers)
    if n == 0:
        return ""
    if verbose:
        print("        header attrs:",
              [a for a in dir(headers[0]) if not a.startswith("_")])
    pairs = []
    for i in range(n):
        h = headers[i]
        info = []
        for attr in ("actor0", "actor1", "body0", "body1", "path0", "path1"):
            if hasattr(h, attr):
                info.append(f"{attr}={getattr(h, attr)}")
        pairs.append(" ".join(info[:4]))
    return f"{n} 个接触: " + " | ".join(pairs[:6])


def run_case(backend, dyn, tag, tau_extra, n_steps):
    backend.reset_to(Q_F)
    tau = dyn.gravity_vector(Q_F) + tau_extra
    for k in range(n_steps):
        backend.apply_arm_torques(tau)
        backend.step()
        try:
            rep = backend._physx_sim.get_contact_report()
        except Exception as e:
            print(f"    step {k}: get_contact_report 失败 {e}")
            return
        s = fmt_report(rep, verbose=(k == 0))
        if s:
            print(f"    [{tag}] step {k}: {s}")
    print(f"    [{tag}] 结束（{n_steps} 步）")


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    import omni.physx  # SimulationApp 就绪后才能 import
    backend._physx_sim = omni.physx.get_physx_simulation_interface()
    dyn = B601NominalDynamics()
    try:
        print("=== [A] 纯重力基线 ===")
        run_case(backend, dyn, "A", np.zeros(6), 10)
        print("=== [B] 闭环净反馈 ===")
        run_case(backend, dyn, "B", TAU_FB, 50)
    finally:
        backend.close()
    print("=== CONTACT REPORT DONE ===")


if __name__ == "__main__":
    main()
