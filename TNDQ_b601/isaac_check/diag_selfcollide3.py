#!/usr/bin/env python3
"""自碰撞终局诊断 v3 —— 接口层 arm_self_collision 开关对照。

历史失败路径（勿重走）：
  - diag_selfcollide2：物理运行中 CreateAttribute -> C++ 层静默中止；
  - v3a：Isaac 6.0 Articulation view 无 apply_articulation_settings。
正解：物理初始化前 PhysxArticulationAPI.Apply（已并入
IsaacB601Backend._set_arm_self_collision_usd，构造参数 arm_self_collision）。

    [A] --mode base  默认（自碰撞开）：q_ss + tau_cmd 1 s（预期顶死）；
    [B] --mode off   arm_self_collision=False 复测同一指令：
          能动 -> 自碰撞实锤（exp1 伸展构型冻结根因）；
          仍不动 -> PhysX 求解器构型锁死，需换 solver / 减小伸展度。
    （SimulationApp 单进程单实例，两个模式分两次跑）

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/diag_selfcollide3.py --mode base
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/diag_selfcollide3.py --mode off
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_SS = np.array([-0.291, -2.543, -1.195, -1.559, -0.783, 0.234])
TAU_CMD = np.array([0.161, 3.772, -1.215, -0.314, 0.027, 0.001])


def run_tau(backend, tag, tau, hold_s=1.0):
    backend.reset_to(Q_SS)
    n = int(round(hold_s / backend.physics_dt))
    q0 = None
    for k in range(n):
        q, _ = backend.get_joint_state()
        if q0 is None:
            q0 = q.copy()
        backend.apply_arm_torques(tau)
        backend.step()
    q1, _ = backend.get_joint_state()
    dq = q1 - q0
    print(f"[{tag}] dq(1s) = {np.round(dq, 4).tolist()}"
          f"  max|dq| = {float(np.max(np.abs(dq))):.4f} rad", flush=True)
    return dq


def main():
    import argparse

    ap = argparse.ArgumentParser(description="自碰撞对照诊断")
    ap.add_argument("--mode", choices=["base", "off"], required=True,
                    help="base=自碰撞开（默认），off=关自碰撞")
    args = ap.parse_args()

    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(
        headless=True, arm_self_collision=(args.mode == "base"))
    backend.setup()
    try:
        dq = run_tau(backend, f"{args.mode}] tau_cmd 复测", TAU_CMD)
    finally:
        backend.close()

    m = float(np.max(np.abs(dq)))
    print(f"\n=== mode={args.mode}  max|dq| = {m:.4f} rad ===")
    if args.mode == "off":
        if m > 0.05:
            print(">>> 判定：自碰撞实锤 —— 关自碰撞后臂可动，"
                  "exp1 伸展构型冻结根因 = articulation 自碰撞。"
                  "（与 --mode base 的 max|dq|~0.006 对比确认）")
        else:
            print(">>> 判定：自碰撞排除 —— 仍顶死，PhysX 求解器构型锁死，"
                  "需换 solver 迭代 / 减小伸展度。")
    print("=== DIAG SELFCOLLIDE3 DONE ===")


if __name__ == "__main__":
    main()
