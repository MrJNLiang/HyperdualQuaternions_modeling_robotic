#!/usr/bin/env python3
"""伸展构型顶死机制二分 —— measured efforts 对账 + articulation 属性。

diag_selfcollide [2] 已证：q_ss 下 tau_cmd（j2 净 -1.10 N*m）1s 只移动
j2 0.0004 rad —— 臂被物理顶死。本脚本二分机制：

    [1] 指令 vs 实测关节力矩（get_measured_joint_efforts）：若
        measured != cmd，力矩被中间层（drive/限位/摩擦）改写；
    [2] articulation 属性快照：selfCollisionEnabled / solver 类型 /
        关节 friction & armature（PhysxSchema 运行时值）；
    [3] 关闭全部臂 collider 后重施 tau_cmd：能动 -> 接触力（立方体/
        地面/外部）；仍不动 -> 求解器/关节层面。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/diag_effort_audit.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import Q_INIT
from config.b601_dynamics import B601NominalDynamics

Q_SS = np.array([-0.291, -2.543, -1.195, -1.559, -0.783, 0.234])
TAU_CMD = np.array([0.161, 3.772, -1.215, -0.314, 0.027, 0.001])


def audit_props(backend):
    """[2] articulation / 关节运行时属性快照。"""
    from pxr import PhysxSchema, UsdPhysics

    stage = backend.stage
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            pa = PhysxSchema.PhysxArticulationAPI(prim)
            print(f"[2] articulation root {prim.GetPath()}: "
                  f"selfCollision={pa.GetSelfCollisionEnabledAttr().Get()}")
            break
    # 关节级 PhysxJointAPI 属性（friction / armature / maxVelocity）
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        if "gripper" in str(prim.GetPath()):
            continue
        fr = prim.GetAttribute("physxJoint:jointFriction").Get()
        ar = prim.GetAttribute("physxJoint:armature").Get()
        mv = prim.GetAttribute("physxJoint:maxJointVelocity").Get()
        print(f"[2] {prim.GetName()}: jointFriction={fr} "
              f"armature={ar} maxJointVelocity={mv}")


def run_tau(backend, tag, tau, hold_s=1.0):
    backend.reset_to(Q_SS)
    n = int(round(hold_s / backend.physics_dt))
    q0 = measured0 = None
    for k in range(n):
        q, _ = backend.get_joint_state()
        if q0 is None:
            q0 = q.copy()
        backend.apply_arm_torques(tau)
        backend.step()
        if k == 10:
            measured0 = backend.get_measured_joint_efforts()
    q1, _ = backend.get_joint_state()
    m = backend.get_measured_joint_efforts()
    print(f"[{tag}] dq(1s) = {np.round(q1 - q0, 4).tolist()}")
    print(f"    cmd      = {np.round(tau, 3).tolist()}")
    print(f"    measured = {np.round(m, 3).tolist()}")
    print(f"    diff     = {np.round(m - tau, 3).tolist()}")
    return q1


def disable_colliders(backend):
    """关闭机器人全部 collider（USD 层 physics:collisionEnabled=False）。"""
    from pxr import UsdPhysics
    n = 0
    for prim in backend.stage.Traverse():
        p = str(prim.GetPath())
        if not p.startswith("/World/B601"):
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.CreateAttribute("physics:collisionEnabled", bool).Set(False)
            n += 1
    print(f"[3] 已关闭 {n} 个 collider")


def main():
    import argparse

    ap = argparse.ArgumentParser(description="力矩对账诊断")
    ap.add_argument("--headless", type=int, default=1,
                    help="1=无头（默认，快），0=开 Isaac 窗口实时观看")
    args = ap.parse_args()

    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=bool(args.headless))
    backend.setup()
    try:
        # [1] 力矩对账
        run_tau(backend, "1] q_ss + tau_cmd（collider 开）", TAU_CMD)
        # [2] 属性快照
        audit_props(backend)
        # [3] 关 collider 复测（属性变更下一步即生效，无需重建世界）
        disable_colliders(backend)
        run_tau(backend, "3] q_ss + tau_cmd（collider 关）", TAU_CMD)
    finally:
        backend.close()
    print("=== DIAG EFFORT AUDIT DONE ===")


if __name__ == "__main__":
    main()
