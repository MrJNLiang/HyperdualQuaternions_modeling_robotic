#!/usr/bin/env python3
"""自碰撞终局诊断 —— articulation selfCollision 开关 + link 世界位姿。

排除清单（已证）：drive 清零 / jointFriction=0 / armature=0 / 限位不顶
（j4 余量 0.31 rad）/ 重力准 / measured==cmd / 手不接触立方体与地面。
特征：顶死时 j1-j5 冻结而 j6 可动 —— 链中段被锁、末端自由，
PhysX articulation 自碰撞（若开启）在伸展构型的 link 接触嫌疑最大。

    [1] 定位 articulation root，读 selfCollisionEnabled；
    [2] q_ss + tau_cmd 复测（基线，臂顶死）；
    [3] USD 层 author selfCollisionEnabled=False -> 复测；
        动 -> 自碰撞实锤（[4] 定位接触对）；仍不动 -> 求解器锁死；
    [4] link 世界原点位姿表（伸展构型几何目测）。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/diag_selfcollide2.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import Q_INIT
from config.b601_dynamics import B601NominalDynamics

Q_SS = np.array([-0.291, -2.543, -1.195, -1.559, -0.783, 0.234])
TAU_CMD = np.array([0.161, 3.772, -1.215, -0.314, 0.027, 0.001])


def find_articulation_root(backend):
    from pxr import UsdPhysics, PhysxSchema

    for prim in backend.stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim
    return None


def read_self_collision(prim):
    from pxr import PhysxSchema

    pa = PhysxSchema.PhysxArticulationAPI.Get(backend_stage(), prim.GetPath()) \
        if False else None
    attr = prim.GetAttribute("physxArticulation:selfCollisionEnabled")
    return attr.Get() if attr else None


def backend_stage():
    return _STAGE[0]


_STAGE = [None]


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
    print(f"[{tag}] dq(1s) = {np.round(dq, 4).tolist()}")
    return dq


def link_pose_table(backend, tag):
    """link（Geometry 嵌套 Xform 一级+嵌套层）世界原点表 + 非相邻距离。"""
    from pxr import Usd, UsdGeom

    stage = backend.stage
    # 只取 Geometry 下嵌套链的每一级 link Xform（名字 in 集合）
    links = ["base_link", "link1", "link2", "link3", "link4", "link5",
             "link6", "gripper_link", "finger1", "finger2"]
    poses = {}
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if "/World/B601/Geometry/" not in p:
            continue
        name = prim.GetName()
        if name not in links or name in poses:
            continue
        xf = UsdGeom.Xformable(prim)
        if xf:
            m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            t = m.ExtractTranslation()
            poses[name] = np.array([t[0], t[1], t[2]])
    print(f"[{tag}] link 世界原点:")
    for k in links:
        if k in poses:
            print(f"    {k:14s} {np.round(poses[k], 4).tolist()}")
    # 非相邻对距离 < 15cm 的列出（目测嫌疑）
    adj = {("base_link", "link1"), ("link1", "link2"), ("link2", "link3"),
           ("link3", "link4"), ("link4", "link5"), ("link5", "link6"),
           ("link6", "gripper_link"), ("link5", "gripper_link"),
           ("gripper_link", "finger1"), ("gripper_link", "finger2")}
    print("    非相邻对距离 < 0.16 m:")
    ks = [k for k in links if k in poses]
    for i in range(len(ks)):
        for j in range(i + 1, len(ks)):
            a, b = ks[i], ks[j]
            if (a, b) in adj or (b, a) in adj:
                continue
            d = float(np.linalg.norm(poses[a] - poses[b]))
            if d < 0.16:
                print(f"      {a} <-> {b}: {d:.4f} m")
    return poses


def main():
    import argparse

    ap = argparse.ArgumentParser(description="自碰撞诊断")
    ap.add_argument("--headless", type=int, default=1,
                    help="1=无头（默认，快），0=开 Isaac 窗口实时观看")
    args = ap.parse_args()

    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=bool(args.headless))
    backend.setup()
    _STAGE[0] = backend.stage
    try:
        # [1] articulation root + selfCollision
        root = find_articulation_root(backend)
        if root is None:
            print("[1] 未找到 ArticulationRootAPI prim")
        else:
            attr = root.GetAttribute("physxArticulation:selfCollisionEnabled")
            print(f"[1] articulation root = {root.GetPath()}  "
                  f"selfCollisionEnabled = {attr.Get() if attr else '未 authored'}")

        # [2] 基线（顶死复现）
        run_tau(backend, "2] q_ss + tau_cmd（自碰撞现状）", TAU_CMD)

        # [4] link 位姿（顶死构型几何目测）
        link_pose_table(backend, "4] q_ss link 表")

        # [3] 关自碰撞复测
        if root is not None:
            attr = root.GetAttribute("physxArticulation:selfCollisionEnabled")
            if attr is None:
                attr = root.CreateAttribute(
                    "physxArticulation:selfCollisionEnabled", bool)
            attr.Set(False)
            print("[3] 已 author selfCollisionEnabled=False")
            run_tau(backend, "3] q_ss + tau_cmd（自碰撞关）", TAU_CMD)
    finally:
        backend.close()
    print("=== DIAG SELFCOLLIDE2 DONE ===")


if __name__ == "__main__":
    main()
