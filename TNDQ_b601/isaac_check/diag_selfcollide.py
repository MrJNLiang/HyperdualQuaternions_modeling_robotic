#!/usr/bin/env python3
"""伸展构型自碰撞/冻结诊断 —— exp1 新几何 9.7cm 冻结根因。

现象链（results/exp1_setpoint.csv 修复后重跑）：
    - 纯仿真同栈完美收敛（diag_loop_sim A/B/C）；
    - Isaac 中臂 j4 冲过 IK 终态 0.33 rad 后冻结于伸展构型；
    - 冻结时 qddot_ref 持续饱和 30 rad/s^2，j2 净纠正力矩 -1.10 N*m
      拉不动（旧构型 0.184 N*m 即可移动 j2）；
    - 伸展构型（TCP 水平 0.575 m，接近臂展）下唯一能提供任意大反力
      的来源 = link 间自碰撞接触。

本脚本：
    [1] reset 到冻结构型 q_ss -> 纯重力补偿悬挂 1s（对照：接触应仍
        冻结/或弹开；无接触则如 Q_INIT 悬挂小幅漂移）；
    [2] 施加 exp1 末段指令力矩 tau_cmd 1s -> 各关节位移（哪些被顶死）；
    [3] 世界 bbox 几何对账：各 link 的世界 AABB 相交检测（非相邻
        link 对 + 与地平面），报自碰撞嫌疑对；
    [4] 对照组：Q_INIT 悬挂 1s（基线漂移）。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/diag_selfcollide.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import Q_INIT
from config.b601_dynamics import B601NominalDynamics

# exp1（新几何）CSV 末态
Q_SS = np.array([-0.291, -2.543, -1.195, -1.559, -0.783, 0.234])
TAU_CMD = np.array([0.161, 3.772, -1.215, -0.314, 0.027, 0.001])  # 末段指令

# 相邻 link 对（允许接触：关节连接）；base=base_link
_ADJACENT = {
    ("base_link", "link1"), ("link1", "link2"), ("link2", "link3"),
    ("link3", "link4"), ("link4", "link5"), ("link5", "link6"),
    ("link5", "gripper_link"), ("link6", "gripper_link"),
    # 手指与腕部几何上紧贴（USD 同款 finger rel，过滤组已排除指-指）
    ("gripper_link", "finger1"), ("gripper_link", "finger2"),
}


def _bbox_overlaps(backend, tag):
    """世界 AABB 相交检测：非相邻 link 对 + 全部 link vs 地面。"""
    from pxr import Usd, UsdGeom

    stage = backend.stage
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_], False)
    root = stage.GetPrimAtPath(backend._ROBOT_PRIM) \
        if hasattr(backend, "_ROBOT_PRIM") else None
    # 遍历机器人 prim 下所有带 collision 的 mesh/cube prim，按一级 link 名分组
    boxes = {}
    robot_path = "/World/B601"
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if not p.startswith(robot_path) or prim.GetTypeName() == "":
            continue
        # collision prim：Mesh/Cube/ Sphere 且在 Geometry 树内
        if prim.GetTypeName() not in ("Mesh", "Cube", "Sphere", "Cylinder",
                                      "Capsule", "Cone"):
            continue
        # link 名 = Geometry/<link>/... 的第一段
        rest = p[len(robot_path):].strip("/")
        parts = rest.split("/")
        if len(parts) < 3 or parts[0] != "Geometry":
            continue
        link = parts[1]
        try:
            bb = cache.ComputeWorldBound(prim)
            r = bb.ComputeAlignedRange()
            lo, hi = np.array(r.GetMin()), np.array(r.GetMax())
        except Exception:
            continue
        if link not in boxes:
            boxes[link] = [lo, hi]
        else:
            boxes[link][0] = np.minimum(boxes[link][0], lo)
            boxes[link][1] = np.maximum(boxes[link][1], hi)

    names = sorted(boxes)
    print(f"[{tag}] 参与 AABB 的 link（{len(names)}）: {names}")
    n_hit = 0
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            if (a, b) in _ADJACENT or (b, a) in _ADJACENT:
                continue
            lo_a, hi_a = boxes[a]
            lo_b, hi_b = boxes[b]
            if (np.all(lo_a < hi_b) and np.all(hi_a > lo_b)):
                ov = np.minimum(hi_a, hi_b) - np.maximum(lo_a, lo_b)
                print(f"    *** AABB 相交: {a} <-> {b}  重叠量 {np.round(ov,4)}")
                n_hit += 1
    # 地面 z=0（体对角允许 1mm 容差）
    for a in names:
        if boxes[a][0][2] < -1e-3:
            print(f"    *** 触地: {a} z_min={boxes[a][0][2]:.4f}")
            n_hit += 1
    if n_hit == 0:
        print("    无非相邻 AABB 相交 / 触地")
    return n_hit


def run_hold(backend, dyn, tag, q0, tau_bias=None, hold_s=1.0):
    backend.reset_to(q0)
    n = int(round(hold_s / backend.physics_dt))
    q_start = None
    for k in range(n):
        q, _ = backend.get_joint_state()
        if q_start is None:
            q_start = q.copy()
        tau = dyn.gravity_vector(q)
        if tau_bias is not None:
            tau = tau + tau_bias
        backend.apply_arm_torques(tau)
        backend.step()
    q_end, _ = backend.get_joint_state()
    print(f"[{tag}] 悬挂 {hold_s}s 位移 dq = {np.round(q_end - q_start, 4).tolist()}")
    return q_end


def main():
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()

    try:
        # [1] 冻结构型纯重力悬挂（对照 Q_INIT）
        run_hold(backend, dyn, "1] q_ss 重力悬挂", Q_SS)
        run_hold(backend, dyn, "4] Q_INIT 重力悬挂", Q_INIT)

        # [2] 冻结构型 + exp1 末段指令力矩（哪些关节被顶死）
        backend.reset_to(Q_SS)
        n = int(round(1.0 / backend.physics_dt))
        q0 = None
        for k in range(n):
            q, _ = backend.get_joint_state()
            if q0 is None:
                q0 = q.copy()
            backend.apply_arm_torques(TAU_CMD)
            backend.step()
        q1, qd1 = backend.get_joint_state()
        print(f"[2] q_ss + tau_cmd 1s 位移 dq = {np.round(q1 - q0, 4).tolist()}"
              f"  |qd|max={np.abs(qd1).max():.4f} rad/s")

        # [3] 世界 AABB 对账（重力悬挂静止时刻）
        _bbox_overlaps(backend, "3] q_ss AABB")
        backend.reset_to(Q_INIT)
        for _ in range(50):
            q, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(q))
            backend.step()
        _bbox_overlaps(backend, "3b] Q_INIT AABB")
    finally:
        backend.close()
    print("=== DIAG SELFCOLLIDE DONE ===")


if __name__ == "__main__":
    main()
