#!/usr/bin/env python3
"""Q_F 接触对账 v2 —— 修正 link 归属的 AABB 相交/间隙检测。

probe_contact_check v1 的 _bbox_overlaps 在嵌套 Geometry 路径下
link 归属恒为 base_link（0 个有效 link）。本版按路径段中最深的已知
link 名归属，并输出非相邻 link 对的最小分离距离（负值=重叠）。

  [1] teleport Q_F 静置 1 步 -> 全 link AABB 对账；
  [2] B 力矩驱动 20 步 -> 复测（新相交对 = 锁死约束来源）；
  [3] 指尖世界高度（触地嫌疑直接量化）。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_contact_check2.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

Q_F = np.array([-0.2822, -2.6535, -1.4986, -1.3649, -0.7834, 0.1493])
TAU_FB = np.array([0.033, -1.347, 0.625, 0.281, -0.004, 0.003])

_LINKS = ["base_link", "link1", "link2", "link3", "link4", "link5",
          "link6", "gripper_link", "gripper_left", "gripper_right"]
_ADJACENT = {
    ("base_link", "link1"), ("link1", "link2"), ("link2", "link3"),
    ("link3", "link4"), ("link4", "link5"), ("link5", "link6"),
    ("link5", "gripper_link"), ("link6", "gripper_link"),
    ("gripper_link", "gripper_left"), ("gripper_link", "gripper_right"),
    ("link6", "gripper_left"), ("link6", "gripper_right"),
}
_GEOM_TYPES = ("Mesh", "Cube", "Sphere", "Cylinder", "Capsule", "Cone")


def _link_of(path):
    segs = path.strip("/").split("/")
    hits = [s for s in segs if s in _LINKS]
    return hits[-1] if hits else None


def collect_boxes(backend):
    from pxr import Usd, UsdGeom

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_], False)
    boxes = {}
    for prim in backend.stage.Traverse():
        if prim.GetTypeName() not in _GEOM_TYPES:
            continue
        link = _link_of(str(prim.GetPath()))
        if link is None:
            continue
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
    return boxes


def report(tag, boxes):
    names = sorted(boxes)
    print(f"[{tag}] link({len(names)}): {names}")
    hits = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            if (a, b) in _ADJACENT or (b, a) in _ADJACENT:
                continue
            lo_a, hi_a = boxes[a]
            lo_b, hi_b = boxes[b]
            if np.all(lo_a < hi_b) and np.all(hi_a > lo_b):
                ov = np.minimum(hi_a, hi_b) - np.maximum(lo_a, lo_b)
                print(f"    *** 相交: {a} <-> {b}  重叠 {np.round(ov, 4)}")
                hits.append((a, b))
            else:
                sep = np.maximum(lo_a - hi_b, lo_b - hi_a)
                d = float(np.linalg.norm(np.maximum(sep, 0.0)))
                if d < 0.05:
                    print(f"    近距: {a} <-> {b}  分离 {d * 1000:.1f} mm")
    for a in names:
        if boxes[a][0][2] < 2e-3:
            print(f"    *** 触地/近地: {a} z_min={boxes[a][0][2]:.4f}")
    if not hits:
        print("    无非相邻相交")
    return hits


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        backend.reset_to(Q_F)
        backend.apply_arm_torques(dyn.gravity_vector(Q_F))
        backend.step()
        print("=== [1] Q_F 静置 ===", flush=True)
        boxes = collect_boxes(backend)
        report("Q_F 静置", boxes)

        backend.reset_to(Q_F)
        g = dyn.gravity_vector(Q_F)
        for _ in range(20):
            backend.apply_arm_torques(g + TAU_FB)
            backend.step()
        q, _ = backend.get_joint_state()
        print(f"=== [2] B 驱动 20 步 dq={np.round(q - Q_F, 4).tolist()} ===",
              flush=True)
        boxes2 = collect_boxes(backend)
        report("B 20 步", boxes2)

        p_l = boxes.get("gripper_left")
        p_r = boxes.get("gripper_right")
        print("=== [3] 指尖高度 ===", flush=True)
        for nm, b in (("gripper_left", p_l), ("gripper_right", p_r)):
            if b is not None:
                print(f"    {nm}: z_min={b[0][2]:.4f} z_max={b[1][2]:.4f}")
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        backend.close()
    print("=== CONTACT CHECK2 DONE ===")


if __name__ == "__main__":
    main()
