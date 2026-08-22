#!/usr/bin/env python3
"""USD 层惯量直读 —— 逐 link 质量 / 逐关节 rotorInertia+gearRatio。

probe_physx_mass 显示 M_phys 对角比名义大 10~989x 且 j1=j2=j3 完全
相等（疑似转子惯量主导）。本探针不经 PhysX 求解器，直接从 USD 属性
读取：
  [A] 每个 link prim 的 physics:mass（MassAPI，若未 authored 则
      报告 "auto(未author)"）；
  [B] 每个关节 prim 的 physxJoint:rotorInertia / gearRatio /
      maxJointVelocity / drive 参数。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_usd_props.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import ISAAC_ROBOT_USD

LINKS = ["base_link", "link1", "link2", "link3", "link4", "link5",
         "link6", "gripper_link", "gripper_left", "gripper_right"]
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
          "gripper_joint1", "gripper_joint2"]


def main():
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    from isaacsim.core.utils.stage import create_new_stage, get_current_stage
    from pxr import PhysxSchema, UsdPhysics

    create_new_stage()
    stage = get_current_stage()
    prim = stage.DefinePrim("/Robot", "Xform")
    prim.GetReferences().AddReference(ISAAC_ROBOT_USD)

    print("[A] link 质量（USD authored 值）：")
    for name in LINKS:
        found = None
        for p in stage.Traverse():
            if p.GetName() == name:
                found = p
                break
        if found is None:
            print(f"    {name}: <未找到 prim>")
            continue
        mass_api = UsdPhysics.MassAPI(found)
        if mass_api and mass_api.GetMassAttr().HasAuthoredValue():
            m = mass_api.GetMassAttr().Get()
            print(f"    {name}: mass={m}")
        else:
            print(f"    {name}: <未 author mass，由 collider 密度推算>")

    print("[B] 关节属性：")
    for name in JOINTS:
        found = None
        for p in stage.Traverse():
            if p.GetName() == name and p.GetPrimPath().pathString.find(
                    "/Physics/") >= 0:
                found = p
                break
        if found is None:
            print(f"    {name}: <未找到>")
            continue
        pj = PhysxSchema.PhysxJointAPI(found)
        props = [a.GetName() for a in pj.GetPrim().GetAuthoredAttributes()]
        vals = {}
        for aname in props:
            if any(k in aname for k in ("rotor", "armature", "gear",
                                        "friction", "maxJointVelocity",
                                        "drive")):
                vals[aname] = found.GetAttribute(aname).Get()
        print(f"    {name}: {vals}")
    print("=== USD PROPS DONE ===")
    app.close()


if __name__ == "__main__":
    main()
