#!/usr/bin/env python3
"""Isaac Sim 6.0 + B601-DM 资产无头冒烟测试 v5 —— 构型/重力/动力学一致性诊断。

v1-v4 结论：
  - 力矩通道正常（applied/measured 都能读写），运行时 set_gains 可清零 drive（j4 已解锁）
  - 但零力矩下 j2/j3 仍纹丝不动，而实验 D 证明 j3 能被注入力矩推动
  - 矛盾指向：Isaac 中的 q=0 构型或重力与 URDF/Pinocchio 不一致

v5 诊断（drive 保持锁定让臂静止，然后读真实状态）：
  [1] PhysicsScene 重力设置
  [2] q=0 静态：各 link 世界坐标 vs Pinocchio FK（z = 0.0847/0.1402/0.1402/0.1942/0.2317/0.1917）
  [3] q=0 静态：measured_joint_efforts（静态锁定力矩 = -重力矩）
     Pinocchio tau_g = [0, -1.1142, -2.7891, -0.6727, 0, 0.0001]
  [4] 设 q=[0,-1,-1,0,0,0] 静态：measured vs Pinocchio tau_g(该构型)

运行方式：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/smoke_test.py
"""
import numpy as np

ASSET = "/home/liang/Documents/code/HyperdualQuaternions_modeling_robotic/reBot-Isaacsim/usd/reBot_B601_DM/reBot_B601_DM.usda"
ROBOT_PRIM = "/World/reBotArm"

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdPhysics, UsdGeom, Usd
import omni.usd

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 500.0, rendering_dt=1.0 / 60.0)
add_reference_to_stage(usd_path=ASSET, prim_path=ROBOT_PRIM)
art = SingleArticulation(prim_path=ROBOT_PRIM, name="smoke")
world.scene.add(art)
world.reset()
art.initialize()

stage = omni.usd.get_context().get_stage()
ARM = np.arange(6)

# ---- [1] 重力设置 ----
print("[1] PhysicsScene 重力：")
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Scene):
        gdir = prim.GetAttribute("physics:gravityDirection").Get()
        gmag = prim.GetAttribute("physics:gravityMagnitude").Get()
        print(f"    {prim.GetPath()}: direction = {gdir}, magnitude = {gmag}")

# ---- [2] q=0 各 link 世界坐标 ----
LINK_NAMES = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper_link"]
link_prims = {}
for prim in stage.Traverse():
    name = prim.GetName()
    if name in LINK_NAMES and name not in link_prims and prim.IsA(UsdGeom.Xformable):
        link_prims[name] = prim

print("[2] q=0 各 link 世界坐标（Isaac 实测 vs Pinocchio FK 预期）：")
PIN_Z = {"base_link": 0.0, "link1": 0.0847, "link2": 0.1402, "link3": 0.1402,
         "link4": 0.1942, "link5": 0.2317, "link6": 0.1917, "gripper_link": 0.1917}
for name in LINK_NAMES:
    prim = link_prims.get(name)
    if prim is None:
        print(f"    {name}: 未找到 prim")
        continue
    xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = np.array([xform[3][0], xform[3][1], xform[3][2]])
    print(f"    {name:12s}: Isaac 位置 = ({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f})"
          f"   Pinocchio z = {PIN_Z[name]:+.4f}")


# ---- [3] 静态测量力矩（drive 锁定状态，跑 50 步让瞬态衰减） ----
def read_static_torques(label):
    for _ in range(50):
        world.step(render=False)
    m = np.asarray(art.get_measured_joint_efforts(), dtype=float)
    q = np.asarray(art.get_joint_positions(), dtype=float)
    print(f"[{label}] q = {np.round(q[ARM], 4).tolist()}")
    print(f"        measured_tau = {np.round(m[ARM], 4).tolist()}")
    return m, q


print("[3] q=0 静态测量力矩（Pinocchio tau_g = [0, -1.1142, -2.7891, -0.6727, 0, 0.0001]）：")
read_static_torques("3")

# ---- [4] q=[0,-1,-1,0,0,0] 静态测量力矩 ----
art.set_joint_positions(np.array([0, -1.0, -1.0, 0, 0, 0, 0, 0]), joint_indices=np.arange(8))
art.set_joint_velocities(np.zeros(8), joint_indices=np.arange(8))
print("[4] q=[0,-1,-1,0,0,0] 静态测量力矩（Pinocchio tau_g = [0, 0.698, -2.7891, -0.6727, 0, 0]）：")
read_static_torques("4")

world.reset()
app.close()
print("=== SMOKE TEST v5 DONE ===")
