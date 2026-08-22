"""临时内省脚本：PhysxSchema articulation 类的 selfCollision 访问器。"""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import PhysxSchema

names = [n for n in dir(PhysxSchema) if "Articulation" in n]
print("PhysxSchema Articulation 类:", names)
for cls_name in names:
    cls = getattr(PhysxSchema, cls_name)
    meths = [m for m in dir(cls) if "elfCollision" in m]
    print(f"{cls_name}: SelfCollision 相关 = {meths}")

app.close()
