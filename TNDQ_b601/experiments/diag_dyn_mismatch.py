"""对账误差分解诊断：重力 / 质量矩阵 / Coriolis 三路分离定位。"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import B601_URDF, JOINT_LOWER, JOINT_UPPER
from config.b601_dynamics import B601NominalDynamics

import pinocchio as pin  # noqa: E402

dyn = B601NominalDynamics()
model = pin.buildModelFromUrdf(B601_URDF)
jids = [model.getJointId(nm) for nm in ("gripper_joint1", "gripper_joint2")]
red = pin.buildReducedModel(model, jids, pin.neutral(model))
data = red.createData()

rng = np.random.default_rng(7)
q = rng.uniform(0.5 * JOINT_LOWER, 0.5 * JOINT_UPPER)
v = rng.uniform(-1.0, 1.0, 6)
a = rng.uniform(-1.0, 1.0, 6)
z = np.zeros(6)

print("q =", np.round(q, 4))

# (i) 纯重力：分离 质量/质心（提取与合成）
g_m = dyn.rnea(q, z, z, gravity=True)
g_p = np.asarray(pin.rnea(red, data, q, z, z))
print("\n(i) 纯重力 max|d| =", np.abs(g_m - g_p).max())
print("    mine:", np.array2string(g_m, precision=8))
print("    pin :", np.array2string(g_p, precision=8))
print("    diff:", np.array2string(g_m - g_p, precision=3))

# (ii) 零重力 + 单位加速度：质量矩阵 M 对账（分离惯量张量）
red.gravity.linear = np.zeros(3)
M_m = dyn.mass_matrix(q)
print("\n(ii) 零重力 M 列对账（mine vs pin 逐列 max|d|）：")
for k in range(6):
    e = np.zeros(6)
    e[k] = 1.0
    col_p = np.asarray(pin.rnea(red, data, q, z, e))
    print(f"    M[:,{k}]  max|d| = {np.abs(M_m[:, k] - col_p).max():.3e}"
          f"   mine[0..2]={np.round(M_m[:3, k], 8)} pin[0..2]={np.round(col_p[:3], 8)}")

# (iii) 零重力 Coriolis（v≠0, a=0）
C_m = dyn.rnea(q, v, z, gravity=False)
C_p = np.asarray(pin.rnea(red, data, q, v, z))
print("\n(iii) 零重力 Coriolis max|d| =", np.abs(C_m - C_p).max())
print("     mine:", np.array2string(C_m, precision=8))
print("     pin :", np.array2string(C_p, precision=8))
print("     diff:", np.array2string(C_m - C_p, precision=3))

# (iv) 完整 rnea（先恢复重力）
red.gravity.linear = np.array([0.0, 0.0, -9.81])
tau_m = dyn.rnea(q, v, a)
tau_p = np.asarray(pin.rnea(red, data, q, v, a))
print("\n(iv) 完整 RNEA max|d| =", np.abs(tau_m - tau_p).max())
