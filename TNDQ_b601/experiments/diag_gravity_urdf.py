"""重力对账归因实验（绕开 pinocchio 的独立验证）。

三方对比（同一组随机 q）：
  A) B601NominalDynamics.gravity_vector（手写 DH 链 + 回填参数）
  B) 手写 DH 链势能数值梯度（与 A 同链同参数，仅算法路径不同）
  C) URDF 链势能数值梯度（URDF 原始几何 + URDF 惯性原值）

预期归因结论：
  |A-B| ~ 1e-10（RNEA 代数正确，自检 [3] 的扩展采样版）
  |A-C| ~ 1e-6（= 与 pinocchio 对账同量级残差）
  => 残差来源 = DH 表与 URDF 几何的表示差
     （URDF 文本 rpy 6 位截断，如 -1.5708 vs 精确 pi/2，差 3.7e-6 rad），
     与 RNEA 算法、参数提取、pinocchio 均无关。
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import B601_URDF, B601_DH_TABLE
from config.b601_dynamics import (
    B601NominalDynamics, B601_LINK_MASS, B601_LINK_COM, _dh_transform)
from experiments.extract_inertia import parse_urdf, rpy_to_R, axis_rot, _T

links, joints = parse_urdf(B601_URDF)
_GRIP = ["gripper_link", "gripper_left", "gripper_right"]


def link_poses_q(q):
    """任意 q 下各 URDF link 世界位姿（手指 prismatic 锁定 q=0）。"""
    qmap = {f"joint{i+1}": float(q[i]) for i in range(6)}
    poses = {"base_link": np.eye(4)}
    pending = list(joints)
    while pending:
        rest = []
        for j in pending:
            if j["parent"] not in poses:
                rest.append(j)
                continue
            T = poses[j["parent"]]
            T = T @ _T(np.eye(3), j["xyz"]) @ _T(rpy_to_R(j["rpy"]), np.zeros(3))
            if j["type"] != "fixed":
                T = T @ _T(axis_rot(j["axis"], qmap.get(j["name"], 0.0)),
                           np.zeros(3))
            poses[j["child"]] = T.copy()
        if len(rest) == len(pending):
            raise ValueError("关节树不可达")
        pending = rest
    return poses


def potential_urdf(q):
    """URDF 链势能 U = sum m g z_c（link6 按体逐项计，手指 q=0 位形）。"""
    poses = link_poses_q(q)
    U = 0.0
    for i in range(6):
        names = (["link6"] + _GRIP) if i == 5 else [f"link{i+1}"]
        for nm in names:
            Tb = poses[nm]
            b = links[nm]
            c = Tb[:3, :3] @ b["com"] + Tb[:3, 3]
            U += b["mass"] * 9.81 * c[2]
    return U


def potential_dh(q):
    """手写 DH 链势能（与 B601NominalDynamics 同链同参数）。"""
    T = np.eye(4)
    U = 0.0
    for i in range(6):
        a, al, d, off, _ = B601_DH_TABLE[i]
        T = T @ _dh_transform(a, al, d, q[i] + off)
        c = T[:3, :3] @ B601_LINK_COM[i] + T[:3, 3]
        U += B601_LINK_MASS[i] * 9.81 * c[2]
    return U


def num_grad(f, q, h=1e-6):
    return np.array([(f(q + h * e) - f(q - h * e)) / (2 * h)
                     for e in np.eye(6)])


dyn = B601NominalDynamics()
rng = np.random.default_rng(7)
print("三方重力对账（A=RNEA, B=DH 链梯度, C=URDF 链梯度）：")
for k in range(5):
    q = rng.uniform(-1.0, -0.1, 6)
    g_a = dyn.gravity_vector(q)
    g_b = num_grad(potential_dh, q)
    g_c = num_grad(potential_urdf, q)
    print(f"  样本{k}: |A-B|={np.abs(g_a - g_b).max():.3e}  "
          f"|A-C|={np.abs(g_a - g_c).max():.3e}  "
          f"|B-C|={np.abs(g_b - g_c).max():.3e}")
