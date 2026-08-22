#!/usr/bin/env python3
"""USD（PhysX 真值）vs URDF 连杆惯量对账：质量 / 质心 / 惯量主轴。

v9 四跑归因：抓取位深折叠构型 ~1 cm/s 慢漂（重力残差饱和型）。
质量已验相等（t7c4），此处补验质心与惯量张量——USD 导入器若重算
或丢失惯量，深折叠下重力力矩误差可达 N*m 级，恰能解释慢漂。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_inertia_compare.py
"""
import os, sys
import numpy as np
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P


def urdf_inertials():
    tree = ET.parse(P.B601_URDF)
    out = {}
    for link in tree.getroot().findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        m = float(inertial.find("mass").get("value"))
        origin = inertial.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
        inertia = inertial.find("inertia")
        ixx = float(inertia.get("ixx")); ixy = float(inertia.get("ixy"))
        ixz = float(inertia.get("ixz")); iyy = float(inertia.get("iyy"))
        iyz = float(inertia.get("iyz")); izz = float(inertia.get("izz"))
        I = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        out[link.get("name")] = (m, xyz, I)
    return out


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    from pxr import UsdPhysics, PhysxSchema
    stage = backend.stage
    urdf = urdf_inertials()
    print(f"{'link':14s} {'m_USD':>8s} {'m_URDF':>8s}  "
          f"{'com_USD':>26s} {'com_URDF':>26s}  惯量主轴比")
    for prim in stage.Traverse():
        if not prim.IsA(__import__("pxr").UsdGeom.Xformable):
            continue
        rb = UsdPhysics.RigidBodyAPI(prim)
        if not rb:
            continue
        name = prim.GetName()
        mass_api = UsdPhysics.MassAPI(prim)
        m_usd = mass_api.GetMassAttr().Get() if mass_api.GetMassAttr() else None
        com = mass_api.GetCenterOfMassAttr().Get() if mass_api.GetCenterOfMassAttr() else None
        inertia_attr = prim.GetAttribute("physics:inertia")
        inertia = inertia_attr.Get() if inertia_attr and inertia_attr.IsValid() else None
        diag_attr = prim.GetAttribute("physics:diagonalInertia")
        diag = diag_attr.Get() if diag_attr and diag_attr.IsValid() else None
        if m_usd is None or name not in urdf:
            continue
        m_u, xyz_u, I_u = urdf[name]
        com_u = tuple(np.round(xyz_u, 5))
        com_s = tuple(np.round(np.array(com), 5)) if com is not None else None
        # 惯量主轴（特征值）对比
        eig_u = np.sort(np.linalg.eigvalsh(I_u))
        if inertia is not None:
            I_usd = np.array(inertia, dtype=float)
            if I_usd.size == 9:
                I_usd = I_usd.reshape(3, 3)
            else:  # 对角形式
                I_usd = np.diag(I_usd)
            eig_s = np.sort(np.linalg.eigvalsh(I_usd))
            ratio = np.round(eig_s / np.maximum(eig_u, 1e-12), 3)
        elif diag is not None:  # diagonalInertia+principalAxes 形式
            eig_s = np.sort(np.array(diag, dtype=float))
            ratio = np.round(eig_s / np.maximum(eig_u, 1e-12), 3)
        else:
            ratio = "无惯量attr"
        print(f"{name:14s} {m_usd:8.4f} {m_u:8.4f}  "
              f"{str(com_s):>26s} {str(com_u):>26s}  {ratio}")
    backend.close()


if __name__ == "__main__":
    main()
