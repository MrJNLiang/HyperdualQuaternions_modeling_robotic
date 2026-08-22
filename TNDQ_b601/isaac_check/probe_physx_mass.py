#!/usr/bin/env python3
"""PhysX 视图惯量直读 —— USD 资产 vs URDF/名义模型对账。

脉冲法受耦合污染不可靠（probe_mass_compare 出现负值）。改用
articulation view 直读：
  [A] get_masses()：连杆质量 vs URDF 表（base 之外 8 连杆）；
  [B] get_mass_matrices()（若可用）：Q_INIT 处物理 M vs 名义 M̂
      对角/全阵最大相对偏差。

运行：
    ~/isaacsim/python.sh TNDQ_b601/isaac_check/probe_physx_mass.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import Q_INIT


def main():
    from config.b601_dynamics import B601NominalDynamics
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    dyn = B601NominalDynamics()
    try:
        art = backend.articulation
        print("[A] DOF names:", list(art.dof_names), flush=True)
        getter_m = getattr(art, "get_masses", None)
        if getter_m is not None:
            masses = np.asarray(getter_m())[0]
            print("    连杆质量(PhysX) =", np.round(masses, 4).tolist(),
                  flush=True)
        else:
            print("    get_masses 不可用", flush=True)
        print("    URDF 质量参考: link1..6=[0.0841, 0.7104, 0.5249, "
              "0.1911, 0.1500, 0.0187] gripper_link=0.1818 "
              "fingers=0.0423x2", flush=True)

        backend.reset_to(Q_INIT)
        backend.apply_arm_torques(np.zeros(6))
        backend.step()
        getter_mm = getattr(art, "get_mass_matrices", None)
        if getter_mm is None:
            print("[B] get_mass_matrices 不可用", flush=True)
            return
        M8 = np.asarray(getter_mm())[0]
        idx = backend.arm_idx
        M_phys = M8[np.ix_(idx, idx)]
        M_nom = dyn.mass_matrix(Q_INIT)
        print("[B] Q_INIT 处 M 对账（PhysX vs 名义）：", flush=True)
        for j in range(6):
            print(f"    j{j + 1}: M_phys={M_phys[j, j]:.6f}  "
                  f"M̂={M_nom[j, j]:.6f}  "
                  f"ratio={M_phys[j, j] / M_nom[j, j]:6.2f}", flush=True)
        rel = np.abs(M_phys - M_nom) / np.maximum(np.abs(M_nom), 1e-9)
        print(f"    全阵最大相对偏差 = {np.max(rel):.2f}", flush=True)
        print("    M_phys =", np.round(M_phys, 5).tolist(), flush=True)
        print("    M_nom  =", np.round(M_nom, 5).tolist(), flush=True)
    finally:
        backend.close()
    print("=== PHYSX MASS DONE ===")


if __name__ == "__main__":
    main()
