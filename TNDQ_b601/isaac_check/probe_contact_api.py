#!/usr/bin/env python3
"""PhysX 接触报告 API 内省（Isaac Sim 6.0）。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main():
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    try:
        import omni.physx
        print("omni.physx:", [m for m in dir(omni.physx) if not m.startswith("_")])
        try:
            iface = omni.physx.get_physx_interface()
            print("iface contact:", [m for m in dir(iface)
                                     if "contact" in m.lower()])
        except Exception as e:
            print("get_physx_interface failed:", e)
        try:
            sim_iface = omni.physx.get_physx_simulation_interface()
            print("sim_iface contact:", [m for m in dir(sim_iface)
                                         if "contact" in m.lower()])
        except Exception as e:
            print("get_physx_simulation_interface failed:", e)
    finally:
        backend.close()


if __name__ == "__main__":
    main()
