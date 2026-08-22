#!/usr/bin/env python3
"""转储 /World/B601 prim 树类型结构（调试 contact_check2 归属）。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main():
    from interfaces.isaac_interface import IsaacB601Backend

    backend = IsaacB601Backend(headless=True)
    backend.setup()
    try:
        n = 0
        for prim in backend.stage.Traverse():
            p = str(prim.GetPath())
            if not p.startswith("/World/B601"):
                continue
            n += 1
            if n <= 120:
                print(f"{prim.GetTypeName():12s} {p}")
        print("total prims:", n)
    finally:
        backend.close()


if __name__ == "__main__":
    main()
