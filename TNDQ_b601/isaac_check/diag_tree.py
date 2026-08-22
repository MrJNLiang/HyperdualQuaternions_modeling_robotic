#!/usr/bin/env python3
"""prim 树快照（diag_selfcollide AABB 过滤失败的调试）。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main():
    from interfaces.isaac_interface import IsaacB601Backend

    b = IsaacB601Backend(headless=True)
    b.setup()
    try:
        for prim in b.stage.Traverse():
            p = str(prim.GetPath())
            if p.startswith('/World/B601'):
                parts = p.strip('/').split('/')
                if len(parts) <= 5:
                    print(f'{prim.GetTypeName():10s} {p}')
    finally:
        b.close()


if __name__ == "__main__":
    main()
