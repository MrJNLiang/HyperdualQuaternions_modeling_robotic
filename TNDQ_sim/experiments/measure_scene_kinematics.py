"""
从 CoppeliaSim 场景实测 LBR4p 运动学结构（停止态，只读+临时置位后恢复）：
  - q=0 时各关节对象在基座系下的原点与 z 轴（CoppeliaSim 关节轴 = 对象局部 z）
  - 末端 connection 在基座系下的位姿
  - LBR4p / RG2 脚本内容前 40 行（判断是否抢关节控制权）
输出用于回填 config/params.py::KUKA_LBR4_DH。
"""

import os
import sys
import time

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from interfaces.coppeliasim_interface import (
    probe_joint_handles, TIP_PATH_CANDIDATES, BASE_PATH_CANDIDATES,
)


def R_from_quat_xyzw(qx, qy, qz, qw):
    x, y, z, w = qx, qy, qz, qw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    client = RemoteAPIClient(host="localhost", port=23000)
    sim = client.require("sim")
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        time.sleep(0.5)

    joints, _ = probe_joint_handles(sim)
    tip = next(h for h in (sim.getObject(p) for p in TIP_PATH_CANDIDATES
                           if _try(sim, p)) if h is not None)
    base = sim.getObject(BASE_PATH_CANDIDATES[0])

    q_backup = [sim.getJointPosition(h) for h in joints]
    for h in joints:
        sim.setJointPosition(h, 0.0)

    print("=== q=0 时关节对象基座系位姿（轴 = 对象局部 z）===")
    for i, h in enumerate(joints):
        p = np.array(sim.getObjectPosition(h, base))
        qx, qy, qz, qw = sim.getObjectQuaternion(h, base)
        R = R_from_quat_xyzw(qx, qy, qz, qw)
        z = R[:, 2]
        print(f"  joint{i + 1}: p=[{p[0]:+.4f} {p[1]:+.4f} {p[2]:+.4f}]  "
              f"axis=[{z[0]:+.4f} {z[1]:+.4f} {z[2]:+.4f}]")

    p = np.array(sim.getObjectPosition(tip, base))
    qx, qy, qz, qw = sim.getObjectQuaternion(tip, base)
    R = R_from_quat_xyzw(qx, qy, qz, qw)
    print(f"  tip    : p=[{p[0]:+.4f} {p[1]:+.4f} {p[2]:+.4f}]")
    print(f"  tip R (基座系列向量 x|y|z):\n{np.array_str(R, precision=4, suppress_small=True)}")

    for h, qi in zip(joints, q_backup):
        sim.setJointPosition(h, float(qi))

    print("\n=== 脚本内容 ===")
    for h in sim.getObjectsInTree(base, sim.sceneobject_script, 0):
        alias = sim.getObjectAlias(h, 2)
        try:
            txt = sim.getScriptStringParam(h, sim.scriptstringparam_text)
            if isinstance(txt, (bytes, bytearray)):
                txt = txt.decode("utf-8", "replace")
            head = "\n".join(txt.splitlines()[:40])
            print(f"--- {alias} （前 40 行）---\n{head}\n")
        except Exception as exc:
            print(f"--- {alias} 读取失败: {exc}")
    return 0


def _try(sim, path):
    try:
        sim.getObject(path)
        return True
    except Exception:
        return False


if __name__ == "__main__":
    sys.exit(main())
