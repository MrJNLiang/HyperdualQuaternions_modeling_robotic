"""
CoppeliaSim 连接诊断脚本（只读，不启动/停止仿真，不移动任何对象）。

用途：在正式仿真/诊断实验前，验证与本地 CoppeliaSim
（/home/liang/Solfware/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04）的
ZMQ Remote API 连接（localhost:23000，与 hdq_hinf_coppeliasim 项目
跑通的配置一致），并核查场景对象是否满足 TNDQ_sim 对接要求。

运行（dq_hinf conda 环境）：
    /home/liang/miniconda3/envs/dq_hinf/bin/python experiments/check_coppeliasim_connection.py
"""

import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from interfaces.coppeliasim_interface import (
    probe_joint_handles, TIP_PATH_CANDIDATES,
    BASE_PATH_CANDIDATES, CUP_PATH_CANDIDATES, CHAIR_PATH_CANDIDATES,
)


def probe(sim, paths, label):
    for p in paths:
        try:
            h = sim.getObject(p)
            print(f"  [OK]   {label:12s} {p} -> handle {h} "
                  f"({sim.getObjectAlias(h, 2)})")
            return h
        except Exception:
            continue
    print(f"  [MISS] {label:12s} 候选均未命中: {paths}")
    return None


def main():
    host, port = "localhost", 23000
    print(f"=== 1. ZMQ Remote API 连接 {host}:{port} ===")
    client = RemoteAPIClient(host=host, port=port)
    sim = client.require("sim")
    ver = sim.getInt32Param(sim.intparam_program_full_version)
    print(f"  连接成功。CoppeliaSim 版本号 = {ver} "
          f"({ver // 1000000}.{ver // 10000 % 100}.{ver // 100 % 100} rev{ver % 100})")

    print("\n=== 2. 仿真状态与场景 ===")
    state = sim.getSimulationState()
    state_name = {
        sim.simulation_stopped: "stopped",
        sim.simulation_paused: "paused",
        sim.simulation_advancing_running: "running",
    }.get(state, f"code={state}")
    print(f"  仿真状态: {state_name}")
    try:
        scene = sim.getStringParam(sim.stringparam_scene_path_and_name)
        print(f"  当前场景: {scene if scene else '(未保存/空场景)'}")
    except Exception as exc:
        print(f"  场景路径读取失败: {exc}")
    print(f"  引擎步长: {sim.getSimulationTimeStep() * 1e3:.1f} ms")

    print("\n=== 3. 关节句柄探测（候选路径 + 基座子树回退） ===")
    joints, used = probe_joint_handles(sim)
    if joints:
        print(f"  [OK] 命中候选组: {used[0]} ... {used[-1]}")
        for p, h in zip(used, joints):
            q = sim.getJointPosition(h)
            print(f"    {p} -> handle {h}  q = {q:+.4f} rad")
    else:
        print("  [MISS] 候选路径与基座子树遍历均未命中——场景中可能未加载 LBR4+ 模型")

    print("\n=== 4. 其余场景对象 ===")
    probe(sim, TIP_PATH_CANDIDATES, "末端 tip")
    probe(sim, BASE_PATH_CANDIDATES, "基座 base")
    probe(sim, CUP_PATH_CANDIDATES, "杯子 cup")
    probe(sim, CHAIR_PATH_CANDIDATES, "椅子 chair")

    print("\n=== 5. 场景对象树（前 40 项） ===")
    try:
        handles = sim.getObjectsInTree(sim.handle_scene)
        for h in handles[:40]:
            print(f"    {sim.getObjectAlias(h, 2)}")
        if len(handles) > 40:
            print(f"    ... 共 {len(handles)} 个对象")
        if not handles:
            print("    (场景为空)")
    except Exception as exc:
        print(f"    枚举失败: {exc}")

    ok = bool(joints)
    print("\n=== 结论 ===")
    if ok:
        print("  连接与场景核查通过，可进行后续仿真与诊断。")
    else:
        print("  ZMQ 连接正常，但场景缺少机械臂模型，"
              "请在 CoppeliaSim 中加载 KUKALBR4+_sim.ttt 场景后重试。")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
