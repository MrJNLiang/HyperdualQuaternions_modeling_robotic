"""
FK 对齐诊断（场景篇 §1.2 第 5/8/9 项）—— 只在 stopped 状态下操作，不启动仿真。

诊断内容：
  1. TNDQ 链 FK（config/params.py::KUKA_LBR4_DH）vs CoppeliaSim 场景实测
     末端位姿（基座系），多组关节位形 + 逐关节激励，定位 DH 失配行；
  2. 关节动力学控制模式核查（力矩模式 sim.jointdynctrl_force 是否可用）；
  3. LBR4p 自带脚本核查（自带演示脚本会抢关节控制权，必须停用）；
  4. 引擎侧连杆质量/动态属性清单 vs 名义动力学表（重力补偿失配 = 塌臂根源）；
  5. 引擎步长核查（50 ms 默认值对力矩闭环过粗）。

运行（dq_hinf conda 环境，CoppeliaSim 已加载 KUKALBR4+_sim.ttt 且处于停止态）：
    /home/liang/miniconda3/envs/dq_hinf/bin/python experiments/diagnose_fk_alignment.py
"""

import os
import sys

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from config import params
from config.lbr4_dynamics import LBR4_LINK_MASS
from core.kinematics import TNDQSerialChain
from core.dq_algebra import dq_translation, dq_rotation
from interfaces.coppeliasim_interface import (
    probe_joint_handles, TIP_PATH_CANDIDATES, BASE_PATH_CANDIDATES,
    _quat_xyzw_to_wxyz, _pose_to_dq,
)


def read_tip_dq(sim, tip, base):
    """末端相对基座位姿 -> 单位 DQ（[w,x,y,z] 约定）。"""
    p = np.array(sim.getObjectPosition(tip, base))
    r = _quat_xyzw_to_wxyz(sim.getObjectQuaternion(tip, base))
    return _pose_to_dq(p, r)


def pose_residual(x_model, x_sim):
    """位置残差 [m] 与姿态残差 [rad]。"""
    dp = np.linalg.norm(dq_translation(x_model) - dq_translation(x_sim))
    dr = 2.0 * np.arccos(min(1.0, abs(float(
        dq_rotation(x_model) @ dq_rotation(x_sim)))))
    return dp, dr


def set_q(sim, joints, q):
    for h, qi in zip(joints, q):
        sim.setJointPosition(h, float(qi))


def main():
    client = RemoteAPIClient(host="localhost", port=23000)
    sim = client.require("sim")

    state = sim.getSimulationState()
    if state != sim.simulation_stopped:
        print("[warn] 仿真未处于停止态，先停止再诊断 ...")
        sim.stopSimulation()
        import time
        time.sleep(0.5)

    joints, used = probe_joint_handles(sim)
    assert joints, "关节句柄探测失败"
    tip = None
    for pth in TIP_PATH_CANDIDATES:
        try:
            tip = sim.getObject(pth)
            break
        except Exception:
            continue
    base = None
    for pth in BASE_PATH_CANDIDATES:
        try:
            base = sim.getObject(pth)
            break
        except Exception:
            continue
    assert tip is not None and base is not None

    chain = TNDQSerialChain(params.KUKA_LBR4_DH)
    q_backup = np.array([sim.getJointPosition(h) for h in joints])

    # ---- 1. FK 对齐：多组位形 ------------------------------------------------
    print("=== 1. FK 对齐（TNDQ 模型 vs 场景实测，基座系） ===")
    rng = np.random.default_rng(7)
    cases = [("zero", np.zeros(7)),
             ("Q_INIT_TASK", params.Q_INIT_TASK.copy()),
             ("Q_INIT", params.Q_INIT.copy())]
    cases += [(f"rand{k}", rng.uniform(-0.8, 0.8, 7)) for k in range(3)]
    worst = (0.0, 0.0)
    for name, q in cases:
        set_q(sim, joints, q)
        x_model = chain.fkm(q)
        x_sim = read_tip_dq(sim, tip, base)
        dp, dr = pose_residual(x_model, x_sim)
        worst = (max(worst[0], dp), max(worst[1], dr))
        pm = dq_translation(x_model)
        ps = dq_translation(x_sim)
        print(f"  {name:12s} |dp|={dp * 1e3:7.2f} mm  dth={np.rad2deg(dr):7.3f} deg"
              f"   p_model=[{pm[0]:+.3f} {pm[1]:+.3f} {pm[2]:+.3f}]"
              f"   p_sim=[{ps[0]:+.3f} {ps[1]:+.3f} {ps[2]:+.3f}]")
    verdict = "OK（<1mm/0.1°）" if worst[0] < 1e-3 and worst[1] < np.deg2rad(0.1) \
        else "超限 —— DH/基座/末端需修正"
    print(f"  最大残差: |dp|={worst[0] * 1e3:.2f} mm, "
          f"dth={np.rad2deg(worst[1]):.3f} deg -> {verdict}")

    # ---- 2. 逐关节激励定位失配行 ----------------------------------------------
    print("\n=== 2. 逐关节激励（q=0 基础上单关节 +0.4 rad）===")
    for i in range(7):
        q = np.zeros(7)
        q[i] = 0.4
        set_q(sim, joints, q)
        dp, dr = pose_residual(chain.fkm(q), read_tip_dq(sim, tip, base))
        print(f"  joint{i + 1}: |dp|={dp * 1e3:7.2f} mm  dth={np.rad2deg(dr):7.3f} deg")

    set_q(sim, joints, q_backup)   # 恢复原位形

    # ---- 3. 关节模式与力矩接口 ------------------------------------------------
    print("\n=== 3. 关节动力学控制模式 ===")
    for i, h in enumerate(joints):
        try:
            mode = sim.getObjectInt32Param(h, sim.jointintparam_dynctrlmode)
            names = {sim.jointdynctrl_free: "free",
                     sim.jointdynctrl_force: "force",
                     sim.jointdynctrl_velocity: "velocity",
                     sim.jointdynctrl_position: "position",
                     sim.jointdynctrl_spring: "spring"}
            print(f"  joint{i + 1}: dynctrlmode={names.get(mode, mode)}")
        except Exception as exc:
            print(f"  joint{i + 1}: 读取失败 {exc}")

    # ---- 4. LBR4p 自带脚本 -----------------------------------------------------
    print("\n=== 4. LBR4p 子树脚本核查 ===")
    try:
        scripts = sim.getObjectsInTree(base, sim.sceneobject_script, 0)
        if not scripts:
            print("  无脚本对象。")
        for h in scripts:
            alias = sim.getObjectAlias(h, 2)
            try:
                # objintparam_ 通用可见性参数不适用；脚本禁用状态用模型属性读取
                enabled = sim.getObjectInt32Param(
                    h, sim.scriptintparam_enabled)
            except Exception:
                enabled = "?"
            print(f"  {alias}  enabled={enabled}")
    except Exception as exc:
        print(f"  脚本枚举失败: {exc}")

    # ---- 5. 引擎侧质量 vs 名义表 -----------------------------------------------
    print("\n=== 5. 引擎侧连杆质量（动态 shape） vs 名义动力学表 ===")
    shapes = sim.getObjectsInTree(base, sim.sceneobject_shape, 0)
    total = 0.0
    dyn_masses = []
    for h in shapes:
        alias = sim.getObjectAlias(h, 2)
        try:
            static = sim.getObjectInt32Param(h, sim.shapeintparam_static)
            if static:
                continue
            m = sim.getShapeMass(h)
            dyn_masses.append((alias, m))
            total += m
        except Exception:
            continue
    for alias, m in dyn_masses:
        print(f"  {alias:24s} m={m:7.3f} kg")
    print(f"  引擎侧动态质量合计 = {total:.3f} kg；"
          f"名义表合计 = {LBR4_LINK_MASS.sum():.3f} kg")

    # ---- 6. 引擎步长 -------------------------------------------------------------
    print("\n=== 6. 时序 ===")
    print(f"  引擎步长 = {sim.getSimulationTimeStep() * 1e3:.1f} ms "
          f"(params.DT = {params.DT * 1e3:.1f} ms)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
