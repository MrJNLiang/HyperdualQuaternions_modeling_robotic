"""
引擎侧重力保持力矩实测 vs 名义模型 ĝ(q) 逐关节对比。

方法：关节切位置模式、目标 = 当前位形，同步步进跑 3 s 让 PID 完全稳
定后，取最后 1 s 的 sim.getJointForce 平均值 = 引擎真实重力保持力矩
（静态、无瞬态），与 LBR4NominalDynamics.gravity_vector(q) 直接对比。
腕部关节有效惯量仅 ~1e-2 kg m²，ĝ 误差 0.5 N·m 即数十 rad/s² 加速度
—— 这是定位塌臂/提前终止的最直接证据链。

运行（CoppeliaSim 已加载场景、停止态）：
    /home/liang/miniconda3/envs/dq_hinf/bin/python experiments/measure_gravity_torque.py
"""

import os
import sys
import time

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from config import params
from config.lbr4_dynamics import LBR4NominalDynamics
from interfaces.coppeliasim_interface import probe_joint_handles


SETTLE_STEPS = 600      # 3 s @ 5 ms
AVG_STEPS = 200         # 最后 1 s 取平均


def hold_torque(sim, joints, q):
    """位置模式保持 q，返回稳态 getJointForce 平均值（7,）。"""
    for h, qi in zip(joints, q):
        sim.setJointPosition(h, float(qi))
        sim.setJointTargetPosition(h, float(qi))
    sim.setStepping(True)
    sim.startSimulation()
    taus = []
    for k in range(SETTLE_STEPS):
        sim.step()
        if k >= SETTLE_STEPS - AVG_STEPS:
            taus.append([sim.getJointForce(h) for h in joints])
    sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.05)
    return np.mean(np.array(taus), axis=0)


def main():
    client = RemoteAPIClient(host="localhost", port=23000)
    sim = client.require("sim")
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        time.sleep(0.5)

    joints, _ = probe_joint_handles(sim)

    # 停用 LBR4p 演示脚本（避免抢控制权）
    try:
        script = sim.getObject("/LBR4p/Script")
        try:
            sim.setBoolProperty(script, "enabled", False)
        except Exception:
            sim.setObjectInt32Param(script, sim.scriptintparam_enabled, 0)
    except Exception:
        pass

    # 引擎步长 5 ms
    sim.setFloatParam(sim.floatparam_simulation_time_step,
                      params.COPPELIA_DT_TARGET)

    # 备份并切位置模式（高力矩上限保证 PID 能稳住）
    mode_backup = [sim.getObjectInt32Param(h, sim.jointintparam_dynctrlmode)
                   for h in joints]
    q_backup = [sim.getJointPosition(h) for h in joints]
    for h in joints:
        sim.setObjectInt32Param(h, sim.jointintparam_dynctrlmode,
                                sim.jointdynctrl_position)
        sim.setJointTargetForce(h, 300.0)

    dyn = LBR4NominalDynamics(params.KUKA_LBR4_DH,
                              motor_inertia=np.zeros(7))

    poses = [
        ("Q_INIT_TASK", params.Q_INIT_TASK.copy()),
        ("bent", np.array([0.0, 0.6, 0.0, -1.0, 0.0, 0.8, 0.0])),
        ("elbow90", np.array([0.0, 1.2, 0.0, -1.57, 0.0, 0.0, 0.0])),
    ]
    np.set_printoptions(precision=3, suppress=True, linewidth=160)
    for name, q in poses:
        tau_eng = hold_torque(sim, joints, q)
        g_hat = dyn.gravity_vector(q)
        print(f"=== {name}  q={np.round(q, 3)}")
        print(f"  engine hold  = {tau_eng}")
        print(f"  model g_hat  = {g_hat}")
        print(f"  diff         = {tau_eng - g_hat}")

    # 恢复
    for h, m, qi in zip(joints, mode_backup, q_backup):
        sim.setObjectInt32Param(h, sim.jointintparam_dynctrlmode, m)
        sim.setJointPosition(h, float(qi))
    return 0


if __name__ == "__main__":
    sys.exit(main())
