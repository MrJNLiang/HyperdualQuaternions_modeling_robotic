"""
实验一：定点控制 + 无接触基线（Setpoint & Open-Hold Baseline）
—— B601-DM Isaac Sim 力矩级验证。

任务（v14 开指保持模式）：机械臂从原生零位出发，夹爪先从 0 mm
张开到全开 140 mm（GRIPPER_BASELINE_WIDTH = GRIPPER_OPENING）并
保持恒开度，全程不接触/不夹住方块，然后执行完整运动流程：举高
-> 前移 -> 下降前段腕部
转动使夹爪朝下 -> 沿接近轴斜下插入到抓取位 -> 静置 -> 带载提升
（无载荷提升，同轨迹）。此模式排除接触干扰，验证轨迹跟踪精度与
控制稳定性，作为后续接触抓取实验的基线对照（恢复两段式受控
闭合 schedule 即可回到抓取模式）。

任务几何（config/params.py，保持不变）：
    - 初始位形 Q_INIT：模型原生形态（USD/URDF 零位）；
    - 立方体置于 6 cm 支柱顶（径向 0.40 m），顶抓抓取位
      SETPOINT_POS = 指尖端探到立方体中心下方 3 cm；
    - 姿态 R_TOOL_QUAT：tilt=170°（夹爪朝下偏外 10°，开合向水平）。

轨迹：v13 Pinocchio 关节空间规划（默认 --traj pinocchio，
run_lib.build_setpoint_goto_trajectory_pinocchio：全位姿 IK 链 + 关节
Hermite 样条 + pin.rnea 力矩校核，中间路标不停走）；另保留 kinematic
（笛卡尔 quintic+slerp）与 tndq（TNDQ 解析链）两路线作对比。

夹爪：make_gripper_open_hold_schedule —— 0.25 m/s 斜坡 0 -> 140 mm
（全开）后恒保持；每侧净空 47.5 mm，v9 三跑已实证插入段方块零
扰动（首二跑 60/100 mm 净空不足，分别撞落/擦碰方块，已修正），
插入/抓取位/提升全程手指不碰方块侧面。CSV gripper_width 列记录
全程开度。

运行方式（Isaac 官方运行时）：
    ~/isaacsim/python.sh TNDQ_b601/experiments/exp1_setpoint.py
输出：TNDQ_b601/results/exp1_setpoint.csv（列定义见 run_lib._CSV_COLUMNS）
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import GRIPPER_BASELINE_WIDTH, SETPOINT_HOLD_TIME

from run_lib import (run_tndq_experiment, build_setpoint_goto_trajectory,
                     build_setpoint_goto_trajectory_kinematic,
                     build_setpoint_goto_trajectory_pinocchio)


def make_gripper_open_hold_schedule(v_open=0.25):
    """开指保持调度（v14 无接触基线模式）。

    开度从 0 以 v_open 斜坡升到 GRIPPER_BASELINE_WIDTH = 140 mm
    （全开；斜坡防 KP=2000 位置 drive 阶跃响应冲击；同两段闭合的
    快接近段速率 0.25 m/s），随后全程恒保持。对 45 mm 方块每侧
    净空 47.5 mm（首二跑 60 mm 撞落方块、100 mm 插入段 ~8 mm
    擦碰：插入段跟踪滞后峰值 ~1 cm + 10° 倾角占有效净空 ~5 mm，
    净空须显著大于二者之和），确保插入/抓取位/提升全程手指不接触
    方块侧面 —— 排除接触干扰，验证纯轨迹跟踪精度与控制稳定性，
    作接触抓取实验的基线对照。
    """
    t1 = GRIPPER_BASELINE_WIDTH / v_open     # 斜坡终点时刻 [s]

    def schedule(t):
        if t < t1:
            return v_open * t
        return GRIPPER_BASELINE_WIDTH

    return schedule


def main():
    ap = argparse.ArgumentParser(description="B601 实验一：定点控制")
    ap.add_argument("--headless", type=int, default=1,
                    help="1=无头运行（默认，最快），0=可视化")
    ap.add_argument("--csv", type=str, default=None, help="CSV 输出路径")
    ap.add_argument("--self-collide", type=int, default=1,
                    help="1=articulation 自碰撞开（默认），0=关闭"
                         "（诊断/缓解运动中途构型锁）")
    ap.add_argument("--solver-pos-iter", type=int, default=None,
                    help="TGS 位置求解迭代次数（默认 None=资产默认）")
    ap.add_argument("--solver-vel-iter", type=int, default=None,
                    help="TGS 速度求解迭代次数（默认 None=资产默认）")
    ap.add_argument("--traj", type=str, default="pinocchio",
                    choices=["pinocchio", "kinematic", "tndq"],
                    help="期望轨迹生成：pinocchio=关节空间 IK+Hermite"
                         "样条+力矩校核（默认，v13）；kinematic=笛卡尔"
                         "quintic+slerp（v11 对比）；tndq=TNDQ 解析链"
                         "（对比备份）")
    args = ap.parse_args()

    csv_path = args.csv or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "results", "exp1_setpoint.csv")

    # 期望轨迹：从 Q_INIT（原生零位）出发，分段路标（举高/前移/
    # 转腕下降/插指/静置/提升）；默认 Pinocchio 关节空间规划
    # （v13；--traj kinematic/tndq 切回对比路线）。开指保持模式下
    # t_close（闭合时刻）不再使用，仅保留解包。
    if args.traj == "pinocchio":
        traj, t_move, _t_close = build_setpoint_goto_trajectory_pinocchio()
    elif args.traj == "kinematic":
        traj, t_move, _t_close = build_setpoint_goto_trajectory_kinematic()
    else:
        traj, t_move, _t_close = build_setpoint_goto_trajectory()
    duration = t_move + SETPOINT_HOLD_TIME

    # 夹爪调度：开指保持（斜坡张开到 60 mm 后恒保持，全程不接触
    # 方块；见 make_gripper_open_hold_schedule）
    gripper_schedule = make_gripper_open_hold_schedule()

    # 延迟 import：SimulationApp 必须先于一切 isaacsim import 创建
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=bool(args.headless),
                               arm_self_collision=bool(args.self_collide),
                               solver_pos_iter=args.solver_pos_iter,
                               solver_vel_iter=args.solver_vel_iter)
    backend.setup()
    try:
        summary = run_tndq_experiment(
            backend, traj, duration, csv_path,
            gripper_schedule=gripper_schedule,   # 开指保持（140 mm 全开恒开度）
            gripper_init=0.0,                    # 从并拢起步，按调度张开
            label="exp1-setpoint")
    finally:
        if args.headless:
            backend.close()
        else:
            # GUI 模式：不关闭 SimulationApp，保持 viewport 渲染/事件
            # 循环持续运转（world.is_playing() 空转一帧）——只渲染不
            # 推进物理，画面定格最终状态且相机可自由操作。直接
            # time.sleep 会导致渲染循环停转、视角卡死。
            # 终端 Ctrl+C 或关闭窗口退出。
            print("[exp1-setpoint] 实验结束：窗口保持打开，"
                  "可转视角观察；终端 Ctrl+C 退出", flush=True)
            while backend.sim_app is not None and backend.world.is_playing():
                backend.world.render()
            print("[exp1-setpoint] 窗口已关闭，进程退出", flush=True)
    return summary


if __name__ == "__main__":
    main()
