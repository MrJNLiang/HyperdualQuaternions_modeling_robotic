"""
实验一：定点控制 + 抓取（Setpoint & Grasp）—— B601-DM Isaac Sim 力矩级验证。

任务（用户要求 v7）：机械臂从原生零位出发，举高 -> 前移 -> 下降前段
腕部转动使夹爪朝下 -> 下降插指 -> 闭合夹住立方体 -> 带载提升保持。

任务几何（config/params.py，v7 顶抓重设计）：
    - 初始位形 Q_INIT：模型原生形态（USD/URDF 零位）；
    - 立方体置于 6 cm 支柱顶（径向 0.40 m），顶抓抓取位
      SETPOINT_POS = 指尖端探到立方体中心下方 3 cm；
    - 姿态 R_TOOL_QUAT：tilt=170°（夹爪朝下偏外 10°，开合向水平）。

轨迹：v7 分段路标（run_lib.build_setpoint_goto_trajectory，IK 延续
扫描全链 PASS：worst margin 19~39°、sigma>=0.047）；夹爪在抓取位
静置段闭合（t_close），提升段带载。

运行方式（Isaac 官方运行时）：
    ~/isaacsim/python.sh TNDQ_b601/experiments/exp1_setpoint.py
输出：TNDQ_b601/results/exp1_setpoint.csv（列定义见 run_lib._CSV_COLUMNS）
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    GRIPPER_GRASP, GRIPPER_OPENING, SETPOINT_HOLD_TIME,
)

from run_lib import (run_tndq_experiment, build_setpoint_goto_trajectory,
                     build_setpoint_goto_trajectory_kinematic)


def make_gripper_schedule(t_close):
    """两段式受控闭合（v12c：实测接触点模型，diag 保持/夹持裁决锤定）。

    set_gripper(w) 目标 = 2*单指 stroke，真实指间开度 = 2*stroke。
    实测接触点：方块面深度 x≈-28 mm 处垫/轨内面 ≈ frame-10.7 mm，
    接触 stroke≈33.2（开度 66 mm）——闭合指令在 stroke 33.8/34.4
    停驻即夹住。铁律：闭合目标不得越过接触点（旧值 0.040/0.045
    越过接触点 10+ mm 强闭挤飞方块，是历次推飞根因）。
    快接近段 250 mm/s 到 0.075（接触开度 66 mm 上方留 9 mm 净空），
    慢夹段 25 mm/s 到 GRIPPER_GRASP=0.063（接触点过盈 ~1.5 mm/侧，
    KP*过盈 ≈ 3 N/指 -> 摩擦 2μN ≈ 3.6 N > 自重 0.98 N）。五跑教训
    保留：不用阶跃闭合（KP=2000 阶跃响应 ~2.7 m/s 撞击动量）。
    """
    w_fast, w_slow = 0.075, 0.25         # 快接近目标 [m] / 接近速度 [m/s]
    v_squeeze = 0.025                    # 慢夹速度 [m/s]
    t1 = t_close + (GRIPPER_OPENING - w_fast) / w_slow
    t2 = t1 + (w_fast - GRIPPER_GRASP) / v_squeeze

    def schedule(t):
        if t < t_close:
            return GRIPPER_OPENING
        if t < t1:
            return GRIPPER_OPENING - w_slow * (t - t_close)
        if t < t2:
            return w_fast - v_squeeze * (t - t1)
        return GRIPPER_GRASP

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
    ap.add_argument("--traj", type=str, default="kinematic",
                    choices=["kinematic", "tndq"],
                    help="期望轨迹生成：kinematic=笛卡尔 quintic+slerp"
                         "（默认，v11）；tndq=TNDQ 解析链（对比备份）")
    args = ap.parse_args()

    csv_path = args.csv or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "results", "exp1_setpoint.csv")

    # 期望轨迹：从 Q_INIT（原生零位）出发，分段路标（举高/前移/
    # 转腕下降/插指/静置/带载提升）；默认笛卡尔运动学插值（v11，
    # --traj tndq 切回 TNDQ 解析链对比）；t_close 后夹爪慢速斜坡闭合
    if args.traj == "kinematic":
        traj, t_move, t_close = build_setpoint_goto_trajectory_kinematic()
    else:
        traj, t_move, t_close = build_setpoint_goto_trajectory()
    duration = t_move + SETPOINT_HOLD_TIME

    # 夹爪调度：两段式受控闭合（见 make_gripper_schedule 归因注释）
    gripper_schedule = make_gripper_schedule(t_close)

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
            gripper_schedule=gripper_schedule,   # t_close 闭合夹持
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
