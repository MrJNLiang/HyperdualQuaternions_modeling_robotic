"""
实机实验一：定点控制 + 接触抓取 —— B601-DM 真机（Jetson + 达妙 DM 电机）。

与 TNDQ_b601/experiments/exp1_setpoint.py 的仿真入口同构：同一条 TNDQ
力矩级控制链（run_lib.run_tndq_experiment）、同一任务几何与轨迹、同一
夹爪调度；唯一差异是物理后端：
    IsaacB601Backend（仿真力矩直驱）-> RealB601Backend（MIT 力矩直驱）

运行前置（按序完成，见 README.md 第 4 节）：
    [1] config/transforms.py 标定回填（符号/零位/力矩标度/夹爪换算）；
    [2] 就位（独立进程，POS_VEL 位置模式把臂摆到 Q_INIT；闭环无 teleport）：
        cd TNDQ_real && uv run --project vendor/reBotArm_control_py python -c \
            "import sys; sys.path[:0] = ['.']; \
            import config.paths, importlib.util as u; \
            m = u.spec_from_file_location('rb', 'interfaces/real_backend.py'); \
            rb = u.module_from_spec(m); m.loader.exec_module(rb); \
            from config.params import Q_INIT; rb.posvel_goto(Q_INIT)"
    [3] 串口权限：sudo usermod -aG dialout $USER（或 chmod 666 /dev/ttyACM0）。

运行（工作目录 = TNDQ_real/，驱动环境经 --project 指定）：
    uv run --project vendor/reBotArm_control_py python experiments/exp1_real.py --mode openhold  # 空载基线
    uv run --project vendor/reBotArm_control_py python experiments/exp1_real.py --mode grasp     # 接触抓取

输出：TNDQ_real/results/exp1_real_<mode>.csv（列定义与仿真一致，
含 meas* 实测力矩列；cube_* 列为 NaN——真机无仿真目标物）。
"""
import argparse
import importlib.util
import os
import sys
from pathlib import Path

REAL_ROOT = Path(__file__).resolve().parent.parent
VENDOR_ROOT = REAL_ROOT / "vendor" / "reBotArm_control_py"
for _p in (str(REAL_ROOT), str(VENDOR_ROOT)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# --- 导入顺序铁律 ---
# [1] params_real 先行：monkey-patch config.params 到真机口径（DT=10 ms、
#     CTRL_EVERY=1、small_arm 增益组），必须发生在 run_lib 导入之前；
# [2] run_lib / real_backend 经 importlib 显式按文件路径加载（本包自包含
#     副本；按路径加载避免与任意同名顶层包的解析歧义）。
import config.params_real  # noqa: F401,E402


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


real_backend = _load("real_backend",
                     REAL_ROOT / "interfaces" / "real_backend.py")
run_lib = _load("run_lib", REAL_ROOT / "experiments" / "run_lib.py")

from config.params import (CUBE_SIZE, GRIPPER_BASELINE_WIDTH, GRIPPER_GRASP,  # noqa: E402
                           GRIPPER_OPENING, SETPOINT_HOLD_TIME)


# ---------------------------------------------------------------------------
# 夹爪调度（与 exp1_setpoint.py 同口径副本：两段式受控闭合 / 开指保持）
# ---------------------------------------------------------------------------

def make_gripper_schedule(t_close, v_open=0.25, v_approach=0.25,
                          v_press=0.025, w_touch=None, w_grasp=None):
    """两段式受控闭合：全开插入 -> 快接近到临触 -> 慢压过盈夹持。
    真机差异：底层由 Isaac 位置 drive 换成 gripper 电机 MIT 位置保持，
    开度语义（指间宽度 m）与速率限制不变。"""
    w_touch = CUBE_SIZE + 0.002 if w_touch is None else float(w_touch)
    w_grasp = GRIPPER_GRASP if w_grasp is None else float(w_grasp)
    t1 = GRIPPER_OPENING / v_open
    t2 = float(t_close)
    t3 = t2 + (GRIPPER_OPENING - w_touch) / v_approach
    t4 = t3 + (w_touch - w_grasp) / v_press

    def schedule(t):
        if t < t1:
            return v_open * t
        if t < t2:
            return GRIPPER_OPENING
        if t < t3:
            return GRIPPER_OPENING - v_approach * (t - t2)
        if t < t4:
            return w_touch - v_press * (t - t3)
        return w_grasp

    return schedule


def make_gripper_open_hold_schedule(v_open=0.25):
    """开指保持（无接触基线）：斜坡升到全开后恒保持。"""
    t1 = GRIPPER_BASELINE_WIDTH / v_open

    def schedule(t):
        if t < t1:
            return v_open * t
        return GRIPPER_BASELINE_WIDTH

    return schedule


def main():
    ap = argparse.ArgumentParser(description="B601 实机实验一：定点控制")
    ap.add_argument("--csv", type=str, default=None, help="CSV 输出路径")
    ap.add_argument("--traj", type=str, default="pinocchio",
                    choices=["pinocchio", "kinematic", "tndq"],
                    help="期望轨迹生成（与仿真同选项；pinocchio 缺失时"
                         "run_lib 自动降级 kinematic）")
    ap.add_argument("--mode", type=str, default="openhold",
                    choices=["openhold", "grasp"],
                    help="openhold=开指保持无接触基线（真机首跑默认，"
                         "安全）；grasp=接触抓取（完成空载验证后启用）")
    args = ap.parse_args()

    csv_path = args.csv or str(REAL_ROOT / "results"
                               / f"exp1_real_{args.mode}.csv")
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    # 期望轨迹：与仿真完全相同的任务几何与路标（params.py）
    if args.traj == "pinocchio":
        traj, t_move, t_close = \
            run_lib.build_setpoint_goto_trajectory_pinocchio()
    elif args.traj == "kinematic":
        traj, t_move, t_close = \
            run_lib.build_setpoint_goto_trajectory_kinematic()
    else:
        traj, t_move, t_close = run_lib.build_setpoint_goto_trajectory()
    duration = t_move + SETPOINT_HOLD_TIME

    gripper_schedule = (make_gripper_schedule(t_close)
                        if args.mode == "grasp"
                        else make_gripper_open_hold_schedule())

    print(f"[exp1-real] mode={args.mode}  traj={args.traj}  "
          f"duration={duration:.1f}s -> {csv_path}")
    print("[exp1-real] 安全提示：首跑请拆除负载、人员保持臂展外、"
          "手悬急停（Ctrl+C 触发安全关闭）")

    backend = real_backend.RealB601Backend()
    backend.setup()
    try:
        summary = run_lib.run_tndq_experiment(
            backend, traj, duration, csv_path,
            gripper_schedule=gripper_schedule,
            gripper_init=0.0,                    # 从并拢起步，按调度张开
            label=f"exp1-real-{args.mode}")
    finally:
        # 正常结束 / KeyboardInterrupt / HardwareFault 统一走安全关闭：
        # 重力补偿收尾 -> 停总线 -> disable_all 断力矩 -> 断串口
        backend.close()
    return summary


if __name__ == "__main__":
    main()
