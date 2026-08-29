"""
TNDQ_real/scripts/02_gravity_hold.py —— 重力保持验证/手掰示教。
（v3：1:1 复刻 Borot rebotarmcontroller 在本机实机跑通的重力补偿配方，
外加重力源对比诊断；全程不失能、不掉臂。v4：每 0.2 s 终端打印
指令/实测力矩对账（sent/meas），供符号终审与记录。）

与 Borot _gravity_comp_tick 的对应关系：
  kp=7.0 / kd=0.8 位置弹簧 + 冻结目标（运动时目标跟随、静止即锁定）
  tau = tau_g + 积分器（增益 1.0、限幅 ±0.5，吃掉模型残差）
  send_mit 的 vel 传 0（不用带噪速度反馈做阻尼，新批次 register 源
  速度不可靠）；50 Hz 循环；逐关节切 MIT 并立即下发保持帧（无失托窗口）。

--gsrc 选择重力源做 A/B 对比（二者同一份官方 URDF、同一 pinocchio 约定，
启动时打印两套数值，若保持仍失败且两套一致 -> 问题在电机/指令链路）：
  vendor : 厂商库 compute_generalized_gravity（Borot 同款，默认）
  urdf   : 本包 B601NominalDynamics.gravity_vector（论文控制律同款）

运行（工作目录 = TNDQ_real/）：
    python scripts/02_gravity_hold.py                     # drag 手掰，
                                                          #   Ctrl+C 结束
    python scripts/02_gravity_hold.py --pose armup --duration 30
    python scripts/02_gravity_hold.py --gsrc urdf
    python scripts/02_gravity_hold.py --factor 1,1.2,1,1,1,1
"""
import argparse
import socket
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from config.params import Q_INIT  # noqa: E402
from config.params_real import GRIPPER_ACTIVE  # noqa: E402
from config.transforms import JOINT_OFFSET, JOINT_SIGN, TAU_SCALE  # noqa: E402

POSES = {
    "armup": np.array([0.0, -0.8, -0.4, 0.0, 0.0, 0.0]),  # 重力载荷大
    "qinit": Q_INIT,                                       # 静稳定
}
# Borot hardware_manager.py 实测配方（本机实机跑通）
GC_KP = 7.0
GC_KD = 0.8
GC_VEL_TH = 0.05          # 关节级运动检测阈值 [rad/s]（Borot 为末端笛卡尔
                          #   速度阈值，此处等价简化）
GC_RATE = 50.0            # Borot 控制频率
DT = 1.0 / GC_RATE
POS_TOL = 0.03            # 就位容差 [rad]
MONITOR_DT = 0.2          # 终端力矩监视打印间隔 [s]（v4 需求①）：sent=
                          #   本步电机系指令，meas= 上步电机系原始反馈
                          #   （未标度，同坐标系直接可比；新批次 register
                          #   源关节可能恒 0，非故障）
TWIN_UDP_PORT = 47470     # 数字孪生遥测端口（= params_real.TWIN_UDP_PORT，
                          #   与 scripts/ros_twin_bridge.py 对应）
_TWIN_ADDR = ("127.0.0.1", TWIN_UDP_PORT)


# ---------------------------------------------------------------------------
# 两套重力源（启动时互相对照打印）
# ---------------------------------------------------------------------------

def _make_g_vendor():
    """厂商库重力（Borot 同款）：官方 URDF + pinocchio 广义重力。"""
    from reBotArm_control_py.dynamics import compute_generalized_gravity
    from reBotArm_control_py.dynamics import load_dynamics_model
    from reBotArm_control_py.kinematics import pad_q_for_model

    model = load_dynamics_model()
    data = model.createData()

    def g(q):
        import pinocchio as pin
        tau = compute_generalized_gravity(
            model, pad_q_for_model(model, np.asarray(q, dtype=float)), data)
        return np.asarray(tau[:6], dtype=float)

    return g


def _make_g_urdf():
    """本包名义动力学重力（论文控制律同款）。"""
    from config.b601_dynamics import B601NominalDynamics

    dyn = B601NominalDynamics()
    return lambda q: dyn.gravity_vector(q)


def _near_ref(values, reference):
    """角度回卷到参考点 ±pi 邻域（Borot _angles_near_reference 同款，
    防多圈反馈跳变扰动目标）。"""
    d = values - reference
    d = (d + np.pi) % (2.0 * np.pi) - np.pi
    return reference + d


def _hold_gripper(robot, tag=""):
    """夹爪组切 POS_VEL 按当前位置保持：消除 MIT 无命令抖动（joint7）。
    电机 7 静默期（GRIPPER_ACTIVE=False）直接跳过：不切模式不使能
    不发帧，保持上电原状（硬件问题期间口径）。"""
    grp = getattr(robot, "gripper", None)
    if grp is None or not GRIPPER_ACTIVE:
        return
    try:
        pos, _, _ = robot.get_state(request_feedback=True)
        grp.mode_pos_vel()
        grp.enable()
        grp.send_pos_vel(np.asarray([pos[6]]), vlim=np.asarray([0.5]))
        print(f"[02] 夹爪已切 POS_VEL 位置保持{tag}")
    except Exception as e:  # 夹爪异常不阻断主流程
        print(f"[02] 夹爪保持设置失败（忽略）：{e}")


def _soft_disconnect(robot):
    """断串口但**不失能**（v4 安全修正）：vendor disconnect() 内部
    disable_all() 会失能全部电机，输出级关闭即零力矩，非静稳位姿下
    臂必坠——与 finally 的"位置保持、未失能"承诺矛盾。镜像其串口
    关闭序列并跳过失能：PV 位置环在驱动器固件内持续托臂，不依赖
    上位机。"""
    robot.stop_control_loop()
    robot._reg_running = False
    if robot._reg_thread is not None:
        robot._reg_thread.join(timeout=1.0)
        robot._reg_thread = None
    for ctrl in robot._ctrl_map.values():
        ctrl.shutdown()
        time.sleep(0.1)
        ctrl.close()
    robot._ctrl_map.clear()
    robot._motor_map.clear()
    robot._connected = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--factor", type=str, default=None,
                    help="逐关节补偿系数（逗号分隔 6 值），乘在 tau_g 上")
    ap.add_argument("--pose", type=str, default="drag",
                    choices=["drag", *POSES.keys()],
                    help="drag=手掰示教（默认，Ctrl+C 结束）；armup=前伸"
                         "（自动就位）；qinit=Q_INIT")
    ap.add_argument("--vlim", type=float, default=0.3,
                    help="就位段逐关节速度上限 [rad/s]")
    ap.add_argument("--gsrc", type=str, default="vendor",
                    choices=["vendor", "urdf"],
                    help="重力源：vendor=厂商库（Borot 同款，默认）；"
                         "urdf=本包名义动力学")
    args = ap.parse_args()

    factor = (np.ones(6) if args.factor is None
              else np.array([float(v) for v in args.factor.split(",")]))
    drag = args.pose == "drag"
    q_target = None if drag else POSES[args.pose]

    from reBotArm_control_py.actuator import RebotArm

    g_vendor = _make_g_vendor()
    g_urdf = _make_g_urdf()
    g_main = g_vendor if args.gsrc == "vendor" else g_urdf

    robot = RebotArm()
    robot.connect()
    _hold_gripper(robot)                       # joint7 防抖：位置保持
    q_hist = []
    try:
        # ---- [0] 起点读取 + 两套重力对照诊断 ----
        pos0, _, _ = robot.get_state(request_feedback=True)
        q0 = JOINT_SIGN * pos0[:6] + JOINT_OFFSET
        gv0, gu0 = g_vendor(q0), g_urdf(q0)
        print(f"[02] q0={np.round(q0, 4).tolist()}")
        print(f"[02] tau_g vendor={np.round(gv0, 4).tolist()}")
        print(f"[02] tau_g urdf  ={np.round(gu0, 4).tolist()}   "
              f"max|diff|={float(np.max(np.abs(gv0 - gu0))):.2e} N*m")
        print(f"[02] 采用重力源：{args.gsrc}   factor={factor.tolist()}")

        # ---- [1] POS_VEL 就位（需要且不在目标姿态时）----
        if not drag:
            dq0 = float(np.max(np.abs(q0 - q_target)))
            if dq0 > POS_TOL:
                print(f"[02] 距目标姿态 max|dq|={dq0:.2f} rad，"
                      f"POS_VEL 慢速就位（vlim={args.vlim}）")
                robot.arm.mode_pos_vel()
                robot.arm.enable()
                t_go = time.perf_counter()
                while time.perf_counter() - t_go < 60.0:
                    robot.arm.send_pos_vel(q_target,
                                           vlim=np.full(6, args.vlim))
                    time.sleep(0.1)
                    q = robot.arm.get_positions(request_feedback=True)[:6]
                    if float(np.max(np.abs(q - q_target))) < 0.01:
                        time.sleep(1.5)
                        print("[02] 就位完成")
                        break
                else:
                    raise RuntimeError("POS_VEL 就位超时")
                pos0, _, _ = robot.get_state(request_feedback=True)

        # ---- [2] 先使能，再逐关节切 MIT 并立即下发保持帧（Borot 同款，
        #      无失托窗口；冷启动时切换窗口内臂不掉）----
        from motorbridge import Mode

        robot.arm.enable()
        q_hold = pos0[:6].copy()
        tau_g0 = g_main(JOINT_SIGN * q_hold + JOINT_OFFSET) * factor
        for i, jc in enumerate(robot.arm._jcfgs):
            motor = robot._motor_map[jc.name]
            motor.ensure_mode(Mode.MIT, 1000)
            motor.send_mit(float(q_hold[i]), 0.0, GC_KP, GC_KD,
                           float(JOINT_SIGN[i] * tau_g0[i] / TAU_SCALE[i]))
        _hold_gripper(robot, "（MIT 切换后复查）")
        print(f"[02] 重力保持启动：pose={args.pose} gsrc={args.gsrc} "
              f"kp={GC_KP} kd={GC_KD} 积分器限幅±0.5 "
              f"{'（Ctrl+C 结束）' if drag else f'duration={args.duration}s'}")

        # ---- [3] Borot 配方主循环：弹簧 + 重力前馈 + 积分器 ----
        twin_sock = None                       # 数字孪生遥测（尽力而为）
        try:
            twin_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"[02] 数字孪生遥测开启：UDP 127.0.0.1:{TWIN_UDP_PORT} "
                  "-> ros_twin_bridge（网页镜像）")
        except OSError as exc:
            print(f"[02] 遥测 socket 创建失败（忽略）: {exc}")
        q_ref = q_hold.copy()            # 冻结目标（电机系）
        q_ref_u = q0.copy()              # 参考（URDF 系，角度回卷用）
        integral = np.zeros(6)
        t_mon = -1.0                     # 监视打印节拍（首拍必打印）
        t0 = time.perf_counter()
        while True:
            tic = time.perf_counter()
            t = tic - t0
            if not drag and t > args.duration:
                break
            pos, vel, torq = robot.get_state(request_feedback=True)
            q_u = _near_ref(JOINT_SIGN * pos[:6] + JOINT_OFFSET, q_ref_u)
            v_abs = float(np.max(np.abs(vel[:6])))
            if v_abs > 3.5:
                raise RuntimeError(
                    f"t={t:.1f}s 速度尖峰 {v_abs:.2f} rad/s：疑似打滑/碰撞")
            tau_g = g_main(q_u) * factor
            if drag and v_abs > GC_VEL_TH:
                # 运动检测（手在掰/正在下坠）：目标跟随、积分衰减
                q_ref = pos[:6].copy()
                q_ref_u = q_u.copy()
                integral *= 0.9
            else:
                integral += (q_ref_u - q_u) * 1.0
                np.clip(integral, -0.5, 0.5, out=integral)
            tau_motor = JOINT_SIGN * (tau_g + integral) / TAU_SCALE
            robot.arm.send_mit(pos=q_ref, vel=np.zeros(6),
                               kp=np.full(6, GC_KP),
                               kd=np.full(6, GC_KD), tau=tau_motor)
            # 数字孪生遥测（v6.3）：50 Hz 把 URDF 系 q 发 localhost，
            # ros_twin_bridge 转发 /rebotarm/joint_states 供 web 镜像；
            # 单向只读、异常静默——绝不影响保持热路径
            if twin_sock is not None:
                try:
                    twin_sock.sendto(q_u.tobytes(), _TWIN_ADDR)
                except OSError:
                    pass
            # 0.2 s 终端监视（v4 需求①）：本步输入（电机系指令）vs
            # 上步读取（电机系原始反馈，未标度；同坐标系直接可比）
            if t - t_mon >= MONITOR_DT:
                t_mon = t
                print(f"[02] t={t:6.2f}s sent="
                      f"[{' '.join(f'{v:+.2f}' for v in tau_motor)}]  "
                      f"meas=[{' '.join(f'{v:+.2f}' for v in torq[:6])}]",
                      flush=True)
            q_hist.append(q_u.copy())
            elapsed = time.perf_counter() - tic
            if elapsed < DT:
                time.sleep(DT - elapsed)
    except KeyboardInterrupt:
        print("\n[02] 用户中断")
    except RuntimeError as e:
        print(f"\n[02] 保护报警：{e}")
    finally:
        # 安全退出：切回 POS_VEL 按当前位置保持 -> 断连但不失能
        try:
            posf, _, _ = robot.get_state(request_feedback=True)
            robot.arm.mode_pos_vel()
            robot.arm.enable()
            for _ in range(5):
                robot.arm.send_pos_vel(posf[:6], vlim=np.full(6, 0.1))
                time.sleep(0.1)
            print("[02] 已切回 POS_VEL 位置保持（未失能），目视确认后再离开")
            _hold_gripper(robot)
            print("     确要失能：先回稳定姿态（Q_INIT），例如：")
            print("       python -c \"import sys; sys.path.insert(0,'.'); "
                  "import config.paths; from interfaces.real_backend import "
                  f"posvel_goto; posvel_goto({Q_INIT.tolist()})\"")
        finally:
            _soft_disconnect(robot)   # 不失能（disconnect 内部会坠臂）

    q_hist = np.asarray(q_hist)
    if drag and len(q_hist) > 25:
        q_hold_end = q_hist[-25:].mean(axis=0)
        print(f"\n[02] 结束时的姿态（最后 0.5 s 均值）："
              f"q={np.round(q_hold_end, 4).tolist()}")
    elif not drag and len(q_hist) > 10:
        hold = q_hist[len(q_hist) // 2:]
        drift = hold[-1] - hold[0]
        rate = drift / max(len(hold) * DT, 1e-9)
        print(f"\n[02] 保持段漂移：")
        for i in range(6):
            ok = "OK " if abs(rate[i]) < 0.002 else "调 "
            print(f"  j{i + 1}: {drift[i]:+.4f} rad"
                  f"（{rate[i]:+.4f} rad/s）[{ok}]")
    print("[02] 诊断提示：若两套 tau_g 一致且保持仍失败 -> 问题在"
          "电机/指令链路而非重力模型")


if __name__ == "__main__":
    main()
