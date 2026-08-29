"""
B601-DM 真机后端 —— Jetson + 达妙 DM 电机（MIT 模式力矩直驱）。

与 interfaces/isaac_interface.py 的 IsaacB601Backend **方法签名一致**，
experiments/run_lib.py 主控制循环零改动替换（后端可插拔契约）。

架构（双线程，GIL 下共享缓冲区 + 锁）：
    总线线程 500 Hz（RebotArm.start_control_loop）：
        读反馈(pos/vel/torq) -> 标定变换 -> 共享状态
        分相：保持期(启动/心跳超时/收尾： kp=7 弹簧+tau_g+积分器，v3 配方)
             | 控制期(心跳正常： 斜率限制 -> 斜坡混入 -> 纯力矩直驱)
    控制线程 100 Hz（run_lib.run_tndq_experiment）：
        get_joint_state -> TNDQ 控制律 -> apply_arm_torques -> step()

仿真"力矩直驱" -> 真机指令格式的转换（核心）：
    Isaac：set_joint_efforts(tau)，drive 增益清零，tau 为唯一输入；
    真机：达妙 MIT 模式，驱动器内部执行
        tau_motor = kp*(pos_d - pos) + kd*(vel_d - vel) + tau_ff
    令 pos_d = pos_meas、vel_d = vel_meas（kp/kd 项恒零）、
    kp = 0、kd = MIT_KD(小阻尼安全网)、tau_ff = tau_TNDQ，即还原
    纯力矩直驱；tau_ff 下发前经 TAU_SCALE/JOINT_SIGN 标定变换到电机系。
    （与厂商 gravity_joint_sender 的重力前馈保持同构，链路已验证。）

模式切换约定：
    - 闭环实验全程 MIT（send_mit）：驱动器位置环关闭，计算力矩控制律
      为唯一外环；
    - 就位/回零用 POS_VEL（send_pos_vel，本文件 posvel_goto()）：
      独立脚本执行，结束后断连交回 MIT 流程。禁止闭环中混用位置模式
      （驱动器内置位置环 pos_kp=150 会与 TNDQ 力矩环打架）。

Jetson 实时性（算力约束下的主循环优化，数学不变）：
    - 总线/控制热路径全部预分配缓冲 + np.*(out=) 原地运算，无动态
      内存分配；锁内仅做定长数组拷贝；
    - 重力向量 g(q) 由控制线程每步算好快照（run_lib 调用
      update_gravity_snapshot），总线线程零 RNEA 开销；
    - FK/M/crba 走 pinocchio C++ 快速路径（run_lib 自动选择）。

串口配置：通道/波特率/电机 ID 全部在
    reBot-Isaacsim/third_party/reBotArm_control_py/config/rebotarm_dm.yaml
    （channel: /dev/ttyACM0，921600，rate: 500）；本文件不重复定义。

运行环境：uv（motorbridge 原生绑定）；Jetson 需 aarch64 wheel。
"""
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

# 路径引导（TNDQ_b601 + 驱动库 + TNDQ_real 根）
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from config.params import (DT, GRIPPER_OPENING, Q_INIT, TAU_MAX)  # noqa: E402
from config.params_real import (BUS_DT, BUS_RATE_HZ, CONTACT_RESID_COUNT,
                                CONTACT_RESID_TAU, GRIPPER_ACTIVE,
                                ANCHOR_KP, HOLD_INTEG_GAIN,
                                HOLD_INTEG_MAX, HOLD_KP, HOLD_KD,
                                HOLD_RATE_HZ, HOLD_VEL_TH, MIT_KD, MIT_KP,
                                Q_BRINGUP_TOL, TORQUE_RAMP_TIME,
                                TORQUE_SLEW_MAX, TWIN_RATE_HZ,
                                TWIN_UDP_PORT, VEL_SPIKE_MAX,
                                WATCHDOG_TIMEOUT)
from config.transforms import (GRIPPER_M_PER_RAD, GRIPPER_SIGN,
                               GRIPPER_WIDTH_MAX, JOINT_OFFSET, JOINT_SIGN,
                               TAU_SCALE)


class HardwareFault(RuntimeError):
    """硬件级故障（通信超时/速度异常/碰撞残差）——run_lib 主循环捕获后
    记入 CSV abort 字段并经 finally 触发 backend.close() 急停断输出。"""


class RealB601Backend:
    """B601-DM 真机后端（生命周期与约定见模块 docstring）。"""

    def __init__(self):
        # ---- 驱动与线程 ----
        self.robot = None                  # RebotArm 实例（setup 时创建）
        self._lock = threading.Lock()      # 共享缓冲区互斥
        self._running = False

        # ---- 共享状态（URDF/DH 约定系；全部预分配，热路径零分配）----
        self._q = np.zeros(6)              # 关节角 [rad]
        self._qd = np.zeros(6)             # 关节速度 [rad/s]
        self._tau_meas = np.zeros(6)       # 实测力矩（关节系 N*m，已标度）
        self._tau_cmd = np.zeros(6)        # 控制线程目标力矩（限幅后）
        self._tau_sent = np.zeros(6)       # 上一拍实际下发（斜率限制基准）
        self._g_snap = np.zeros(6)         # g(q) 快照（控制线程每步更新）
        self._width_cmd = GRIPPER_OPENING  # 夹爪指间开度目标 [m]
        self._width_meas = 0.0             # 夹爪实测开度 [m]
        self._gripper_active = bool(GRIPPER_ACTIVE)
        #   False = 电机 7 全程静默（不发模式/使能/命令帧；硬件问题
        #   期间口径，params_real 一处开关控制全部夹爪触点）
        self._cmd_time = 0.0               # 控制线程心跳（apply_arm_torques）
        self._fb_time = 0.0                # 最近一次反馈到达时刻
        self._ramp_t0 = None               # 使能斜坡起点（setup 时置位）

        # ---- 数字孪生遥测：总线线程低频把实测 q 发 localhost UDP，
        #      scripts/ros_twin_bridge.py 转发 /rebotarm/joint_states
        #      供 web 镜像（单向只读；TWIN_UDP_PORT=0 关闭）----
        self._twin_sock = None
        self._twin_addr = (("127.0.0.1", TWIN_UDP_PORT)
                           if TWIN_UDP_PORT else None)
        self._twin_every = (max(1, round(BUS_RATE_HZ / TWIN_RATE_HZ))
                            if TWIN_UDP_PORT else 0)
        self._twin_tick = 0
        self._dwell = False                # 驻留纯监视模式（dwell_hold）

        # ---- 控制期锚定兕底（apply_anchor；官方 gravity comp 同思想）----
        self._anchor_mask = None           # 6 维 0/1：1=该关节控制期叠 kp 弹簧
        self._anchor_q = np.zeros(6)       # 锚定参考位姿（short=Q_INIT）
        self._anchor_inv = np.zeros(6)     # 1-mask（预分配热路径用）
        self._kp_vec = np.zeros(6)
        self._pos_vec = np.zeros(6)
        self._tmp_a = np.zeros(6)
        self._tmp_b = np.zeros(6)

        # ---- 热路径临时缓冲（预分配，总线/控制线程各自专用）----
        self._tmp_bus = np.zeros(6)        # 总线线程变换暂存
        self._tmp_resid = np.zeros(6)      # 残差检查暂存
        self._resid_count = 0              # 残差连续超阈拍数
        self._fault = None                 # 总线线程检测到的降级原因
        self._no_tau_idx = ()              # 寄存器状态源关节下标（无力矩测量）

        # ---- 保持期状态（v3 配方：kp 弹簧锚定 + tau_g + 积分器；
        #      启动期/看门狗降级/收尾共用，02_gravity_hold 同款）----
        self._hold_q = np.zeros(6)         # 保持目标（电机系；冻结/跟随）
        self._hold_integ = np.zeros(6)     # 积分器（URDF 系 N*m）
        self._hold_tick = 0                # 慢速刷新计数（HOLD_RATE_HZ 口径）
        self._hold_every = max(1, int(round(BUS_RATE_HZ / HOLD_RATE_HZ)))
        self._in_hold = True               # 当前是否保持期（状态转换打印）
        self._tau_hold_motor = np.zeros(6)  # 保持期前馈（电机系，慢速刷新）
        self._hold_kp = np.full(6, HOLD_KP)
        self._hold_kd = np.full(6, HOLD_KD)
        self._hold_vel0 = np.zeros(6)      # send_mit vel=0（register 源
                                           #   速度不可靠，02 v3 同款）
        self._vel_zero = np.zeros(6)       # 控制期 vel_d=0（阻尼网，见 _send_mit）

        # ---- 墙钟配速（step() 时间基准）----
        self._t_next = None

        # ---- 动力学（重力快照/降级补偿用；pinocchio RNEA 后端）----
        self._dyn = None

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def setup(self):
        """连接 -> POS_VEL 慢速就位 Q_INIT（进程内）-> 逐关节无缝切 MIT
        -> 启动 500 Hz 总线线程。一次连接全程不失能（Borot 式）。

        就位与闭环同进程同连接：不存在"就位脚本退出失能 -> 掉臂"的
        窗口；MIT 切换逐关节进行并立即下发保持帧（kp=7 托底 + g(q)），
        无失托间隙（Borot _enter_gravity_compensation_mode 同款）。
        """
        if self.robot is not None:
            raise RuntimeError("setup() 已执行过")
        # 延迟导入：motorbridge 为原生绑定，导入即占用串口资源
        from reBotArm_control_py.actuator import RebotArm
        from config.b601_dynamics import B601NominalDynamics

        self._dyn = B601NominalDynamics()
        self.robot = RebotArm()
        self.robot.connect()
        self._no_tau_idx = tuple(self.robot.reg_arm_indices())
        if self._gripper_active:
            self._hold_gripper_posvel("（setup 初始）")  # joint7 防抖

        # [1] POS_VEL 慢速就位到 Q_INIT（不在目标附近时；与闭环同一
        #     连接，绝不失能；结束后驱动器位置环持续托臂）
        pos, _, _ = self.robot.get_state(request_feedback=True)
        q0 = JOINT_SIGN * pos[:6] + JOINT_OFFSET
        dq0 = float(np.max(np.abs(q0 - Q_INIT)))
        if dq0 > 0.03:
            print(f"[real] 距 Q_INIT max|dq|={dq0:.2f} rad，"
                  "POS_VEL 慢速就位（vlim=0.3）")
            self.robot.arm.mode_pos_vel()
            self.robot.arm.enable()
            t_go = time.perf_counter()
            while time.perf_counter() - t_go < 60.0:
                self.robot.arm.send_pos_vel(Q_INIT, vlim=np.full(6, 0.3))
                time.sleep(0.1)
                p = self.robot.arm.get_positions(request_feedback=True)[:6]
                if float(np.max(np.abs(p - Q_INIT))) < 0.01:
                    time.sleep(1.5)
                    break
            else:
                raise RuntimeError("POS_VEL 就位超时（60 s 未收敛）")
            pos, _, _ = self.robot.get_state(request_feedback=True)
            print("[real] 就位完成")

        # [1b] 总线健康闸门：连续 20 拍（≥0.4 s）读反馈成功且读数稳定
        #      才放行进入 MIT。usbipd 转发通道不稳时（2026-08-29 事故：
        #      joint3 写超时 + 反馈半截数据骗过就位判定，带病进入力矩
        #      控制后 t=0 超限 abort、通道死亡后固件失联失能塌臂），
        #      在此挡下并指引检修——绝不让臂在坏通道上进力矩环。
        n_ok, q_ref = 0, None
        t_hc = time.perf_counter()
        while time.perf_counter() - t_hc < 5.0:
            try:
                pos_h, _, _ = self.robot.get_state(request_feedback=True)
            except Exception:  # noqa: BLE001
                n_ok = 0
                time.sleep(0.02)
                continue
            if q_ref is None:
                q_ref = pos_h[:6].copy()
            elif float(np.max(np.abs(pos_h[:6] - q_ref))) > 0.02:
                n_ok = 0                    # 读数跳变：反馈不可信，重计
                q_ref = pos_h[:6].copy()
            else:
                n_ok += 1
            if n_ok >= 20:
                break
            time.sleep(0.02)
        if n_ok < 20:
            try:
                self.robot.arm.send_pos_vel(Q_INIT, vlim=np.full(6, 0.3))
            except Exception:  # noqa: BLE001
                pass
            raise RuntimeError(
                "总线健康检查未通过（反馈持续超时/跳变）——串口通道不稳，"
                "拒绝进入力矩控制。Windows 侧 usbipd detach 后重新 attach"
                "（或换 USB 口/线），跑 00_check_bus.py 验证后重试；参见"
                " 串口启动.md")
        print(f"[real] 总线健康检查通过（连续 {n_ok} 拍读数稳定）")

        # [2] 逐关节切 MIT 并立即下发保持帧（kp=7/kd=0.8 托底 + g(q)），
        #     总线线程随接力（斜坡起点 = 纯重力补偿，无掉臂窗口）
        from motorbridge import Mode

        q_hold = pos[:6].copy()
        tau_g = self._dyn.gravity_vector(
            JOINT_SIGN * q_hold + JOINT_OFFSET)
        self.robot.arm.enable()
        for i, jc in enumerate(self.robot.arm._jcfgs):
            motor = self.robot._motor_map[jc.name]
            motor.ensure_mode(Mode.MIT, 1000)
            motor.send_mit(float(q_hold[i]), 0.0, 7.0, 0.8,
                           float(JOINT_SIGN[i] * tau_g[i] / TAU_SCALE[i]))
        self.robot.arm._mode = "mit"
        if self.robot.has_gripper and self._gripper_active:
            self.robot.gripper.mode_mit()   # 总线线程每拍 MIT 保持夹爪
        # （夹爪静默期跳过：不切模式不使能，电机 7 保持上电原状）

        # 使能前先读一拍状态，初始化重力快照（斜坡起点 = 纯重力补偿）
        # 与保持期目标（电机系冻结位姿：总线线程 v3 配方弹簧锚定点）
        self._g_snap[:] = tau_g
        self._tau_sent[:] = self._g_snap
        self._hold_q[:] = q_hold

        self._ramp_t0 = time.perf_counter()
        self._cmd_time = self._ramp_t0
        self._fb_time = self._ramp_t0
        if self._twin_addr is not None:
            self._twin_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"[real] 数字孪生遥测开启：UDP 127.0.0.1:{TWIN_UDP_PORT} "
                  f"-> ros_twin_bridge（{TWIN_RATE_HZ:.0f} Hz）")
        self._running = True
        self.robot.start_control_loop(self._bus_cycle, rate=BUS_RATE_HZ)
        self._t_next = time.perf_counter()
        print(f"[real] 后端就绪：总线 {BUS_RATE_HZ:.0f} Hz（MIT 力矩直驱）、"
              f"控制周期 {DT * 1000:.0f} ms、力矩斜坡 {TORQUE_RAMP_TIME} s")

    def close(self):
        """安全关闭（Borot 式，全程不失能）：停闭环 -> 逐关节无缝切
        POS_VEL（MIT 末帧固件持续生效 + 每关节切后立即位置帧，无失托
        窗口）-> 断串口（不断使能）。
    
        急停路径（HardwareFault / KeyboardInterrupt）同样走这里：不再
        做"重力补偿 0.3 s -> disable"（失能即坠臂）；改为位置环保持，
        电机维持力矩，臂不会动。后续重新运行脚本可直接接管（setup
        自带慢速就位），或用 Borot 网页端手动失能/�搫臂。
        """
        if self.robot is None:
            return
        self._running = False
        try:
            time.sleep(0.3)                    # 总线线程保持期配方稳定托臂
            self.robot.stop_control_loop()
        finally:
            try:
                # 逐关节无缝切 POS_VEL（setup 的镜像操作）：停止发帧后
                # 电机保持最后一帧 MIT（kp=7 弹簧 + tau_g）持续出力，
                # 逐关节 ensure_mode(POS_VEL) 后立即写位置目标——全程
                # 无零力矩窗口（组级 mode_pos_vel 的逐关节 sleep 0.05 s
                # + 尾部 0.2 s 失托窗口弃用）
                from motorbridge import Mode
    
                pos, _, _ = self.robot.get_state(request_feedback=True)
                self.robot.arm.enable()
                for i, jc in enumerate(self.robot.arm._jcfgs):
                    motor = self.robot._motor_map[jc.name]
                    motor.ensure_mode(Mode.POS_VEL, 1000)
                    motor.send_pos_vel(float(pos[i]), 0.1)
                self.robot.arm._mode = "pos_vel"
                if self.robot.has_gripper and self._gripper_active:
                    self.robot.gripper.mode_pos_vel()
                    self.robot.gripper.send_pos_vel(
                        np.array([pos[6]]), vlim=np.array([0.5]))
                print("[real] 已切 POS_VEL 位置保持（未失能）：臂由驱动器"
                      "位置环托住；确要断电请用 Borot 网页端或手动失能")
            except Exception as exc:  # noqa: BLE001
                print(f"[real] 位置保持收尾异常（跳过）: {exc}")
            try:
                soft_disconnect(self.robot)    # 关闭串口（不失能）；
                                               #   勿用 robot.disconnect——
                                               #   内部 disable_all 坠臂
            except Exception as exc:  # noqa: BLE001
                print(f"[real] soft_disconnect 异常: {exc}")
            if self._twin_sock is not None:
                try:
                    self._twin_sock.close()
                except OSError:
                    pass
                self._twin_sock = None
            print("[real] 后端已关闭，串口释放（电机保持使能）")

    def __enter__(self):
        self.setup()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # ------------------------------------------------------------------
    # 总线线程（500 Hz；热路径零分配）
    # ------------------------------------------------------------------

    def _bus_cycle(self, robot, dt):
        """一拍总线：读反馈 -> 变换 -> 分相（保持期 | 控制期）-> 下发。

        保持期（启动 / 控制心跳超时 / 收尾）：v3 实证配方托臂——
        kp=7 弹簧锚定冻结目标 + tau_g 前馈 + 积分器（02_gravity_hold
        同款）。控制期：kp=0 纯力矩直驱（tau_cmd 经斜率限制/斜坡），
        控制律为唯一外环。两相各自保证无失托窗口。
        """
        # [1] 读反馈（厂商库：request_feedback + poll，pos/vel/torq 7 维
        #     = arm6 + gripper1，yaml joints 顺序）
        pos, vel, torq = robot.get_state(request_feedback=True)

        # [2] 电机系 -> URDF 系（原地变换，无分配）
        with self._lock:
            np.multiply(JOINT_SIGN, pos[:6], out=self._tmp_bus)
            np.add(self._tmp_bus, JOINT_OFFSET, out=self._q)
            np.multiply(JOINT_SIGN, vel[:6], out=self._qd)
            np.multiply(JOINT_SIGN, torq[:6], out=self._tmp_bus)
            np.multiply(TAU_SCALE, self._tmp_bus, out=self._tau_meas)
            if robot.has_gripper:
                self._width_meas = float(
                    GRIPPER_M_PER_RAD * GRIPPER_SIGN * pos[6])
            self._fb_time = time.perf_counter()

        # [2b] 驻留纯监视：POS_VEL 目标已锁存驱动器（固件位置环自持，
        #      见 dwell_hold），只读反馈 + 遥测，绝不下发任何指令帧
        #      （夹爪同机理：末帧锁存；静默期本就零写入）
        if self._dwell:
            self._twin_publish()
            return

        # [3] 分相：控制心跳超期或收尾 -> 保持期配方（MIT_KP=0 纯前馈
        #     在建模残差下托不住臂——"就位后垮"根因）
        now = time.perf_counter()
        watchdog = now - self._cmd_time > WATCHDOG_TIMEOUT
        if not self._running or watchdog:
            if not self._in_hold:
                print("[real] 切入保持期（v3 配方：kp=7 弹簧 + 重力补偿 "
                      "+ 积分器）" + ("：控制心跳超时" if watchdog
                                      else "：收尾阶段"))
            elif watchdog and self._fault != "watchdog":
                print("[real] 警告：控制线程心跳超时，总线层保持配方托臂")
            self._fault = "watchdog" if watchdog else None
            self._in_hold = True
            self._hold_cycle(robot, pos, vel)
        else:
            if self._in_hold:
                self._in_hold = False
                print("[real] 控制心跳恢复，切回控制律透传（kp=0 纯力矩直驱）")
            self._fault = None

            # [4] 斜率限制（防指令跳变冲击轻腕；每拍最多 SLEW*BUS_DT）
            dmax = TORQUE_SLEW_MAX * dt
            with self._lock:
                np.subtract(self._tau_cmd, self._tau_sent, out=self._tmp_bus)
                np.clip(self._tmp_bus, -dmax, dmax, out=self._tmp_bus)
                np.add(self._tau_sent, self._tmp_bus, out=self._tau_sent)

                # [5] 使能斜坡：重力补偿 -> 全控制线性混入
                alpha = (now - self._ramp_t0) / TORQUE_RAMP_TIME
                if alpha < 1.0:
                    self._tau_sent *= alpha
                    np.multiply(self._g_snap, 1.0 - alpha, out=self._tmp_bus)
                    self._tau_sent += self._tmp_bus
                tau_out = self._tau_sent

            # [6] MIT 下发（URDF 系 -> 电机系逆变换）
            self._send_mit(robot, pos, vel, tau_out)

        # [7] 夹爪：MIT 位置保持（宽度目标 -> 电机位置，标定换算；
        #     两相都发——保持期跳过会退化为"最后帧保持"）。静默期
        #     完全跳过（电机 7 零写入；读反馈不受影响）
        if robot.has_gripper and self._gripper_active:
            g_target = GRIPPER_SIGN * self._width_cmd / GRIPPER_M_PER_RAD
            robot.gripper.send_mit(pos=np.array([g_target]),
                                   vel=np.zeros(1),
                                   kp=np.array([4.0]), kd=np.array([0.5]),
                                   tau=np.zeros(1))

        # [8] 数字孪生遥测（实现见 _twin_publish）
        self._twin_publish()

    def _twin_publish(self):
        """数字孪生遥测：低频打包实测 q 发 localhost（UDP 单向只读，
        异常静默——遥测故障绝不影响控制热路径）。锁外读 _q 可能撕裂
        （新旧混合），但数值连续且 20 ms 后被下一包覆盖，监视用途
        无害；50 Hz 小分配不属严格热路径纪律范畴。dwell 纯监视模式
        下复用（届时它是总线线程唯一活动）。"""
        if self._twin_sock is not None:
            self._twin_tick += 1
            if self._twin_tick >= self._twin_every:
                self._twin_tick = 0
                try:
                    self._twin_sock.sendto(self._q.tobytes(),
                                           self._twin_addr)
                except OSError:
                    pass

    def dwell_hold(self, note=""):
        """驻留模式（v6.1）：切 POS_VEL 位置保持后，总线线程转纯监视
        （只读反馈 + 数字孪生遥测，不再下发任何指令帧），进程不退出，
        直到调用方显式 close()。

        依据：POS_VEL 目标由驱动器固件位置环自持执行——目标锁存后
        即使上位机死亡/串口断开臂仍被托住（Borot 网页端同款机理）；
        对比保持期 MIT 配方依赖通道 50 Hz 持续发帧，通道死 = 失联
        失能塌臂（2026-08-29 事故）。驻留 = 把臂交给不依赖上位机的
        固件层，同时保留观测（遥测）与随时接管（close 幂等）。"""
        if self._dwell or self.robot is None:
            return
        with self._lock:
            pos_m = JOINT_SIGN * (self._q - JOINT_OFFSET)  # -> 电机系
        try:
            from motorbridge import Mode
            for jc in self.robot.arm._jcfgs:
                self.robot._motor_map[jc.name].ensure_mode(
                    Mode.POS_VEL, 1000)
            self.robot.arm._mode = "pos_vel"
            self.robot.arm.send_pos_vel(pos_m, vlim=np.full(6, 0.3))
        except Exception as exc:  # noqa: BLE001
            print(f"[real] 驻留切 POS_VEL 异常（不进入驻留，总线线程"
                  f"继续保持期配方发帧）: {exc}")
            return
        self._dwell = True
        print(f"[real] 驻留：POS_VEL 位置保持"
              f"{('（' + note + '）') if note else ''}——固件位置环自持、"
              "通道中断免疫；总线线程仅监视（遥测继续）；Ctrl+C 关闭"
              "后端")

    def _hold_cycle(self, robot, pos, vel):
        """保持期一拍（v3 实证配方，02_gravity_hold / Borot 同款）：
        50 Hz 口径重算 tau_g + 积分器（总线 500 Hz 下每 _hold_every 拍），
        每拍下发 kp=HOLD_KP 弹簧（锚定冻结目标）+ 前馈。速度超阈
        （手掰/外力扰动）时目标跟随实测并衰减积分（防积分顶死对抗
        操作者）；静止即锁定目标。_tau_sent 同步为 URDF 系真值：
        恢复控制期时斜率限制基准连续、监视对账不虚假。"""
        self._hold_tick += 1
        if self._hold_tick >= self._hold_every:
            self._hold_tick = 0
            q_u = JOINT_SIGN * pos[:6] + JOINT_OFFSET
            if float(np.max(np.abs(vel[:6]))) > HOLD_VEL_TH:
                self._hold_q[:] = pos[:6]        # 目标跟随（电机系）
                self._hold_integ *= 0.9          # 积分衰减
            else:
                q_ref_u = JOINT_SIGN * self._hold_q + JOINT_OFFSET
                self._hold_integ += ((q_ref_u - q_u) * HOLD_INTEG_GAIN)
                np.clip(self._hold_integ, -HOLD_INTEG_MAX, HOLD_INTEG_MAX,
                        out=self._hold_integ)
            tau_hold = self._dyn.gravity_vector(q_u) + self._hold_integ
            np.multiply(JOINT_SIGN, tau_hold, out=self._tau_hold_motor)
            np.divide(self._tau_hold_motor, TAU_SCALE,
                      out=self._tau_hold_motor)
            with self._lock:
                self._tau_sent[:] = tau_hold      # URDF 系（斜率基准）
        robot.arm.send_mit(pos=self._hold_q, vel=self._hold_vel0,
                           kp=self._hold_kp, kd=self._hold_kd,
                           tau=self._tau_hold_motor)

    def _hold_gripper_posvel(self, tag=""):
        """夹爪组切 POS_VEL 按当前位置保持：消除 MIT 无命令抖动。
        电机 7 静默期（GRIPPER_ACTIVE=False）直接跳过。"""
        if not self.robot.has_gripper or not self._gripper_active:
            return
        try:
            pos, _, _ = self.robot.get_state(request_feedback=True)
            self.robot.gripper.mode_pos_vel()
            self.robot.gripper.enable()
            self.robot.gripper.send_pos_vel(np.array([pos[6]]),
                                            vlim=np.array([0.5]))
            print(f"[real] 夹爪已切 POS_VEL 位置保持{tag}")
        except Exception as exc:  # noqa: BLE001
            print(f"[real] 夹爪保持设置失败（忽略）：{exc}")

    def apply_anchor(self, q_ref, mask):
        """控制期锚定兕底（官方 gravity comp 同思想，_GC_KP 同值）：
        mask=1 的关节在 MIT 控制期叠加 kp=ANCHOR_KP 弹簧锚向 q_ref
        （抗模型误差/摩擦导致的缓慢垂移——kp=0 纯力矩对之零刚度，
        2026-08-29 实测斜坡期臂垂 0.085 rad）；mask=0 任务关节保持
        kp=0 纯力矩直驱（控制律唯一外环，不与弹簧对抗）。仅低自由度
        验证（--traj short 只动 j1）使用。"""
        m = np.asarray(mask, dtype=float)[:6]
        self._anchor_q[:] = np.asarray(q_ref, dtype=float)[:6]
        self._anchor_mask = m
        self._anchor_inv[:] = 1.0 - m
        print("[real] 锚定兕底开启：关节 "
              f"{[i + 1 for i in range(6) if m[i] > 0.5]} kp={ANCHOR_KP}"
              " 锚参考位姿；其余关节 kp=0 纯力矩直驱")

    def _send_mit(self, robot, pos, vel, tau_urdf):
        """MIT 指令下发：tau_ff = 标定逆变换(tau_urdf)。

        v5：vel_d 由实测改为 0——kd 项从恒零变为 -MIT_KD*vel 真速度
        阻尼（j1-3 0.5、j4-6 0.1 N*m*s/rad，"阻尼网"名副其实）：
        [1] 崩溃保护：进程崩溃时总线线程同死、软件看门狗失效，固件
            持续执行的最后帧即本帧——vel_d=0 让它在固件内对速度
            反向出力，臂受阻尼滑行减速而非无阻尼失控；这是唯一能
            覆盖崩溃场景的毫秒级自动保护（固件层，不依赖上位机）；
        [2] 耗散项：qd=0.3 rad/s 时阻尼力矩 0.15/0.03 N*m（j1-3/j4-6），
            short 轨迹 qd≈0.045 时仅 ~0.02 N*m——有界耗散扰动，对
            ISS/定理 3 裕度有利（d(t) 证书覆盖），跟踪滞后由任务
            反馈补偿（与 JOINT_DAMPING 注入同机理）。
        v6.2：apply_anchor 开启后，不动关节 pos_d=锚定参考 +
        kp=ANCHOR_KP（官方 _GC_KP 同值）兕底；任务关节 pos_d=实测、
        kp=0 纯力矩（与 v5 相同）。热路径无分配（预分配向量）。"""
        np.multiply(JOINT_SIGN, tau_urdf, out=self._tmp_bus)
        np.divide(self._tmp_bus, TAU_SCALE, out=self._tmp_bus)
        if self._anchor_mask is not None:
            # 锚定兕底：mask=1 -> pos_d=q_ref, kp=ANCHOR_KP；
            #            mask=0 -> pos_d=q_meas, kp=0
            np.multiply(self._anchor_mask, ANCHOR_KP, out=self._kp_vec)
            np.multiply(self._anchor_mask, self._anchor_q, out=self._tmp_a)
            np.multiply(self._anchor_inv, pos[:6], out=self._tmp_b)
            np.add(self._tmp_a, self._tmp_b, out=self._pos_vec)
            robot.arm.send_mit(pos=self._pos_vec, vel=self._vel_zero,
                               kp=self._kp_vec, kd=MIT_KD,
                               tau=self._tmp_bus)
        else:
            robot.arm.send_mit(pos=pos[:6], vel=self._vel_zero,
                               kp=MIT_KP, kd=MIT_KD, tau=self._tmp_bus)

    # ------------------------------------------------------------------
    # 控制循环接口（与 IsaacB601Backend 同签名；run_lib 直调）
    # ------------------------------------------------------------------

    def reset_to(self, q_init, gripper_width=None):
        """真机无 teleport：校验当前构型已在目标附近（setup() 已自带
        POS_VEL 慢速就位，正常情况此处必过），否则拒绝起步。"""
        q, _ = self.get_joint_state()
        dq = np.abs(q - np.asarray(q_init, dtype=float))
        if float(np.max(dq)) > Q_BRINGUP_TOL:
            raise RuntimeError(
                f"真机当前构型距目标 max|dq|={np.max(dq):.3f} rad > 容差 "
                f"{Q_BRINGUP_TOL}：setup() 就位未生效或实验中臂被外力移动，"
                f"请检查后重启实验（setup 会自动慢速就位）")
        if gripper_width is not None:
            self.set_gripper(gripper_width)

    def get_joint_state(self):
        """(q, qd) [rad, rad/s]，URDF/DH 约定系，joint1..joint6 顺序。"""
        with self._lock:
            return self._q.copy(), self._qd.copy()

    # 用户侧别名（与规划文档方法名一致）
    read_state = get_joint_state

    def get_measured_joint_efforts(self):
        """实测关节力矩 [N*m]（关节系；run_lib CSV meas* 列诊断用）。"""
        with self._lock:
            return self._tau_meas.copy()

    def get_commanded_joint_efforts(self):
        """实际下发的指令力矩 [N*m]（关节系，经使能斜坡/斜率限制后的
        真值；run_lib 0.2 s 终端监视用——与 get_measured_joint_efforts
        成对对账"这步输入 vs 上步读取"）。"""
        with self._lock:
            return self._tau_sent.copy()

    def apply_arm_torques(self, tau):
        """写力矩指令缓冲（控制线程心跳）；二次限幅复核 TAU_MAX。"""
        np.clip(tau, -TAU_MAX, TAU_MAX, out=self._tau_cmd)
        self._cmd_time = time.perf_counter()

    # 用户侧别名
    apply_torque = apply_arm_torques

    def update_gravity_snapshot(self, g_of_q):
        """控制线程每步回填 g(q) 快照：总线线程斜坡/看门狗降级零 RNEA。"""
        self._g_snap[:] = g_of_q

    def set_gripper(self, width_m):
        """指间开度目标 [m]（clip 到物理行程）。电机 7 静默期忽略：
        不写目标、不下发（run_lib 主循环/reset_to 的夹爪调用无害化）。"""
        if not self._gripper_active:
            return
        self._width_cmd = float(np.clip(width_m, 0.0, GRIPPER_WIDTH_MAX))

    def get_gripper_width(self):
        """实测指间开度 [m]（标定换算，未标定时返回目标值兜底）。"""
        with self._lock:
            return self._width_meas

    def get_cube_pose(self):
        """真机无仿真目标物：返回 NaN（CSV cube_* 列支持；接视觉后替换）。"""
        return np.full(3, np.nan), np.full(4, np.nan)

    def step(self, render=None):
        """墙钟配速：真机无物理步进，推进一个 DT = 睡到下一拍边界。

        落后超过 2 拍视为时间基准失效（Jetson 过载/总线阻塞），抛
        HardwareFault 走急停路径——比带着过期状态继续下发安全。
        """
        self._t_next += DT
        sleep_t = self._t_next - time.perf_counter()
        if sleep_t > 0.0:
            time.sleep(sleep_t)
        elif sleep_t < -2.0 * DT:
            raise HardwareFault(
                f"控制循环墙钟落后 {-sleep_t * 1000:.1f} ms（> 2 拍），"
                f"时间基准失效，安全终止")

    # ------------------------------------------------------------------
    # 硬件安全检查（run_lib 主循环每控制步调用；异常 -> 急停）
    # ------------------------------------------------------------------

    def hardware_safety_check(self):
        """通信超时 / 速度异常 / 碰撞-卡滞残差三重检查，故障抛
        HardwareFault（run_lib 捕获 -> abort 记录 -> close() 断输出）。

        与 run_lib 既有保护层的关系：关节限位（check_joint_limits）、
        预测制动治理器、奇异点阻尼均在控制律层原样保留；本检查补充
        真机独有的物理通道故障（仿真中不存在）。
        """
        now = time.perf_counter()
        # [1] 通信超时：反馈停更（串口断开/桥接器掉电）
        if now - self._fb_time > WATCHDOG_TIMEOUT:
            raise HardwareFault(
                f"通信超时：反馈 {(now - self._fb_time) * 1000:.0f} ms 未更新")
        # [1b] 新批次寄存器状态源停更（xout 读回线程失效）
        if self.robot is not None and \
                self.robot.reg_state_age() > WATCHDOG_TIMEOUT:
            raise HardwareFault(
                f"通信超时：新批次 xout 状态 {self.robot.reg_state_age() * 1000:.0f}"
                f" ms 未更新")
        with self._lock:
            # [2] 速度异常：编码器跳变或失控（阈值 2x 首跑 QDOT_MAX）
            vmax = float(np.max(np.abs(self._qd)))
            if vmax > VEL_SPIKE_MAX:
                raise HardwareFault(f"关节速度异常 {vmax:.2f} rad/s")
            # [3] 碰撞/卡滞残差：实测 vs 实际下发（含斜坡/斜率后的真值）
            #     新批次关节无力矩测量（tau_meas 恒 0），跳过免误报
            np.subtract(self._tau_meas, self._tau_sent, out=self._tmp_resid)
            for i in self._no_tau_idx:
                self._tmp_resid[i] = 0.0
        if float(np.max(np.abs(self._tmp_resid))) > CONTACT_RESID_TAU:
            self._resid_count += 1
            if self._resid_count >= CONTACT_RESID_COUNT:
                raise HardwareFault(
                    f"力矩残差持续超阈（>{CONTACT_RESID_TAU} N*m x "
                    f"{self._resid_count} 拍）：疑似碰撞/卡滞")
        else:
            self._resid_count = 0


# ---------------------------------------------------------------------------
# 断开与就位（独立脚本调用）
# ---------------------------------------------------------------------------

def soft_disconnect(robot):
    """断串口但**不失能**（v4 安全修正，E1 上游核查发现）。

    vendor RebotArm.disconnect() 内部 disable_all() 会失能全部电机：
    输出级关闭即零力矩，非静稳定位姿（Q_INIT 除外）下臂必坠——与
    close()/posvel_goto 的"位置保持、未失能"承诺直接矛盾（急停/
    收尾路径最后一步坠臂隐患）。本函数镜像 vendor 的串口关闭序列
    并跳过失能：电机保持使能，PV 位置环在驱动器固件内持续托臂
    （不依赖上位机），MIT 末帧亦持续生效。vendor 为官方冻结拷贝
    不改动，故在此镜像实现。
    """
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


def posvel_goto(q_target, vlim=0.3, settle=1.5, timeout=60.0):
    """位置模式慢速就位：把臂从当前姿态摆到 q_target（如 Q_INIT）。

    独立进程运行（与 MIT 闭环互斥占用串口）：连接 -> POS_VEL -> 使能
    -> 下发位置目标 -> 轮询收敛 -> 失能断连。闭环脚本随后以 MIT 重新
    接管（RealB601Backend.setup）。
    vlim [rad/s] 逐关节速度上限：默认 0.3（保守慢速；需要时可传更小，
    如 vlim=0.1；yaml 额定上限 5/3 不建议接近）。
    """
    from reBotArm_control_py.actuator import RebotArm

    q_target = np.asarray(q_target, dtype=float).reshape(-1)
    robot = RebotArm()
    robot.connect()
    try:
        robot.arm.mode_pos_vel()
        robot.arm.enable()
        t0 = time.perf_counter()
        robot.arm.send_pos_vel(q_target, vlim=np.full(6, float(vlim)))
        # 轮询收敛（驱动器内插值运动，到位后位置误差稳定在小邻域）
        while time.perf_counter() - t0 < timeout:
            time.sleep(0.1)
            robot.arm.send_pos_vel(q_target, vlim=np.full(6, float(vlim)))
            q = robot.arm.get_positions(request_feedback=True)[:6]
            if float(np.max(np.abs(q - q_target))) < 0.01:
                time.sleep(settle)      # 静置稳定
                qf = robot.arm.get_positions(request_feedback=True)[:6]
                print(f"[goto] 就位完成：max|dq|="
                      f"{float(np.max(np.abs(qf - q_target))):.4f} rad")
                return qf
        raise RuntimeError(f"posvel_goto 超时（{timeout} s 未收敛）")
    finally:
        # Borot 式收尾：位置环接管当前/目标位置后断连，不失能——
        # 驱动器持续托臂，后续闭环 setup 可直接接管
        posf, _, _ = robot.get_state(request_feedback=True)
        robot.arm.mode_pos_vel()
        robot.arm.enable()
        for _ in range(5):
            robot.arm.send_pos_vel(posf[:6], vlim=np.full(6, 0.1))
            time.sleep(0.1)
        print("[goto] 已切 POS_VEL 位置保持（未失能）")
        soft_disconnect(robot)


if __name__ == "__main__":
    # 无参自检：只连接 + 读反馈 + 打印标定前原始值（不使能、不下发）
    from reBotArm_control_py.actuator import RebotArm

    r = RebotArm()
    r.connect()
    pos, vel, torq = r.get_state(request_feedback=True)
    print(f"[check] 原始反馈 pos={np.round(pos, 4).tolist()}")
    print(f"[check] 原始反馈 vel={np.round(vel, 4).tolist()}")
    print(f"[check] 原始反馈 torq={np.round(torq, 4).tolist()}")
    print(f"[check] URDF 系 q={np.round(JOINT_SIGN * pos[:6] + JOINT_OFFSET, 4).tolist()}"
          f"（Q_INIT={np.round(Q_INIT, 4).tolist()}）")
    r.disconnect()
