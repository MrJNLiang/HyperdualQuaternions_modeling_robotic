"""ROS 通道版 B601 后端 —— 与 TNDQ_real/interfaces/real_backend.py
RealB601Backend **方法签名一致**，experiments/run_lib 主循环零改动替换。

与直连版的唯一结构差异：vendor 串口总线线程（500 Hz 直写 motorbridge）
替换为官方 rebotarmcontroller ROS2 透传通道——

    反馈：/rebotarm/joints/<joint>/state（电机系原始值，官方 100 Hz 发布）
              -> 订阅回调做 标定变换（电机系 -> URDF 系）-> 共享缓冲
    命令：发送线程 send_rate_hz（默认 200 Hz）分相组帧 ->
          /rebotarm/joints/<joint>/cmd（JointMotorCmd mode=MODE_MIT）
              -> 官方 passthrough 逐帧 send_mit 写串口总线
    模式：/rebotarm/set_mode 服务（"mit" 进入 / "pos_vel" 收尾托臂）

安全架构 1:1 镜像直连版（RealB601Backend），语义逐条对应：
    [分相]   保持期（启动/心跳超时/收尾）v3 实证配方：kp=HOLD_KP 弹簧
             锚定冻结目标 + tau_g 前馈 + 积分器（HOLD_RATE_HZ 刷新）；
             控制期 kp=ANCHOR_KP 弹簧锚定最新实测位置 + 论文力矩律
             前馈（斜率限制 -> 使能斜坡），保证末帧失联仍有托臂能力。
    [看门狗] 控制线程心跳 apply_arm_torques 超时 WATCHDOG_TIMEOUT ->
             发送线程自动降级保持配方（进程内共享时钟，不经 DDS）。
             进程崩溃时发送线程同死、软件看门狗失效——与直连版相同，
             最后一层毫秒级保护靠控制期末帧 kd=MIT_KD 阻尼网（vel_d=0）
             在固件内持续生效。
    [收尾]   close()：保持配方稳定 0.3 s -> 停发送线程（末帧 = 保持帧，
             固件持续执行）-> set_mode("pos_vel")（官方包冻结当前位置
             启动 SDK 位置环托臂）——与 soft_disconnect "位置保持、
             未失能"承诺等价。
    [闸门]   setup() 进入 MIT 前做反馈健康检查（连续 20 拍读数稳定），
             镜像直连版 [1b] 总线健康闸门；反馈持续超时/跳变拒绝进入
             力矩控制。
    [硬检]   hardware_safety_check：反馈停更（最旧关节数据龄）+
             速度异常 VEL_SPIKE_MAX -> HardwareFault -> run_lib 急停。
             直连版的碰撞残差检查不可移植（ROS 通道无法甄别哪些关节
             属新批次无力矩测量源），不实现。

模式切换窗口说明：set_mode("mit") 走官方组级 mode_mit（逐电机切换，
存在 <10 ms 的默认零目标窗口）——启动前置条件是臂已由 bringup（POS_VEL
慢速就位）摆到 Q_INIT 伸直静稳位姿，该窗口内重力载荷最小、漂移可忽略；
set_mode 返回后本发送线程立即以保持帧接力。

运行前置：官方 bringup 已启动（rebotarmcontroller 节点 connect + enable，
默认 pos_vel 模式），串口由官方进程独占；本包不接触任何串口资源。
"""

from __future__ import annotations

import threading
import time

import numpy as np

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import JointMotorCmd, JointMotorState
from rebotarm_msgs.msg import ArmStatus
from rebotarm_msgs.srv import SetMode

# 以下 import 依赖 tndq_controller.paths.bootstrap() 已执行（见模块尾注）
from config.params import DT, GRIPPER_OPENING, TAU_MAX
from config.params_real import (
    ANCHOR_KP, GRIPPER_ACTIVE, HOLD_INTEG_GAIN, HOLD_INTEG_MAX, HOLD_KP,
    HOLD_KD, HOLD_RATE_HZ, HOLD_VEL_TH, MIT_KD, Q_BRINGUP_TOL,
    TORQUE_RAMP_TIME, TORQUE_SLEW_MAX, VEL_SPIKE_MAX, WATCHDOG_TIMEOUT,
)
from config.transforms import (
    GRIPPER_M_PER_RAD, GRIPPER_SIGN, GRIPPER_WIDTH_MAX, JOINT_OFFSET,
    JOINT_SIGN, TAU_SCALE,
)
from config.b601_dynamics import B601NominalDynamics


class HardwareFault(RuntimeError):
    """硬件级故障（反馈停更/速度异常）——run_lib 按类名捕获后记入
    CSV abort 字段，并经 finally 触发 backend.close() 安全收尾。"""


class RosChannelBackend:
    """官方 ROS2 透传通道上的 B601 后端（run_lib 可插拔契约）。"""

    def __init__(self, node, arm_namespace: str = "rebotarm",
                 joint_names=None, send_rate_hz: float = 200.0,
                 fb_stale_timeout: float = 0.05) -> None:
        self._node = node
        self._ns = arm_namespace.strip("/")
        self.joint_names = list(joint_names) if joint_names else \
            [f"joint{i}" for i in range(1, 7)]
        self._n = len(self.joint_names)
        self._send_rate = float(send_rate_hz)
        self._fb_stale_timeout = float(fb_stale_timeout)

        # ---- 共享状态（URDF 系；回调写 / 控制线程读，锁内仅定长拷贝）----
        self._lock = threading.Lock()
        self._q_motor = np.zeros(self._n)      # 电机系最新（发送线程 pos_d 用）
        self._q = np.zeros(self._n)            # URDF 系
        self._qd = np.zeros(self._n)
        self._tau_meas = np.zeros(self._n)     # 实测力矩（URDF 系，已标度）
        self._fb_time = np.zeros(self._n)      # 每关节最近反馈到达（perf_counter）
        self._fb_time_max = 0.0

        # ---- 控制线程 -> 发送线程（镜像 RealB601Backend 命名）----
        self._tau_cmd = np.zeros(self._n)      # apply_arm_torques 目标
        self._tau_sent = np.zeros(self._n)     # 斜率限制后的实际下发（URDF 系）
        self._g_snap = np.zeros(self._n)       # g(q) 快照（run_lib 每步回填）
        self._cmd_time = 0.0                   # 控制心跳
        self._ramp_t0 = None                   # 使能斜坡起点

        # ---- 保持期状态（v3 配方，02_gravity_hold / 直连版同款）----
        self._hold_q = np.zeros(self._n)       # 保持目标（电机系冻结）
        self._hold_integ = np.zeros(self._n)   # 积分器（URDF 系 N*m）
        self._hold_tick = 0
        self._hold_every = max(1, int(round(self._send_rate / HOLD_RATE_HZ)))
        self._in_hold = True
        self._tau_hold_motor = np.zeros(self._n)
        self._hold_kp = np.full(self._n, HOLD_KP)
        self._hold_kd = np.full(self._n, HOLD_KD)
        self._vel_zero = np.zeros(self._n)

        # ---- 生命周期 ----
        self._mit_active = False               # setup() 后 True
        self._closing = False                  # close() 收尾（强制保持配方）
        self._send_stop = threading.Event()
        self._send_thread: threading.Thread | None = None
        self._t_next = None                    # step() 墙钟基准（reset_to 初始化）
        self._lag_resync = 0                   # step() 跳拍重置计数（诊断）
        self._vel_spike_run = 0                # 速度超阈连续拍数（去抖）
        self._ctrl_kp = np.full(self._n, ANCHOR_KP)  # 控制期锚定弹簧
                                               # （安全不变量：任何一帧都托得住臂）
        self._stop_requested = False           # ROS 服务请求急停（step 抛 KI）
        self._dyn_hold = B601NominalDynamics()  # 保持期 g(q)（与 run_lib 实例分离）
        self._gripper_cmd_width: float | None = None
        self._gripper_active = bool(GRIPPER_ACTIVE)

        # ---- ROS IO ----
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # 官方 arm_status 为 latched（TRANSIENT_LOCAL），用于校验关节表
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._arm_status: ArmStatus | None = None
        self._status_sub = node.create_subscription(
            ArmStatus, f"/{self._ns}/arm_status", self._on_arm_status,
            status_qos)
        self._state_subs = []
        for i, name in enumerate(self.joint_names):
            self._state_subs.append(node.create_subscription(
                JointMotorState, f"/{self._ns}/joints/{name}/state",
                self._make_state_cb(i), sensor_qos))
        # 官方 cmd 订阅端为 RELIABLE（motor_passthrough），发布端必须匹配
        cmd_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._cmd_pubs = {
            name: node.create_publisher(
                JointMotorCmd, f"/{self._ns}/joints/{name}/cmd", cmd_qos)
            for name in self.joint_names
        }
        if self._gripper_active:
            self._gripper_pub = node.create_publisher(
                JointMotorCmd, f"/{self._ns}/gripper/cmd", cmd_qos)
        else:
            self._gripper_pub = None
        self._mode_client = node.create_client(
            SetMode, f"/{self._ns}/set_mode")

    # ------------------------------------------------------------------
    # ROS 回调 / 服务
    # ------------------------------------------------------------------

    def _on_arm_status(self, msg: ArmStatus) -> None:
        self._arm_status = msg

    def _make_state_cb(self, i: int):
        def _cb(msg: JointMotorState) -> None:
            now = time.perf_counter()
            q_m = float(msg.position)
            qd_m = float(msg.velocity)
            tau_m = float(msg.torque)
            with self._lock:
                self._q_motor[i] = q_m
                self._q[i] = JOINT_SIGN[i] * q_m + JOINT_OFFSET[i]
                self._qd[i] = JOINT_SIGN[i] * qd_m
                self._tau_meas[i] = TAU_SCALE[i] * (JOINT_SIGN[i] * tau_m)
                self._fb_time[i] = now
                self._fb_time_max = now
        return _cb

    def _call_set_mode(
        self, mode: str, timeout: float = 20.0, pump_executor: bool = False,
    ) -> bool:
        """同步调用官方 /rebotarm/set_mode。

        正常服务回调中由 MultiThreadedExecutor 的其他线程处理响应；
        主执行器退出后的 SIGINT 收尾则必须临时泵送 future，否则 close()
        会一直等到超时，MIT 模式无法交回官方 POS_VEL 托臂。
        """
        if not self._mode_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                f"/{self._ns}/set_mode 服务不可用——官方 rebotarmcontroller"
                " 未启动？")
        req = SetMode.Request()
        req.mode = mode
        future = self._mode_client.call_async(req)
        t0 = time.monotonic()
        if pump_executor:
            import rclpy
            remaining = max(0.0, timeout - (time.monotonic() - t0))
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=remaining,
            )
        else:
            while not future.done():
                if time.monotonic() - t0 > timeout:
                    raise RuntimeError(f"set_mode({mode!r}) 响应超时")
                time.sleep(0.005)
        if not future.done():
            raise RuntimeError(f"set_mode({mode!r}) 响应超时")
        resp = future.result()
        if not resp.success:
            raise RuntimeError(f"set_mode({mode!r}) 被官方包拒绝: {resp.message}")
        self._node.get_logger().info(f"官方模式已切换: {mode}")
        return True

    # ------------------------------------------------------------------
    # 生命周期（镜像 RealB601Backend.setup / close）
    # ------------------------------------------------------------------

    def setup(self):
        """反馈健康闸门 -> 官方组级切 MIT -> 保持帧接力（发送线程）。

        前置：官方 bringup 已 connect+enable（pos_vel 默认模式）；臂已
        由 bringup 服务慢速就位 Q_INIT（组级切 MIT 的 <10 ms 默认目标
        窗口在伸直静稳位姿下无害）。幂等：重复调用直接返回。
        """
        if self._mit_active:
            return
        # [0] 关节表校验（arm_status latched，正常立即到达）
        self._verify_joint_names()
        # [1] 反馈健康闸门（镜像直连版 [1b]：连续 20 拍读数稳定才放行）
        self._wait_feedback_healthy()
        # [2] 官方组级切 MIT（内部会停掉 pos_vel 控制循环）
        self._call_set_mode("mit")
        # [3] 初始化重力快照/保持目标，启动发送线程立即接力保持帧
        with self._lock:
            q_m = self._q_motor.copy()
            q_u = self._q.copy()
        tau_g = self._dyn_hold.gravity_vector(q_u)
        self._g_snap[:] = tau_g
        self._tau_sent[:] = tau_g
        self._hold_q[:] = q_m
        self._ramp_t0 = time.perf_counter()
        self._cmd_time = self._ramp_t0
        self._closing = False
        self._send_stop.clear()
        self._send_thread = threading.Thread(
            target=self._send_loop, name="tndq-ros-send", daemon=True)
        self._send_thread.start()
        self._mit_active = True
        self._node.get_logger().info(
            f"后端就绪：发送线程 {self._send_rate:.0f} Hz（官方透传通道）、"
            f"控制周期 10 ms、力矩斜坡 {TORQUE_RAMP_TIME} s、保持期配方 "
            f"kp={HOLD_KP}/kd={HOLD_KD}")

    def _verify_joint_names(self) -> None:
        deadline = time.monotonic() + 2.0
        while self._arm_status is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if self._arm_status is None:
            self._node.get_logger().warn(
                "未收到 /arm_status（latched），跳过关节表校验")
            return
        official = list(self._arm_status.joint_names)
        if official[:self._n] != self.joint_names:
            raise RuntimeError(
                f"关节表不匹配：本包配置 {self.joint_names}，官方包 "
                f"{official}——请修正 tndq_controller.yaml joint_names")

    def _wait_feedback_healthy(self, timeout: float = 15.0) -> None:
        """等 6 关节反馈齐 + 连续 20 拍（20 ms 间隔）读数稳定 <0.02 rad。

        总时长有界（约 timeout + 最后一轮内层 5 s）：跳变不重置总计时，
        超时必返回并携带最后一次失败原因（否则 start 服务会无限无响应）。
        """
        t0 = time.monotonic()
        last_reason = "反馈未到齐（/rebotarm/joints/*/state 无数据）"
        while time.monotonic() - t0 < timeout:
            with self._lock:
                ready = bool(np.all(self._fb_time > 0.0))
                q_ref = None if not ready else self._q_motor.copy()
            if not ready:
                time.sleep(0.05)
                continue
            n_ok = 0
            hc0 = time.monotonic()
            while time.monotonic() - hc0 < 5.0:
                time.sleep(0.02)
                with self._lock:
                    q_now = self._q_motor.copy()
                jump = float(np.max(np.abs(q_now - q_ref)))
                if jump > 0.02:
                    n_ok = 0                     # 读数跳变：反馈不可信，重计
                    last_reason = (f"读数持续跳变（相邻拍 {jump:.3f} rad "
                                   "> 0.02）——臂未静稳或反馈抖动")
                    q_ref = q_now
                else:
                    n_ok += 1
                if n_ok >= 20:
                    break
            if n_ok >= 20:
                self._node.get_logger().info(
                    f"反馈健康检查通过（连续 {n_ok} 拍读数稳定）")
                return
            time.sleep(0.2)                      # 跳变未消：稍候重试（不重置总计时）
        raise RuntimeError(
            f"反馈健康检查未通过（{timeout:.0f}s 超时）：{last_reason}。"
            "检查 ros2 topic hz /rebotarm/joints/joint1/state，并确认臂已"
            "由 bringup 就位静稳")

    def close(self, pump_executor: bool = False):
        """安全收尾（镜像直连版 Borot 式，全程不失能）：保持配方稳定
        0.3 s -> 停发送线程（末帧 = 保持帧，固件持续执行托臂）->
        set_mode("pos_vel")（官方包冻结当前位置、启动 SDK 位置环接力）。
        急停路径（HardwareFault / KeyboardInterrupt / SIGINT）同样走这里。
        ``pump_executor=True`` 用于主执行器已停止后的 SIGINT 收尾。
        """
        if not self._mit_active:
            return
        self._closing = True                     # 发送线程切保持配方
        time.sleep(0.3)                          # 保持帧稳定托臂
        self._send_stop.set()
        if self._send_thread is not None:
            self._send_thread.join(timeout=2.0)
            self._send_thread = None
        try:
            # 停发帧 -> 组级切 pos_vel（官方 _start_pos_vel_loop 冻结当前
            # 位置托臂）；切换窗口内末帧 MIT 弹簧仍在固件内生效
            self._call_set_mode("pos_vel", pump_executor=pump_executor)
        except Exception as exc:                 # noqa: BLE001
            self._node.get_logger().error(
                f"set_mode(pos_vel) 收尾失败（末帧保持仍在托臂，请人工"
                f"介入）: {exc}")
        self._mit_active = False
        self._node.get_logger().info(
            "后端已关闭：官方包 pos_vel 位置保持接管（未失能）")

    def __enter__(self):
        self.setup()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # ------------------------------------------------------------------
    # 发送线程（默认 200 Hz；分相逻辑镜像直连版 _bus_cycle）
    # ------------------------------------------------------------------

    def _send_loop(self) -> None:
        dt = 1.0 / self._send_rate
        next_t = time.perf_counter()
        err_count = 0
        while not self._send_stop.is_set():
            now = time.perf_counter()
            if now < next_t:
                time.sleep(min(next_t - now, 0.002))
                continue
            next_t += dt
            if next_t < now:                     # 落后过多：重置基准
                next_t = now + dt
            try:
                self._send_tick(dt)
                err_count = 0
            except Exception as exc:             # noqa: BLE001
                err_count += 1
                if err_count <= 3 or err_count % 100 == 0:
                    self._node.get_logger().error(
                        f"发送线程异常（累计 {err_count} 拍）: {exc}")

    def _send_tick(self, dt: float) -> None:
        now = time.perf_counter()
        with self._lock:
            q_m = self._q_motor.copy()
        watchdog = now - self._cmd_time > WATCHDOG_TIMEOUT

        if (not self._mit_active) or self._closing or watchdog:
            if watchdog and not self._in_hold:
                self._node.get_logger().warn(
                    "控制心跳超时，切入保持期（kp=7 弹簧 + 重力补偿 + "
                    "积分器）")
            self._in_hold = True
            self._hold_refresh(q_m)
            self._publish_mit(self._hold_q, self._vel_zero,
                              self._hold_kp, self._hold_kd,
                              self._tau_hold_motor)
            return

        if self._in_hold:
            self._in_hold = False
            self._node.get_logger().info(
                "控制心跳恢复，切回控制律透传（kp=0 纯力矩直驱）")

        # [1] 斜率限制（每拍最多 TORQUE_SLEW_MAX*dt，护轻腕关节）
        dmax = TORQUE_SLEW_MAX * dt
        with self._lock:
            delta = np.clip(self._tau_cmd - self._tau_sent, -dmax, dmax)
            self._tau_sent += delta
            tau = self._tau_sent.copy()

        # [2] 使能斜坡：重力补偿 -> 全控制线性混入
        alpha = (now - self._ramp_t0) / TORQUE_RAMP_TIME
        if alpha < 1.0:
            tau = tau * alpha + self._g_snap * (1.0 - alpha)

        # [3] MIT 下发（URDF 系 -> 电机系逆变换）。
        #
        # 安全不变量（v2）：控制期帧 kp=ANCHOR_KP(7)、pos_d=最新实测
        # 位置——正常运行时弹簧项 kp*(q_send-q) 仅为"反馈龄 x 速度"
        # 量级（10 ms x 0.3 rad/s -> ~0.02 N*m，微弱附加阻尼，按论文
        # 诚实条款计入 d(t)）；而进程/通道死亡时固件锁存的末帧是
        # "kp=7 弹簧锚死点 + 末次力矩前馈"，臂被托住不塌（对比旧版
        # kp=0 纯力矩：末帧零刚度，崩溃即塌臂）。kd=MIT_KD 阻尼网同前。
        tau_motor = JOINT_SIGN * tau / TAU_SCALE
        self._publish_mit(q_m, self._vel_zero,
                          self._ctrl_kp, MIT_KD, tau_motor)

    def _hold_refresh(self, q_m: np.ndarray) -> None:
        """保持期配方慢速刷新（HOLD_RATE_HZ 口径；镜像 _hold_cycle）。"""
        self._hold_tick += 1
        if self._hold_tick < self._hold_every:
            return
        self._hold_tick = 0
        with self._lock:
            q_u = self._q.copy()
            qd = self._qd.copy()
        if float(np.max(np.abs(qd))) > HOLD_VEL_TH:
            self._hold_q[:] = q_m                # 目标跟随（手掰/外力扰动）
            self._hold_integ *= 0.9              # 积分衰减防顶死
        else:
            q_ref_u = JOINT_SIGN * self._hold_q + JOINT_OFFSET
            self._hold_integ += (q_ref_u - q_u) * HOLD_INTEG_GAIN
            np.clip(self._hold_integ, -HOLD_INTEG_MAX, HOLD_INTEG_MAX,
                    out=self._hold_integ)
        tau_hold = self._dyn_hold.gravity_vector(q_u) + self._hold_integ
        self._tau_hold_motor[:] = JOINT_SIGN * tau_hold / TAU_SCALE
        with self._lock:
            self._tau_sent[:] = tau_hold         # URDF 系（斜率基准连续）

    def _publish_mit(self, pos_m, vel, kp, kd, tau_m) -> None:
        for i, name in enumerate(self.joint_names):
            msg = JointMotorCmd()
            msg.mode = JointMotorCmd.MODE_MIT
            msg.use_pos = True
            msg.use_vel = True
            msg.use_kp = True
            msg.use_kd = True
            msg.use_tau = True
            msg.pos = float(pos_m[i])
            msg.vel = float(vel[i])
            msg.kp = float(kp[i])
            msg.kd = float(kd[i])
            msg.tau = float(tau_m[i])
            self._cmd_pubs[name].publish(msg)

    def publish_posvel(self, q_urdf, vlim: float = 0.3) -> None:
        """组级 POS_VEL 位置目标（bringup 就位用；官方透传同步更新其
        内部 _q_target，由官方 SDK 500 Hz 位置循环平滑执行）。"""
        q_urdf = np.asarray(q_urdf, dtype=float)[:self._n]
        q_m = JOINT_SIGN * (q_urdf - JOINT_OFFSET)
        for i, name in enumerate(self.joint_names):
            msg = JointMotorCmd()
            msg.mode = JointMotorCmd.MODE_POS_VEL
            msg.use_pos = True
            msg.use_vlim = True
            msg.pos = float(q_m[i])
            msg.vlim = float(vlim)
            self._cmd_pubs[name].publish(msg)

    def _publish_gripper(self, width_m: float) -> None:
        if self._gripper_pub is None:
            return
        # 官方 passthrough mode=1 分支把 cmd.pos 解释为指间开度 [m]
        msg = JointMotorCmd()
        msg.mode = JointMotorCmd.MODE_POS_VEL
        msg.use_pos = True
        msg.pos = float(np.clip(width_m, 0.0, GRIPPER_WIDTH_MAX))
        self._gripper_pub.publish(msg)

    # ------------------------------------------------------------------
    # 控制循环接口（与 RealB601Backend 同签名；run_lib 直调）
    # ------------------------------------------------------------------

    def reset_to(self, q_init, gripper_width=None):
        """真机无 teleport：校验当前构型已在目标附近（bringup 就位后
        必过），否则拒绝起步。顺带初始化 step() 墙钟基准（直连版同款；
        v2 修复：旧版漏初始化，首次 step() 即 TypeError 杀死实验线程）。"""
        q, _ = self.get_joint_state()
        dq = np.abs(q - np.asarray(q_init, dtype=float))
        if float(np.max(dq)) > Q_BRINGUP_TOL:
            raise RuntimeError(
                f"当前构型距目标 max|dq|={np.max(dq):.3f} rad > 容差 "
                f"{Q_BRINGUP_TOL}：先调用 bringup 服务慢速就位（或检查"
                f"臂是否被外力移动）")
        self._t_next = time.perf_counter()
        self._lag_resync = 0
        if gripper_width is not None:
            self.set_gripper(gripper_width)

    def get_joint_state(self):
        """(q, qd) [rad, rad/s]，URDF/DH 约定系，joint1..joint6 顺序。"""
        with self._lock:
            return self._q.copy(), self._qd.copy()

    read_state = get_joint_state

    def get_measured_joint_efforts(self):
        with self._lock:
            return self._tau_meas.copy()

    def get_commanded_joint_efforts(self):
        with self._lock:
            return self._tau_sent.copy()

    def apply_arm_torques(self, tau):
        """写力矩指令缓冲（控制线程心跳）；二次限幅复核 TAU_MAX。"""
        np.clip(tau, -TAU_MAX, TAU_MAX, out=self._tau_cmd[:len(tau)])
        self._cmd_time = time.perf_counter()

    apply_torque = apply_arm_torques

    def update_gravity_snapshot(self, g_of_q):
        """控制线程每步回填 g(q) 快照：发送线程斜坡/看门狗降级零 RNEA。"""
        self._g_snap[:] = g_of_q

    def set_gripper(self, width_m):
        """指间开度目标 [m]（夹爪静默期 GRIPPER_ACTIVE=False 忽略）。"""
        if not self._gripper_active:
            return
        w = float(np.clip(width_m, 0.0, GRIPPER_WIDTH_MAX))
        self._gripper_cmd_width = w
        self._publish_gripper(w)

    def get_gripper_width(self):
        """ROS 通道版未接夹爪反馈换算（官方 joint_states 含视觉缩放，
        不作物理口径）：返回最近目标值兜底（run_lib CSV 列支持）。"""
        return (self._gripper_cmd_width
                if self._gripper_cmd_width is not None else GRIPPER_OPENING)

    def get_cube_pose(self):
        """无视觉目标物：NaN（CSV cube_* 列支持；接视觉后替换）。"""
        return np.full(3, np.nan), np.full(4, np.nan)

    def step(self, render=None):
        """墙钟配速：推进一个 DT = 睡到下一拍边界。

        v2 去过敏（WSL2 + DDS 非实时环境，毫秒级尖峰是常态）：落后
        不再直接抛 HardwareFault 终止——落后 > 2 拍时跳拍重置基准并
        计数（缺拍期间发送线程末帧力矩由固件持续执行，且控制期帧
        自带 kp=7 弹簧安全网，物理层无危险）；仅当单次落后超过
        WATCHDOG_TIMEOUT（发送线程已降级保持期，实验时间基准与
        物理脱节）才终止。stop 服务请求经 KeyboardInterrupt 优雅
        终止（run_lib 保存 CSV 后收尾）。"""
        if self._stop_requested:
            self._stop_requested = False
            raise KeyboardInterrupt("stop 服务请求终止")
        if self._t_next is None:                 # reset_to 未走到（防御）
            self._t_next = time.perf_counter()
        self._t_next += DT
        sleep_t = self._t_next - time.perf_counter()
        if sleep_t > 0.0:
            time.sleep(sleep_t)
            return
        lag = -sleep_t
        if lag > WATCHDOG_TIMEOUT:
            raise HardwareFault(
                f"控制循环墙钟落后 {lag * 1000:.1f} ms（> 看门狗阈值 "
                f"{WATCHDOG_TIMEOUT * 1000:.0f} ms，发送线程已降级保持期），"
                "时间基准失效，安全终止")
        if lag > 2.0 * DT:                       # 跳拍重置：不终止，只记账
            self._t_next = time.perf_counter()
            self._lag_resync += 1
            if self._lag_resync <= 5 or self._lag_resync % 50 == 0:
                self._node.get_logger().warn(
                    f"控制循环落后 {lag * 1000:.1f} ms，跳拍重置基准"
                    f"（累计 {self._lag_resync} 次；计入实现层扰动）")

    def request_stop(self):
        """线程安全请求终止当前实验（下次 step() 触发 KeyboardInterrupt）。"""
        self._stop_requested = True

    # ------------------------------------------------------------------
    # 硬件安全检查（run_lib 每控制步调用；异常 -> 急停）
    # ------------------------------------------------------------------

    def hardware_safety_check(self):
        """反馈停更 / 速度异常两重检查（直连版残差检查因通道无法甄别
        无力矩测量关节而不移植，见模块 docstring）。

        v2 去抖：速度超阈需连续 3 个控制步（30 ms）才判故障——旧版
        单拍即停，DDS 乱序/差分毛刺会误杀实验；30 ms 内发送线程斜率
        限制 + 固件 kd 阻尼网仍在物理层兜底。"""
        now = time.perf_counter()
        with self._lock:
            fb_age = now - float(np.min(self._fb_time))
            vmax = float(np.max(np.abs(self._qd)))
        if fb_age > self._fb_stale_timeout:
            raise HardwareFault(
                f"反馈停更：最旧关节 {fb_age * 1000:.0f} ms 未更新"
                f"（阈值 {self._fb_stale_timeout * 1000:.0f} ms）")
        if vmax > VEL_SPIKE_MAX:
            self._vel_spike_run += 1
            if self._vel_spike_run >= 3:
                raise HardwareFault(
                    f"关节速度异常 {vmax:.2f} rad/s（连续 "
                    f"{self._vel_spike_run} 拍 > {VEL_SPIKE_MAX}）")
        else:
            self._vel_spike_run = 0

    # ------------------------------------------------------------------
    # 诊断快照（节点遥测用）
    # ------------------------------------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            q = self._q.copy()
            qd = self._qd.copy()
            tau_sent = self._tau_sent.copy()
            tau_cmd = self._tau_cmd.copy()
            fb_age = (time.perf_counter() - float(np.max(self._fb_time))
                      if self._fb_time_max > 0 else float("inf"))
        now = time.perf_counter()
        return {
            "q": q, "qd": qd, "tau_sent": tau_sent, "tau_cmd": tau_cmd,
            "phase": ("idle" if not self._mit_active else
                      "closing" if self._closing else
                      "hold" if (self._in_hold or
                                 now - self._cmd_time > WATCHDOG_TIMEOUT)
                      else "control"),
            "ramp_alpha": (1.0 if self._ramp_t0 is None else
                           min(1.0, (now - self._ramp_t0) / TORQUE_RAMP_TIME)),
            "fb_age_ms": fb_age * 1000.0,
        }
