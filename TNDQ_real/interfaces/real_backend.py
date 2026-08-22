"""
B601-DM 真机后端 —— Jetson + 达妙 DM 电机（MIT 模式力矩直驱）。

与 interfaces/isaac_interface.py 的 IsaacB601Backend **方法签名一致**，
experiments/run_lib.py 主控制循环零改动替换（后端可插拔契约）。

架构（双线程，GIL 下共享缓冲区 + 锁）：
    总线线程 500 Hz（RebotArm.start_control_loop）：
        读反馈(pos/vel/torq) -> 标定变换 -> 共享状态
        看门狗 -> 取指令缓冲 -> 斜率限制 -> 使能斜坡混入 -> MIT 下发
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
                                CONTACT_RESID_TAU, MIT_KD, MIT_KP,
                                Q_BRINGUP_TOL, TORQUE_RAMP_TIME,
                                TORQUE_SLEW_MAX, VEL_SPIKE_MAX,
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
        self._cmd_time = 0.0               # 控制线程心跳（apply_arm_torques）
        self._fb_time = 0.0                # 最近一次反馈到达时刻
        self._ramp_t0 = None               # 使能斜坡起点（setup 时置位）

        # ---- 热路径临时缓冲（预分配，总线/控制线程各自专用）----
        self._tmp_bus = np.zeros(6)        # 总线线程变换暂存
        self._tmp_resid = np.zeros(6)      # 残差检查暂存
        self._resid_count = 0              # 残差连续超阈拍数
        self._fault = None                 # 总线线程检测到的降级原因

        # ---- 墙钟配速（step() 时间基准）----
        self._t_next = None

        # ---- 动力学（重力快照/降级补偿用；pinocchio RNEA 后端）----
        self._dyn = None

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def setup(self):
        """连接总线 -> MIT 模式 -> 使能 -> 启动 500 Hz 总线线程。"""
        if self.robot is not None:
            raise RuntimeError("setup() 已执行过")
        # 延迟导入：motorbridge 为原生绑定，导入即占用串口资源
        from reBotArm_control_py.actuator import RebotArm
        from config.b601_dynamics import B601NominalDynamics

        self._dyn = B601NominalDynamics()
        self.robot = RebotArm()
        self.robot.connect()
        self.robot.arm.mode_mit()
        if self.robot.has_gripper:
            self.robot.gripper.mode_mit()

        # 使能前先读一拍状态，初始化重力快照（斜坡起点 = 纯重力补偿）
        pos, _, _ = self.robot.get_state(request_feedback=True)
        q0 = JOINT_SIGN * pos[:6] + JOINT_OFFSET
        self._g_snap[:] = self._dyn.gravity_vector(q0)
        self._tau_sent[:] = self._g_snap

        self.robot.enable_all()
        self._ramp_t0 = time.perf_counter()
        self._cmd_time = self._ramp_t0
        self._fb_time = self._ramp_t0
        self._running = True
        self.robot.start_control_loop(self._bus_cycle, rate=BUS_RATE_HZ)
        self._t_next = time.perf_counter()
        print(f"[real] 后端就绪：总线 {BUS_RATE_HZ:.0f} Hz（MIT 力矩直驱）、"
              f"控制周期 {DT * 1000:.0f} ms、力矩斜坡 {TORQUE_RAMP_TIME} s")

    def close(self):
        """安全关闭：重力补偿收尾 -> 停总线线程 -> 断使能 -> 断串口。

        急停路径（HardwareFault / KeyboardInterrupt）同样走这里：
        先以重力补偿托住臂 0.3 s（避免突然断使能坠臂），再 disable。
        """
        if self.robot is None:
            return
        self._running = False
        try:
            time.sleep(0.3)                    # 总线线程此时只发 g(q)（见 _bus_cycle）
            self.robot.stop_control_loop()
        finally:
            try:
                self.robot.disable_all()       # 切断电机力矩输出
            except Exception as exc:  # noqa: BLE001
                print(f"[real] disable 异常: {exc}")
            try:
                self.robot.disconnect()        # 关闭串口
            except Exception as exc:  # noqa: BLE001
                print(f"[real] disconnect 异常: {exc}")
            print("[real] 后端已关闭：电机失能、串口释放")

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
        """一拍总线：读反馈 -> 变换 -> 看门狗 -> 斜率限制 -> 斜坡混入 -> 下发。"""
        if not self._running:
            # 收尾阶段：只发重力补偿托臂，等待 close() 停线程
            pos, vel, _ = robot.get_state(request_feedback=True)
            np.multiply(JOINT_SIGN, pos[:6], out=self._tmp_bus)
            np.add(self._tmp_bus, JOINT_OFFSET, out=self._tmp_bus)
            self._send_mit(robot, pos, vel, self._g_snap)
            return

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

        # [3] 看门狗：控制线程心跳超期 -> 降级纯重力补偿
        now = time.perf_counter()
        if now - self._cmd_time > WATCHDOG_TIMEOUT:
            tau_target = self._g_snap
            if self._fault != "watchdog":
                self._fault = "watchdog"
                print("[real] 警告：控制线程心跳超时，总线层降级重力补偿")
        else:
            tau_target = self._tau_cmd
            self._fault = None

        # [4] 斜率限制（防指令跳变冲击轻腕；每拍最多 SLEW*BUS_DT）
        dmax = TORQUE_SLEW_MAX * dt
        with self._lock:
            np.subtract(tau_target, self._tau_sent, out=self._tmp_bus)
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

        # [7] 夹爪：MIT 位置保持（宽度目标 -> 电机位置，标定换算）
        if robot.has_gripper:
            g_target = GRIPPER_SIGN * self._width_cmd / GRIPPER_M_PER_RAD
            robot.gripper.send_mit(pos=np.array([g_target]),
                                   vel=np.zeros(1),
                                   kp=np.array([4.0]), kd=np.array([0.5]),
                                   tau=np.zeros(1))

    def _send_mit(self, robot, pos, vel, tau_urdf):
        """MIT 指令下发：tau_ff = 标定逆变换(tau_urdf)，kp=0/kd=MIT_KD，
        pos/vel 目标取实测值（kp/kd 项恒零，等效纯力矩直驱 + 阻尼网）。"""
        np.multiply(JOINT_SIGN, tau_urdf, out=self._tmp_bus)
        np.divide(self._tmp_bus, TAU_SCALE, out=self._tmp_bus)
        robot.arm.send_mit(pos=pos[:6], vel=vel[:6],
                           kp=MIT_KP, kd=MIT_KD, tau=self._tmp_bus)

    # ------------------------------------------------------------------
    # 控制循环接口（与 IsaacB601Backend 同签名；run_lib 直调）
    # ------------------------------------------------------------------

    def reset_to(self, q_init, gripper_width=None):
        """真机无 teleport：校验当前构型已在目标附近（由 POS_VEL 就位
        脚本预先完成，见 posvel_goto），否则拒绝起步。"""
        q, _ = self.get_joint_state()
        dq = np.abs(q - np.asarray(q_init, dtype=float))
        if float(np.max(dq)) > Q_BRINGUP_TOL:
            raise RuntimeError(
                f"真机当前构型距目标 max|dq|={np.max(dq):.3f} rad > 容差 "
                f"{Q_BRINGUP_TOL}：请先运行就位脚本（posvel_goto -> "
                f"Q_INIT）再启动闭环实验")
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
        """指间开度目标 [m]（clip 到物理行程）。"""
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
        with self._lock:
            # [2] 速度异常：编码器跳变或失控（阈值 2x 首跑 QDOT_MAX）
            vmax = float(np.max(np.abs(self._qd)))
            if vmax > VEL_SPIKE_MAX:
                raise HardwareFault(f"关节速度异常 {vmax:.2f} rad/s")
            # [3] 碰撞/卡滞残差：实测 vs 实际下发（含斜坡/斜率后的真值）
            np.subtract(self._tau_meas, self._tau_sent, out=self._tmp_resid)
        if float(np.max(np.abs(self._tmp_resid))) > CONTACT_RESID_TAU:
            self._resid_count += 1
            if self._resid_count >= CONTACT_RESID_COUNT:
                raise HardwareFault(
                    f"力矩残差持续超阈（>{CONTACT_RESID_TAU} N*m x "
                    f"{self._resid_count} 拍）：疑似碰撞/卡滞")
        else:
            self._resid_count = 0


# ---------------------------------------------------------------------------
# POS_VEL 就位（独立脚本调用，闭环之前执行）
# ---------------------------------------------------------------------------

def posvel_goto(q_target, vlim=1.0, settle=1.5, timeout=60.0):
    """位置模式慢速就位：把臂从当前姿态摆到 q_target（如 Q_INIT）。

    独立进程运行（与 MIT 闭环互斥占用串口）：连接 -> POS_VEL -> 使能
    -> 下发位置目标 -> 轮询收敛 -> 失能断连。闭环脚本随后以 MIT 重新
    接管（RealB601Backend.setup）。
    vlim [rad/s] 取保守值（yaml 额定 5/3 的一半以下）。
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
        robot.arm.disable()
        robot.disconnect()


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
