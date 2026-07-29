"""
CoppeliaSim ZMQ Remote API 接口 —— KUKA LBR4+（LWR4+）7R 力矩模式对接。

对接目标（场景篇《KUKALBR4p场景_定点与圆周扰动对比实验设计.md》§1/§8）：
把 run_simulation.py 的内部积分器（式 5.1 的加速度级理想被控对象）替换为
CoppeliaSim 动力学引擎，闭环结构变为（总方案 §5.1 力矩模式）：

    [传感层]   sim.getJointPosition/Velocity  -> q, q̇        （替代内部状态）
    [FK 层]    TNDQ 链连乘（式 3.4）           -> x, ξ, J, J̇q̇（免构造，式 3.5）
    [误差层]   HDQ 误差元素（定理 1/2）        -> e_ξ, e_z, A
    [控制层]   几何一致计算力矩律（式 5.2）    -> q̈_ref
    [力矩层]   τ = M̂ q̈_ref + Ĉ q̇ + ĝ（§2.4，M̂/Ĉ/ĝ 取 Gaz [11] 名义模型）
    [执行层]   sim.setJointTargetForce(h, τ)   （关节须处于力矩动态控制模式）
    [步进]     sim.step()  —— 同步模式 sim.setStepping(True)，
               控制频率与引擎步长的时序契约见场景篇 §1.2 第 4 项

模型失配声明（定理 3 的诚实性条款）：引擎侧真实 M,C,g 与名义 M̂,Ĉ,ĝ 之差
折算为式 (5.1) 的 w_dyn，由 H∞ 增益条件 (5.6a) / ISS 极限球 (5.7) 兜底——
这正是 CoppeliaSim 对接实验相对内部理想仿真的核心增量（总方案 §5.1）。

坐标系与四元数约定（场景篇 §1.2 第 8/9 项）：
- CoppeliaSim 四元数为 [x,y,z,w]，内部 DQ 约定为 [w,x,y,z]，读取时转换；
- TNDQ 链建在基座系：所有 sim 读到的世界系位姿需左乘基座位姿逆
  （read_tip_pose_dq(relative_to_base=True) 已处理）；
- 场景对象路径存在两处历史记录（/lbr4p_joint_i 与 /LBR4p/jointi），
  connect() 按候选列表逐一探测（场景篇 §1.2 第 1 项核查的代码化）。

依赖：pip install coppeliasim-zmqremoteapi-client（导入失败时延迟报错，
不影响 --backend internal 的纯 numpy 运行）。
"""

import time

import numpy as np

from core.dq_algebra import q_mul, q_conj, dq_mul, dq_conj

# 延迟导入：仅在真正连接时才需要 ZMQ 客户端
try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    _HAS_ZMQ = True
except ImportError:      # pragma: no cover - 环境无该包时走内部后端
    RemoteAPIClient = None
    _HAS_ZMQ = False


class CoppeliaSimError(RuntimeError):
    """CoppeliaSim 对接异常基类（连接失败 / 对象缺失 / 模式错误）。"""


class CoppeliaSimConnectionError(CoppeliaSimError):
    """无法建立 ZMQ 连接或场景未加载。"""


# ---------------------------------------------------------------------------
# 场景对象路径候选（场景篇 §1.2 第 1/2 项：两处历史记录不一致，运行时探测）
# ---------------------------------------------------------------------------

JOINT_PATH_CANDIDATES = [
    # 记录 A：hdq_hinf_coppeliasim/sim/joint_names.py（曾实际跑通）
    ["/lbr4p_joint_%d" % (i + 1) for i in range(7)],
    # 记录 B：本文件旧占位（层级式别名）
    ["/LBR4p/joint%d" % (i + 1) for i in range(7)],
    # 记录 C：CoppeliaSim 自带模型的常见别名
    ["/LBR4p/LBR4p_joint%d" % (i + 1) for i in range(7)],
]

TIP_PATH_CANDIDATES = ["/LBR4p/connection", "/lbr4p_tip", "/LBR4p/link8_resp"]
BASE_PATH_CANDIDATES = ["/LBR4p", "/lbr4p"]
CUP_PATH_CANDIDATES = ["/Cup", "/cup", "/Cup[0]"]
CHAIR_PATH_CANDIDATES = ["/Chair", "/chair", "/DiningChair", "/sofa"]


def _quat_xyzw_to_wxyz(q_xyzw):
    """CoppeliaSim [x,y,z,w] -> 内部 [w,x,y,z]，并归一化防漂移。"""
    r = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=float)
    return r / np.linalg.norm(r)


def _pose_to_dq(p, r_wxyz):
    """位置 + 单位四元数 -> 单位 DQ：x = r + ε·½ p r（式 2.1）。"""
    qd = 0.5 * q_mul(np.r_[0.0, np.asarray(p, dtype=float)], r_wxyz)
    return np.r_[r_wxyz, qd]


class CoppeliaSimLBR4Interface:
    """
    KUKA LBR4+ 7R 场景的 ZMQ Remote API 客户端（力矩/速度双模式）。

    典型用法（式 5.1 由引擎实现，控制栈不变——本仓库接口契约）：

        itf = CoppeliaSimLBR4Interface()
        itf.connect()                          # 探测句柄 + 力矩模式 + 同步步进
        itf.start()
        q, q_dot = itf.read_joint_states()
        ... TNDQ FK + 式 (5.2) -> qddot_ref -> τ = M̂q̈_ref + Ĉq̇ + ĝ ...
        itf.send_joint_targets(tau, mode="torque")
        itf.step()
        ...
        itf.disconnect()
    """

    def __init__(self, joint_names=None, host="localhost", port=23000):
        # joint_names 显式给定时只用给定路径；否则按候选列表探测
        self.joint_names = joint_names
        self.host = host
        self.port = port
        self.client = None
        self.sim = None
        self.joint_handles = []
        self.n = 0
        self.tip_handle = None
        self.base_handle = -1          # -1 = 世界系
        self.cup_handle = None
        self.chair_handle = None
        self.sim_dt = None             # 引擎步长（场景篇 §1.2 第 4 项）
        self._running = False
        self._qdot_target = None       # 速度降级模式的积分状态（场景篇 §8 预案）

    # -- 连接与场景核查（场景篇 §1.2 清单的代码化） ---------------------------

    def connect(self, torque_mode=True):
        """
        建立连接并完成里程碑 0 的自动核查：
          1. ZMQ 连接（失败 -> CoppeliaSimConnectionError）；
          2. 关节句柄探测（候选路径逐组尝试，§1.2 第 1 项）；
          3. tip/base/杯子/椅子句柄（§1.2 第 2/6/7 项，杯椅缺失仅警告）；
          4. 力矩动态控制模式设置（§1.2 第 3 项——若场景保存为位置/速度
             PID 模式，此处强制切到 sim.jointdynctrl_force，否则实验退化为
             "比较内置 PID"，即总方案 §2.2(i) 的混淆变量）；
          5. 同步步进 sim.setStepping(True)（§1.2 第 4 项时序契约）。
        """
        if not _HAS_ZMQ:
            raise CoppeliaSimConnectionError(
                "缺少 coppeliasim-zmqremoteapi-client 包："
                "pip install coppeliasim-zmqremoteapi-client")
        try:
            self.client = RemoteAPIClient(host=self.host, port=self.port)
            self.sim = self.client.require("sim")
        except Exception as exc:
            raise CoppeliaSimConnectionError(
                f"无法连接 CoppeliaSim ZMQ 服务 {self.host}:{self.port}，"
                f"请确认软件已启动且场景 KUKALBR4+_sim.ttt 已加载: {exc}") from exc

        # 2. 关节句柄探测
        candidates = ([self.joint_names] if self.joint_names
                      else JOINT_PATH_CANDIDATES)
        self.joint_handles, used = [], None
        for paths in candidates:
            try:
                self.joint_handles = [self.sim.getObject(p) for p in paths]
                used = paths
                break
            except Exception:
                self.joint_handles = []
        if not self.joint_handles:
            raise CoppeliaSimError(
                "关节路径探测失败（候选: %s）。请运行 print_scene_inventory() "
                "枚举场景对象树后回填 joint_names。" % JOINT_PATH_CANDIDATES)
        self.joint_names = used
        self.n = len(self.joint_handles)
        print("[coppeliasim] 关节句柄:")
        for p, h in zip(used, self.joint_handles):
            print(f"    {p} -> {h}  ({self.sim.getObjectAlias(h, 2)})")

        # 3. tip / base / 杯子 / 椅子
        self.tip_handle = self._probe(TIP_PATH_CANDIDATES, "末端 tip", required=True)
        base = self._probe(BASE_PATH_CANDIDATES, "基座", required=False)
        self.base_handle = -1 if base is None else base
        self.cup_handle = self._probe(CUP_PATH_CANDIDATES, "杯子", required=False)
        self.chair_handle = self._probe(CHAIR_PATH_CANDIDATES, "椅子", required=False)

        # 4. 力矩动态控制模式（场景篇 §1.2 第 3 项）
        if torque_mode:
            self._enable_torque_mode()

        # 5. 同步步进 + 引擎步长核查
        self.sim.setStepping(True)
        self.sim_dt = float(self.sim.getSimulationTimeStep())
        print(f"[coppeliasim] 引擎步长 = {self.sim_dt * 1e3:.1f} ms（同步模式）")
        return self

    def _probe(self, paths, label, required):
        """按候选路径探测单个对象句柄。"""
        for p in paths:
            try:
                h = self.sim.getObject(p)
                print(f"[coppeliasim] {label}: {p} -> {h}")
                return h
            except Exception:
                continue
        msg = f"[coppeliasim] 未找到{label}（候选 {paths}）"
        if required:
            raise CoppeliaSimError(msg + "，请核查场景对象树。")
        print(msg + "，相关实验（定点目标/接触扰动）将使用配置默认值。")
        return None

    def _enable_torque_mode(self):
        """把 7 个关节切换到动力学力矩控制模式（sim.jointdynctrl_force）。
        理论意义：保证被控对象是 M q̈ + C q̇ + g = τ（式 3.1）本身，
        而非内置 PID 包裹后的位置环（总方案 §2.2 运动学/动力学不可比论证）。"""
        for h in self.joint_handles:
            try:
                self.sim.setObjectInt32Param(
                    h, self.sim.jointintparam_dynctrlmode,
                    self.sim.jointdynctrl_force)
            except Exception:
                # 兼容旧版 API：力矩模式经典实现 = 大速度目标 + 力矩上限
                # （send_joint_targets 的降级路径会用到）
                print(f"[coppeliasim][warn] 关节 {h} 无法设置 dynctrl_force，"
                      f"将采用速度目标+力矩上限的经典力矩模式")

    # -- 仿真生命周期 -----------------------------------------------------------

    def start(self):
        """启动仿真（若在运行先停止，保证初始状态一致）。"""
        try:
            self.sim.stopSimulation()
            time.sleep(0.3)
        except Exception:
            pass
        self.sim.startSimulation()
        self._running = True

    def step(self):
        """推进一个引擎步（同步模式）。控制频率契约：外层每次 step 前
        必须完成 读取->控制->下发 的完整回路（场景篇 §1.2 第 4 项）。"""
        self.sim.step()

    def disconnect(self):
        """安全断开：力矩清零 -> 停仿真。任何异常路径都应最终走到这里
        （run_simulation.py 用 try/finally 保证）。"""
        if self.sim is None:
            return
        try:
            if self._running:
                if self._qdot_target is not None:
                    self.send_joint_targets(np.zeros(self.n), mode="velocity")
                else:
                    self.send_joint_targets(np.zeros(self.n), mode="torque")
                time.sleep(0.05)
                self.sim.stopSimulation()
        except Exception:
            pass
        finally:
            self._running = False

    # -- 传感层：关节状态与末端位姿 ---------------------------------------------

    def read_joint_states(self):
        """读取 (q, q̇)，shape 各为 (n,)。
        这是控制栈的唯一状态入口（论文 §6：控制器只需 q, q̇ 与 TNDQ 链输出）；
        读取的是引擎积分后的真实状态，天然包含动力学效应与接触影响。"""
        q = np.array([self.sim.getJointPosition(h) for h in self.joint_handles])
        q_dot = np.array([self.sim.getJointVelocity(h) for h in self.joint_handles])
        return q, q_dot

    def set_joint_positions(self, q):
        """直接设置关节角（仅初始化用，如 E4 大姿态误差初始位形；
        运行中禁止调用——会破坏引擎动力学状态）。"""
        for h, qi in zip(self.joint_handles, np.asarray(q, dtype=float)):
            self.sim.setJointPosition(h, float(qi))

    def read_tip_pose_dq(self, relative_to_base=True):
        """
        末端真实位姿 -> 单位 DQ  x = r + ε·½ p r（式 2.1）。

        用途：FK 对齐诊断（场景篇 §1.2 第 9 项）——与 TNDQ 链 FK 比对，
        残差应 < 1 mm / 0.1°；不用于闭环反馈（反馈走关节空间 + 名义 FK，
        保证控制端/仿真端模型一致性，见总方案 §5.1）。
        注意四元数顺序转换：CoppeliaSim [x,y,z,w] -> 内部 [w,x,y,z]。
        """
        ref = self.base_handle if relative_to_base else -1
        p = np.array(self.sim.getObjectPosition(self.tip_handle, ref))
        r = _quat_xyzw_to_wxyz(self.sim.getObjectQuaternion(self.tip_handle, ref))
        return _pose_to_dq(p, r)

    def read_object_position(self, handle, relative_to_base=True):
        """任意对象位置（基座系）；用于读取杯子位置以生成定点/圆周目标
        （场景篇 §2：杯子 = 定点目标 + 圆心参照）。"""
        ref = self.base_handle if relative_to_base else -1
        return np.array(self.sim.getObjectPosition(handle, ref), dtype=float)

    def cup_position(self, default=None):
        """杯子在基座系下的位置；场景缺失时回退到配置默认值。"""
        if self.cup_handle is not None:
            return self.read_object_position(self.cup_handle)
        if default is not None:
            print("[coppeliasim][warn] 场景无杯子对象，定点目标使用配置默认值")
            return np.asarray(default, dtype=float)
        raise CoppeliaSimError("场景中未找到杯子且未提供默认目标位置。")

    # -- 执行层：控制指令下发 -----------------------------------------------------

    def send_joint_targets(self, cmd, mode="torque"):
        """
        下发控制指令。

        mode="torque"（主路径，总方案 §5.1）：
            cmd = τ（N m），τ 应已由名义计算力矩接口装配：
            τ = M̂ q̈_ref + Ĉ q̇ + ĝ（§2.4；M̂/Ĉ/ĝ 取 Gaz [11] 名义模型，
            config/lbr4_dynamics.py）。实现：sim.setJointTargetForce。
            兼容旧引擎的经典力矩模式：目标速度置 ±大值、|τ| 作力矩上限。

        mode="velocity"（降级预案，场景篇 §8）：
            cmd = q̈_ref，内部积分为速度目标发给引擎速度环；
            速度环残差计入扰动 d(t)，结论适用范围相应收窄（总方案 §7 条款 1）。
        """
        cmd = np.asarray(cmd, dtype=float).reshape(self.n)
        if mode == "torque":
            for h, tau_i in zip(self.joint_handles, cmd):
                try:
                    self.sim.setJointTargetForce(h, float(tau_i))
                except Exception:
                    # 经典力矩模式：速度目标带符号大值 + 力矩幅值上限
                    self.sim.setJointTargetVelocity(
                        h, float(np.sign(tau_i)) * 1e3)
                    self.sim.setJointMaxForce(h, float(abs(tau_i)))
        elif mode == "velocity":
            if self._qdot_target is None:
                self._qdot_target = np.zeros(self.n)
            dt = self.sim_dt or 1e-3
            self._qdot_target = self._qdot_target + cmd * dt   # q̈_ref 积分
            for h, v in zip(self.joint_handles, self._qdot_target):
                self.sim.setJointTargetVelocity(h, float(v))
        else:
            raise ValueError(f"未知控制模式 '{mode}'（torque / velocity）")

    # -- 扰动注入辅助（场景篇 §6） -------------------------------------------------

    def attach_cup_to_tip(self):
        """E7 变体之一 / 场景篇 §6.2：把杯子附着到末端（突加负载）。
        等效 ΔM/Δg 阶跃偏差 -> 定理 3(d) ISS 极限球核验。"""
        if self.cup_handle is None:
            print("[coppeliasim][warn] 无杯子对象，跳过突加负载注入")
            return False
        self.sim.setObjectParent(self.cup_handle, self.tip_handle, True)
        print("[coppeliasim] 杯子已附着到末端（突加负载扰动生效）")
        return True

    def read_contact_force_norm(self):
        """E7 / 场景篇 §6.3：读取当前步与场景的接触合力范数，
        作为非建模接触扰动的能量估计（sim.getContactInfo）。"""
        total = 0.0
        try:
            idx = 0
            while True:
                info = self.sim.getContactInfo(
                    self.sim.handle_all, self.sim.handle_all, idx)
                if not info or not info[0]:
                    break
                # info = (objectHandles, point, force, normal)
                total += float(np.linalg.norm(info[2][:3]))
                idx += 1
        except Exception:
            pass
        return total

    # -- 场景清单（里程碑 0 存档工具） ---------------------------------------------

    def print_scene_inventory(self, save_path=None):
        """枚举场景对象树（形状质量/动态属性、关节模式），对应场景篇 §1.1
        自检脚本；save_path 给定时同时存档为文本清单。"""
        lines = []
        for h in self.sim.getObjectsInTree(self.sim.handle_scene):
            alias = self.sim.getObjectAlias(h, 2)
            otype = self.sim.getObjectType(h)
            if otype == self.sim.sceneobject_shape:
                try:
                    mass = self.sim.getShapeMass(h)
                    static = self.sim.getObjectInt32Param(
                        h, self.sim.shapeintparam_static)
                    lines.append(f"{alias:50s} shape  mass={mass:8.3f}  "
                                 f"dynamic={not static}")
                except Exception:
                    lines.append(f"{alias:50s} shape")
            elif otype == self.sim.sceneobject_joint:
                lines.append(f"{alias:50s} joint  mode={self.sim.getJointMode(h)}")
            else:
                lines.append(f"{alias:50s} type={otype}")
        text = "\n".join(lines)
        print(text)
        if save_path:
            with open(save_path, "w") as f:
                f.write(text + "\n")
            print(f"[coppeliasim] 场景清单已存档: {save_path}")
        return text
