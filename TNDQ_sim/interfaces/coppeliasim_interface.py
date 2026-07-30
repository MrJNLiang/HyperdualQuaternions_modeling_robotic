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
    # 记录 D：KUKALBR4+_sim.ttt 实测层级（4.10 默认别名 joint/link 嵌套，
    # 2026-07 连接诊断确认：joint_i = /LBR4p/joint(/link/joint)^{i-1}）
    ["/LBR4p/joint" + "/link/joint" * i for i in range(7)],
]

TIP_PATH_CANDIDATES = ["/LBR4p/connection", "/lbr4p_tip", "/LBR4p/link8_resp"]
BASE_PATH_CANDIDATES = ["/LBR4p", "/lbr4p"]
CUP_PATH_CANDIDATES = ["/Cup", "/cup", "/Cup[0]"]
CHAIR_PATH_CANDIDATES = ["/diningChair", "/Chair", "/chair", "/DiningChair", "/sofa"]


def probe_joint_handles(sim, candidates=JOINT_PATH_CANDIDATES, n_expected=7):
    """探测 7 个旋转关节句柄，返回 (handles, used_paths)。

    先按候选路径逐组尝试；全部落空则回退为基座子树遍历：
    sim.getObjectsInTree(base, sceneobject_joint) 按树序返回串联链关节
    （场景篇 §1.2 第 1 项——路径记录不可靠时以场景实际结构为准）。
    """
    for paths in candidates:
        try:
            return [sim.getObject(p) for p in paths], paths
        except Exception:
            continue
    # 回退：遍历基座子树收集关节（串联链树序 = 关节序）
    for base_path in BASE_PATH_CANDIDATES:
        try:
            base = sim.getObject(base_path)
        except Exception:
            continue
        handles = sim.getObjectsInTree(base, sim.sceneobject_joint, 0)
        if len(handles) >= n_expected:
            handles = handles[:n_expected]
            paths = [sim.getObjectAlias(h, 2) for h in handles]
            return handles, paths
    return [], None


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
        self._grasp_sensor = None      # 抓取刚性附着的力传感器句柄（S3）
        self._cup_parent0 = None       # 杯子原父对象（附着前保存，便于恢复）
        self._coll_robot = None        # 净距监控用对象集合（S3 无穿模验证）
        self._coll_chair = None

    # -- 连接与场景核查（场景篇 §1.2 清单的代码化） ---------------------------

    def connect(self, torque_mode=True, engine_dt=None):
        """
        建立连接并完成里程碑 0 的自动核查：
          1. ZMQ 连接（失败 -> CoppeliaSimConnectionError）；
          2. 关节句柄探测（候选路径逐组尝试，§1.2 第 1 项）；
          3. tip/base/杯子/椅子句柄（§1.2 第 2/6/7 项，杯椅缺失仅警告）；
          4. 力矩动态控制模式设置（§1.2 第 3 项——若场景保存为位置/速度
             PID 模式，此处强制切到 sim.jointdynctrl_force，否则实验退化为
             "比较内置 PID"，即总方案 §2.2(i) 的混淆变量）；
          5. 停用 LBR4p 自带演示线程脚本（2026-07 诊断：场景自带
             /LBR4p/Script 会在仿真启动后用 moveToConfig 驱动关节到
             预设位形，与外部力矩控制冲突，必须停用；RG2 夹爪脚本
             只控夹爪自身关节，保留）；
          6. 引擎步长改写（engine_dt 给定时；场景默认 50 ms 对力矩闭环
             过粗，需降到 5 ms 量级，以实际读回值为准）；
          7. 动力学引擎强制 MuJoCo + 关节 armature 回写（2026-07 诊断：
             Bullet 在 7R+RG2 长链大质量比下约束求解失真，零力矩自由
             落体加速度非物理、重力保持力矩仅为真值 1/6，力矩模式不可
             用；MuJoCo 多体链精确。CoppeliaSim 默认 armature=2.0 与名义
             模型电机惯量表不一致，统一写入 LBR4_MOTOR_INERTIA，使引擎
             与控制器名义模型 M_total = M_links + diag(B) 严格一致）；
          8. 机器人↔环境碰撞屏蔽（2026-07 诊断：S1 预抓取位姿下 RG2
             指尖与杯/椅共域是任务几何的必然（LBR4 臂展限制，工具朝下
             时无法悬停于杯口 +0.21 m 夹爪长度之上），引擎接触力高达
             数千 N，启动瞬间冲量把关节踢过限位触发 [abort]；跟踪实验
             中杯/椅只是目标参照物，通过 respondable 全局掩码分组屏蔽
             机器人↔椅子/杯子碰撞，保留椅↔地板、杯↔椅支撑）；
          9. 同步步进 sim.setStepping(True)（§1.2 第 4 项时序契约）。
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

        # 2. 关节句柄探测（候选路径 + 基座子树遍历回退）
        candidates = ([self.joint_names] if self.joint_names
                      else JOINT_PATH_CANDIDATES)
        self.joint_handles, used = probe_joint_handles(self.sim, candidates)
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

        # 5. 停用 LBR4p 自带演示脚本（保留 RG2 夹爪脚本）
        self._disable_demo_scripts()

        # 6. 引擎步长改写（仅停止态可改；失败则警告并沿用实际值）
        if engine_dt is not None:
            try:
                self.sim.setFloatParam(
                    self.sim.floatparam_simulation_time_step, float(engine_dt))
            except Exception as exc:
                print(f"[coppeliasim][warn] 引擎步长改写失败（{exc}），"
                      f"请在 GUI 把仿真 dt 设为 custom；将沿用场景当前步长")

        # 7. 动力学引擎：强制 MuJoCo + 关节 armature = 名义电机惯量表
        self._configure_engine_dynamics()

        # 8. 机器人↔椅子/杯子碰撞屏蔽（respondable 全局掩码分组）
        self._shield_environment_collision()

        # 9. 同步步进 + 引擎步长核查
        self.sim.setStepping(True)
        self.sim_dt = float(self.sim.getSimulationTimeStep())
        if engine_dt is not None and abs(self.sim_dt - engine_dt) > 1e-9:
            print(f"[coppeliasim][warn] 引擎步长实际为 "
                  f"{self.sim_dt * 1e3:.1f} ms（目标 {engine_dt * 1e3:.1f} ms），"
                  f"控制周期以实际值为准")
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

    def _disable_demo_scripts(self):
        """停用基座子树内会驱动手臂关节的自带脚本（LBR4p 演示线程脚本）。
        判据：脚本挂在 LBR4p 模型根上（别名以 /LBR4p/Script 结尾）；
        RG2 子树脚本只控夹爪自身关节，不在停用之列。"""
        if self.base_handle == -1:
            return
        try:
            scripts = self.sim.getObjectsInTree(
                self.base_handle, self.sim.sceneobject_script, 0)
        except Exception:
            return
        for h in scripts:
            alias = self.sim.getObjectAlias(h, 2)
            if "RG2" in alias:
                continue                      # 夹爪控制脚本保留
            disabled = False
            try:                              # 4.6+ 属性 API
                self.sim.setBoolProperty(h, "enabled", False)
                disabled = True
            except Exception:
                try:                          # 旧版参数 API 回退
                    self.sim.setObjectInt32Param(
                        h, self.sim.scriptintparam_enabled, 0)
                    disabled = True
                except Exception:
                    pass
            print(f"[coppeliasim] 自带脚本 {alias} "
                  f"{'已停用' if disabled else '停用失败（请在 GUI 中手动禁用）'}")

    def _configure_engine_dynamics(self):
        """动力学引擎配置（2026-07 诊断固化，仅停止态可改）：

        1) 强制 MuJoCo：场景默认 Bullet 在 7R+RG2 长链、大质量比
           （2.7 kg 连杆 vs 0.045 kg 夹爪件）下约束求解失真：实测零力矩
           自由落体关节加速度非物理（joint1 重力矩≈0 却有 ±60 rad/s²），
           joint2 重力保持力矩仅 ~7 N·m（按质量分布应为 40.4），力矩
           控制完全不可用；MuJoCo 实测保持力矩与名义模型一致。
        2) 关节 armature 回写：CoppeliaSim 默认 2.0 kg·m²，与名义模型
           电机惯量表 LBR4_MOTOR_INERTIA 不一致（腕部 0.15 vs 2.0 差 13
           倍，前馈严重低估 -> 大瞬态 + 零空间漂移撞限位）；统一写入
           名义表，引擎 M_total = M_links + diag(B) 与控制器严格一致。
        3) 基座固定（本次诊断的根因）：.ttt 场景把 /LBR4p 基座 shape
           保存为 dynamic（static=0）且与世界无任何刚性连接（父子层级
           不构成动力学约束）——整台机器人是自由漂浮体，力矩一发
           整机即前倾翻倒（实测：全关节锁死零指令 1 s 内基座位移
           563 mm、翻转 115°），并使单关节平衡力矩失真（joint2
           实测 22 vs 模型 40.4 N·m，四引擎一致；基座 static 化后恢复
           36.5/40.4，残差为 RG2 实验污染）。固定机械臂基座是力矩
           控制实验的标准边界条件，运行时强制 static。
        """
        try:
            from config.lbr4_dynamics import LBR4_MOTOR_INERTIA
        except ImportError:                    # 相对导入回退（脚本直跑）
            from ..config.lbr4_dynamics import LBR4_MOTOR_INERTIA
        try:
            if self.sim.getInt32Param(self.sim.intparam_dynamic_engine) \
                    != self.sim.physics_mujoco:
                self.sim.setInt32Param(self.sim.intparam_dynamic_engine,
                                       self.sim.physics_mujoco)
                print("[coppeliasim] 动力学引擎已切换为 MuJoCo"
                      "（Bullet 在本链上力矩模式失真）")
        except Exception as exc:
            print(f"[coppeliasim][warn] 引擎切换失败（{exc}），"
                  f"请在 GUI 中手动选 MuJoCo")
        try:
            for h, B in zip(self.joint_handles,
                            LBR4_MOTOR_INERTIA[:self.n]):
                self.sim.setEngineFloatParam(
                    self.sim.mujoco_joint_armature, h, float(B))
            print("[coppeliasim] 关节 armature 已写入名义电机惯量表 "
                  f"{np.round(LBR4_MOTOR_INERTIA[:self.n], 3).tolist()}")
        except Exception as exc:
            print(f"[coppeliasim][warn] armature 写入失败（{exc}），"
                  f"引擎侧电机惯量与名义模型可能失配")
        # 3) 基座固定：.ttt 保存为 dynamic 且无刚性连接 -> 整机翻倒
        if self.base_handle != -1:
            try:
                if self.sim.getObjectInt32Param(
                        self.base_handle, self.sim.shapeintparam_static) == 0:
                    self.sim.setObjectInt32Param(
                        self.base_handle, self.sim.shapeintparam_static, 1)
                    print("[coppeliasim] 基座 shape 已固定（static）："
                          ".ttt 保存为 dynamic 自由体，力矩模式下整机会翻倒")
            except Exception as exc:
                print(f"[coppeliasim][warn] 基座固定失败（{exc}），"
                      f"请在 GUI 中取消 LBR4p 基座的 dynamic 属性")

    def _shield_environment_collision(self):
        """屏蔽机器人（含 RG2）与椅子/杯子之间的碰撞响应。

        原理：shapeintparam_respondable_mask 低 8 位 = 局部掩码（同模型
        链内相邻体，保持不动），高 8 位 = 全局掩码（不同模型间，两 shape
        碰撞当且仅当全局掩码按位与非零）。分组：
            机器人子树   全局掩码 -> 0x01
            椅子子树     全局掩码 -> 0x02（杯子是椅子的子对象，同组，
                                        杯↔椅支撑碰撞走局部掩码不受影响）
            地板等其余   保持默认 0xff（与两组都相交 -> 支撑碰撞保留）
        效果：0x01 & 0x02 = 0，机器人与椅/杯不再产生接触力；椅/杯与
        地板 0x02 & 0xff != 0 照常支撑。"""
        def _set_group(root, group_bits, label):
            shapes = self.sim.getObjectsInTree(
                root, self.sim.sceneobject_shape, 0)
            n_set = 0
            for s in shapes:
                try:
                    if not self.sim.getObjectInt32Param(
                            s, self.sim.shapeintparam_respondable):
                        continue
                    mask = self.sim.getObjectInt32Param(
                        s, self.sim.shapeintparam_respondable_mask)
                    self.sim.setObjectInt32Param(
                        s, self.sim.shapeintparam_respondable_mask,
                        (mask & 0x00FF) | (group_bits << 8))
                    n_set += 1
                except Exception:
                    continue
            print(f"[coppeliasim] {label} 全局碰撞掩码 -> "
                  f"0x{group_bits:02X}（{n_set} 个 respondable shape）")

        if self.base_handle != -1:
            _set_group(self.base_handle, 0x01, "机器人子树")
        if self.chair_handle is not None:
            _set_group(self.chair_handle, 0x02, "椅子/杯子子树")
        elif self.cup_handle is not None:
            _set_group(self.cup_handle, 0x02, "杯子")

    # -- 仿真生命周期 -----------------------------------------------------------

    def start(self):
        """启动仿真（若在运行先停止，保证初始状态一致）。
        2026-07 修复：stopSimulation 会重置 ZMQ 客户端的同步步进状态，
        若不在 startSimulation 前重新 setStepping(True)，引擎会实时自由
        运行——力矩未下发时机械臂直接重力塌落，并在数个控制步内
        撞限位触发 [abort]（本次诊断实测的提前终止根源之一）。"""
        try:
            self.sim.stopSimulation()
            time.sleep(0.3)
        except Exception:
            pass
        self.sim.setStepping(True)     # stop 后必须重新声明同步步进
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
        等效 ΔM/Δg 阶跃偏差 -> 定理 3(d) ISS 极限球核验。
        注：仅改写场景层级父子关系，不构成刚性动力学连接；
        S3 抓取实验请用 attach_cup_rigid()（力传感器刚性连接）。"""
        if self.cup_handle is None:
            print("[coppeliasim][warn] 无杯子对象，跳过突加负载注入")
            return False
        self.sim.setObjectParent(self.cup_handle, self.tip_handle, True)
        print("[coppeliasim] 杯子已附着到末端（突加负载扰动生效）")
        return True

    # -- S3 抓取-搬运实验辅助（experiments/run_grasp_circle.py） ------------------

    def move_cup(self, pos_base):
        """把杯子挪到基座系位置 pos_base（仅限 start() 前调用）。

        S3 前提：默认杯位 y=0.625 处内撑式抓取链不可达（IK 残差 43 mm，
        控制器追不可达目标 -> 末端垂坠穿模），挪到 params.CUP_POS_GRASP
        后全链可达（参数节注释中的 58 点扫描证据）。保持杯子原姿态与
        父对象（椅子子树）不变，仅平移。"""
        if self.cup_handle is None:
            print("[coppeliasim][warn] 无杯子对象，跳过挪杯")
            return False
        pos = np.asarray(pos_base, dtype=float).reshape(3)
        self.sim.setObjectPosition(self.cup_handle, self.base_handle,
                                   pos.tolist())
        print(f"[coppeliasim] 杯子已挪至基座系 {np.round(pos, 3).tolist()}")
        return True

    def lock_gripper_fingers(self):
        """锁定 RG2 手指关节（位置伺服钉在当前开度）。

        手指关节若无人驱动会在动力学中自由下垂/摆动，把静态标定的
        杯沿净距余量吃掉（2026-07 noload 审计：静态 6 mm -> 动态 0 mm，
        最近点正是 leftTouch/rightTouch 指尖体）。S3 采用内撑式固定
        开度抓取，手指无需运动 -> 直接位置伺服锁死（start 前调用）。"""
        joints = [h for h in self.sim.getObjectsInTree(
                      self.tip_handle, self.sim.sceneobject_joint, 0)
                  if h not in self.joint_handles]
        locked = 0
        for h in joints:
            try:
                q0 = self.sim.getJointPosition(h)
                self.sim.setObjectInt32Param(
                    h, self.sim.jointintparam_dynctrlmode,
                    self.sim.jointdynctrl_position)
                self.sim.setJointTargetPosition(h, q0)
                self.sim.setJointTargetForce(h, 100.0, True)
                locked += 1
            except Exception:
                continue
        print(f"[coppeliasim] RG2 手指关节已锁定 {locked} 个"
              f"（位置伺服钉在当前开度）")
        return locked

    def _tip_dynamic_shape(self):
        """末端子树内第一个 dynamic shape（RG2 基座体）：力传感器刚性
        附着的物理父体。tip 本身是 dummy，挂在其上不构成动力学连接；
        力传感器必须介于两个 dynamic shape 之间才能传递刚性约束。"""
        parent = self.sim.getObjectParent(self.tip_handle)
        for h in [parent] + list(self.sim.getObjectsInTree(
                parent, self.sim.sceneobject_shape, 0)):
            try:
                if self.sim.getObjectType(h) != self.sim.sceneobject_shape:
                    continue
                if not self.sim.getObjectInt32Param(
                        h, self.sim.shapeintparam_static):
                    return h
            except Exception:
                continue
        return None

    def attach_cup_rigid(self, load_mass=None):
        """S3 抓取模拟：力传感器刚性附着（真实动力学连接）。

        结构：末端 dynamic shape -> forceSensor -> 杯子（dynamic）。
        CoppeliaSim 中 forceSensor 是刚性连接件：杯子进入机器人动力学
        树（质量/惯量真实作用到关节，区别于 setObjectParent 的纯层级
        挂接），且 readForceSensor 可测附着点交互力旋量 =“抓握力”代理。

        load_mass 给定时同时改写杯质量（模拟装水杯，放大负载效应；
        控制器名义模型不含杯 -> 负载 = 模型失配扰动，定理 3(c)/(d)
        证书兜底）。碰撞掩码：杯子保持 0x02 组（与机器人 0x01 不相
        交 -> 指↔杯无接触力，防刚性闭环爆炸；与椅/地板照常碰撞：
        附着后负载下垂阶段杯仍由椅面真实支撑，接触力可被
        read_contact_force_norm 监测，且椅↔杯不会幽灵穿透）。
        运行态调用（静止保持段中点，无速度跳变冲击）。"""
        if self.cup_handle is None:
            print("[coppeliasim][warn] 无杯子对象，跳过刚性附着")
            return False
        anchor = self._tip_dynamic_shape()
        if anchor is None:
            print("[coppeliasim][warn] 末端子树无 dynamic shape，"
                  "回退到层级挂接（无动力学载荷）")
            return self.attach_cup_to_tip()
        self._cup_parent0 = self.sim.getObjectParent(self.cup_handle)
        # 力传感器放在杯心位姿（附着点 = 杯心，测得的力旋量即杯对
        # 机器人的全部反作用：重力 + 惯性力 + 向心项）；
        # intParams=[滤波类型, 样本数, 连续超阈数, 0, 0]，
        # floatParams=[尺寸, 力阈值, 矩阈值, 0, 0]，options=0 关闭断裂阈值
        sensor = self.sim.createForceSensor(
            0, [0, 1, 1, 0, 0], [0.02, 0.0, 0.0, 0.0, 0.0])
        self.sim.setObjectAlias(sensor, "graspSensor")
        self.sim.setObjectPose(
            sensor, self.sim.getObjectPose(self.cup_handle, -1), -1)
        self.sim.setObjectParent(sensor, anchor, True)
        self.sim.setObjectParent(self.cup_handle, sensor, True)
        self._grasp_sensor = sensor
        if load_mass is not None:
            try:
                m0 = self.sim.getShapeMass(self.cup_handle)
                self.sim.setShapeMass(self.cup_handle, float(load_mass))
                print(f"[coppeliasim] 杯质量 {m0:.3f} -> {load_mass:.3f} kg"
                      f"（模拟满杯负载）")
            except Exception as exc:
                print(f"[coppeliasim][warn] 杯质量改写失败（{exc}）")
        # 保留杯↔椅/地板碰撞响应（杯保持 0x02 组不变）：附着后负载
        # 下垂阶段杯仍由椅面真实支撑（无幽灵穿透），提杯后自然脱离；
        # 接触力由 read_contact_force_norm(cup) 记录，是需求 4 的直接
        # 观测量（2026-07 教训：清掉掩码后 0.5 kg 下垂让杯幽灵穿入
        # 椅面，几何审计报零净距）
        print("[coppeliasim] 杯子已经力传感器刚性附着到末端"
              f"（anchor={self.sim.getObjectAlias(anchor, 2)}）")
        return True

    def read_grasp_wrench(self):
        """读取抓取附着点力旋量 (F[3], M[3])（传感器系）；未附着或
        读取失败返回零。这是“抓握力/负载力”的直接测量：静态时
        ≈杯重，圆周运动时叠加向心/切向惯性力。"""
        if self._grasp_sensor is None:
            return np.zeros(3), np.zeros(3)
        try:
            res, F, M = self.sim.readForceSensor(self._grasp_sensor)
            if res > 0:
                return np.asarray(F, dtype=float), np.asarray(M, dtype=float)
        except Exception:
            pass
        return np.zeros(3), np.zeros(3)

    def setup_clearance_monitor(self):
        """建立机器人↔椅子/杯子净距监控集合（sim.checkDistance）。

        碰撞响应被掩码屏蔽后，几何净距是“无穿模”的独立审计量：
        全程净距 > 0 即证明参考轨迹与实际运动都未与环境体相交
        （S3 验收条件；内撑段指尖在杯内空气中，净距仍为正）。"""
        try:
            self._coll_robot = self.sim.createCollection(0)
            self.sim.addItemToCollection(
                self._coll_robot, self.sim.handle_tree, self.base_handle, 0)
            if self.chair_handle is not None:
                self._coll_chair = self.sim.createCollection(0)
                self.sim.addItemToCollection(
                    self._coll_chair, self.sim.handle_tree,
                    self.chair_handle, 0)
                # 杯子单独监控（附着后属机器人树，从椅子集合剔除无意义，
                # 改用 handle 直接测）
            return True
        except Exception as exc:
            print(f"[coppeliasim][warn] 净距监控初始化失败（{exc}）")
            self._coll_robot = None
            return False

    def read_clearances(self, cup_attached=False):
        """读取 (机器人↔椅子, 机器人↔杯子) 最小净距 [m]；
        不可用时返回 np.nan。cup_attached=True 后杯已入机器人树，
        机器人↔杯净距改报 杯↔椅（带载运动中真正需要审计的对）。"""
        d_chair = d_cup = np.nan
        if self._coll_robot is None:
            return d_chair, d_cup
        try:
            if self._coll_chair is not None:
                res = self.sim.checkDistance(
                    self._coll_robot, self._coll_chair, 0.0)
                if res[0]:
                    d_chair = float(res[1][6])
            if self.cup_handle is not None:
                a = self._coll_chair if cup_attached else self._coll_robot
                if a is not None:
                    res = self.sim.checkDistance(self.cup_handle, a, 0.0)
                    if res[0]:
                        d_cup = float(res[1][6])
        except Exception:
            pass
        return d_chair, d_cup

    def read_contact_force_norm(self, obj=None):
        """E7 / 场景篇 §6.3：读取当前步接触合力范数，作为非建模
        接触扰动的能量估计（sim.getContactInfo）。obj 给定时只统计
        涉及该对象的接触（S3：只看杯子，排除椅↔地板支撑力底噪）。"""
        total = 0.0
        target = self.sim.handle_all if obj is None else obj
        try:
            idx = 0
            while True:
                info = self.sim.getContactInfo(
                    self.sim.handle_all, target, idx)
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
