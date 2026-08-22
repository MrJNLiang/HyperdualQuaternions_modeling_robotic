"""
Isaac Sim 力矩级后端 —— B601-DM 六关节力矩直驱 + 平行夹爪位置 drive。

职责边界（与未来真机后端同签名，experiments/ 脚本零改动迁移）：
    setup()                              创建 SimulationApp 与物理场景
    reset_to(q, gripper_width)           关节状态 teleport（世界复位后）
    get_joint_state() -> (q, qd)         关节角/角速度 [rad, rad/s]
    apply_arm_torques(tau)               关节力矩 [N*m]（限幅在控制律层）
    get_measured_joint_efforts() -> tau  实测关节力矩（诊断）
    set_gripper(width_m)                 指间开度目标（位置 drive）
    get_gripper_width() -> width         实测指间开度
    get_cube_pose() -> (pos, quat)       立方体世界位姿（[w,x,y,z] 四元数）
    get_ee_pose() -> (pos, quat)         gripper_link 世界位姿（USD xform）
    step(render=None)                    一个物理步（DT = 2 ms）
    run_selfcheck()                      FK 对齐 + 重力补偿保持综合自检
    close()                              关闭 SimulationApp

坐标与符号约定：
    - 世界系 = URDF base_link 系（机器人 base_link 原点置于世界原点、
      立于地面 z=0）；DH 链基座 = 世界系平移 B601_BASE_PREFIX；
    - USD 关节角与 URDF/DH 同号同零位（三重证据：USD 限位与 URDF 限位
      逐关节同号、URDF importer 帧语义、isaac_check/smoke_test.py 实测
      FK 对齐）；
    - 力矩与关节角同约定（正力矩产生正角加速度）。

力矩直驱（drive 清零双保险）：
    USD 资产 joint1-6 已带 DriveAPI:angular 且 maxForce = TAU_MAX
    （27/27/27/7/7/7 N*m），但未 authored stiffness/damping。本接口：
      [1] USD 层（world.reset() 前）：RevoluteJoint 的 angular drive
          显式 authored stiffness = damping = 0（articulation 初始化时
          读取）；
      [2] 运行时：set_gains(0, 0)（smoke_test v1-v4 验证运行时清零
          可行）。
    清零后 drive 恒输出 0，set_joint_efforts 的力矩成为唯一关节输入；
    maxForce 保持资产 authored 值，作为物理层力矩兜底限幅。

夹爪：PrismaticJoint 位置 drive（运行时 set_gains 设 GRIPPER_KP /
    GRIPPER_KD，maxForce = 100 N 资产 authored）。set_gripper(width_m)
    语义为指间开度，单指行程 = width/2（USD 限位 [0, 0.0715] m、
    q=0 两指并拢；指厚偏置未计入，抓取余量覆盖）。

运行方式（Isaac 官方运行时，pip 版 isaacsim 6.0）：
    ~/isaacsim/python.sh TNDQ_b601/interfaces/isaac_interface.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    B601_BASE_PREFIX, B601_DH_TABLE, B601_TOOL_ANGLE, CUBE_FRICTION,
    CUBE_MASS, CUBE_POS, CUBE_SIZE, CUBE_YAW, GRIPPER_KD, GRIPPER_KP,
    GRIPPER_OPENING, ISAAC_PHYSICS_DT, ISAAC_RENDER_DT, ISAAC_ROBOT_PRIM,
    ISAAC_ROBOT_USD, JOINT_LOWER, JOINT_UPPER, PEDESTAL_H, PEDESTAL_SIZE,
    Q_INIT,
)


def _quat_rz(angle):
    """绕 z 轴旋转的单位四元数 [w,x,y,z]。"""
    half = 0.5 * float(angle)
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)])


def _quat_to_R(r):
    """单位四元数 [w,x,y,z] -> 旋转矩阵（列 = 本体轴在世界系的像）。"""
    w, x, y, z = r
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _rot_angle(R_a, R_b):
    """两旋转矩阵的测地角 [rad]。"""
    tr = np.trace(R_a.T @ R_b)
    return float(np.arccos(np.clip(0.5 * (tr - 1.0), -1.0, 1.0)))


class IsaacB601Backend:
    """Isaac Sim B601-DM 力矩级后端（生命周期与用法见模块 docstring）。"""

    # 手指 / gripper_link prim（相对 ISAAC_ROBOT_PRIM 的路径，嵌套层级
    # 与 USD 资产 Geometry 子树一致）
    _FINGER_RELS = (
        "/Geometry/base_link/link1/link2/link3/link4/link5/link6"
        "/gripper_link/gripper_left",
        "/Geometry/base_link/link1/link2/link3/link4/link5/link6"
        "/gripper_link/gripper_right",
    )
    _GRIPPER_LINK_REL = (
        "/Geometry/base_link/link1/link2/link3/link4/link5/link6/gripper_link"
    )

    def __init__(self, headless=True, physics_dt=None, render_dt=None,
                 arm_self_collision=True, solver_pos_iter=None,
                 solver_vel_iter=None, probe_size=None):
        # 只存配置；一切 isaacsim/pxr import 延迟到 setup()（SimulationApp
        # 必须先于它们创建）
        self.headless = bool(headless)
        self.physics_dt = ISAAC_PHYSICS_DT if physics_dt is None else float(physics_dt)
        self.render_dt = ISAAC_RENDER_DT if render_dt is None else float(render_dt)
        # articulation 自碰撞开关（USD 资产未 authored，PhysX 默认开）；
        # 诊断/缓解 exp1 伸展构型冻结时可关闭，见 _set_arm_self_collision_usd
        self.arm_self_collision = bool(arm_self_collision)
        # TGS 求解器迭代次数（None = 资产默认）：伸展构型下默认迭代可能
        # 不收敛 -> 表观构型锁（力矩已施加但臂不动），提高可缓解
        self.solver_pos_iter = solver_pos_iter
        self.solver_vel_iter = solver_vel_iter
        # 接触包络探针（诊断用）：边长 probe_size 的微小立方体，
        # 逐点 teleport 到 gripper 附近网格，用位移响应测绘占据空间
        self.probe_size = probe_size
        self.sim_app = None
        self.world = None
        self.articulation = None
        self.cube = None
        self.probe = None
        self.stage = None
        self.arm_idx = None      # joint1..joint6 的 DOF 下标
        self.grip_idx = None     # gripper_joint1/2 的 DOF 下标
        self._grip_upper = None  # 单指行程上限 [m]

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def setup(self):
        """创建 SimulationApp 与物理场景（每进程仅一次）。"""
        if self.world is not None:
            raise RuntimeError("setup() 已执行过")
        if not os.path.isfile(ISAAC_ROBOT_USD):
            raise FileNotFoundError(
                f"B601 USD 资产不存在: {ISAAC_ROBOT_USD}（可用环境变量 "
                f"B601_USD 指定其他路径）")
        try:
            from isaacsim import SimulationApp
        except ImportError as exc:
            raise RuntimeError(
                "未检测到 Isaac Sim Python 环境，请使用官方运行时：\n"
                "    ~/isaacsim/python.sh <script>.py") from exc

        self.sim_app = SimulationApp({"headless": self.headless})

        from isaacsim.core.api import World
        from isaacsim.core.api.materials.physics_material import PhysicsMaterial
        from isaacsim.core.api.objects import (
            DynamicCuboid, FixedCuboid, GroundPlane)
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.stage import (
            add_reference_to_stage, get_current_stage)

        # --- [1] 世界 + 地平面 ---
        self.world = World(stage_units_in_meters=1.0,
                           physics_dt=self.physics_dt,
                           rendering_dt=self.render_dt)
        ground_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/ground_material",
            static_friction=CUBE_FRICTION,
            dynamic_friction=CUBE_FRICTION,
            restitution=0.1,
        )
        self.world.scene.add(GroundPlane(
            prim_path="/World/GroundPlane", name="ground_plane",
            z_position=0.0, physics_material=ground_material))
        self._add_lighting()

        # --- [2] 机器人（引用 reBot-Isaacsim 仓库 USD 资产）---
        # 资产部分贴图引用历史导出路径 ~/reBotArm_control_py/...（headless
        # 物理仿真不受影响；可视化模式可设 REBOT_TEXTURE_SYMLINK=1 后运行
        # reBot-Isaacsim 接收端脚本创建符号链接，此处仅提示不处理）
        add_reference_to_stage(usd_path=ISAAC_ROBOT_USD, prim_path=ISAAC_ROBOT_PRIM)
        self.stage = get_current_stage()
        self._zero_arm_drive_usd()          # 必须在 world.reset() 之前
        self._set_arm_self_collision_usd()  # 同上（物理初始化后不可改）
        self._set_articulation_solver_iters()  # 同上
        self._add_finger_collision_group()

        # --- [3] 抓取立方体 ---
        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_FRICTION,
            dynamic_friction=0.5,
            restitution=0.1,
        )
        # z 加 1 mm 空隙避免与地面共面接触的初始抖动（反馈控制下无影响）
        cube_pos = np.array(CUBE_POS, dtype=float)
        cube_pos[2] += 1e-3
        self.cube = DynamicCuboid(
            prim_path="/World/GraspCube", name="grasp_cube",
            position=cube_pos, orientation=_quat_rz(CUBE_YAW),
            scale=np.array([CUBE_SIZE] * 3), mass=CUBE_MASS,
            physics_material=cube_material)
        self.world.scene.add(self.cube)

        # --- [3b] 支柱（静态，v7 顶抓：立方体置于柱顶）---
        pedestal = FixedCuboid(
            prim_path="/World/Pedestal", name="pedestal",
            position=np.array([CUBE_POS[0], CUBE_POS[1], PEDESTAL_H / 2]),
            size=1.0,
            scale=np.array([PEDESTAL_SIZE, PEDESTAL_SIZE, PEDESTAL_H]),
            physics_material=cube_material)
        self.world.scene.add(pedestal)

        # --- [3c] 接触包络探针（可选，诊断用；初始放远处）---
        if self.probe_size is not None:
            self.probe = DynamicCuboid(
                prim_path="/World/ProbeCube", name="probe_cube",
                position=np.array([1.2, 0.8, 0.3]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
                size=1.0, scale=np.array([self.probe_size] * 3),
                mass=0.005, physics_material=cube_material)
            self.world.scene.add(self.probe)

        # --- [4] articulation 初始化 + 关节映射 + 运行时增益 ---
        # 用 view 层 Articulation（SingleArticulation 白名单不含
        # set_gains / set_joint_position_targets）；view 方法形状 (M, K)，
        # M=1 单臂，读写用 _row / _flat 适配
        self.articulation = Articulation(
            prim_paths_expr=ISAAC_ROBOT_PRIM, name="b601")
        self.world.scene.add(self.articulation)
        self.world.reset()
        self.articulation.initialize()
        self._setup_joint_mapping()
        self._apply_drive_gains()
        print(f"[isaac] 场景就绪：USD={ISAAC_ROBOT_USD}\n"
              f"[isaac]           prim={ISAAC_ROBOT_PRIM}  "
              f"dt={self.physics_dt:.4f}s  立方体={np.round(CUBE_POS, 4).tolist()}")

    def _add_lighting(self):
        """基础灯光（DomeLight + DistantLight，无 Nucleus 依赖）。"""
        from isaacsim.core.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdLux

        stage = get_current_stage()
        if not stage.GetPrimAtPath("/World/DomeLight").IsValid():
            dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
            dome.CreateIntensityAttr(450.0)
        if not stage.GetPrimAtPath("/World/DistantLight").IsValid():
            distant = UsdLux.DistantLight.Define(
                stage, Sdf.Path("/World/DistantLight"))
            distant.CreateIntensityAttr(500.0)
            distant.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 45.0, 0.0))

    def _zero_arm_drive_usd(self):
        """world.reset() 前显式 authored 手臂 drive 增益为 0（双保险之一）。

        只处理 RevoluteJoint（joint1-6）；手指 PrismaticJoint 的位置
        drive 保留，由运行时 set_gains 配置。
        """
        from pxr import UsdPhysics

        n = 0
        for prim in self.stage.Traverse():
            if not prim.IsA(UsdPhysics.RevoluteJoint):
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive is None:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(0.0)
            n += 1
        if n != 6:
            print(f"[isaac] 警告：USD 层清零了 {n} 个 RevoluteJoint drive（预期 6）")

    def _set_arm_self_collision_usd(self):
        """world.reset() 前 author articulation selfCollisionEnabled。

        USD 资产未 authored 该 flag（PhysX 默认 = 自碰撞开）。诊断史：
        物理运行中 CreateAttribute/apply_articulation_settings 均不可行
        （前者 C++ 层静默中止，后者 Isaac 6.0 view 层无此方法），
        必须在物理初始化前用 PhysxArticulationAPI.Apply 设置。
        伸展构型下非相邻 link 对距离仅 ~0.10 m（link4<->link6），
        若自碰撞在极限构型触发，可用 arm_self_collision=False 缓解。
        """
        from pxr import PhysxSchema, UsdPhysics

        if self.arm_self_collision:
            return                       # 保持资产默认（自碰撞开）
        root = None
        for prim in self.stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                root = prim
                break
        if root is None:
            print("[isaac] 警告：未找到 articulation root，自碰撞开关未生效")
            return
        pa = PhysxSchema.PhysxArticulationAPI.Apply(root)
        pa.GetEnabledSelfCollisionsAttr().Set(False)   # physxArticulation:
        #                                              #   enabledSelfCollisions
        print(f"[isaac] 已关闭 articulation 自碰撞（root={root.GetPath()}）")

    def _set_articulation_solver_iters(self):
        """world.reset() 前 author TGS 求解器迭代次数（可选）。

        背景：exp1 在伸展构型出现"力矩已施加、非限位、无接触、自碰撞
        开关无差异，但臂完全不动"的表观构型锁——TGS 默认迭代在
        病态（近全伸/惯量病态条件数）构型不收敛的典型症状。
        """
        from pxr import PhysxSchema, UsdPhysics

        if self.solver_pos_iter is None and self.solver_vel_iter is None:
            return
        root = None
        for prim in self.stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                root = prim
                break
        if root is None:
            print("[isaac] 警告：未找到 articulation root，求解器迭代未设置")
            return
        pa = PhysxSchema.PhysxArticulationAPI.Apply(root)
        if self.solver_pos_iter is not None:
            pa.GetSolverPositionIterationCountAttr().Set(
                int(self.solver_pos_iter))
        if self.solver_vel_iter is not None:
            pa.GetSolverVelocityIterationCountAttr().Set(
                int(self.solver_vel_iter))
        print(f"[isaac] 求解器迭代：pos={self.solver_pos_iter} "
              f"vel={self.solver_vel_iter}")

    def _add_finger_collision_group(self):
        """左右手指互不碰撞（CollisionGroup colliders + filteredGroups 自引用，
        与 reBot-Isaacsim 接收端同方案；手臂各 link 间碰撞不受影响）。"""
        from pxr import Sdf, UsdPhysics

        group_path = Sdf.Path("/World/GripperFingerNoCollideGroup")
        group = UsdPhysics.CollisionGroup.Define(self.stage, group_path)
        colliders = group.GetCollidersCollectionAPI()
        if colliders is None:
            colliders = UsdPhysics.CollectionAPI.Apply(group.GetPrim(), "colliders")
        includes = colliders.GetIncludesRel()
        if includes is None:
            includes = colliders.CreateIncludesRel()
        for rel in self._FINGER_RELS:
            path = ISAAC_ROBOT_PRIM + rel
            if self.stage.GetPrimAtPath(path).IsValid():
                includes.AddTarget(Sdf.Path(path))
        filtered = group.GetFilteredGroupsRel()
        if filtered is None:
            filtered = group.CreateFilteredGroupsRel()
        filtered.AddTarget(group_path)

    def _setup_joint_mapping(self):
        """DOF 名单 -> joint1..6 / gripper_joint1/2 下标；限位对账（USD）。"""
        from pxr import UsdPhysics

        names = list(self.articulation.dof_names)
        try:
            self.arm_idx = np.array(
                [names.index(f"joint{i}") for i in range(1, 7)], dtype=int)
            self.grip_idx = np.array(
                [names.index(f"gripper_joint{i}") for i in (1, 2)], dtype=int)
        except ValueError as exc:
            raise RuntimeError(f"B601 DOF 名单异常: {names}") from exc
        # 限位直接读 USD 关节 prim（Articulation view 无 dof_properties；
        # RevoluteJoint 限位 authored 为度，PrismaticJoint 为米）
        arm_lo, arm_hi, grip_upper = [], [], []
        for prim in self.stage.Traverse():
            if prim.GetName().startswith("joint") and prim.IsA(UsdPhysics.RevoluteJoint):
                lo = np.deg2rad(prim.GetAttribute("physics:lowerLimit").Get())
                hi = np.deg2rad(prim.GetAttribute("physics:upperLimit").Get())
                arm_lo.append(lo)
                arm_hi.append(hi)
            elif (prim.GetName().startswith("gripper_joint")
                  and prim.IsA(UsdPhysics.PrismaticJoint)):
                grip_upper.append(prim.GetAttribute("physics:upperLimit").Get())
        self._grip_upper = float(min(grip_upper))
        print(f"[isaac] DOF={names}  手指行程上限={self._grip_upper:.4f} m")
        if len(arm_lo) == 6:
            err = float(np.max(np.abs(np.array(arm_lo) - JOINT_LOWER)
                               + np.abs(np.array(arm_hi) - JOINT_UPPER)))
            if err > 0.05:
                print(f"[isaac] 警告：USD 关节限位与 URDF 差异 {err:.3f} rad")
        else:
            print(f"[isaac] 警告：USD 层找到 {len(arm_lo)} 个 RevoluteJoint（预期 6）")

    def _apply_drive_gains(self):
        """运行时增益（双保险之二）：手臂 0/0 力矩直驱，手指位置 drive。"""
        self.articulation.set_gains(
            np.zeros((1, 6)), np.zeros((1, 6)), joint_indices=self.arm_idx)
        self.articulation.set_gains(
            np.full((1, len(self.grip_idx)), GRIPPER_KP),
            np.full((1, len(self.grip_idx)), GRIPPER_KD),
            joint_indices=self.grip_idx)

    @staticmethod
    def _row(vec):
        """一维关节向量 -> view 层写入形状 (1, K)。"""
        return np.asarray(vec, dtype=float).reshape(1, -1)

    @staticmethod
    def _flat(values):
        """view 层返回值 (M, K) -> 一维 (num_dof,)。"""
        return np.asarray(values, dtype=float).reshape(-1)

    def _require_setup(self):
        if self.articulation is None:
            raise RuntimeError("先调用 setup()")

    # ------------------------------------------------------------------
    # 状态读写（控制循环接口）
    # ------------------------------------------------------------------

    def reset_to(self, q_init, gripper_width=None):
        """teleport 关节状态（不重建世界）。

        手臂 drive 已清零，reset 后无支撑——调用方须在首个 step 前施加
        力矩（实验脚本标准模式：读状态 -> 控制律 -> 施力矩 -> step）。
        """
        self._require_setup()
        q_init = np.asarray(q_init, dtype=float).reshape(-1)
        if q_init.shape != (6,):
            raise ValueError(f"q_init 维度 {q_init.shape} != (6,)")
        self.articulation.set_joint_positions(
            self._row(q_init), joint_indices=self.arm_idx)
        self.articulation.set_joint_velocities(
            self._row(np.zeros(6)), joint_indices=self.arm_idx)
        width = GRIPPER_OPENING if gripper_width is None else gripper_width
        stroke = float(np.clip(0.5 * float(width), 0.0, self._grip_upper))
        self.articulation.set_joint_positions(
            self._row(np.full(len(self.grip_idx), stroke)),
            joint_indices=self.grip_idx)
        self.articulation.set_joint_velocities(
            self._row(np.zeros(len(self.grip_idx))), joint_indices=self.grip_idx)
        self.set_gripper(width)

    def get_joint_state(self):
        """(q, qd)：关节角/角速度 [rad, rad/s]，joint1..joint6 顺序。"""
        self._require_setup()
        q = self._flat(self.articulation.get_joint_positions())[self.arm_idx]
        qd = self._flat(self.articulation.get_joint_velocities())[self.arm_idx]
        return q.copy(), qd.copy()

    def get_measured_joint_efforts(self):
        """实测关节力矩 [N*m]（含 drive/接触贡献，诊断用）。"""
        self._require_setup()
        m = self._flat(self.articulation.get_measured_joint_efforts())
        return m[self.arm_idx].copy()

    def apply_arm_torques(self, tau):
        """施加关节力矩 [N*m]（clip 由控制律层的治理器负责）。"""
        self._require_setup()
        tau = np.asarray(tau, dtype=float).reshape(-1)
        if tau.shape != (6,):
            raise ValueError(f"tau 维度 {tau.shape} != (6,)")
        self.articulation.set_joint_efforts(
            self._row(tau), joint_indices=self.arm_idx)

    def set_gripper(self, width_m):
        """指间开度目标 [m]（单指行程 = width/2，clip 到 USD 限位）。"""
        self._require_setup()
        stroke = float(np.clip(0.5 * float(width_m), 0.0, self._grip_upper))
        self.articulation.set_joint_position_targets(
            self._row(np.full(len(self.grip_idx), stroke)),
            joint_indices=self.grip_idx)

    def get_gripper_width(self):
        """实测指间开度 [m]（2*平均单指行程的一阶近似）。"""
        self._require_setup()
        q = self._flat(self.articulation.get_joint_positions())[self.grip_idx]
        return 2.0 * float(np.mean(q))

    def get_cube_pose(self):
        """立方体世界位姿 (pos[3], quat[w,x,y,z])。"""
        self._require_setup()
        pos, quat = self.cube.get_world_pose()
        return (np.asarray(pos, dtype=float).copy(),
                np.asarray(quat, dtype=float).copy())

    def get_probe_pose(self):
        """探针立方体世界位姿 (pos[3], quat[w,x,y,z])（诊断用）。"""
        self._require_setup()
        if self.probe is None:
            raise RuntimeError("探针未启用（probe_size=None）")
        pos, quat = self.probe.get_world_pose()
        return (np.asarray(pos, dtype=float).copy(),
                np.asarray(quat, dtype=float).copy())

    def get_ee_pose(self):
        """gripper_link 世界位姿 (pos[3], quat[w,x,y,z])——读 USD xform。

        与 DQ FK 对账时注意：DH 链基座 = 世界系平移 B601_BASE_PREFIX。
        """
        from pxr import Usd, UsdGeom

        self._require_setup()
        prim = self.stage.GetPrimAtPath(ISAAC_ROBOT_PRIM + self._GRIPPER_LINK_REL)
        if not prim.IsValid():
            raise RuntimeError("gripper_link prim 未找到")
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        gf_quat = xf.ExtractRotation().GetQuaternion()
        quat = np.array([gf_quat.GetReal(), *gf_quat.GetImaginary()], dtype=float)
        quat /= np.linalg.norm(quat)
        pos = np.array(xf.ExtractTranslation(), dtype=float)
        return pos, quat

    def step(self, render=None):
        """一个物理步（self.physics_dt）；render 默认 headless 时 False。"""
        self._require_setup()
        if render is None:
            render = not self.headless
        self.world.step(render=bool(render))

    def close(self):
        # GUI 模式（headless=False，Script Editor 内运行时 SimulationApp
        # 被桩替换）：不 reset 世界，保留实验结束时的最终画面供观察；
        # headless 模式照旧 reset 清理物理状态
        if self.world is not None and self.headless:
            self.world.reset()
        if self.sim_app is not None:
            self.sim_app.close()
            self.sim_app = None

    def __enter__(self):
        if self.world is None:
            self.setup()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    # ------------------------------------------------------------------
    # 综合自检
    # ------------------------------------------------------------------

    def run_selfcheck(self, q_probe=None, n_steps=250):
        """FK 对齐 + 重力补偿保持 + 力矩通道综合自检。

        流程：teleport q_probe -> 循环施加 tau = g(q)（B601NominalDynamics
        名义重力，与 Isaac 场景同参数源 URDF）-> step -> 读关节漂移与
        gripper_link 世界位姿，对账 DQ FK。

        判据（阈值含义，非物理常数）：
          - FK 位置/姿态对齐 2e-3 m / 2e-2 rad（USD quatf 数值精度 +
            URDF rpy 六位截断 + probe 保持中的微小位移，预期 <<1e-3）；
          - 重力补偿保持 0.5 s 关节漂移 0.05 rad（离散积分误差，预期
            <<0.005；显著超标说明力矩通道未生效或场景重力/构型不一致）。
        返回 result dict（含 ok 布尔）。
        """
        from config.b601_dynamics import B601NominalDynamics
        from core.dq_algebra import (
            dq_mul, dq_rot_z, dq_rotation, dq_translation)
        from core.kinematics import TNDQSerialChain

        self._require_setup()
        if q_probe is None:
            q_probe = np.asarray(Q_INIT, dtype=float)
        q_probe = np.asarray(q_probe, dtype=float).reshape(-1)

        dyn = B601NominalDynamics()
        chain = TNDQSerialChain(B601_DH_TABLE)
        e_dq = dq_rot_z(B601_TOOL_ANGLE)

        # [1] 场景重力
        from pxr import UsdPhysics
        for prim in self.stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                gdir = prim.GetAttribute("physics:gravityDirection").Get()
                gmag = prim.GetAttribute("physics:gravityMagnitude").Get()
                print(f"[selfcheck][1] PhysicsScene 重力: direction={gdir} "
                      f"magnitude={gmag}")

        # [2] drive 增益读回（view 层 get_gains 返回 (kps, kds)）
        try:
            kps, kds = self.articulation.get_gains(joint_indices=self.arm_idx)
            print(f"[selfcheck][2] 手臂 drive 读回: "
                  f"kps={np.round(self._flat(kps), 3).tolist()} "
                  f"kds={np.round(self._flat(kds), 3).tolist()}（应全 0）")
        except Exception as exc:  # noqa: BLE001（API 缺失时仅降级为跳过）
            print(f"[selfcheck][2] drive 读回 API 不可用: {exc}")

        # [3] teleport probe 位形
        self.reset_to(q_probe, GRIPPER_OPENING)
        print(f"[selfcheck][3] probe q={np.round(q_probe, 4).tolist()}")

        # [4] 重力补偿保持
        drift = np.zeros((n_steps, 6))
        for k in range(n_steps):
            q, _ = self.get_joint_state()
            self.apply_arm_torques(dyn.gravity_vector(q))
            self.step()
            drift[k] = q - q_probe
        drift_max = float(np.max(np.abs(drift)))
        q_final, _ = self.get_joint_state()
        tau_m = self.get_measured_joint_efforts()
        g_expect = dyn.gravity_vector(q_probe)
        print(f"[selfcheck][4] 重力补偿保持 {n_steps} 步 "
              f"({n_steps * self.physics_dt:.2f} s)："
              f"漂移 max|dq|={drift_max:.4f} rad")
        print(f"           g(q_probe)      = {np.round(g_expect, 3).tolist()}")
        print(f"           measured_efforts= {np.round(tau_m, 3).tolist()}")

        # [5] FK 对齐（Isaac gripper_link 世界位姿 vs DQ FK + 基座前缀）
        pos_usd, quat_usd = self.get_ee_pose()
        x = dq_mul(chain.fkm(q_probe), e_dq)
        p_expect = np.asarray(dq_translation(x), dtype=float) + B601_BASE_PREFIX
        r_expect = np.asarray(dq_rotation(x), dtype=float)
        dp = float(np.linalg.norm(pos_usd - p_expect))
        dth = _rot_angle(_quat_to_R(quat_usd), _quat_to_R(r_expect))
        print(f"[selfcheck][5] FK 对齐: dp={dp:.2e} m  dtheta={dth:.2e} rad")
        print(f"           Isaac  pos={np.round(pos_usd, 5).tolist()}")
        print(f"           DQ-FK  pos={np.round(p_expect, 5).tolist()}")

        # [6] 立方体位姿
        cube_pos, cube_quat = self.get_cube_pose()
        print(f"[selfcheck][6] 立方体 pos={np.round(cube_pos, 4).tolist()} "
              f"quat={np.round(cube_quat, 4).tolist()}")

        result = {
            "fk_pos_err": dp,
            "fk_ang_err": dth,
            "drift_max": drift_max,
            "q_final": q_final,
            "cube_pos": cube_pos,
            "cube_quat": cube_quat,
        }
        result["ok"] = (dp < 2e-3 and dth < 2e-2 and drift_max < 0.05)
        return result


if __name__ == "__main__":
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    ok = False
    try:
        result = backend.run_selfcheck()
        ok = bool(result["ok"])
    except Exception:  # noqa: BLE001（自检脚本：任何异常都转化为 FAIL）
        import traceback
        traceback.print_exc()
    finally:
        # 总结行必须在 close() 之前：SimulationApp teardown 后 print 会被丢弃
        print("=== ISAAC INTERFACE SELFCHECK {} ===".format(
            "PASS" if ok else "FAIL"), flush=True)
        backend.close()
