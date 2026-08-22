"""
Pinocchio 关节空间轨迹生成器（v13，替代笛卡尔路标插值）。

设计定位（用户 v13 要求）：
    原方案（traj_kinematic / trajectory_generator）在任务空间逐段
    quintic/余弦插值，每个路标强制零速零加速 —— 全程 5 次完整启停、
    运动生硬，且规划层完全不感知关节空间与动力学。本模块改用
    Pinocchio 做完整的关节空间规划：
      [1] 全位姿 IK（阻尼最小二乘 + SE(3) 对数误差，pin.log6）逐路标
          求解，上一路标解热启动下一路标，保证关节路径连续、分支一致；
      [2] 关节构型分段三次 Hermite 样条插值：中间路标保留非零通过
          速度（无停走），抓取静置路标两侧零速（dwell 语义），C1 连续；
      [3] 动力学可行性校核（pin.rnea 名义力矩 vs TAU_MAX、关节速度
          vs QDOT_MAX）：超限则等比拉长全段时长并重算样条；
      [4] 输出 FK 与解析一/二阶导数（pin.forwardKinematics + 帧速度/
          经典加速度），在接口边界组装为控制律所需的 x_breve_d / xi_d /
          xi_dot_d（与 traj_kinematic.KinematicTrajectoryAdapter 同构）。

    约束（用户指定）：只替换路径规划模块；TNDQ 反馈控制律、误差系统、
    控制器架构零改动 —— 本模块输出字段与既有两路线完全同构。

建模约定：
    - URDF 直接建模（config.params.B601_URDF），手指 prismatic 关节
      锁定（与 config.b601_dynamics.pinocchio_cross_check 同口径）；
    - URDF base_link 系 = 世界系；期望位置进 TNDQ 链前经 w2dh（减
      B601_BASE_PREFIX），与另两条轨迹路线一致；
    - 关节角与 URDF/USD/DH 同号同零位（verify_dh.py 已证）；
    - ★帧对齐（随机 30 构型实证，max|dp|=2.2e-6 m / max dθ=1.5e-5
      rad）：pin gripper_link FK == B601TCPChain FK（DH FK * 尾变换
      E=Rz(B601_TOOL_ANGLE)）+ B601_BASE_PREFIX 平移。故任务姿态
      （R_TOOL_QUAT 等，gripper_link 物理帧约定）与 pin IK/FK 帧天然
      一致，无需任何 E 补偿；位置进控制链表示时减 B601_BASE_PREFIX
      （与 w2dh 同口径）。
      （v13 首测陷阱：误判差 90° 并引入 _E_R 补偿反而使全路标不收敛；
      真因是任务姿态通道取帧，修正后直接对齐。）

运行环境：
    - dq_hinf conda 环境（pin 2.7.0）与 Isaac Sim 官方运行时均可；
    - Isaac 运行时内 pinocchio 经 pip_prebundle 安装（cmeel 布局），
      PYTHONPATH 目录不处理 .pth，需先 import cmeel_pth 登记前缀 ——
      _import_pinocchio() 自动处理。

自检：python3 -m simdata.traj_pinocchio（需 pinocchio 与 TNDQ_b601 根
在 sys.path；离线无 Isaac 依赖）。
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _import_pinocchio():
    """导入 pinocchio（含 Isaac pip_prebundle 的 cmeel 前缀引导）。"""
    try:
        import pinocchio as pin
        return pin
    except ImportError:
        try:
            import cmeel_pth  # noqa: F401  Isaac cmeel 布局 .pth 引导
            import pinocchio as pin
            return pin
        except ImportError as exc:
            raise ImportError(
                "pinocchio 未安装。Isaac 运行时：~/isaacsim/python.sh -m "
                "pip install pin；conda 环境：pip install pin==2.7.0") from exc


pin = _import_pinocchio()

# 控制律接口依赖（v14 实时化重构：从 evaluate() 提升到模块级，
# 免去每控制步的 import 机器开销）
from core.dq_algebra import dq_vec6
from core.tndq_algebra import twist_from_tndq, twist_dot_from_tndq
from simdata.trajectory_generator import _pose_tndq_from_rp_derivatives


# ---------------------------------------------------------------------------
# 四元数工具（[w,x,y,z]，独立实现，与 traj_kinematic 同约定）
# ---------------------------------------------------------------------------

def _qmul(a, b):
    """Hamilton 四元数乘法 [w,x,y,z]。

    v13 首版 y 分量末项误写为 ax*bx（应为 az*bx），导致 r_dot/r_ddot
    姿态导数带 3e-3 rad/s 级偏差，由自检角速度差分对账定位。
    """
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _quat_from_R(R):
    """旋转矩阵 -> 单位四元数 [w,x,y,z]（Shepperd 数值稳定分支）。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        q = np.array([0.25 * s,
                      (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s,
                      (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = np.array([(R[2, 1] - R[1, 2]) / s,
                      0.25 * s,
                      (R[0, 1] + R[1, 0]) / s,
                      (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = np.array([(R[0, 2] - R[2, 0]) / s,
                      (R[0, 1] + R[1, 0]) / s,
                      0.25 * s,
                      (R[1, 2] + R[2, 1]) / s])
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = np.array([(R[1, 0] - R[0, 1]) / s,
                      (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s,
                      0.25 * s])
    return q / np.linalg.norm(q)


def quat_geodesic_angle(r_a, r_b):
    """两单位四元数的测地角 [rad]（自检用；输入先归一化）。"""
    a = np.asarray(r_a, dtype=float) / np.linalg.norm(r_a)
    b = np.asarray(r_b, dtype=float) / np.linalg.norm(r_b)
    return 2.0 * np.arccos(np.clip(min(1.0, abs(float(np.dot(a, b)))),
                                     -1.0, 1.0))


# ---------------------------------------------------------------------------
# B601 Pinocchio 模型：reduced 模型 + IK + FK 导数
# ---------------------------------------------------------------------------

class B601PinModel:
    """B601 reduced 模型（手指锁定）+ 全位姿 IK + FK 位姿导数出口。"""

    IK_MARGIN = np.deg2rad(5.0)   # IK 内限位余量（与 ik_lib 同口径）

    def __init__(self):
        from config.params import B601_URDF, JOINT_LOWER, JOINT_UPPER

        model_full = pin.buildModelFromUrdf(B601_URDF)
        jids = [model_full.getJointId(nm)
                for nm in ("gripper_joint1", "gripper_joint2")]
        self.model = pin.buildReducedModel(
            model_full, jids, pin.neutral(model_full))
        if self.model.nq != 6:
            raise RuntimeError(
                f"B601 reduced 模型 nq={self.model.nq} != 6（URDF 关节"
                f"名单异常：{[self.model.names[i] for i in range(self.model.njoints)]}）")
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId("gripper_link")
        self.lo = np.asarray(JOINT_LOWER, dtype=float) + self.IK_MARGIN
        self.hi = np.asarray(JOINT_UPPER, dtype=float) - self.IK_MARGIN
        # FK 对账保险：pin gripper_link vs DH 链 FK*E（B601TCPChain 同约定，
        # 世界/base_link 系；verify_dh.py 表示差阈值）——帧约定错位时
        # 此处即 FAIL，不让错误进入 IK/轨迹。
        from config.params import (B601_BASE_PREFIX, B601_DH_TABLE,
                                   B601_TOOL_ANGLE, Q_INIT)
        from core.dq_algebra import dq_rot_z, dq_rotation, dq_translation
        from core.kinematics import TNDQSerialChain
        from core.tndq_algebra import TNDQ

        chain = TNDQSerialChain(B601_DH_TABLE)
        e_bar = TNDQ.from_constant(dq_rot_z(B601_TOOL_ANGLE))
        x_ref = (chain.fk_tndq(Q_INIT) * e_bar).to_dq()
        p_ref = np.asarray(dq_translation(x_ref), dtype=float) \
            + np.asarray(B601_BASE_PREFIX, dtype=float)
        r_ref = np.asarray(dq_rotation(x_ref), dtype=float)
        oMf = self.fk_se3(Q_INIT)
        dp = float(np.linalg.norm(oMf.translation - p_ref))
        dth = float(np.arccos(np.clip(
            0.5 * (np.trace(oMf.rotation.T @ _quat_to_R(r_ref)) - 1.0),
            -1.0, 1.0)))
        if dp > 5e-3 or dth > 5e-3:
            raise RuntimeError(
                f"Pinocchio FK 与控制链 FK 不对齐：dp={dp:.2e} m、"
                f"dtheta={dth:.2e} rad（帧/零位约定错位）")

    def fk_se3(self, q):
        """gripper_link 位姿 SE3（URDF base_link 系）。"""
        pin.framesForwardKinematics(self.model, self.data,
                                    np.asarray(q, dtype=float))
        return self.data.oMf[self.frame_id]

    def solve_ik(self, oM_des, q_guess, it_max=5000, eps=1e-8):
        """阻尼最小二乘全位姿 IK（SE3 对数误差，LOCAL 配套雅可比）。

        返回 (q, 残差范数, 是否收敛)；迭代内投影回限位盒
        [JOINT_LOWER/UPPER ± IK_MARGIN]。
        """
        q = np.clip(np.asarray(q_guess, dtype=float), self.lo, self.hi)
        damp = 1e-6
        resid = np.inf
        for _ in range(it_max):
            oMf = self.fk_se3(q)
            err = pin.log6(oMf.actInv(oM_des)).vector
            resid = float(np.linalg.norm(err))
            if resid < eps:
                return q, resid, True
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.frame_id, pin.LOCAL)
            dq = J.T @ np.linalg.solve(J @ J.T + damp * np.eye(6), err)
            n = float(np.linalg.norm(dq))
            if n > 0.5:                      # 步长限幅（大误差线性化保护）
                dq *= 0.5 / n
            q = np.clip(q + dq, self.lo, self.hi)
        return q, resid, False

    def solve_ik_robust(self, oM_des, q_guess, n_random=48, n_perturb=16,
                        seed=0):
        """热启动 IK + 扰动/随机多初值回退（大位姿跳变路标防局部极小）。

        先试热启动；不收敛则候选初值族 = 热启动扰动（±0.8 rad）+
        限位盒内均匀随机（固定种子，确定性），逐一起点求解（单起点
        迭代限 it_max=2500，防单点耗尽），收敛解中取限位余量最大者
        （与 ik_lib.solve_ik_multi 同策略）。实测：举高路标（大位姿
        跳变）均匀随机吸引盆小，需 ~50+ 初值；后续路标热启动即收敛。
        返回 (q, resid, ok)；全部失败时 ok=False。
        """
        q, resid, ok = self.solve_ik(oM_des, q_guess)
        if ok:
            return q, resid, ok
        rng = np.random.default_rng(seed)
        cands = [np.clip(q_guess + rng.uniform(-0.8, 0.8, 6),
                         self.lo, self.hi) for _ in range(n_perturb)]
        cands += [rng.uniform(self.lo, self.hi) for _ in range(n_random)]
        best, best_margin, best_resid = None, -np.inf, resid
        for q0 in cands:
            q_i, resid_i, ok_i = self.solve_ik(oM_des, q0, it_max=2500)
            if not ok_i:
                continue
            margin = float(np.min(np.minimum(q_i - self.lo,
                                             self.hi - q_i)))
            if margin > best_margin:
                best, best_margin, best_resid = q_i, margin, resid_i
        if best is None:
            return q, resid, False
        return best, best_resid, True


# ---------------------------------------------------------------------------
# 关节空间分段三次 Hermite 样条（C1，中间路标非零通过速度）
# ---------------------------------------------------------------------------

class JointHermiteSpline:
    """关节空间分段三次 Hermite 样条 q(t)，解析 (q, qd, qdd)。

    t_knots 递增（可含重复节点 = 零长 dwell 段，求值时跳过）。
    t < t0 取首节点（v=0）；t >= t_end 取末节点（末速度构造为 0）。
    """

    def __init__(self, t_knots, Q, V):
        self.t = np.asarray(t_knots, dtype=float)
        self.Q = np.asarray(Q, dtype=float)
        self.V = np.asarray(V, dtype=float)
        if not (len(self.t) == len(self.Q) == len(self.V)):
            raise ValueError("t_knots / Q / V 长度不一致")

    def evaluate(self, t):
        """(q, qd, qdd) @ t。"""
        t = float(t)
        if t <= self.t[0]:
            return self.Q[0].copy(), np.zeros(6), np.zeros(6)
        if t >= self.t[-1]:
            return self.Q[-1].copy(), np.zeros(6), np.zeros(6)
        i = int(np.searchsorted(self.t, t, side="right")) - 1
        while self.t[i + 1] <= self.t[i]:      # 零长段（重复 dwell 节点）
            i += 1
        h = self.t[i + 1] - self.t[i]
        u = (t - self.t[i]) / h
        u2, u3 = u * u, u * u * u
        H00 = 2 * u3 - 3 * u2 + 1
        H10 = u3 - 2 * u2 + u
        H01 = -2 * u3 + 3 * u2
        H11 = u3 - u2
        dH00 = 6 * u2 - 6 * u
        dH10 = 3 * u2 - 4 * u + 1
        dH01 = -6 * u2 + 6 * u
        dH11 = 3 * u2 - 2 * u
        ddH00 = 12 * u - 6
        ddH10 = 6 * u - 4
        ddH01 = -12 * u + 6
        ddH11 = 6 * u - 2
        q = (H00 * self.Q[i] + H10 * h * self.V[i]
             + H01 * self.Q[i + 1] + H11 * h * self.V[i + 1])
        qd = (dH00 * self.Q[i] + dH10 * h * self.V[i]
              + dH01 * self.Q[i + 1] + dH11 * h * self.V[i + 1]) / h
        qdd = (ddH00 * self.Q[i] + ddH10 * h * self.V[i]
               + ddH01 * self.Q[i + 1] + ddH11 * h * self.V[i + 1]) / h ** 2
        return q, qd, qdd


# ---------------------------------------------------------------------------
# goto 轨迹：IK 链 -> 关节样条 -> 动力学/速度校核 -> 控制律接口
# ---------------------------------------------------------------------------

class PinocchioGotoTrajectory:
    """Pinocchio 关节空间 goto 轨迹（run_lib 构建器消费）。

    legs: [(p_world, r_quat, duration, dwell), ...]（世界系 gripper 原点
    + 单位四元数姿态，与 run_lib 另两构建器同语义）。q_start 为起始关节
    构型（Q_INIT）。构建流程：IK 链 -> Hermite 样条 -> 力矩/速度校核
    （必要时等比拉长时长）。t_total / t_grasp_arrival 为校核后的实际时序。
    """

    TORQUE_FRAC = 0.7          # 名义力矩预算比例（留负载/残差余量）
    QDOT_FRAC = 0.9            # 额定速度预算比例
    LIMIT_MIN_MARGIN = np.deg2rad(2.0)   # 样条全程最小限位余量下限

    def __init__(self, q_start, legs, verbose=True):
        from config.params import B601_BASE_PREFIX, QDOT_MAX, TAU_MAX

        self.pin_model = B601PinModel()
        self.base_prefix = np.asarray(B601_BASE_PREFIX, dtype=float)
        self.tau_budget = self.TORQUE_FRAC * np.asarray(TAU_MAX, dtype=float)
        self.qdot_budget = self.QDOT_FRAC * np.asarray(QDOT_MAX, dtype=float)

        # ---- [1] IK 链：逐路标全位姿 IK，前解热启动 ----
        q_knots = [np.asarray(q_start, dtype=float).copy()]
        q_guess = q_knots[0]
        self.ik_resid = []
        for k, (p_w, r_q, _T, _dw) in enumerate(legs):
            # 任务姿态 = gripper_link 物理帧（与 pin FK 帧直接一致，无补偿）
            R_pin = np.asarray(_quat_to_R(np.asarray(r_q, dtype=float)),
                               dtype=float)
            oM_des = pin.SE3(R_pin, np.asarray(p_w, dtype=float))
            q_sol, resid, ok = self.pin_model.solve_ik_robust(
                oM_des, q_guess, seed=k)
            if not ok:
                raise RuntimeError(
                    f"Pinocchio IK 路标 {k} 未收敛：残差 {resid:.3e}"
                    f"（p={np.round(p_w, 3).tolist()}）")
            q_knots.append(q_sol)
            self.ik_resid.append(resid)
            q_guess = q_sol
            if verbose:
                m = float(np.min(np.minimum(
                    q_sol - self.pin_model.lo,
                    self.pin_model.hi - q_sol))) + self.pin_model.IK_MARGIN
                print(f"[traj-pin] IK 路标{k}: 残差={resid:.2e}  "
                      f"限位余量={np.rad2deg(m):.1f} deg")
        self.q_knots = q_knots

        # ---- [2][3] 样条 + 校核（时长可被拉长）----
        self._legs = [(float(T), float(dw)) for _p, _r, T, dw in legs]
        self.time_scale = 1.0
        for _ in range(3):
            self._build_spline(self.time_scale)
            scale_needed = self._feasibility_ratio()
            if scale_needed <= 1.0 + 1e-3:
                break
            self.time_scale *= min(scale_needed, 2.0)
            if verbose:
                print(f"[traj-pin] 动力学/速度校核：时长拉长 x"
                      f"{self.time_scale:.3f}")
        self._check_joint_limits()

        self.t_total = float(self.spline.t[-1])
        # 抓取路标到达时刻（dwell 起点）：逐腿累加（time_scale 拉长后仍正确）
        t_arr = 0.0
        for k, (T, dw) in enumerate(self._legs):
            t_arr += T * self.time_scale
            if dw > 0.0:
                break
        self.t_grasp_arrival = float(t_arr)
        if verbose:
            qd_max, tau_max = self._sample_extrema()
            print(f"[traj-pin] 就绪：t_total={self.t_total:.2f}s "
                  f"（scale={self.time_scale:.3f}）  max|qd|={qd_max:.3f} "
                  f"rad/s  max|tau|={tau_max:.3f} N*m")

    # ---- 内部构建 ----

    def _build_spline(self, scale):
        """按时间缩放系数构建节点时序 + 通过速度 + Hermite 样条。

        dwell 语义：静置段两端节点（到达点 / 静置终点）通过速度强制
        为 0，其余中间节点用中心差分非零过速（C1 不停走）。
        """
        t_knots, Q = [0.0], [self.q_knots[0]]
        t = 0.0
        zero_v_idx = set()
        for k, (T, dw) in enumerate(self._legs):
            t += T * scale
            t_knots.append(t)
            Q.append(self.q_knots[k + 1])
            if dw > 0.0:                     # dwell：重复节点恒置段
                zero_v_idx.add(len(Q) - 1)   # 到达节点零速
                t += dw * scale
                t_knots.append(t)
                Q.append(self.q_knots[k + 1])
                zero_v_idx.add(len(Q) - 1)   # 静置终点零速
        t_knots = np.array(t_knots)
        Q = np.array(Q)
        n = len(Q)
        V = np.zeros_like(Q)
        for i in range(1, n - 1):
            if i in zero_v_idx:
                continue
            h_p = t_knots[i + 1] - t_knots[i]
            h_m = t_knots[i] - t_knots[i - 1]
            if h_p <= 0.0 or h_m <= 0.0:     # 零长段相邻：零速保护
                continue
            V[i] = 0.5 * ((Q[i + 1] - Q[i]) / h_p
                          + (Q[i] - Q[i - 1]) / h_m)
        self.spline = JointHermiteSpline(t_knots, Q, V)

    def _sample(self, n_pts=2000):
        """均匀采样 (t, q, qd, qdd)。"""
        ts = np.linspace(0.0, self.spline.t[-1], n_pts)
        Q = np.empty((n_pts, 6))
        QD = np.empty_like(Q)
        QDD = np.empty_like(Q)
        for k, t in enumerate(ts):
            Q[k], QD[k], QDD[k] = self.spline.evaluate(t)
        return ts, Q, QD, QDD

    def _sample_extrema(self):
        _ts, _Q, QD, QDD = self._sample()
        model, data = self.pin_model.model, self.pin_model.data
        tau_max = 0.0
        for k in range(len(_ts)):
            tau = np.asarray(pin.rnea(model, data, _Q[k], QD[k], QDD[k]))
            tau_max = max(tau_max, float(np.max(np.abs(tau))))
        return float(np.max(np.abs(QD))), tau_max

    def _feasibility_ratio(self):
        """所需时长放大系数（力矩按平方律、速度按线性律折算）。"""
        _ts, _Q, QD, QDD = self._sample()
        model, data = self.pin_model.model, self.pin_model.data
        r_torque, r_qdot = 1.0, 1.0
        for k in range(len(_ts)):
            tau = np.abs(np.asarray(
                pin.rnea(model, data, _Q[k], QD[k], QDD[k])))
            r_torque = max(r_torque, float(np.max(tau / self.tau_budget)))
            r_qdot = max(r_qdot, float(np.max(np.abs(QD[k])
                                              / self.qdot_budget)))
        return max(np.sqrt(r_torque), r_qdot)

    def _check_joint_limits(self):
        _ts, Q, _QD, _QDD = self._sample()
        from config.params import JOINT_LOWER, JOINT_UPPER
        margin = np.min(np.minimum(
            Q - np.asarray(JOINT_LOWER), np.asarray(JOINT_UPPER) - Q))
        if margin < self.LIMIT_MIN_MARGIN:
            raise RuntimeError(
                f"Pinocchio 样条越限位：最小余量 {np.rad2deg(margin):.2f} "
                f"deg < {np.rad2deg(self.LIMIT_MIN_MARGIN):.2f} deg")

    # ---- 控制律接口（与 KinematicTrajectoryAdapter 同构）----

    def evaluate(self, t):
        """evaluate(t) -> dict（x_bar_d/x_breve_d/x_d/xi_d/xi_dot_d）。

        期望位置为 DH 基座系（减 B601_BASE_PREFIX），姿态为 gripper_link
        物理姿态（与 B601TCPChain 同约定）——run_lib 主循环直接消费。
        """
        q, qd, qdd = self.spline.evaluate(t)
        model, data = self.pin_model.model, self.pin_model.data
        pin.forwardKinematics(model, data, q, qd, qdd)
        # pin 2.7：updateFramePlacements(model, data)；新版改名
        # updateFrameKinematics 并带 ReferenceFrame 参
        try:
            pin.updateFrameKinematics(model, data, pin.LOCAL_WORLD_ALIGNED)
        except (AttributeError, TypeError):
            pin.updateFramePlacements(model, data)
        oMf = data.oMf[self.pin_model.frame_id]
        vf = pin.getFrameVelocity(model, data, self.pin_model.frame_id,
                                  pin.LOCAL_WORLD_ALIGNED)
        af = pin.getFrameClassicalAcceleration(
            model, data, self.pin_model.frame_id, pin.LOCAL_WORLD_ALIGNED)

        # pin gripper_link 帧 = 控制链末端帧（B601TCPChain 约定，已实证）
        p = oMf.translation - self.base_prefix       # DH 基座系（平移不变）
        p_dot = np.asarray(vf.linear, dtype=float)   # 帧原点速度（世界轴）
        p_ddot = np.asarray(af.linear, dtype=float)  # 经典加速度 = 原点二阶导

        r = _quat_from_R(oMf.rotation)
        omega = np.asarray(vf.angular, dtype=float)
        omega_dot = np.asarray(af.angular, dtype=float)
        om = np.r_[0.0, omega]
        r_dot = 0.5 * _qmul(om, r)
        r_ddot = (0.5 * _qmul(np.r_[0.0, omega_dot], r)
                  + 0.25 * _qmul(om, _qmul(om, r)))

        x_bar_d = _pose_tndq_from_rp_derivatives(
            r, r_dot, r_ddot, p, p_dot, p_ddot)
        return {
            "x_bar_d": x_bar_d,
            "x_breve_d": x_bar_d.to_hdq(),
            "x_d": x_bar_d.to_dq(),
            "xi_d": dq_vec6(twist_from_tndq(x_bar_d)),
            "xi_dot_d": dq_vec6(twist_dot_from_tndq(x_bar_d)),
        }


def _quat_to_R(r):
    """单位四元数 [w,x,y,z] -> 旋转矩阵（IK 目标构造用）。"""
    w, x, y, z = r / np.linalg.norm(r)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


# ---------------------------------------------------------------------------
# 离线自检：python3 -m simdata.traj_pinocchio
# ---------------------------------------------------------------------------

def _selfcheck():
    from config.params import (
        B601_BASE_PREFIX, B601_DH_TABLE, GOTO_GRASP_DWELL, GOTO_T_DESC,
        GOTO_T_LIFT, GOTO_T_LIFTLOAD, GOTO_T_ROTDESC, GOTO_T_TRANS,
        GOTO_TOP_Z, GRASP_APPROACH_POS, LIFT_POS, Q_INIT, R_TOOL_QUAT,
        SETPOINT_POS,
    )
    from core.dq_algebra import dq_rotation, dq_translation
    from core.kinematics import TNDQSerialChain

    from config.params import B601_TOOL_ANGLE
    from core.dq_algebra import dq_rot_z
    from core.tndq_algebra import TNDQ

    chain = TNDQSerialChain(B601_DH_TABLE)
    # 与 run_lib.B601TCPChain 同约定：FK * 尾变换 E = gripper_link 姿态
    x_init = (chain.fk_tndq(Q_INIT)
              * TNDQ.from_constant(dq_rot_z(B601_TOOL_ANGLE))).to_dq()
    p0 = np.asarray(dq_translation(x_init), dtype=float) + B601_BASE_PREFIX
    r0 = np.asarray(dq_rotation(x_init), dtype=float)

    legs = [
        (np.array([p0[0], p0[1], GOTO_TOP_Z]), r0, GOTO_T_LIFT, 0.0),
        (np.array([GRASP_APPROACH_POS[0], GRASP_APPROACH_POS[1], GOTO_TOP_Z]),
         r0, GOTO_T_TRANS, 0.0),
        (GRASP_APPROACH_POS, R_TOOL_QUAT, GOTO_T_ROTDESC, 0.0),
        (SETPOINT_POS, R_TOOL_QUAT, GOTO_T_DESC, GOTO_GRASP_DWELL),
        (LIFT_POS, R_TOOL_QUAT, GOTO_T_LIFTLOAD, 0.0),
    ]
    traj = PinocchioGotoTrajectory(Q_INIT, legs)

    # [1] 路标到位精度（IK 解 FK 回放；节点时序含 dwell 重复节点，
    # 逐腿累加到达时刻而非直接索引 spline.t）
    print("[1] 路标到位精度：")
    ok = True
    t_arr = 0.0
    for k, (p_w, r_q, T, dw) in enumerate(legs):
        t_arr += T * traj.time_scale
        des = traj.evaluate(t_arr)
        t_arr += dw * traj.time_scale
        from core.dq_algebra import dq_rotation as _rot, dq_translation as _tr
        p_got = np.asarray(_tr(des["x_d"]), dtype=float) + B601_BASE_PREFIX
        r_got = np.asarray(_rot(des["x_d"]), dtype=float)
        dp = float(np.linalg.norm(p_got - np.asarray(p_w)))
        dth = quat_geodesic_angle(r_got, np.asarray(r_q, dtype=float))
        ok &= dp < 1e-4 and dth < 1e-4
        print(f"    路标{k}: dp={dp:.2e} m  dtheta={dth:.2e} rad")

    # [2] 导数一致性（中心差分 vs 解析；采样限制在样条段内部：
    # 段间 C1 但二阶导折角，跨段差分会污染对账）
    print("[2] 导数一致性（中心差分对账）：")
    rng = np.random.default_rng(0)
    h = 1e-4
    e_p1 = e_p2 = e_r1 = 0.0
    segs = [(ta, tb) for ta, tb in zip(traj.spline.t[:-1], traj.spline.t[1:])
            if tb - ta > 4 * h]
    for _ in range(60):
        ta, tb = segs[rng.integers(len(segs))]
        t = rng.uniform(ta + 2 * h, tb - 2 * h)
        d0, d1, d2 = traj.evaluate(t - h), traj.evaluate(t), traj.evaluate(t + h)
        p_, p_0, p_2 = (np.asarray(dq_translation(d["x_d"]), dtype=float)
                        for d in (d1, d0, d2))
        fd1 = (p_2 - p_0) / (2 * h)
        from core.tndq_algebra import twist_from_tndq
        from core.dq_algebra import dq_vec6
        xi = dq_vec6(twist_from_tndq(d1["x_bar_d"]))
        w_ = xi[:3]
        v_O = xi[3:]                       # xi = [w; v_O]，v_O = pd - w x p
        p_dot_got = v_O + np.cross(w_, p_)
        e_p1 = max(e_p1, float(np.linalg.norm(fd1 - p_dot_got)))
        fd2 = (p_2 - 2 * p_ + p_0) / h ** 2
        # p_ddot 数值差分含 O(h^2) 误差，阈值放宽
        e_p2 = max(e_p2, min(
            float(np.linalg.norm(fd2 - _pddot_from(d1))), 9.9))
        r_0 = np.asarray(dq_rotation(d0["x_d"]), dtype=float)
        r_2 = np.asarray(dq_rotation(d2["x_d"]), dtype=float)
        r_1 = np.asarray(dq_rotation(d1["x_d"]), dtype=float)
        # 姿态：差分角速度 vs xi 角速度通道（ṙ=½[0,ω]r =>
        # ω = 2 vec(ṙ r*)，中心差分 dr = (r2-r0)/(2h)，估计式
        # ω ≈ 2 vec(dr conj(r1))）
        dr = (r_2 - r_0) / (2 * h)
        w_fd = 2.0 * _qmul(dr, _qconj(r_1))[1:]
        e_r1 = max(e_r1, float(np.linalg.norm(w_fd - w_)))
    print(f"    max|pd_fd - pd|     = {e_p1:.2e} m/s   "
          f"[{'PASS' if e_p1 < 1e-3 else 'FAIL'}]")
    print(f"    max|pdd_fd - pdd|   = {e_p2:.2e} m/s2 (数值差分，仅参考)")
    print(f"    max|w_fd - w|       = {e_r1:.2e} rad/s "
          f"[{'PASS' if e_r1 < 1e-3 else 'FAIL'}]")

    # [3] dwell 段恒置（抓取静置）
    t_a = traj.t_grasp_arrival
    d_a, d_b = traj.evaluate(t_a + 0.1), traj.evaluate(t_a + 1.0)
    dp_dw = float(np.linalg.norm(
        np.asarray(dq_translation(d_a["x_d"]), dtype=float)
        - np.asarray(dq_translation(d_b["x_d"]), dtype=float)))
    print(f"[3] dwell 恒置: dp={dp_dw:.2e} m  "
          f"[{'PASS' if dp_dw < 1e-12 else 'FAIL'}]")
    print(f"    t_total={traj.t_total:.3f}s  t_grasp_arrival="
          f"{t_a:.3f}s")
    print("=== TRAJ_PINOCCHIO SELFCHECK {} ===".format(
        "PASS" if ok and e_p1 < 1e-3 and e_r1 < 1e-3 and dp_dw < 1e-12
        else "FAIL"))


def _qconj(a):
    return np.array([a[0], -a[1], -a[2], -a[3]])


def _pddot_from(d):
    """从 xi_dot/xi 反推期望 p_ddot（自检对账用）：
    v_O = pd - w x p  =>  a_O = pdd - wdot x p - w x pd。"""
    from core.tndq_algebra import twist_from_tndq, twist_dot_from_tndq
    from core.dq_algebra import dq_vec6
    xi = dq_vec6(twist_from_tndq(d["x_bar_d"]))
    xi_dot = dq_vec6(twist_dot_from_tndq(d["x_bar_d"]))
    w, v_O = xi[:3], xi[3:]
    wd, a_O = xi_dot[:3], xi_dot[3:]
    from core.dq_algebra import dq_translation
    p = np.asarray(dq_translation(d["x_d"]), dtype=float)
    pd = v_O + np.cross(w, p)
    return a_O + np.cross(wd, p) + np.cross(w, pd)


if __name__ == "__main__":
    _selfcheck()
