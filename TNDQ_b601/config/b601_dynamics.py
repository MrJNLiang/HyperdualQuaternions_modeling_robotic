"""
B601-DM（reBot Arm，6 自由度）名义刚体动力学模型 —— 递归牛顿-欧拉（RNEA）。

理论依据（与 TNDQ_sim/config/lbr4_dynamics.py 同构，适配 B601）
----------------------------------------------------------------
1. 拉格朗日刚体动力学：
       M(q) qdd + C(q, qd) qd + g(q) = tau + tau_ext
   满足性质 P1（M 有界正定）、P2（Mdot - 2C 斜对称）、P3（参数线性）。
2. 名义计算力矩接口（control/control_law.py::nominal_computed_torque）：
       tau = Mhat qdd_ref + Chat qd + ghat
   qdd_ref 由 TNDQ 几何一致控制律给出；模型失配折算为加速度级扰动
   w_dyn，由 Hinf/ISS 证书兜底。
3. 惯性参数来源：reBot-Isaacsim 官方 URDF
   （urdf/reBot_B601_DM/urdf/reBot_B601_DM.urdf，其 FK 已与 Isaac Sim
   场景实测、Pinocchio 三方对齐，可作黄金参考）。由
   experiments/extract_inertia.py 一次性提取：URDF 惯性 -> q=0 位姿 ->
   DH 连杆系变换（多刚体平行轴合成）；gripper 子树（gripper_link +
   双手指，prismatic q=0 静态位形）并入 DH 连杆 6，手指开合对腕部
   动力学影响 ~1e-3 kg*m，作受控模型失配处理。

与 LBR4 版的差异
----------------
- 惯量张量为完整 3x3（URDF 含非对角项；LBR4 名义表为对角）；
- B601_MOTOR_INERTIA = 0：URDF 无 <dynamics> armature/friction 数据，
  且 Isaac 接口层将关节 drive/armature 清零（力矩直驱），电机折算
  惯量项 B qdd = 0，M_total = M_links；
- 关节限位非对称（params.JOINT_LOWER/JOINT_UPPER），check_joint_limits
  做双向比较（LBR4 对称限位取绝对值）；
- 附 pinocchio_cross_check：URDF 直接建模（buildReducedModel 锁定手指
  关节）与自写 RNEA 随机 (q, v, a) 全量对账。

实现说明
--------
- 运动学约定与 core/kinematics.py 完全一致：标准 DH，关节因子
  A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)。
- RNEA 采用 Siciliano《Robotics: MPC》式 (7.107)-(7.114) 的标准 DH
  形式，重力经 pdd_0 = -g_0 注入前向递推（免单独重力项推导）。
- M(q) 由单位加速度列向量法组装：M[:, k] = RNEA(q, 0, e_k) 且关重力；
  Chat qd + ghat = RNEA(q, qd, 0)；ghat = RNEA(q, 0, 0)。
- mismatch_scale 统一缩放质量/惯量（P3 参数线性 => 失配沿参数方向
  线性传播），模拟"控制器名义模型 != 被控对象真实模型"。

自检（TNDQ_b601 目录下）：
    python3 -m config.b601_dynamics            # [1]-[3]（纯 numpy）
    # [4] Pinocchio 对账需 pinocchio（dq_hinf 环境已装 2.7.0）：
    /home/liang/miniconda3/envs/dq_hinf/bin/python -m config.b601_dynamics
"""

import numpy as np

from config.params import (
    B601_DH_TABLE, JOINT_LOWER, JOINT_UPPER, TAU_MAX)

# ---------------------------------------------------------------------------
# 名义惯性参数表（experiments/extract_inertia.py 从官方 URDF 提取，
# 高精度回填；DH 连杆系 = 标准 DH 帧 i = A_1..A_i 之后，q=0 位形）
#
# 每连杆: 质量 m_i (kg)、质心 c_i (DH 连杆 i 系，3,)、质心系惯量
# 张量 I_i (3x3, kg m^2)。连杆 6 为聚合体（link6 + gripper_link +
# 双手指，合计 0.2852 kg）。整机（不含 base_link）1.9457 kg。
# ---------------------------------------------------------------------------

B601_LINK_MASS = np.array([
    0.084073650, 0.710394385, 0.524941641,
    0.191139211, 0.150004102, 0.285183116,
])

B601_LINK_COM = np.array([
    [-0.020071942, -0.035777821, +0.000522463],
    [-0.131755366, +0.003050430, -0.030929929],
    [-0.118252553, -0.026072649, -0.029460981],
    [+0.017069815, -0.000333648, -0.051523069],
    [+0.000001695, -0.001213486, -0.005049331],
    [-0.000046630, -0.100822656, -0.000000542],
])

B601_LINK_INERTIA = np.array([
    [[+8.539307e-05, -8.187889e-06, +5.360831e-11],
     [-8.187889e-06, +8.038677e-05, +1.615321e-10],
     [+5.360831e-11, +1.615321e-10, +5.088860e-05]],
    [[+2.500266e-04, -1.607163e-06, +2.507611e-09],
     [-1.607163e-06, +1.863220e-03, +4.365537e-07],
     [+2.507611e-09, +4.365537e-07, +1.860077e-03]],
    [[+2.047059e-04, -2.163000e-04, +3.315711e-06],
     [-2.163000e-04, +1.204507e-03, +4.338565e-07],
     [+3.315711e-06, +4.338565e-07, +1.255681e-03]],
    [[+7.181843e-05, -5.192289e-08, -1.123019e-05],
     [-5.192289e-08, +1.011415e-04, -2.218259e-07],
     [-1.123019e-05, -2.218259e-07, +1.129001e-04]],
    [[+5.171870e-05, -9.860032e-09, +7.409944e-10],
     [-9.860032e-09, +5.436569e-05, +2.825543e-06],
     [+7.409944e-10, +2.825543e-06, +6.791966e-05]],
    [[+3.080450e-04, -4.527125e-07, +1.647563e-06],
     [-4.527125e-07, +2.681580e-04, -1.799518e-08],
     [+1.647563e-06, -1.799518e-08, +4.651548e-04]],
])

# 电机转子折算惯量 B_i = n_i^2 J_rotor,i [kg m^2]（关节轴侧）。
# B601-DM URDF 无 <dynamics> armature 数据（已核查），Isaac 接口层将
# 关节 drive/armature 同步清零实现力矩直驱，故名义 B = 0：
# M_total(q) = M_links(q)，不引入附加 Coriolis 项，P1/P2 仍成立。
# 若真机辨识出折算惯量，回填此表并保持接口层 armature 一致即可。
B601_MOTOR_INERTIA = np.zeros(6)

GRAVITY = np.array([0.0, 0.0, -9.81])   # 基座系重力加速度 [m/s^2]

_EZ = np.array([0.0, 0.0, 1.0])         # 标准 DH 关节轴（z_{i-1}）


def _dh_transform(a, alpha, d, theta):
    """标准 DH 齐次变换 A_i = Rz(theta) Tz(d) Tx(a) Rx(alpha)
    （与 core/kinematics.py::tndq_joint_factor_dh 一致）。"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


class B601NominalDynamics:
    """
    B601-DM 名义动力学后端：提供 Mhat(q)、Chat(q,qd)qd、ghat(q)，供名义
    计算力矩接口 tau = Mhat qdd_ref + Chat qd + ghat 与内部力矩级被控
    对象使用。

    参数
    ----
    dh_table       : 与 core/kinematics.TNDQSerialChain 相同格式的 DH 表；
                     None = params.B601_DH_TABLE
    mismatch_scale : 参数失配实验的统一缩放因子（1.0 = 无失配）
    motor_inertia  : 电机转子折算惯量对角元 (n,)；None = 默认表（零）
    mass/com/inertia : 惯性参数注入（experiments/extract_inertia.py
                     交叉对账用提取值实例化，核验提取算法本身）；
                     None = 本模块回填表
    """

    def __init__(self, dh_table=None, mismatch_scale=1.0, motor_inertia=None,
                 mass=None, com=None, inertia=None):
        self.dh = np.asarray(
            B601_DH_TABLE if dh_table is None else dh_table, dtype=float)
        self.n = len(self.dh)
        s = float(mismatch_scale)
        # 质量与惯量按同一因子缩放（P3 参数线性）；质心位置不缩放
        base_m = B601_LINK_MASS if mass is None else np.asarray(mass, float)
        base_c = B601_LINK_COM if com is None else np.asarray(com, float)
        base_I = (B601_LINK_INERTIA if inertia is None
                  else np.asarray(inertia, float))
        self.m = base_m.reshape(self.n).copy() * s
        self.com = base_c.reshape(self.n, 3).copy()
        self.I = base_I.reshape(self.n, 3, 3).copy() * s
        base_B = (B601_MOTOR_INERTIA if motor_inertia is None
                  else np.asarray(motor_inertia, float))
        self.B = base_B.reshape(self.n) * s

    # -- 内部：逐关节变换 -----------------------------------------------------

    def _link_transforms(self, q):
        """返回各关节因子 A_i 的旋转 R_i 与平移 p_i（在系 i-1 下表达）。"""
        Rs, ps = [], []
        for i in range(self.n):
            a, alpha, d, off, _ = self.dh[i]
            A = _dh_transform(a, alpha, d, q[i] + off)
            Rs.append(A[:3, :3])
            ps.append(A[:3, 3])
        return Rs, ps

    # -- RNEA 核心 -------------------------------------------------------------

    def rnea(self, q, q_dot, q_ddot, gravity=True):
        """
        递归牛顿-欧拉逆动力学：tau = RNEA(q, qd, qdd)。

        前向递推（Siciliano 式 7.107-7.111，全部量在连杆 i 系下表达）：
            w_i  = R^T(w_{i-1} + qd_i z0)
            wd_i = R^T(wd_{i-1} + qdd_i z0 + qd_i w_{i-1} x z0)
            pdd_i = R^T pdd_{i-1} + wd_i x r_i + w_i x (w_i x r_i)
        重力经 pdd_0 = -g_0 注入（等效基座向上加速）。

        反向递推（式 7.112-7.114）：
            f_i = R_{i+1} f_{i+1} + m_i pdd_{c,i}
            mu_i = -f_i x (r_i + c_i) + R_{i+1} mu_{i+1}
                   + (R_{i+1} f_{i+1}) x c_i + I_i wd_i + w_i x (I_i w_i)
            tau_i = mu_i^T (R^T z0)          （转动关节，轴 = z_{i-1}）
        """
        q = np.asarray(q, dtype=float)
        q_dot = np.asarray(q_dot, dtype=float)
        q_ddot = np.asarray(q_ddot, dtype=float)
        Rs, ps = self._link_transforms(q)

        w = np.zeros(3)                           # w_0
        wd = np.zeros(3)                          # wd_0
        a = -GRAVITY if gravity else np.zeros(3)  # pdd_0 = -g_0（重力注入）

        w_list, wd_list, ac_list = [], [], []
        for i in range(self.n):
            RT = Rs[i].T
            z = RT @ _EZ                          # 关节轴 z_{i-1} 在系 i 下
            r = RT @ ps[i]                        # o_{i-1}->o_i 在系 i 下
            w_new = RT @ w + q_dot[i] * z
            wd_new = RT @ wd + q_ddot[i] * z + q_dot[i] * np.cross(RT @ w, z)
            a_new = RT @ a + np.cross(wd_new, r) + np.cross(w_new, np.cross(w_new, r))
            # 质心加速度 pdd_{c,i}
            c = self.com[i]
            ac = a_new + np.cross(wd_new, c) + np.cross(w_new, np.cross(w_new, c))
            w, wd, a = w_new, wd_new, a_new
            w_list.append(w)
            wd_list.append(wd)
            ac_list.append(ac)

        tau = np.zeros(self.n)
        f_next = np.zeros(3)
        mu_next = np.zeros(3)
        R_next = np.eye(3)                        # R_{n+1}（末端无外力）
        for i in range(self.n - 1, -1, -1):
            RT = Rs[i].T
            z = RT @ _EZ
            r = RT @ ps[i]
            c = self.com[i]
            F = self.m[i] * ac_list[i]
            f = R_next @ f_next + F
            mu = (-np.cross(f, r + c)
                  + R_next @ mu_next
                  + np.cross(R_next @ f_next, c)
                  + self.I[i] @ wd_list[i]
                  + np.cross(w_list[i], self.I[i] @ w_list[i]))
            tau[i] = mu @ z
            f_next, mu_next, R_next = f, mu, Rs[i]
        # 电机转子折算惯量项 B qdd（常数对角阵，不产生 Coriolis 交叉项；
        # B601 名义 B = 0，真机辨识后回填即自动生效）
        tau = tau + self.B * q_ddot
        return tau

    # -- 动力学量装配 -----------------------------------------------------------

    def gravity_vector(self, q):
        """ghat(q) = RNEA(q, 0, 0)。"""
        zeros = np.zeros(self.n)
        return self.rnea(q, zeros, zeros, gravity=True)

    def coriolis_plus_gravity(self, q, q_dot):
        """Chat(q,qd)qd + ghat(q) = RNEA(q, qd, 0)（计算力矩接口只需此组合项）。"""
        return self.rnea(q, q_dot, np.zeros(self.n), gravity=True)

    def mass_matrix(self, q):
        """Mhat(q)：单位加速度列向量法，M[:,k] = RNEA(q,0,e_k)|_{无重力}。
        性质 P1：对称正定（模块自检中核验）。"""
        zeros = np.zeros(self.n)
        M = np.empty((self.n, self.n))
        for k in range(self.n):
            e = np.zeros(self.n)
            e[k] = 1.0
            M[:, k] = self.rnea(q, zeros, e, gravity=False)
        return 0.5 * (M + M.T)   # 数值对称化

    def computed_torque(self, q, q_dot, qddot_ref):
        """名义计算力矩接口：tau = Mhat qdd_ref + Chat qd + ghat。
        所有对比控制器共用的力矩出口（公平性约束）。"""
        return self.mass_matrix(q) @ np.asarray(qddot_ref, dtype=float) \
            + self.coriolis_plus_gravity(q, q_dot)

    def forward_dynamics(self, q, q_dot, tau):
        """正动力学 qdd = M^{-1}(tau - Cqd - g)：内部力矩级被控对象。"""
        M = self.mass_matrix(q)
        h = self.coriolis_plus_gravity(q, q_dot)
        return np.linalg.solve(M, np.asarray(tau, dtype=float) - h)


# ---------------------------------------------------------------------------
# 安全限幅工具（B601-DM 额定值，config/params.py 中央化）
# ---------------------------------------------------------------------------

def clip_torque(tau):
    """力矩限幅（TAU_MAX 额定 effort）；返回 (限幅后 tau, 是否触发饱和)。"""
    tau = np.asarray(tau, dtype=float)
    lim = TAU_MAX[:tau.shape[0]]
    clipped = np.clip(tau, -lim, lim)
    return clipped, bool(np.any(np.abs(tau) > lim))


def check_joint_limits(q, margin=0.02):
    """关节限位检查（非对称限位，留 margin 弧度余量）；
    返回越限关节索引列表。"""
    q = np.asarray(q, dtype=float)
    lo = JOINT_LOWER[:q.shape[0]] + margin
    hi = JOINT_UPPER[:q.shape[0]] - margin
    return [int(i) for i in np.where((q < lo) | (q > hi))[0]]


# ---------------------------------------------------------------------------
# Pinocchio 交叉对账（延迟 import：核心模块不依赖 pinocchio）
# ---------------------------------------------------------------------------

def pinocchio_cross_check(dyn, n_samples=20, seed=7, tol=1e-5):
    """Pinocchio 交叉对账：URDF 直接建模（锁定手指于 q=0）vs 自写 RNEA。

    q 映射：experiments/verify_dh.py 已证 DH q 与 URDF q 逐关节同号
    同值；手指 prismatic 关节锁定值 0 与参数提取位形一致
    （experiments/extract_inertia.py [3]）。

    阈值说明（tol=1e-5 N*m）：实测残差 ~2e-6 N*m 不是算法误差，而是
    DH 表（闭式精确值，如 alpha = pi/2）与 URDF 文本几何（rpy 6 位
    截断，如 -1.5708，每处偏 3.7e-6 rad）的固有表示差。归因证据链
    （experiments/diag_gravity_urdf.py，绕开 pinocchio 的独立验证）：
      |RNEA - DH 链势能梯度|   ~ 4e-10（RNEA 代数正确）
      |RNEA - URDF 链势能梯度| ~ 1.2e-6（= 链表示差，与 pin 同量级）
    物理量级：um 级姿态表示差，较 TAU_MAX 低 7 个数量级；对账目的
    （抓单位/符号/帧错位等算法级 bug，误差 1e-1~1e1 量级）灵敏度
    仍高 4 个数量级以上。

    返回 True/False；pinocchio 未安装返回 None（打印跳过提示）。
    """
    try:
        import pinocchio as pin
    except ImportError:
        print("    [SKIP] pinocchio 未安装"
              "（dq_hinf 环境：pip install pin==2.7.0）")
        return None
    from config.params import B601_URDF

    model = pin.buildModelFromUrdf(B601_URDF)
    jids = [model.getJointId(nm) for nm in ("gripper_joint1", "gripper_joint2")]
    red = pin.buildReducedModel(model, jids, pin.neutral(model))
    data = red.createData()

    rng = np.random.default_rng(seed)
    lo, hi = 0.5 * JOINT_LOWER, 0.5 * JOINT_UPPER   # 限位中部采样
    max_err = 0.0
    for _ in range(n_samples):
        q = rng.uniform(lo, hi)
        v = rng.uniform(-1.0, 1.0, dyn.n)
        acc = rng.uniform(-1.0, 1.0, dyn.n)
        tau_mine = dyn.rnea(q, v, acc)
        tau_pin = np.asarray(pin.rnea(red, data, q, v, acc))
        max_err = max(max_err, float(np.abs(tau_mine - tau_pin).max()))
    print(f"    reduced 模型 nq={red.nq}（锁定手指），"
          f"{n_samples} 组随机 (q, v, a) 样本")
    print(f"    max |tau_mine - tau_pin| = {max_err:.3e} N*m")
    ok = bool(max_err < tol)
    print(f"    [{'PASS' if ok else 'FAIL'}] 对账阈值 {tol:.0e} N*m")
    return ok


# ---------------------------------------------------------------------------
# 模块自检：python3 -m config.b601_dynamics
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    from config.params import Q_INIT

    dyn = B601NominalDynamics()
    rng = np.random.default_rng(0)
    q = Q_INIT + 0.3 * rng.standard_normal(dyn.n)
    qd = rng.standard_normal(dyn.n)
    qdd = rng.standard_normal(dyn.n)

    # [1] 一致性：RNEA(q,qd,qdd) == M qdd + Cqd + g（装配 vs 直接递推）
    lhs = dyn.rnea(q, qd, qdd)
    rhs = dyn.mass_matrix(q) @ qdd + dyn.coriolis_plus_gravity(q, qd)
    e1 = float(np.max(np.abs(lhs - rhs)))
    print(f"[1] RNEA vs M*qdd + C*qd + g 残差: {e1:.3e}  "
          f"[{'PASS' if e1 < 1e-10 else 'FAIL'}]")

    # [2] 性质 P1：M 对称正定
    M = dyn.mass_matrix(q)
    eig = np.linalg.eigvalsh(M)
    ok2 = bool(eig.min() > 0.0)
    print(f"[2] M 特征值范围: {eig.min():.6e} ... {eig.max():.6e}  "
          f"[{'PASS' if ok2 else 'FAIL'}]")

    # [3] 重力向量 vs 势能数值梯度（独立核验，含符号约定）
    def potential(qv):
        # 逐连杆质心世界高度求势能 U = sum m_i g h_i（DH 基座系，
        # 重力场均匀 => 与基座前缀平移无关）
        T = np.eye(4)
        U = 0.0
        for i in range(dyn.n):
            a_, al, d, off, _ = dyn.dh[i]
            T = T @ _dh_transform(a_, al, d, qv[i] + off)
            pc = T[:3, :3] @ dyn.com[i] + T[:3, 3]
            U += dyn.m[i] * 9.81 * pc[2]
        return U

    h = 1e-6
    g_num = np.array([(potential(q + h * e) - potential(q - h * e)) / (2 * h)
                      for e in np.eye(dyn.n)])
    e3 = float(np.max(np.abs(g_num - dyn.gravity_vector(q))))
    print(f"[3] g(q) RNEA vs 势能梯度 残差: {e3:.3e}  "
          f"[{'PASS' if e3 < 1e-6 else 'FAIL'}]")

    # [4] Pinocchio 交叉对账（URDF 黄金参考）
    print("[4] Pinocchio 交叉对账:")
    pinocchio_cross_check(dyn)
