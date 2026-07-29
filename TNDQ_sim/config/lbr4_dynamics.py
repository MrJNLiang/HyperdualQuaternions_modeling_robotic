"""
KUKA LBR4+（LWR4+）名义刚体动力学模型 —— 递归牛顿-欧拉算法（RNEA）。

理论依据
--------
1. 拉格朗日刚体动力学（总方案文档 §3.1，式 3.1）：
       M(q) q̈ + C(q, q̇) q̇ + g(q) = τ + τ_ext
   满足性质 P1（M 有界正定）、P2（Ṁ - 2C 斜对称）、P3（参数线性）。
2. 名义计算力矩接口（论文 §2.4 / control/control_law.py::nominal_computed_torque）：
       τ = M̂ q̈_ref + Ĉ q̇ + ĝ
   其中 q̈_ref 由 TNDQ 几何一致控制律（式 5.2）给出；模型失配
   (M - M̂), (C - Ĉ), (g - ĝ) 折算为式 (5.1) 的加速度级扰动 w_dyn，
   由定理 3(c)/(d) 的 H∞/ISS 证书兜底。
3. 惯性参数来源：Gaz–Flacco–De Luca, "Identifying the Dynamic Model Used by
   the KUKA LWR: A Reverse Engineering Approach", ICRA 2014（总方案文献 [11]）。
   [11] 给出的是最小参数集（动力学系数），本文件将其展开为逐连杆
   （质量 / 质心 / 惯性张量）的名义表；**数值为按 [11] 模型量级设定的名义
   占位值**，正式实验前须执行场景篇 §1.2 第 5 项核查：用
   sim.getShapeMass / sim.getShapeInertia 读取 CoppeliaSim 引擎侧参数回填，
   两者之差即受控的模型失配源（总方案 §5.1）。

实现说明
--------
- 运动学约定与 core/kinematics.py 完全一致：标准 DH，关节因子
  A_i = Rz(θ_i) Tz(d_i) Tx(a_i) Rx(α_i)（附录 B.1），连杆坐标系 = A_i 之后的系。
- RNEA 采用 Siciliano《Robotics: MPC》式 (7.107)–(7.114) 的标准 DH 形式，
  重力经 p̈_0 = -g_0 注入前向递推（免单独重力项推导）。
- M(q) 由单位加速度列向量法组装：M[:, k] = RNEA(q, 0, e_k) 且关重力；
  Ĉq̇ + ĝ = RNEA(q, q̇, 0)；ĝ = RNEA(q, 0, 0)。
- E3 参数失配实验（总方案 §5.3）：mismatch_scale 统一缩放质量/惯量，
  模拟"控制器名义模型 ≠ 被控对象真实模型"。
"""

import numpy as np

# ---------------------------------------------------------------------------
# 名义惯性参数表（依据 [11] Gaz–Flacco–De Luca LWR4+ 辨识模型量级）
#
# 每行: [质量 m_i (kg), 质心 c_i 在连杆 i 系下坐标 (3,), 惯性张量对角元 (3,)]
# LWR4+ 整机约 16 kg（不含底座），质量沿链递减；质心近似位于连杆几何中心。
# 【注意】正式对接实验前必须按场景引擎参数回填（场景篇 §1.2 第 5 项）。
# ---------------------------------------------------------------------------

LBR4_LINK_MASS = np.array([2.7, 2.7, 2.7, 2.7, 1.7, 1.6, 0.3])   # kg

LBR4_LINK_COM = np.array([          # 连杆 i 坐标系下的质心位置 [m]
    [0.0,  0.02, -0.17],            # link1: 沿 -z 回指肩部
    [0.0, -0.17,  0.02],            # link2
    [0.0,  0.02, -0.20],            # link3
    [0.0, -0.20,  0.02],            # link4
    [0.0,  0.02, -0.20],            # link5
    [0.0, -0.02,  0.00],            # link6（腕部短连杆）
    [0.0,  0.00, -0.06],            # link7（法兰）
])

LBR4_LINK_INERTIA = np.array([      # 质心系惯性张量对角元 [kg m^2]
    [0.030, 0.030, 0.010],
    [0.030, 0.010, 0.030],
    [0.030, 0.030, 0.010],
    [0.030, 0.010, 0.030],
    [0.015, 0.015, 0.006],
    [0.006, 0.006, 0.003],
    [0.001, 0.001, 0.001],
])

# 电机转子折算惯量 B_i = n_i² J_rotor,i [kg m^2]（关节轴侧）。
# LWR4+ 为谐波减速高传动比关节（n≈100–160），折算惯量远大于腕部
# 连杆惯量——Gaz [11] 辨识模型本身含电机惯量项；若缺此项，末端接触力
# 经 Jᵀ 映射后会在小惯量腕关节产生非物理的数百 rad/s² 瞬时加速度。
# M_total(q) = M_links(q) + diag(B)，不引入附加 Coriolis 项（B 为常数），
# 性质 P1/P2 仍成立。数值为按 [11] 量级设定的名义占位值。
LBR4_MOTOR_INERTIA = np.array([1.20, 1.20, 0.80, 0.80, 0.30, 0.30, 0.15])

# 关节限位（LWR4+ 手册值）：奇数关节 ±170°，偶数关节 ±120°
LBR4_JOINT_LIMITS = np.deg2rad(
    np.array([170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 170.0]))

# 额定关节力矩上限 [N m]（LWR4+ 手册量级；场景篇 §6.3 安全上限，
# 任一控制器触发饱和即记录，饱和处理统一为限幅）
LBR4_TORQUE_LIMITS = np.array([176.0, 176.0, 100.0, 100.0, 100.0, 38.0, 38.0])

GRAVITY = np.array([0.0, 0.0, -9.81])   # 基座系重力加速度 [m/s^2]

_EZ = np.array([0.0, 0.0, 1.0])         # 标准 DH 关节轴（z_{i-1}）


def _dh_transform(a, alpha, d, theta):
    """标准 DH 齐次变换 A_i = Rz(theta) Tz(d) Tx(a) Rx(alpha)（附录 B.1 约定，
    与 core/kinematics.py::tndq_joint_factor_dh 一致）。"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


class LBR4NominalDynamics:
    """
    LBR4+ 名义动力学后端：提供 M̂(q)、Ĉ(q,q̇)q̇、ĝ(q)，供名义计算力矩接口
    τ = M̂ q̈_ref + Ĉ q̇ + ĝ（论文 §2.4）与内部力矩级被控对象使用。

    参数
    ----
    dh_table       : 与 core/kinematics.TNDQSerialChain 相同格式的 DH 表
    mismatch_scale : E3 参数失配实验的统一缩放因子（1.0 = 无失配；
                     例如 1.2 表示控制器名义质量/惯量整体高估 20%）
    """

    def __init__(self, dh_table, mismatch_scale=1.0):
        self.dh = np.asarray(dh_table, dtype=float)
        self.n = len(self.dh)
        s = float(mismatch_scale)
        # 质量与惯量按同一因子缩放（P3 参数线性 ⇒ 失配沿参数方向线性传播）
        self.m = LBR4_LINK_MASS[:self.n] * s
        self.com = LBR4_LINK_COM[:self.n].copy()
        self.I = np.array([np.diag(row) for row in LBR4_LINK_INERTIA[:self.n]]) * s
        # 电机折算惯量（对角阵，同受失配因子缩放）
        self.B = LBR4_MOTOR_INERTIA[:self.n] * s

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
        递归牛顿-欧拉逆动力学：τ = RNEA(q, q̇, q̈)。

        前向递推（Siciliano 式 7.107–7.111，全部量在连杆 i 系下表达）：
            ω_i  = Rᵀ(ω_{i-1} + q̇_i z0)
            ω̇_i  = Rᵀ(ω̇_{i-1} + q̈_i z0 + q̇_i ω_{i-1}×z0)
            p̈_i  = Rᵀ p̈_{i-1} + ω̇_i×r_i + ω_i×(ω_i×r_i)
        重力经 p̈_0 = -g_0 注入（等效基座向上加速）。

        反向递推（式 7.112–7.114）：
            f_i = R_{i+1} f_{i+1} + m_i p̈_{c,i}
            μ_i = -f_i×(r_i+c_i) + R_{i+1}μ_{i+1} + (R_{i+1}f_{i+1})×c_i
                  + I_i ω̇_i + ω_i×(I_i ω_i)
            τ_i = μ_iᵀ (Rᵀ z0)          （转动关节，轴 = z_{i-1}）
        """
        q = np.asarray(q, dtype=float)
        q_dot = np.asarray(q_dot, dtype=float)
        q_ddot = np.asarray(q_ddot, dtype=float)
        Rs, ps = self._link_transforms(q)

        w = np.zeros(3)                          # ω_0
        wd = np.zeros(3)                         # ω̇_0
        a = -GRAVITY if gravity else np.zeros(3)  # p̈_0 = -g_0（重力注入）

        w_list, wd_list, ac_list = [], [], []
        for i in range(self.n):
            RT = Rs[i].T
            z = RT @ _EZ                          # 关节轴 z_{i-1} 在系 i 下的表达
            r = RT @ ps[i]                        # o_{i-1}→o_i 在系 i 下的表达
            w_new = RT @ w + q_dot[i] * z
            wd_new = RT @ wd + q_ddot[i] * z + q_dot[i] * np.cross(RT @ w, z)
            a_new = RT @ a + np.cross(wd_new, r) + np.cross(w_new, np.cross(w_new, r))
            # 质心加速度 p̈_{c,i}
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
        # 电机转子折算惯量项 B q̈（常数对角阵，不产生 Coriolis 交叉项）
        tau = tau + self.B * q_ddot
        return tau

    # -- 动力学量装配 -----------------------------------------------------------

    def gravity_vector(self, q):
        """ĝ(q) = RNEA(q, 0, 0)。"""
        zeros = np.zeros(self.n)
        return self.rnea(q, zeros, zeros, gravity=True)

    def coriolis_plus_gravity(self, q, q_dot):
        """Ĉ(q,q̇)q̇ + ĝ(q) = RNEA(q, q̇, 0)（计算力矩接口只需此组合项）。"""
        return self.rnea(q, q_dot, np.zeros(self.n), gravity=True)

    def mass_matrix(self, q):
        """M̂(q)：单位加速度列向量法，M[:,k] = RNEA(q,0,e_k)|_{无重力}。
        性质 P1：对称正定（模块自检中核验）。"""
        zeros = np.zeros(self.n)
        M = np.empty((self.n, self.n))
        for k in range(self.n):
            e = np.zeros(self.n)
            e[k] = 1.0
            M[:, k] = self.rnea(q, zeros, e, gravity=False)
        return 0.5 * (M + M.T)   # 数值对称化

    def computed_torque(self, q, q_dot, qddot_ref):
        """名义计算力矩接口（论文 §2.4）：τ = M̂ q̈_ref + Ĉ q̇ + ĝ。
        这是所有对比控制器共用的力矩出口（总方案 §5.2 公平性约束）。"""
        return self.mass_matrix(q) @ np.asarray(qddot_ref, dtype=float) \
            + self.coriolis_plus_gravity(q, q_dot)

    def forward_dynamics(self, q, q_dot, tau):
        """正动力学 q̈ = M⁻¹(τ - Cq̇ - g)：内部力矩级被控对象
        （--backend internal --plant torque 时替代式 (5.1) 的加速度级理想对象）。"""
        M = self.mass_matrix(q)
        h = self.coriolis_plus_gravity(q, q_dot)
        return np.linalg.solve(M, np.asarray(tau, dtype=float) - h)


def clip_torque(tau):
    """力矩限幅（场景篇 §6.3 安全上限）；返回 (限幅后 τ, 是否触发饱和)。"""
    tau = np.asarray(tau, dtype=float)
    lim = LBR4_TORQUE_LIMITS[:tau.shape[0]]
    clipped = np.clip(tau, -lim, lim)
    return clipped, bool(np.any(np.abs(tau) > lim))


def check_joint_limits(q, margin=0.02):
    """关节限位检查（留 margin 弧度余量）；返回越限关节索引列表。"""
    q = np.asarray(q, dtype=float)
    lim = LBR4_JOINT_LIMITS[:q.shape[0]] - margin
    return [int(i) for i in np.where(np.abs(q) > lim)[0]]


# ---------------------------------------------------------------------------
# 模块自检：python3 -m config.lbr4_dynamics
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    from config.params import KUKA_LBR4_DH, Q_INIT

    dyn = LBR4NominalDynamics(KUKA_LBR4_DH)
    rng = np.random.default_rng(0)
    q = Q_INIT + 0.3 * rng.standard_normal(7)
    qd = rng.standard_normal(7)
    qdd = rng.standard_normal(7)

    # 1) 一致性：RNEA(q,q̇,q̈) == M q̈ + Cq̇ + g（装配路径 vs 直接递推）
    lhs = dyn.rnea(q, qd, qdd)
    rhs = dyn.mass_matrix(q) @ qdd + dyn.coriolis_plus_gravity(q, qd)
    print("RNEA vs M*qdd + Cqd + g  残差:", np.max(np.abs(lhs - rhs)))

    # 2) 性质 P1：M 对称正定
    M = dyn.mass_matrix(q)
    eig = np.linalg.eigvalsh(M)
    print("M 特征值范围:", eig.min(), "...", eig.max(), "（应全为正）")

    # 3) 重力向量 vs 势能数值梯度（独立核验）
    def potential(qv):
        # 逐连杆质心世界高度求势能 U = Σ m_i g h_i
        T = np.eye(4)
        U = 0.0
        for i in range(dyn.n):
            a, al, d, off, _ = dyn.dh[i]
            T = T @ _dh_transform(a, al, d, qv[i] + off)
            pc = T[:3, :3] @ dyn.com[i] + T[:3, 3]
            U += dyn.m[i] * 9.81 * pc[2]
        return U

    g_num = np.array([
        (potential(q + h * np.eye(7)[k]) - potential(q - h * np.eye(7)[k])) / (2 * h)
        for k, h in [(k, 1e-6) for k in range(7)]])
    g_rnea = dyn.gravity_vector(q)
    print("g(q) RNEA vs 势能梯度 残差:", np.max(np.abs(g_num - g_rnea)))
