# TNDQ 论文理论 ↔ TNDQ_sim 代码对应与实现伪代码

> **文档性质**：理论–代码对照手册。理论侧只标注 `docs/TNDQ论文初稿_运动学重构_误差体系与控制律.md`（下称"论文"）的章节号与定理关键式子；代码侧给出 `TNDQ_sim/` 项目的**文件名**与逐函数对应的**伪代码**（伪代码与项目实现逐行同构，仅省略类型检查与日志）。
>
> 论文记号沿用初稿表 0：$\hat a$ 四元数、$\hat{\underline a}$ 单位 DQ、$\breve a$ HDQ、$\bar a$ TNDQ。

---

## 0. 总体映射表

| 论文章节 | 理论内容 | 关键式子 | 代码文件 | 核心符号 |
|---|---|---|---|---|
| §2.1 | 四元数 / DQ 代数、单位位姿 DQ | (2.1) | `core/dq_algebra.py` | `q_mul`, `dq_mul`, `dq_from_r_p`, `dq_conj` |
| §2.1 | 纯 DQ 与 $\mathrm{vec}_6$ 同构 | 表 0 | `core/dq_algebra.py` | `dq_vec6`, `vec6_to_pure_dq`, `dq_pure_part` |
| §2.2 | 空间 twist、伴随 $\mathrm{Ad}$、李括号 $\mathrm{ad}$ | (2.2) | `core/dq_algebra.py` | `dq_twist`, `dq_Ad`, `dq_ad` |
| §2.3 | HDQ 代数、乘法 (Leibniz)、逐通道共轭 | (2.3)(2.4) | `core/tndq_algebra.py` | `class HDQ` |
| §3.1 | TNDQ 代数 $\mathcal A_2=\widehat{\mathbb H}[\sigma]/(\sigma^3)$、乘法、共轭 | 定义 1，(3.1)(3.2) | `core/tndq_algebra.py` | `class TNDQ` |
| §3.2 | 曲线 TNDQ 表示、连乘法则 | (3.3a)(3.3)(3.4) | `core/kinematics.py` | `TNDQSerialChain.fk_tndq` |
| §3.2 | 导出量 $\boldsymbol\xi,\dot{\boldsymbol\xi},\dot J\dot{\boldsymbol q}$ | (3.5) | `core/tndq_algebra.py`, `core/kinematics.py` | `twist_from_tndq`, `twist_dot_from_tndq`, `fk_outputs` |
| §3.3 | HDQ 截断与乘法相容 | 命题 2，(3.6)(3.7) | `core/tndq_algebra.py` | `TNDQ.to_hdq` |
| §3.4 | 单位性约束族、残差监测与重投影 | (3.8)(3.9) | `core/tndq_algebra.py`, `core/dq_algebra.py` | `unit_constraint_residuals`, `reproject_tndq`, `dq_pose_normalize` |
| 附录 B.1 | 单关节因子的 TNDQ 表示（DH） | 附录 B.1 | `core/kinematics.py` | `tndq_joint_factor_dh` |
| §2.2 | 几何雅可比（前缀伴随结构） | $\mathrm{vec}_6\boldsymbol\xi=J\dot{\boldsymbol q}$ | `core/kinematics.py` | `TNDQSerialChain.jacobian` |
| §3.2/§4.2 | 期望轨迹的 TNDQ 表示（$\sigma^2$ 通道供前馈） | (3.3a) | `simdata/trajectory_generator.py` | `TrajectoryBase.evaluate`, `_pose_tndq_from_rp_derivatives` |
| §4.3 | 定理 1：HDQ 误差元素、几何一致 twist 误差 | (4.1)(4.2)(4.3)(4.4) | `control/error_system.py` | `hdq_error`, `twist_error_from_hdq` |
| §4.4 | 定理 2：输出误差级联运动学 | (4.5) | `control/error_system.py` | `output_error`, `A_matrix`, `full_error_state` |
| §5.2 | C1 几何一致计算力矩律（本文律） | (5.2)，引理 1 (5.3)(5.4) | `control/control_law.py` | `geometric_computed_torque_law`, `feedforward_term`, `damped_pinv` |
| §6.4 C2 | 忠实 [Ch20] resolved-acceleration 基线（Ad 搬运 twist 误差 + 螺旋对数位姿反馈） | [Ch20]-(32)–(35) | `control/control_law.py` | `dq_chandra2020_law`（位姿反馈用 `core/dq_algebra.py::dq_log2_vec6`） |
| §6.4 C2-abl | 朴素 twist 差消融基线（差分前馈，**非文献律**） | §6.4 折减更正框 | `control/control_law.py` | `dq_ctc_law` |
| §6.4 C3 | 一阶 DQ H∞ 基线（[P2] 律 + 速度伺服桥接） | [P2]-(12) | `control/control_law.py` | `dq_hinf_kinematic_law`, `velocity_to_accel_ref` |
| §2.4/(5.1b) | 名义计算力矩接口 $\boldsymbol\tau=\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+\hat C\dot{\boldsymbol q}+\hat{\boldsymbol g}$ | (5.1b) | `config/lbr4_dynamics.py` | `LBR4NominalDynamics.computed_torque`, `mass_matrix`, `coriolis_plus_gravity` |
| §5.3 | 存储函数、耗散等式、H∞/均方界证书 | 定理 3，(5.4a)(5.6a)(5.6b)(5.7) | `control/performance.py` | `storage_function`, `check_hinf_condition_merged/split`, `tightest_certified_l2_gain`, `iss_ultimate_bound`, `PerformanceAccumulator`, `ResidualDisturbanceEstimator` |
| §5.4 | 近恒等线性化通道、1/4 旋转折减、极点分配、刚度标度律 | (5.8)(5.9) | `control/gain_design.py` | `c1_channels`, `c3_channels`, `design_from_poles`, `design_matching_c3`, `screen` |
| §6.2 | 闭环信息流水线（输入→FK→误差→控制→力矩→监控） | §6.2 流水线图 | `run_simulation.py`, `experiments/run_grasp_circle.py` | `main()` 主循环 |
| §6.1/§6.4 | 机器人模型、增益三档、基线参数、数值参数 | §6.1/§6.4 | `config/params.py` | `KUKA_LBR4_DH`, `GAIN_SETS`, `CH20_*`, `DQC_*`, `DQH_*` |
| §6.3 | S3 抓取-搬运-圆周实验（各律调度、附着、敏感条件） | §6.3 | `experiments/run_grasp_circle.py` | 主循环 `law` 分支 |

---

## 1. §2.1–2.2 DQ 代数基础 → `core/dq_algebra.py`

**理论锚点**：单位 DQ 位姿 $\hat{\underline x}=\hat r+\varepsilon\tfrac12 p\hat r$（式 (2.1)）；空间 twist $\boldsymbol\xi=2\dot{\hat{\underline x}}\hat{\underline x}^*$（式 (2.2)）；伴随 $\mathrm{Ad}_{\hat{\underline x}}\boldsymbol a=\hat{\underline x}\boldsymbol a\hat{\underline x}^*$ 与李括号 $\mathrm{ad}_{\boldsymbol a}\boldsymbol b=\tfrac12(\boldsymbol{ab}-\boldsymbol{ba})$（§2.2）。数组约定：四元数 4 元 `[w,x,y,z]`，DQ 8 元 `[主部4; 对偶部4]`。

```python
# ---- 四元数层（§2.1） ----
def q_mul(a, b):                    # Hamilton 积
    aw, ax, ay, az = a;  bw, bx, by, bz = b
    return [aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw]

def q_conj(q):                      # q* = η − μ
    return [q[0], -q[1], -q[2], -q[3]]

def skew(v):                        # 叉积阵 [v]×（定理 2 的 A 用）
    return [[ 0,   -v[2],  v[1]],
            [ v[2],  0,   -v[0]],
            [-v[1],  v[0],  0  ]]

# ---- DQ 层（§2.1，ε² = 0） ----
def dq_mul(a, b):                   # (a0+εa1)(b0+εb1) = a0b0 + ε(a0b1+a1b0)
    ar, ad = a[:4], a[4:];  br, bd = b[:4], b[4:]
    return concat(q_mul(ar, br),                     # 主部
                  q_mul(ar, bd) + q_mul(ad, br))     # 对偶部（Leibniz 形）

def dq_conj(x):                     # 逐分量四元数共轭（§2.1）
    return concat(q_conj(x[:4]), q_conj(x[4:]))

def dq_from_r_p(r, p):              # 式 (2.1)：x̂ = r + ε·½ p r
    p_quat = concat([0.0], p)
    return concat(r, 0.5 * q_mul(p_quat, r))

def dq_translation(x):              # (2.1) 反解：p = 2 q_d r*
    return (2.0 * q_mul(x[4:], q_conj(x[:4])))[1:4]

def dq_vec6(xi):                    # vec6：纯 DQ → [ω; v] ∈ R⁶
    return concat(xi[1:4], xi[5:8])

def vec6_to_pure_dq(v):             # vec6 逆同构
    return [0, v[0], v[1], v[2],  0, v[3], v[4], v[5]]

def dq_pure_part(x):                # 投影到纯 DQ（清标量/对偶标量部）
    x[0] = 0.0;  x[4] = 0.0;  return x

def dq_twist(x, x_dot):             # 式 (2.2)：ξ = 2 ẋ x*
    return 2.0 * dq_mul(x_dot, dq_conj(x))

# ---- 伴随与李括号（§2.2） ----
def dq_Ad(x, a):                    # Ad_x a = x a x*（twist 搬运）
    return dq_mul(dq_mul(x, a), dq_conj(x))

def dq_ad(a, b):                    # ad_a b = ½(ab − ba)
    return 0.5 * (dq_mul(a, b) - dq_mul(b, a))
```

**对应关系**：`dq_from_r_p` ↔ (2.1)；`dq_twist` ↔ (2.2)；`dq_Ad`/`dq_ad` ↔ §2.2 定义（也是式 (5.2) 前馈的两个构件）。另有 DH 因子构件 `dq_rot_z / dq_rot_x / dq_trans_z / dq_trans_x`（附录 B.1 的常数尾段用）。

---

## 2. §2.3 HDQ 代数 → `core/tndq_algebra.py::class HDQ`

**理论锚点**：HDQ 乘法（式 (2.3)，$\varepsilon^*$ 通道即 Leibniz 法则）与逐通道 DQ 共轭（式 (2.4)，共轭与求导交换）。存储：`2×8` 数组，`ch[0]=主通道`、`ch[1]=导数通道`。

```python
class HDQ:                          # ǎ = a0 + ε* a1，ε*² = 0
    def __init__(self, a0, a1):
        self.ch = [a0, a1]          # 两个 DQ(8) 通道

    def __mul__(self, other):       # 式 (2.3)
        a0, a1 = self.ch;  b0, b1 = other.ch
        return HDQ(dq_mul(a0, b0),                      # 0 通道：位姿复合
                   dq_mul(a0, b1) + dq_mul(a1, b0))     # 1 通道：Leibniz

    def conj(self):                 # 式 (2.4)：逐通道 DQ 共轭（定理 1 的 (x̆_d)*）
        return HDQ(dq_conj(self.ch[0]), dq_conj(self.ch[1]))
```

一次 HDQ 乘 = 3 次 DQ 乘（`a0b0`、`a0b1`、`a1b0`）——定理 1 注记"一次 HDQ 乘法（3 次 DQ 乘）"的出处。

---

## 3. §3.1 TNDQ 代数 → `core/tndq_algebra.py::class TNDQ`

**理论锚点**：定义 1——$\mathcal A_2=\widehat{\mathbb H}[\sigma]/(\sigma^3)$，元素 $\bar a=\hat{\underline a}_0+\sigma\hat{\underline a}_1+\tfrac12\sigma^2\hat{\underline a}_2$（式 (3.1)）；乘法（式 (3.2)）；逐通道共轭（定义 1 后）。存储：`3×8` 数组；**注意** `ch[2]` 存的是**原始二阶导数**（系数 $\tfrac12$ 住在基元 $\tfrac12\sigma^2$ 里，与定义 1 一致）。

```python
class TNDQ:                         # ā = a0 + σ a1 + ½σ² a2，σ³ = 0
    def __init__(self, a0, a1=0, a2=0):
        self.ch = [a0, a1, a2]      # 三个 DQ(8) 通道：位姿/一阶导/二阶导

    def __mul__(self, other):       # 式 (3.2)，σ³=0 截断乘法
        a0, a1, a2 = self.ch;  b0, b1, b2 = other.ch
        c0 = dq_mul(a0, b0)
        c1 = dq_mul(a0, b1) + dq_mul(a1, b0)                       # 一阶 Leibniz
        c2 = dq_mul(a0, b2) + 2.0*dq_mul(a1, b1) + dq_mul(a2, b0)  # 二阶 Leibniz
        return TNDQ(c0, c1, c2)

    def conj(self):                 # 逐通道 DQ 共轭（与 d/dt 交换）
        return TNDQ(dq_conj(self.ch[0]), dq_conj(self.ch[1]), dq_conj(self.ch[2]))

    # ---- 截断塔（表 1 / 命题 2） ----
    def to_hdq(self):               # 式 (3.6)：ā|_HDQ = a0 + ε* a1（只取前两通道）
        return HDQ(self.ch[0], self.ch[1])

    def to_dq(self):                # σ⁰ 通道（DQ 截断）
        return self.ch[0]

    @staticmethod
    def from_pose_derivatives(x, x_dot, x_ddot):   # 式 (3.3a) 曲线表示
        return TNDQ(x, x_dot, x_ddot)
```

**命题 2 的实现含义**：`to_hdq` 只是"取前两个数组"；由于式 (3.2) 的 0/1 通道不含 `b2/a2`（滤过性），先乘后截与先截后乘逐字相同——误差层只消费 HDQ 截断是**无损**的（§4.2 的程序依据）。

---

## 4. §3.2 链正运动学与导出量 → `core/kinematics.py` + `core/tndq_algebra.py`

### 4.1 单关节因子（附录 B.1）

**理论锚点**：旋转关节因子沿局部螺旋轴 $\boldsymbol s=k$（纯 DQ）满足 $\dot{\hat{\underline x}}_i=\tfrac12\dot q_i\boldsymbol s_i\hat{\underline x}_i$、$\ddot{\hat{\underline x}}_i=\tfrac12\ddot q_i\boldsymbol s_i\hat{\underline x}_i+\tfrac14\dot q_i^2\boldsymbol s_i^2\hat{\underline x}_i$；DH 因子 $A_i=R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i)$，只有 $R_z$ 依赖关节变量。

```python
S_LOCAL_Z = pure_DQ([0,0,1])        # 局部 z 轴螺旋轴 s = k

def tndq_joint_factor_dh(a, alpha, d, theta, theta_dot, theta_ddot):
    Rz    = dq_rot_z(theta)
    s_Rz  = dq_mul(S_LOCAL_Z, Rz)
    ss_Rz = dq_mul(S_LOCAL_Z, s_Rz)

    Rz_dot  = 0.5 * theta_dot  * s_Rz                        # 附录 B.1 闭式
    Rz_ddot = 0.5 * theta_ddot * s_Rz + 0.25 * theta_dot**2 * ss_Rz
    rot_bar = TNDQ(Rz, Rz_dot, Rz_ddot)

    tail = dq_mul(dq_mul(dq_trans_z(d), dq_trans_x(a)), dq_rot_x(alpha))
    return rot_bar * TNDQ.from_constant(tail)   # 常数尾段导数通道为 0，按 (3.2) 相乘
```

### 4.2 连乘法则（命题 1 / 式 (3.4)）与雅可比

```python
class TNDQSerialChain:
    def fk_tndq(self, q, q_dot=None, q_ddot=None):      # 式 (3.4)：x̄ = Π x̄_i
        x_bar = TNDQ.identity()
        for i, (a, alpha, d, theta0, _) in enumerate(self.dh_table):
            x_bar_i = tndq_joint_factor_dh(a, alpha, d,
                                           theta     = theta0 + q[i],
                                           theta_dot = q_dot[i],
                                           theta_ddot= q_ddot[i])
            x_bar = x_bar * x_bar_i                     # TNDQ 乘 (3.2)，O(n)
        return x_bar                                    # 三通道 = (x̂, ẋ̂, ẍ̂)

    def jacobian(self, q):                              # §2.2：vec6 ξ = J q̇
        J = zeros(6, n);  prefix = dq_identity()
        for i, (a, alpha, d, theta0, _) in enumerate(self.dh_table):
            J[:, i] = dq_vec6(dq_Ad(prefix, S_LOCAL_Z))   # 前缀伴随搬运螺旋轴（[P1]）
            Rz   = dq_rot_z(theta0 + q[i])
            tail = dq_mul(dq_mul(dq_trans_z(d), dq_trans_x(a)), dq_rot_x(alpha))
            prefix = dq_mul(prefix, dq_mul(Rz, tail))
        return J
```

### 4.3 导出量（式 (3.5)）与 FK 输出束

**理论锚点**：$\boldsymbol\xi=2\dot{\hat{\underline x}}\hat{\underline x}^*$，$\dot{\boldsymbol\xi}=2\ddot{\hat{\underline x}}\hat{\underline x}^*-\tfrac12\boldsymbol\xi^2$（取纯部），$\mathrm{vec}_6\dot{\boldsymbol\xi}=\dot J\dot{\boldsymbol q}+J\ddot{\boldsymbol q}$；**令 $\ddot{\boldsymbol q}=0$ 连乘一次即单独读出 $\dot J\dot{\boldsymbol q}$**（式 (5.2) 的补偿项免构造获得）。

```python
# core/tndq_algebra.py
def twist_from_tndq(x_bar):         # (3.5) 第一式
    return dq_twist(x_bar.ch[0], x_bar.ch[1])           # ξ = 2 ẋ x*

def twist_dot_from_tndq(x_bar):     # (3.5) 第二式（推导见附录 B.2）
    x, x_dot, x_ddot = x_bar.ch
    xi = dq_twist(x, x_dot)
    xi_dot = 2.0 * dq_mul(x_ddot, dq_conj(x)) - 0.5 * dq_mul(xi, xi)
    return dq_pure_part(xi_dot)     # 沿真曲线纯部即 ξ̇

# core/kinematics.py
def fk_outputs(self, q, q_dot, q_ddot=None, with_jacobian=True):
    x_bar0 = self.fk_tndq(q, q_dot, None)               # q̈=0 链
    Jdot_qdot = dq_vec6(twist_dot_from_tndq(x_bar0))    # (3.5) 免构造读出 J̇q̇
    if q_ddot is None:                                  # 实测链 q̈ 未知 → 恒用 q̈=0 链
        x_bar, xi_dot_vec = x_bar0, Jdot_qdot
    else:
        x_bar = self.fk_tndq(q, q_dot, q_ddot)
        xi_dot_vec = dq_vec6(twist_dot_from_tndq(x_bar))   # J̇q̇ + J q̈
    xi = twist_from_tndq(x_bar)
    c0, c1, c2 = unit_constraint_residuals(x_bar)       # (3.8)，见 §5 本文档
    return { "x_bar": x_bar,            # (3.3a) TNDQ 表示
             "x_breve": x_bar.to_hdq(), # (3.6) HDQ 截断（命题 2 无损）→ 误差层
             "x": x_bar.to_dq(),        # σ⁰ 通道位姿
             "xi": dq_vec6(xi),         # (3.5)
             "xi_dot": xi_dot_vec,
             "Jdot_qdot": Jdot_qdot,    # (5.2) 补偿项
             "c0": c0, "c1": c1, "c2": c2,
             "J": self.jacobian(q) }    # with_jacobian 时
```

---

## 5. §3.4 单位性约束族与重投影 → `core/tndq_algebra.py`, `core/dq_algebra.py`

**理论锚点**：约束族 (3.8) 是 $\bar x\bar x^*=1$（式 (3.9)，单位群 $\mathcal U_2$）的三通道；数值漂移监测 $c_0=\|\hat{\underline x}\hat{\underline x}^*-1\|$、$c_1=\|\mathrm{Sc}(2\dot{\hat{\underline x}}\hat{\underline x}^*)\|$、$c_2=\|\mathrm{Sc}(\dot{\boldsymbol\xi})\|$；超阈值重投影（0 阶归一化、1 阶 $\dot{\hat{\underline x}}\mapsto\tfrac12\boldsymbol\xi_{\mathrm{proj}}\hat{\underline x}$、2 阶由 (3.5) 反解）。

```python
def dq_scalar_parts(x):             # Sc(·)：取标量部与对偶标量部 [x[0], x[4]]
def dq_unit_residual(x):            # c0 = ||x x* − 1||
    return norm(dq_mul(x, dq_conj(x)) - dq_identity())

def unit_constraint_residuals(x_bar):          # 式 (3.8) 三残差
    x, x_dot, x_ddot = x_bar.ch
    c0 = dq_unit_residual(x)
    xi = dq_twist(x, x_dot)
    c1 = norm(dq_scalar_parts(xi))             # = ||Sc(2 ẋ x*)||
    xi_sq    = dq_mul(xi, xi)
    acc_term = 2.0 * dq_mul(x_ddot, dq_conj(x))
    c2 = norm(dq_scalar_parts(acc_term) - 0.5 * dq_scalar_parts(xi_sq))
    return c0, c1, c2                          # 沿真曲线解析为 0

def reproject_tndq(x_bar):                     # §3.4 重投影
    x, x_dot, x_ddot = x_bar.ch
    x_new = dq_pose_normalize(x)               # 0 阶：归一化 r，按当前 p 重建对偶部
    xi_proj = dq_pure_part(dq_twist(x_new, x_dot))
    x_dot_new  = 0.5 * dq_mul(xi_proj, x_new)                    # 1 阶：ẋ = ½ ξ x
    xi_dot_proj = dq_pure_part(2.0 * dq_mul(x_ddot, dq_conj(x_new))
                               - 0.5 * dq_mul(xi_proj, xi_proj))
    x_ddot_new = 0.5 * dq_mul(xi_dot_proj + 0.5*dq_mul(xi_proj, xi_proj), x_new)
    return TNDQ(x_new, x_dot_new, x_ddot_new)  # 2 阶：(3.5) 第二式反解
```

---

## 6. 期望轨迹的 TNDQ 表示 → `simdata/trajectory_generator.py`

**理论锚点**：期望链按 (3.3a) 建模（§4.2："正运动学与期望轨迹用 TNDQ 建模，前馈需要 $\sigma^2$ 通道"）；$(r,p)$ 及其两阶导数解析给出后按式 (2.1) 逐阶组装——(3.8) 精确成立，无数值微分；$\boldsymbol\xi_d,\dot{\boldsymbol\xi}_d$ 由 (3.5) 读出。

```python
def _pose_tndq_from_rp_derivatives(r, r_dot, r_ddot, p, p_dot, p_ddot):
    pq, pq_dot, pq_ddot = pure(p), pure(p_dot), pure(p_ddot)
    x      = concat(r,      0.5 * q_mul(pq, r))                               # (2.1)
    x_dot  = concat(r_dot,  0.5 * (q_mul(pq_dot, r) + q_mul(pq, r_dot)))      # d/dt (2.1)
    x_ddot = concat(r_ddot, 0.5 * (q_mul(pq_ddot, r) + 2.0*q_mul(pq_dot, r_dot)
                                   + q_mul(pq, r_ddot)))                      # d²/dt² (2.1)
    return TNDQ.from_pose_derivatives(x, x_dot, x_ddot)     # (3.3a)

class TrajectoryBase:
    def evaluate(self, t):          # 每周期调用（§6.2 输入层）
        r, r_dot, r_ddot, p, p_dot, p_ddot = self._rp_derivatives(t)  # 各轨迹类解析给出
        x_bar_d = _pose_tndq_from_rp_derivatives(...)
        return { "x_bar_d":   x_bar_d,               # σ² 通道 → (5.2) 前馈 ξ̇_d
                 "x_breve_d": x_bar_d.to_hdq(),      # (3.6) 截断 → 误差层（定理 1）
                 "x_d":       x_bar_d.to_dq(),
                 "xi_d":      dq_vec6(twist_from_tndq(x_bar_d)),      # (3.5)
                 "xi_dot_d":  dq_vec6(twist_dot_from_tndq(x_bar_d)) } # (3.5)
```

具体轨迹类（`LineTrajectoryTNDQ` / `CircleTrajectoryTNDQ` / `SetpointTrajectoryTNDQ` / `CupCircleTrajectoryTNDQ` / `goto_trajectory` / `CompositeTrajectoryTNDQ`）只实现 `_rp_derivatives(t)`；旋转曲线导数闭式与附录 B.1 同构（`_rotation_derivatives`：$\dot r=\tfrac12\dot\phi\,\hat n r$、$\ddot r=\tfrac12\ddot\phi\,\hat n r+\tfrac14\dot\phi^2\hat n^2 r$）。S3 七相位时间线由 `experiments/run_grasp_circle.py` 用 `CompositeTrajectoryTNDQ` 串接（§6.3）。

---

## 7. §4 误差体系（定理 1 / 定理 2）→ `control/error_system.py`

### 7.1 定理 1：HDQ 误差元素（式 (4.1)–(4.3)）

**理论锚点**：$\breve{\tilde x}=\breve x(\breve x_d)^*$（式 (4.1)），一次乘法同时得 $\tilde x=\hat{\underline x}\hat{\underline x}_d^*$ 与 $\dot{\tilde x}=\dot{\hat{\underline x}}\hat{\underline x}_d^*+\hat{\underline x}\dot{\hat{\underline x}}_d^*$（式 (4.2)）；几何一致 twist 误差 $\tilde{\boldsymbol\xi}=2\dot{\tilde x}\tilde x^*$、$e_\xi=\mathrm{vec}_6\tilde{\boldsymbol\xi}$（式 (4.3)），无扰时等于 $\boldsymbol\xi-\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d$（式 (4.4)）；$\tilde{\boldsymbol\xi}$ 对 unwinding 翻转 $\tilde x\to-\tilde x$ 不变（定理 1(i)）。

```python
def hdq_error(x_breve, x_breve_d):              # 式 (4.1)
    x_breve_tilde = x_breve * x_breve_d.conj()  # HDQ 乘 (2.3)，共轭取 (2.4)
    # 通道 0 = x̃ = x x_d*（右不变位姿误差，[P2] 原样）
    # 通道 1 = ẋ̃ = ẋ x_d* + x ẋ_d*（Leibniz，式 (4.2)）

    if x_breve_tilde.ch[0][0] < 0.0:            # η̃ < 0 → 整体翻转（定理 1(i) 允许；
        x_breve_tilde = HDQ(-x_breve_tilde.ch[0], -x_breve_tilde.ch[1])
    return x_breve_tilde                        #  强制留在 η̃ > 0 工作域，定理 3(b) 第 2 步）

def twist_error_from_hdq(x_breve_tilde):        # 式 (4.3)
    x_tilde, x_tilde_dot = x_breve_tilde.ch
    xi_tilde = 2.0 * dq_mul(x_tilde_dot, dq_conj(x_tilde))
    xi_tilde = dq_pure_part(xi_tilde)           # 纯性解析成立（附录 A.1），投影防数值漂移
    return dq_vec6(xi_tilde), xi_tilde          # e_ξ；无扰时 = ξ − Ad_x̃ ξ_d（式 (4.4)）
```

### 7.2 定理 2：输出误差与 $A(\tilde x)$（式 (4.5)）

**理论锚点**：$e_z=[\mathcal O;\mathcal T]$，$\mathcal O=-\mathrm{Im}\,\tilde r$、$\mathcal T=\tilde p$（[P2] 输出原样）；$\dot e_z=A(\tilde x)e_\xi$，$A=\begin{bmatrix}-\tfrac12(\tilde\eta I_3+[\mathcal O]_\times)&0\\ -[\mathcal T]_\times&I_3\end{bmatrix}$，$\tilde x\to1$ 时 $A\to A_0=\mathrm{diag}(-\tfrac12I_3,I_3)$。

```python
def output_error(x_tilde):                      # §4.4（[P2] 输出，向下兼容 §4.5）
    r_tilde = x_tilde[:4]
    O = -r_tilde[1:4]                           # O = −Im r̃
    T = dq_translation(x_tilde)                 # T = p̃ = 2 q_d r*
    return concat(O, T), O, T

def A_matrix(x_tilde):                          # 定理 2，式 (4.5)
    e_z, O, T = output_error(x_tilde)
    eta_tilde = x_tilde[0]
    A = zeros(6, 6)
    A[:3, :3] = -0.5 * (eta_tilde * eye(3) + skew(O))
    A[3:, :3] = -skew(T)
    A[3:, 3:] = eye(3)
    return A                                    # ė_z = A(x̃) e_ξ

def full_error_state(x_breve, x_breve_d):       # §6.2 误差层，一次调用出全部误差量
    x_breve_tilde = hdq_error(x_breve, x_breve_d)      # (4.1)(4.2)
    x_tilde       = x_breve_tilde.to_dq()
    e_xi, xi_tilde = twist_error_from_hdq(x_breve_tilde)   # (4.3)
    e_z, O, T     = output_error(x_tilde)
    A             = A_matrix(x_tilde)                    # (4.5)
    return {"x_breve_tilde": ..., "x_tilde": x_tilde, "xi_tilde": xi_tilde,
            "e_xi": e_xi, "e_z": e_z, "O": O, "T": T, "A": A}
```

误差状态共 12 维（$e_z$ 6 + $e_\xi$ 6），**不含加速度层**——§4.2 的结构性取舍在代码里体现为：误差层输入只有两个 HDQ（各 2 通道），`full_error_state` 无任何加速度估计/差分。

---

## 8. §5.2 控制律 C1（本文律）→ `control/control_law.py`

**理论锚点**：式 (5.2)
$$\ddot{\boldsymbol q}_{\mathrm{ref}}=J^{+}\Bigl(\underbrace{\mathrm{vec}_6\bigl(\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d\bigr)}_{u_{\mathrm{ff}}\ \text{（引理 1）}}-K_d e_\xi-A^{\top}K_p e_z-\dot J\dot{\boldsymbol q}\Bigr)$$
前馈与误差动态非反馈项精确相消（定理 3(a) 证明第 3 步）；$K_p$ 写在 $A^\top$ **内侧**（§5.2 说明 (c)，Lyapunov 交叉项精确相消对任意对称正定 $K_p$ 成立）；$\dot J\dot{\boldsymbol q}$ 由 TNDQ 链免构造读出（式 (3.5)）。

```python
def damped_pinv(J, damping=1e-6):               # 阻尼伪逆（假设 (A1)；奇异邻域
    return J.T @ inv(J @ J.T + damping**2 * eye(6))   # JJ⁺≠I 残差归 d_ex，诚实条款 (i)）

def feedforward_term(x_tilde, xi_tilde, xi_d_vec6, xi_dot_d_vec6):   # 引理 1 / (5.3)(5.4)
    xi_d     = vec6_to_pure_dq(xi_d_vec6)
    xi_dot_d = vec6_to_pure_dq(xi_dot_d_vec6)
    Ad_xi_d     = dq_Ad(x_tilde, xi_d)          # Ad_x̃ ξ_d：搬运期望速度
    Ad_xi_dot_d = dq_Ad(x_tilde, xi_dot_d)      # Ad_x̃ ξ̇_d：搬运期望加速度
    transport   = dq_ad(xi_tilde, Ad_xi_d)      # ad_ξ̃ Ad_x̃ ξ_d：输运修正
    return dq_vec6(Ad_xi_dot_d + transport)

def geometric_computed_torque_law(err, xi_d_vec6, xi_dot_d_vec6,
                                  J, Jdot_qdot, K_d, k_p, damping=1e-6):   # 式 (5.2)
    u_ff = feedforward_term(err["x_tilde"], err["xi_tilde"], xi_d_vec6, xi_dot_d_vec6)

    K_p = asarray(k_p)                          # 标量或对称正定 6×6（diag(p_O I3, p_T I3)）
    pose = K_p @ err["e_z"] if K_p.ndim == 2 else K_p * err["e_z"]
    u_fb = -K_d @ err["e_xi"] - err["A"].T @ pose     # −K_d e_ξ − AᵀK_p e_z（K_p 在内侧）

    u_task = u_ff + u_fb - Jdot_qdot            # 任务空间指令（−J̇q̇ 来自 (3.5)）
    qddot_ref = damped_pinv(J, damping) @ u_task
    return qddot_ref, u_task
```

**定理 3(a) 级联标准形（式 (5.5)）在代码中的对应**：$\dot e_z=Ae_\xi$ 由定理 2 保证（`A_matrix`），$\dot e_\xi=-K_de_\xi-A^\top K_pe_z+d$ 中的 $d$ 不显式计算，而是由 `ResidualDisturbanceEstimator`（见 §13.4）从闭环动态**反演**出来做证书核算。

---

## 9. §6.4 基线 C2（忠实 [Ch20] resolved-acceleration 律）→ `control/control_law.py::dq_chandra2020_law`

**理论锚点**：§6.4 C2 段。按原文式 (32)–(35) 与式 (2) 逐项移植：①twist 误差取**经伴随搬运**的差 $\boldsymbol\omega_e=\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d-\boldsymbol\xi$（式 (32)；在本项目右不变约定下 $\boldsymbol\omega_e=-e_\xi$，与定理 1 同一对象）；②位姿反馈取螺旋对数（式 (35)），原文的 $+2K_P\ln x_e$ 在本项目误差取向后换算为 $-K_P\,\mathrm{vec}_6(2\ln\tilde x)$（换算依据：$\tfrac{d}{dt}\mathrm{vec}_6(2\ln\tilde x)=e_\xi=-\boldsymbol\omega_e$，已由 `tests/test_math_properties.py` T11/T12 数值锁定）；③$\dot{\boldsymbol\xi}_d$ 与 $\dot J\dot{\boldsymbol q}$ 均按原文取**解析量**（期望链 $\sigma^2$ 通道 + 式 (3.5) 免构造读出），无差分。与 C1 的唯一结构差异是位姿反馈的整形形式（螺旋对数 vs $A^\top$）：前者导数映射在 $\phi\to\pi$ 奇异且无对任意正定增益成立的耗散等式结构。

```python
# 调用侧（experiments/run_grasp_circle.py，主循环 C2 分支）：
qddot_ref, _ = dq_chandra2020_law(err, des["xi_d"], des["xi_dot_d"],
                                  fk["J"], fk["Jdot_qdot"],
                                  params.CH20_K_V, params.CH20_K_P, damping)
# ξ̇_d 与 J̇q̇ 为解析量，无差分状态

# 律本体：
def dq_chandra2020_law(err, xi_d_vec6, xi_dot_d_vec6, J, Jdot_qdot, K_v, K_P, damping):
    u_ff   = feedforward_term(err["x_tilde"], err["xi_tilde"], xi_d_vec6, xi_dot_d_vec6)
    omega_e = -err["e_xi"]                                  # (32)：Ad ξ_d − ξ
    ell     = dq_log2_vec6(err["x_tilde"])                  # vec₆(2 ln x̃)（螺旋坐标）
    u_pose  = -(K_P @ ell)                                  # (35) 的约定换算（见上）
    a_cmd   = u_ff + K_v @ omega_e + u_pose
    u_task  = a_cmd - Jdot_qdot                             # 原文式 (2)
    qddot_ref = damped_pinv(J, damping) @ u_task
    return qddot_ref, u_task
```

配平：近恒等逐通道线性化为 $\ddot{\boldsymbol\ell}+K_v\dot{\boldsymbol\ell}+K_P\boldsymbol\ell=0$（$\boldsymbol\ell=\mathrm{vec}_6(2\ln\tilde x)$，旋转通道 $\phi\boldsymbol n\approx-2\mathcal O$ 自带因子 2，**无** $1/4$ 亦无 $1/2$ 折减）。`config/params.py::CH20_K_V = 24 I₆`、`CH20_K_P = 80 I₆`——极点 $\{-4,-20\}$、DC 刚度 80，与 C1-tuned/C3 逐通道恒等。

### 9b. §6.4 基线 C2-abl（朴素 twist 差消融档，**非文献律**）→ `control/control_law.py::dq_ctc_law`

**理论锚点**：§6.4 C2-abl 段 + 折减更正框。本稿早期版本曾把本档标注为"[Ch20] 类"，与原文不符（[Ch20] 式 (32) 的 twist 误差含 Ad 搬运，忠实代表为上节 C2），现更名 C2-abl 作消融用途。结构差异（相对 C1）：①朴素 twist 差 $\boldsymbol\xi_d-\boldsymbol\xi$（无 $\mathrm{Ad}_{\tilde x}$ 搬运 → §4.1 伪项）；②无 $A^\top$ 整形（无定理 3 证书）；③$\dot{\boldsymbol\xi}_d$ 与 $\dot J\dot{\boldsymbol q}$ 用**数值差分**（一拍滞后 + 差分噪声）。近恒等线性化折减因子为 $1/2$（只有 $\dot{\mathcal O}=-\tfrac12\tilde\omega$ 一个 $\tfrac12$）：$\ddot{\mathcal O}+K_\omega\dot{\mathcal O}+\tfrac{p_O}{2}\mathcal O=-\tfrac12d_\omega$。

```python
# 调用侧（experiments/run_grasp_circle.py，主循环 C2-abl 分支）：
xi_dot_d_num  = (des["xi_d"] - xi_d_prev) / dt_ctrl        # 差分 ξ̇_d（无 σ² 通道）
Jdot_qdot_num = (fk["J"] - J_prev) / dt_ctrl @ qdot_meas   # 差分 J̇q̇
qddot_ref, _  = dq_ctc_law(err, fk["xi"], des["xi_d"], xi_dot_d_num, Jdot_qdot_num,
                           fk["J"], params.DQC_K_D, params.DQC_K_P, damping)
xi_d_prev, J_prev = copy(des["xi_d"]), copy(fk["J"])       # 一拍状态

# 律本体：
def dq_ctc_law(err, xi_vec6, xi_d_vec6, xi_dot_d_num, Jdot_qdot_num,
               J, K_d, K_p, damping=1e-6):
    Kp = K_p if K_p is matrix else K_p * eye(6)
    u_pose = concat(Kp[:3,:3] @ err["O"], -(Kp[3:,3:] @ err["T"]))   # [+p_O O; −p_T T]
    e_v    = xi_d_vec6 - xi_vec6                     # 朴素 twist 差（§4.1 伪项）
    u_task = xi_dot_d_num + K_d @ e_v + u_pose - Jdot_qdot_num
    qddot_ref = damped_pinv(J, damping) @ u_task
    return qddot_ref, u_task
```

配平：`config/params.py::DQC_K_D = 24 I₆`、`DQC_K_P = diag(160 I₃, 80 I₃)`——$p_O/2=80$ 与 C1-tuned 旋转刚度 $p_O/4=80$、C3 DC 刚度 80 对齐，闭环极点同为 $\{-4,-20\}$（§6.4 更正框）。

---

## 10. §6.4 基线 C3（一阶 DQ H∞ + 速度伺服桥接）→ `control/control_law.py`

**理论锚点**：忠实移植 [P2] 式 (12)：$\dot{\boldsymbol q}_{\mathrm{cmd}}=J^+\bigl([k_O\mathcal O;-k_T\mathcal T]+\mathrm{vec}_6(\tilde x\boldsymbol\xi_d\tilde x^*)\bigr)$，$k_O=\sqrt2/\gamma_O$、$k_T=\sqrt2/\gamma_T$；括号内即 $\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d$（定理 1 注记）。速度级指令经内环速度伺服（含一拍差分前馈）桥接到共享力矩接口——差分前馈的一拍滞后/噪声放大是 C3 缺二阶通道的真实属性。

```python
def dq_hinf_kinematic_law(err, xi_d_vec6, gamma_O, gamma_T):   # [P2]-(12)，速度级
    kO = sqrt(2.0) / gamma_O;  kT = sqrt(2.0) / gamma_T
    feedback    = concat(kO * err["O"], -kT * err["T"])
    feedforward = dq_vec6(dq_Ad(err["x_tilde"], vec6_to_pure_dq(xi_d_vec6)))  # Ad_x̃ ξ_d
    return feedback + feedforward

def velocity_to_accel_ref(qdot_cmd, qdot_cmd_prev, q_dot, dt, k_servo):   # 加速度桥接
    ff = zeros_like(qdot_cmd) if qdot_cmd_prev is None \
         else (qdot_cmd - qdot_cmd_prev) / dt          # 一拍差分前馈（无 ξ̇_d 解析通道）
    return ff + k_servo * (qdot_cmd - q_dot)           # + 速度伺服项

# 调用侧（experiments/run_grasp_circle.py，主循环 C3 分支）：
task_vel  = dq_hinf_kinematic_law(err, des["xi_d"], params.DQH_GAMMA_O, params.DQH_GAMMA_T)
qdot_cmd  = damped_pinv(fk["J"], max(damping, params.DQH_DAMPING)) @ task_vel
qddot_ref = velocity_to_accel_ref(qdot_cmd, qdot_cmd_prev, qdot_meas, dt_ctrl,
                                  params.DQH_K_SERVO)
qdot_cmd_prev = qdot_cmd
```

等效线性化（`gain_design.c3_channels`）：旋转通道极点 $\{-k_O/2,-K_{\mathrm{servo}}\}$、平移 $\{-k_T,-K_{\mathrm{servo}}\}$；$k_O=8,k_T=4,K_{\mathrm{servo}}=20$（`params.DQH_*`）时两通道 DC 刚度均为 80。

---

## 11. §2.4 / (5.1b) 名义动力学与力矩层 → `config/lbr4_dynamics.py`

**理论锚点**：计算力矩接口 $\boldsymbol\tau=\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+\hat C\dot{\boldsymbol q}+\hat{\boldsymbol g}$（式 (5.1b)，§2.4）；真实闭环 $\ddot{\boldsymbol q}=\ddot{\boldsymbol q}_{\mathrm{ref}}+\boldsymbol w_{\mathrm{dyn}}$（式 (5.1)），失配 $\Delta M,\Delta C,\Delta g$（带载时名义模型不含杯 → §6.1 的实验变量）折算为扰动，由定理 3(c)/(d) 兜底。

```python
class LBR4NominalDynamics:          # RNEA（Siciliano 标准 DH 形式），Gaz[14] 名义参数
    def rnea(self, q, q_dot, q_ddot, gravity=True): ...    # 逆动力学 τ = RNEA(q,q̇,q̈)

    def mass_matrix(self, q):       # M̂(q)：单位加速度列向量法
        for k in range(n):
            M[:, k] = self.rnea(q, zeros, e_k, gravity=False)
        return M                    # 对称正定（性质 P1）

    def coriolis_plus_gravity(self, q, q_dot):   # Ĉq̇ + ĝ = RNEA(q, q̇, 0)
        return self.rnea(q, q_dot, zeros, gravity=True)

    def computed_torque(self, q, q_dot, qddot_ref):        # 式 (5.1b)，各律共用力矩出口
        return self.mass_matrix(q) @ qddot_ref + self.coriolis_plus_gravity(q, q_dot)

    def forward_dynamics(self, q, q_dot, tau):             # 内部力矩级对象（式 (5.1a)）
        return solve(self.mass_matrix(q), tau - self.coriolis_plus_gravity(q, q_dot))
```

`mismatch_scale` 参数实现 E3 条件（控制器端名义惯性整体缩放 → $\Delta M\neq0$，式 (5.1) 的 $\boldsymbol w_{\mathrm{dyn}}$）。力矩饱和裁剪 `clip_torque` 与关节限位 `check_joint_limits` 也在本文件（饱和残差计入 $d(t)$，诚实条款）。

---

## 12. §5.3 定理 3 证书核算 → `control/performance.py`

### 12.1 存储函数（式 (5.4a)）

```python
def storage_function(e_xi, e_z, k_p):           # V = ½‖e_ξ‖² + ½ e_zᵀ K_p e_z
    K_p = k_p if k_p is matrix else k_p * eye(6)   # 标量 k_p 或对称正定 K_p
    return 0.5 * (e_xi @ e_xi) + 0.5 * (e_z @ (K_p @ e_z))

def storage_function_split(e_xi, e_z, k_p):     # 定理 3(c-2) 通道拆分 V = V_ω + V_v
    V_w = 0.5*(e_xi[:3]@e_xi[:3]) + 0.5*(e_z[:3] @ (K_p[:3,:3] @ e_z[:3]))
    V_v = 0.5*(e_xi[3:]@e_xi[3:]) + 0.5*(e_z[3:] @ (K_p[3:,3:] @ e_z[3:]))
    return V_w, V_v
```

### 12.2 H∞ 判据（式 (5.6a)/(5.6b)）与最紧增益

```python
def check_hinf_condition_merged(K_d, kappa, gamma_a):    # 定理 3(c-1)，Schur 补判据
    lam_min = min(eigvalsh(K_d))
    level   = 0.5 * (1.0/kappa + 1.0/gamma_a**2)         # K_d ≽ level·I₆
    return lam_min >= level, lam_min, level              # §6.4 读法 A

def check_hinf_condition_split(K_omega, K_v, kappa_w, gamma_w, kappa_v, gamma_v):
    ...                                                  # 定理 3(c-2)：(5.6b) 逐通道

def tightest_certified_l2_gain(K_d):            # §5.3 注记 (ii) / 附录 C.3：
    return 1.0 / min(eigvalsh(K_d))             # 认证 L2 增益天花板 1/λmin(K_d)，§6.4 读法 B
```

### 12.3 均方极限界（式 (5.7)，定理 3(d)）

```python
def iss_ultimate_bound(K_d, d_inf_norm):        # limsup RMS(e_ξ) ≤ D / λ_eff
    lam_min = min(eigvalsh(K_d))
    return d_inf_norm / lam_min                 # α→0 极限（λ_eff = λmin − α·λmax）
```

注意语义（代码 docstring 与论文定理 3(d) 注记一致）：这是**稳态窗 RMS 界**，不是逐点 ISS 极限球（$V$ 在 $e_z$ 方向无耗散，附录 C.5）。

### 12.4 在线核算：扰动反演与能量账本

```python
class ResidualDisturbanceEstimator:             # §6.5(6)：反演证书通道等效扰动
    # 由闭环动态 (5.5) 第二式反解：
    #   ė_ξ = −K_d e_ξ − AᵀK_p e_z + d   =>   d̂ = ė_ξ + K_d e_ξ + AᵀK_p e_z
    def update(self, e_xi, e_z, A):
        d_raw = (e_xi - self.e_xi_prev) / self.dt + self.K_d @ e_xi \
                + A.T @ (self.K_p @ e_z)         # 数值微分 ė_ξ
        self.e_xi_prev = copy(e_xi)
        self.d_filt = lowpass(d_raw)             # 20 Hz 低通（差分噪声不进证书）
        return self.d_filt                       # 纯诊断量，不进控制律

class PerformanceAccumulator:                   # 定理 3(c) 能量账本
    def update(self, e_xi, e_z, d_vec6, dt, d_injected=None):
        # 累计 ∫‖e_ξ‖²dt vs γ_a²∫‖d‖²dt + 2V(0)（式 (5.6)）、逐通道 (5.6')，
        # 并对照精确耗散等式 dV = −e_ξᵀK_d e_ξ + e_ξᵀ d（定理 3(c) 证明第一步）
        ...
```

---

## 13. §5.4 近恒等线性化与增益整定 → `control/gain_design.py`

**理论锚点**：式 (5.8) 两通道解耦二阶模型
$$\ddot{\mathcal O}+K_\omega\dot{\mathcal O}+\tfrac14K_{p,O}\mathcal O=-\tfrac12 d_\omega,\qquad \ddot{\mathcal T}+K_v\dot{\mathcal T}+k_{p,T}\mathcal T=+d_v$$
（1/4 旋转刚度折减：$\dot{\mathcal O}=-\tfrac12\tilde\omega$ 与 $A_0^\top$ 各贡献一个 $\tfrac12$）；极点分配规则 $K_\omega=K_v=(a+b)$、$p_T=ab$、$p_O=4ab$；静态刚度标度律 (5.9)。

```python
def channel_metrics(a1, a0, d_coeff, dt=None):  # 通道 ë + a1 ė + a0 e = d_coeff·d
    poles = roots([1.0, a1, a0])                # 极点、ζ、ωn、2% 调节时间、
    return dict(poles=..., zeta=..., static_gain=abs(d_coeff)/a0,   # 静态误差增益 (5.9)，
                pole_dt=max(abs(poles))*dt)     # 离散化余量 max|p|·Δt

def c1_channels(K_d, K_p, dt=None):             # (5.8) 逐字实现（论文 §5.4 交叉校验点）
    K_omega, K_v = dq_blocks(K_d);  p_O, p_T = dq_blocks(K_p)
    return {"rotation":    channel_metrics(K_omega, p_O / 4.0, -0.5, dt),  # p_O/4、−½
            "translation": channel_metrics(K_v,     p_T,       +1.0, dt)}  # p_T 、+1

def c3_channels(gamma_O, gamma_T, k_servo, dt=None):   # C3 + 伺服的等效通道
    kO = SQRT2/gamma_O;  kT = SQRT2/gamma_T
    return {"rotation":    channel_metrics(kO/2 + k_servo, kO*k_servo/2, -0.5, dt),
            "translation": channel_metrics(kT + k_servo,   kT*k_servo,   +1.0, dt)}

def design_from_poles(dominant, ratio=5.0):     # §5.4(ii) 极点分配规则
    p1 = abs(dominant);  p2 = ratio * p1
    return dict(K_omega=p1+p2, K_v=p1+p2, p_T=p1*p2, p_O=4.0*p1*p2)
    # tuned = {-4,-20}：K=24, p_T=80, p_O=320；fast = {-6,-30}：K=36, p_T=180, p_O=720

def design_matching_c3(gamma_O, gamma_T, k_servo):     # d→(O,T) 传递函数与 C3 恒等
    ch = c3_channels(...);  return dict(K_omega=ch.rot.a1, p_O=4*ch.rot.a0, ...)

def screen(g, dt, kappa, gamma_a, qddot_max, e_xi_ref, e_z_ref, ...):  # 可行性筛选
    K_d, K_p = gains_to_matrices(g)
    level = 0.5 * (1.0/kappa + 1.0/gamma_a**2)
    ok_cert = min(diag(K_d)) >= level           # (5.6a) 读法 A 可行性阀值
    ok_disc = max|pole|·dt <= 0.15              # 显式积分稳定裕度
    ok_damp = min(zeta_rot, zeta_trans) >= 1.0  # 接触工况无超调
    ok_eff  = λmax(K_d)|e_ξ| + ½λmax(K_p)|e_z| <= qddot_max   # 加速度预算
    return dict(feasible=all(...), cert_level=level,
                l2_certified=1.0/lam_min, ...)  # 读法 B 认证增益（与 A 分开输出）
```

`params.GAIN_SETS` 三档（§6.4 表）即由上述规则生成：`base = {K_d=8I₆, k_p=16}`（未补偿 1/4 折减的对照档）、`tuned = {K_d=24I₆, K_p=diag(320I₃,80I₃)}`、`fast = {K_d=36I₆, K_p=diag(720I₃,180I₃)}`。

---

## 14. §6.2 闭环主流水线 → `run_simulation.py`（内部后端）/ `experiments/run_grasp_circle.py`（CoppeliaSim + 各律调度）

两个主循环的控制器段**逐层同构**（论文 §6.2 流水线图）；`run_grasp_circle.py` 额外承担 §6.3 的各律调度（`--law tndq/dq-chandra/dq-ctc/dq-hinf`）、刚性附着与敏感条件。

```python
# ======================= 每控制步（dt = 5 ms） =======================
# [输入层] 传感
q, q_dot = plant_or_interface.read_joint_states()      # 编码器 + 速度观测
if noise: q_meas, qdot_meas = noise(q, q_dot)          # E6：控制器只见带噪量
else:     q_meas, qdot_meas = q, q_dot

# [FK 层] TNDQ 链（式 (3.4)）；q̈ 未知 → q̈=0 链给出 ξ 与 J̇q̇（式 (3.5)）
fk  = chain.fk_outputs(q_meas, qdot_meas, q_ddot=None, with_jacobian=True)
des = trajectory.evaluate(t)                           # 期望 TNDQ（§6 本文档）
# fk["x_breve"] = 实测 HDQ 截断（命题 2）；des["x_breve_d"] = 期望 HDQ 截断

# [误差层] 定理 1/2
err = full_error_state(fk["x_breve"], des["x_breve_d"])   # → e_ξ, e_z, A, x̃, ξ̃

# [奇异监控] σ_min(J) < 1e-3 → 阻尼 1e-6 → 5e-2（残差计入 d(t)，诚实条款 (i)）
damping = params.PINV_DAMPING
if svd(fk["J"])[-1] < params.SINGULARITY_TOL: damping = params.SINGULARITY_DAMPING

# [控制层] 各律只切换 q̈_ref 的计算分支（§6.3 公平协议）
if law == "tndq":      # C1：式 (5.2)（§8 本文档）
    qddot_ref, _ = geometric_computed_torque_law(err, des["xi_d"], des["xi_dot_d"],
                                                 fk["J"], fk["Jdot_qdot"], K_d, k_p, damping)
elif law == "dq-chandra":  # C2：忠实 [Ch20] 律，解析前馈（§9 本文档）
    qddot_ref, _ = dq_chandra2020_law(err, des["xi_d"], des["xi_dot_d"],
                                      fk["J"], fk["Jdot_qdot"], CH20_K_V, CH20_K_P, damping)
elif law == "dq-ctc":  # C2-abl：差分前馈消融律（§9b 本文档）
    qddot_ref, _ = dq_ctc_law(...)
else:                  # C3：[P2] H∞ 律 + 速度伺服桥接（§10 本文档）
    qddot_ref = velocity_to_accel_ref(damped_pinv(J) @ dq_hinf_kinematic_law(...), ...)

# [安全预算]（各律共用，§6.3 协议③）
Jp = damped_pinv(fk["J"], damping)
N  = eye(n) - Jp @ fk["J"]                             # 7R 零空间投影
qddot_ref += N @ (K_NS*(q_center - q_meas) - D_NS*qdot_meas)   # 零空间居中
if norm(qddot_ref) > QDDOT_MAX:                        # 加速度限幅 40 rad/s²
    qddot_ref *= QDDOT_MAX / norm(qddot_ref)           # 饱和残差计入 d(t)
qddot_ref, governed = joint_safety_governor(q, q_dot, qddot_ref)  # 速度限幅+限位制动

# [力矩层] 式 (5.1b)，各律同一出口（协议②：名义模型均不含杯）
tau = dyn_ctrl.computed_torque(q_meas, qdot_meas, qddot_ref)
tau, sat = clip_torque(tau)                            # 力矩饱和裁剪

# [输出层] 下发 + 推进
interface.send_joint_targets(tau, mode="torque");  interface.step()

# [监控层] d̂ 反演 + V 账本 + 残差监测 + 落盘（§12 本文档）
d_hat = d_est.update(err["e_xi"], err["e_z"], err["A"])
V     = perf.update(err["e_xi"], err["e_z"], d_hat, dt, d_injected)
if fk["c0"] > C0_TOL or fk["c1"] > C1_TOL or fk["c2"] > C2_TOL: warn()   # (3.8)
logger.log(t, e_z, e_xi, qddot_ref, tau, x_d, xi_d, xi_dot_d, x, xi, V, c0, c1, c2, ...)
```

**实验条件装配**（`run_simulation.py::build_condition`，对应 §5.1 扰动通道）：`l2`→$L_2$ 有限能量脉冲（定理 3(c)）、`bias`→$L_\infty$ 偏差（定理 3(d)）、`mismatch`→E3 参数失配（$\Delta M$）、`noise`→E6、`contact`→$\tau_{\mathrm{ext}}=J^\top[0;F_{\mathrm{ext}}]$。`run_grasp_circle.py` 的 S3 负载维：`--mode load` 在 hold 段中点 `attach_cup_rigid(CUP_LOAD_MASS=0.25)`，名义模型不含杯 → 持续偏差型 $d_{\mathrm{ex}}$（§6.1）。

---

## 15. 参数对照 → `config/params.py`

| 论文位置 | 参数 | 代码符号 | 取值 |
|---|---|---|---|
| §6.1 | LBR4+ 标准 DH 表 | `KUKA_LBR4_DH` | $d=[0.251,0,0.4,0,0.39,0,0.078]$ m（S-R-S 7R） |
| §6.4 C1 三档 | $K_d, K_p=\mathrm{diag}(p_OI_3,p_TI_3)$ | `GAIN_SETS["base"/"tuned"/"fast"]` | base：$8I_6,16$；tuned：$24I_6,(320,80)$；fast：$36I_6,(720,180)$ |
| §6.4 读法 A | $\kappa,\gamma_a$ | `KAPPA=1.0`, `GAMMA_A=0.5` | 判据阀值 $2.5\le\lambda_{\min}(K_d)$ |
| §6.4 C2 | $K_v,K_P$（忠实 [Ch20]，无折减） | `CH20_K_V=24I_6`, `CH20_K_P=80I_6` | 同预算 $\{-4,-20\}$ |
| §6.4 C2-abl | $K_d,p_O,p_T$（1/2 折减配平） | `DQC_K_D=24I_6`, `DQC_K_P=diag(160I_3,80I_3)` | 同预算 $\{-4,-20\}$ |
| §6.4 C3 | $k_O=8,k_T=4,K_{\mathrm{servo}}=20$ | `DQH_GAMMA_O=√2/8`, `DQH_GAMMA_T=√2/4`, `DQH_K_SERVO=20` | DC 刚度 80 |
| §5.1 / 诚实条款 (i) | 阻尼伪逆 $\lambda$ | `PINV_DAMPING=1e-6`（奇异时 `SINGULARITY_DAMPING=5e-2`） | — |
| §6.3 协议③ | 加速度限幅 | `QDDOT_MAX=40` rad/s² | 饱和计入 $d(t)$ |
| §6.1 | 控制步长 | `COPPELIA_DT_TARGET=5e-3`（内部 `DT=1e-3`） | 200 Hz |
| §6.3 负载 | 杯质量（未建模） | `CUP_LOAD_MASS=0.25` | 持续 $d_{\mathrm{ex}}$ 源 |
| §6.3 敏感条件 | 噪声 $\sigma_q,\sigma_{\dot q}$；降频倍数 | `NOISE_SIGMA_Q=5e-5`, `NOISE_SIGMA_QDOT=1e-3`, `GRASP_CTRL_DECIM=3` | noise / coarse-dt |
| §3.4 | 残差报警阈值 | `C0_TOL=C1_TOL=C2_TOL=1e-9`, `REPROJECT_EVERY=50` | (3.8) 监测 |

---

## 16. 一句话索引

- **算代数**（DQ/HDQ/TNDQ 乘、共轭、截断、vec6）→ `core/dq_algebra.py`、`core/tndq_algebra.py`；
- **算运动学**（链连乘 (3.4)、$\dot J\dot q$ 免构造读出 (3.5)、雅可比、约束残差 (3.8)）→ `core/kinematics.py`；
- **算误差**（定理 1/2：$\breve{\tilde x}$、$e_\xi$、$e_z$、$A$）→ `control/error_system.py`；
- **算指令**（C1 式 (5.2) / C2 忠实 [Ch20] 律 / C2-abl 差分消融 / C3 [P2]+桥接）→ `control/control_law.py`；
- **算力矩**（式 (5.1b)，RNEA 名义模型）→ `config/lbr4_dynamics.py`；
- **算证书**（定理 3：$V$、(5.6a/b)、(5.7)、$\hat d$ 反演）→ `control/performance.py`；
- **算增益**（(5.8)/(5.9) 通道模型、极点分配、可行性）→ `control/gain_design.py`；
- **跑闭环**（§6.2 流水线、各律调度、S3 实验）→ `run_simulation.py`、`experiments/run_grasp_circle.py`；
- **定参数**（DH 表、三档增益、基线参数、数值预算）→ `config/params.py`。
