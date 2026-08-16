### 4.1 DQ 误差体系在动力学环境的三个缺口

[P2] 的误差体系：右不变位姿误差 $\tilde x=\hat{\underline x}\hat{\underline x}_d^{\,*}$、误差函数 $\tilde z=1-\tilde x$、6 维输出 $e_z=[\mathcal O;\mathcal T]$（$\mathcal O=-\mathrm{Im}\,\tilde r$，$\mathcal T=\tilde p$）；扰动为速度级加性信号 $\boldsymbol v_w,\boldsymbol v_c\in L_2$。进入力矩接口时：

1. **位姿与速度误差相互分离，位姿反馈无证书结构**：须先澄清一个常见误读——[Ch20] 的 twist 误差**已经伴随映射搬运**（其式 (32) $\boldsymbol\omega_e=\mathrm{Ad}\,\boldsymbol\xi_d-\boldsymbol\xi$，与本文 (4.4) 的 $-e_\xi$ 同一），故"速度误差无几何一致定义"不是对 [Ch20] 的指控；真正的缺口有二：(a) 其位姿反馈取螺旋对数 $2\ln x_e$，该映射的导数在旋转角 $\phi\to\pi$ 处奇异（大误差工况失效），且此整形方式不能给出定理 3 所依赖的、对**任意**对称正定 $K_p$ 成立的精确耗散等式；(b) [P2] 只有位姿一层，速度误差仅隐式存在于前馈中，无独立误差通道。两者共同的结构性事实是：位姿误差与 twist 误差**分别定义、互不生成**——而它们由同一次误差乘法生成，恰是定理 2 把输出误差闭合为与原系统同型的级联运动学 $\dot e_z=A(\tilde x)e_\xi$ 的前提。另注：朴素坐标差 $\boldsymbol\xi-\boldsymbol\xi_d$（两 twist 分属不同位姿的切空间，直接相减混入伪项 $(\mathrm{Ad}_{\tilde x}-\mathrm{id})\boldsymbol\xi_d$，其范数 $\lesssim2\|\mathcal O\|\,\|\boldsymbol\xi_d\|+\|\mathcal T\|\,\|\omega_d\|$）在 [P2]、[Ch20] 中均**未被采用**，本文 §6.4 将其实现为消融档（C2-abl，仅供代码层面的结构消融，测量数据存档不报告）以量化该伪项的代价。
2. **加速度层扰动无入口**：动力学的主要不确定性（惯性参数偏差、力矩误差）作用在加速度层，[P2] 的速度级扰动通道无法表达。
3. **偏差型不确定性不满足 L₂**：$\Delta M,\Delta C,\Delta g$、摩擦残差是持续偏差，无限时域能量无穷，L₂ 增益指标对其空洞成立。


### 3.4 单位群与截断同态

单位性 $\hat{\underline x}\hat{\underline x}^*=1$ 沿曲线逐阶求导给出约束族：

$$
\hat{\underline x}\hat{\underline x}^*=1,\qquad
\dot{\hat{\underline x}}\hat{\underline x}^*+\hat{\underline x}\dot{\hat{\underline x}}^*=0,\qquad
\ddot{\hat{\underline x}}\hat{\underline x}^*+2\dot{\hat{\underline x}}\dot{\hat{\underline x}}^*+\hat{\underline x}\ddot{\hat{\underline x}}^*=0 .
\tag{3.6}
$$

三式并非三个独立条件，而是同一个代数等式按 $\sigma$ 幂次拆出的三个项：由命题 1（取 $\hat{\underline y}=\hat{\underline x}^*$）与 TNDQ 共轭的逐项定义（定义 1 后）有 $\bar x\,\bar x^{\,*}=\bar x\,\overline{(x^*)}=\overline{x x^*}$（其中 $\bar x^{\,*}=\overline{(x^*)}$，因 DQ 共轭与求导交换，逐项共轭即曲线 $x^*(t)$ 的 TNDQ 表示），其三个项依次为 (3.6) 的三个左端。故

$$
\text{(3.6) 三式同时成立}\iff \bar x\,\bar x^{\,*}=1\ \text{在}\ \mathcal A_2\ \text{中},
\tag{3.7}
$$

即位姿曲线的 TNDQ 表示落在单位群

$$
\mathcal U_2\triangleq\{\bar a\in\mathcal A_2:\bar a\bar a^{\,*}=1\}
$$

上。$\mathcal U_2$ 对乘法封闭：由共轭的反自同构性（定义 1 后），$\bar a,\bar b\in\mathcal U_2\Rightarrow(\bar a\bar b)(\bar a\bar b)^*=\bar a\bar b\bar b^{\,*}\bar a^{\,*}=\bar a\bar a^{\,*}=1$。以下补足群的其余公理，并验证 HDQ 截断诱导的群同态。

> **命题 2（HDQ 截断与乘法相容）**：定义**截断映射** $\Pi:\mathcal A_2\to$ HDQ 代数（§2.3）为
>
> $$
> \Pi\bigl(\hat{\underline a}_0+\sigma\hat{\underline a}_1+\tfrac12\sigma^2\hat{\underline a}_2\bigr)\triangleq\hat{\underline a}_0+\varepsilon^*\hat{\underline a}_1,
> \tag{3.8}
> $$
>
> 即保留前两个项、丢弃 $\sigma^2$ 项并把 $\sigma$ 改记 $\varepsilon^*$（程序上即只取前两个 DQ 数组）。则对任意 $\bar a,\bar b\in\mathcal A_2$，
>
> $$
> \Pi\bigl(\bar a\,\bar b\bigr)=\Pi(\bar a)\,\Pi(\bar b),\qquad\text{先乘后截 = 先截后乘} .
> \tag{3.9}
> $$
>
> 特别地，对曲线的 TNDQ 表示 $\bar x$（式 (3.3a)）施行截断恰得 §2.3 的 HDQ 表示：$\Pi(\bar x)=\hat{\underline x}+\varepsilon^*\dot{\hat{\underline x}}=\breve x$，且由 (3.3) 有 $\Pi(\overline{xy})=\breve x\,\breve y$。
>
> **证明**：对照 (3.2) 与 (2.3)：乘积的 $\sigma^0,\sigma^1$ 两个项只依赖两因子的 $\sigma^0,\sigma^1$ 两个项（$\sigma^2$ 项无法向低阶“回流”），且其表达式与 (2.3) 逐字相同。故先乘后取前两个项，与先取前两个项再乘，结果一致。∎
>
> 命题 2 的实际意义：**丢弃 $\sigma^2$ 项是无损操作**——只要后续运算只发生在前两个项（程序上：只取前两个数组），结果与从未携带过第三项完全一致。这为 §4 的结构性决策（误差体系定义在 HDQ 截断上）提供了严格依据。

> **命题 3（$\mathcal U_2$ 的群公理）**：$\mathcal U_2$ 在乘法 (3.2) 下构成群（封闭性已由上式计算给出）：
>
> (i) **结合律**：对任意 $\bar a,\bar b,\bar c\in\mathcal A_2$，$(\bar a\bar b)\bar c=\bar a(\bar b\bar c)$；
>
> (ii) **单位元**：$1$（即 $\hat{\underline a}_0=1$、$\hat{\underline a}_1=\hat{\underline a}_2=0$）属于 $\mathcal U_2$，且对一切 $\bar a$，$1\,\bar a=\bar a\,1=\bar a$；
>
> (iii) **逆元**：$\bar a\in\mathcal U_2\Rightarrow\bar a^{\,*}\in\mathcal U_2$ 且 $\bar a^{\,*}\bar a=\bar a\bar a^{\,*}=1$。
>
> **证明**：(i) 把 (3.2) 读作截断多项式乘法：记 $\hat{\underline a}_0'=\hat{\underline a}_0$、$\hat{\underline a}_1'=\hat{\underline a}_1$、$\hat{\underline a}_2'=\tfrac12\hat{\underline a}_2$，则 $\bar a=\sum_{m=0}^{2}\sigma^m\hat{\underline a}_m'$，且乘积的 $\sigma^m$ 项为 $\sum_{k+l=m}\hat{\underline a}_k'\hat{\underline b}_l'$。于是 $(\bar a\bar b)\bar c$ 与 $\bar a(\bar b\bar c)$ 的 $\sigma^m$ 项同为
>
> $$
> \sum_{k+l+j=m}\hat{\underline a}_k'\hat{\underline b}_l'\hat{\underline c}_j',
> $$
>
> 双重求和与加括号方式无关、各项内 DQ 乘法结合，故对每个 $m$ 逐项相等（$m>2$ 的项因 $\sigma^3=0$ 而一律为零，两侧截断一致）。
>
> (ii) 直接代入 (3.2)：$\sigma^0,\sigma^1,\sigma^2$ 项分别为 $\hat{\underline a}_0,\hat{\underline a}_1,\tfrac12\hat{\underline a}_2$，即 $1\,\bar a=\bar a$；$\bar a\,1=\bar a$ 同理。又 $1^{\,*}=1$，故 $1\in\mathcal U_2$。
>
> (iii) 共轭是对合：逐项定义给出 $\bar a^{\,**}=\bar a$。由 (i) 与 $\bar a\in\mathcal U_2$，
> $$
> \bar a\,(\bar a^{\,*}\bar a)=(\bar a\bar a^{\,*})\,\bar a=\bar a .
> $$
> 设 $\bar a^{\,*}\bar a=1+\sigma p+\tfrac12\sigma^2 q$（$p,q$ 为一般 DQ；$\sigma^0$ 项为 $1$，因 $\hat{\underline a}_0^{\,*}\hat{\underline a}_0=1$——由 $\bar a\bar a^{\,*}=1$ 的 $\sigma^0$ 项知 $\hat{\underline a}_0$ 是单位 DQ），代入上式并对照 (3.2)：$\sigma^1$ 项给 $\hat{\underline a}_0p=0$，$\sigma^2$ 项给 $\hat{\underline a}_0q+2\hat{\underline a}_1p=0$。单位 DQ 可逆，故 $p=q=0$，即 $\bar a^{\,*}\bar a=1$。再由对合性，$\bar a^{\,*}(\bar a^{\,*})^{\,*}=\bar a^{\,*}\bar a=1$，故 $\bar a^{\,*}\in\mathcal U_2$，$\bar a^{\,*}$ 是 $\bar a$ 的双侧逆元。∎
>
> 群结构使 (3.6) 沿链自动保持：各关节因子的 TNDQ 表示逐个属于 $\mathcal U_2$（附录 B.1），按 (3.4) 连乘得到的链 TNDQ 由封闭性自动属于 $\mathcal U_2$——与命题 1 互相印证。

> **命题 4（HDQ 截断是群同态）**：$\Pi(\mathcal U_2)$ 恰为单位 HDQ 群 $\mathcal U_1\triangleq\{\breve a:\breve a\breve a^{\,*}=1\}$；配合命题 2 的乘法相容性，$\Pi|_{\mathcal U_2}:\mathcal U_2\to\mathcal U_1$ 是群同态（保单位元与保逆元：由 (3.8) 有 $\Pi(1)=1$；TNDQ 与 HDQ 的共轭同为逐项 DQ 共轭，故 $\Pi(\bar a^{\,*})=\Pi(\bar a)^{\,*}$）。
>
> **证明**：包含方向。$\bar a\bar a^{\,*}=1$ 的 $\sigma^0$ 项为 $\hat{\underline a}_0\hat{\underline a}_0^{\,*}=1$、$\sigma^1$ 项为 $\hat{\underline a}_0\hat{\underline a}_1^{\,*}+\hat{\underline a}_1\hat{\underline a}_0^{\,*}=0$（即 (3.6) 的前两式），代入 (2.3) 与 (2.4)，
> $$
> \Pi(\bar a)\,\Pi(\bar a)^{\,*}=\hat{\underline a}_0\hat{\underline a}_0^{\,*}+\varepsilon^*\bigl(\hat{\underline a}_0\hat{\underline a}_1^{\,*}+\hat{\underline a}_1\hat{\underline a}_0^{\,*}\bigr)=1+\varepsilon^*\cdot0=1 .
> $$
> 满射方向。给定单位 HDQ $\breve a=\hat{\underline a}_0+\varepsilon^*\hat{\underline a}_1$（即 $\hat{\underline a}_0\hat{\underline a}_0^{\,*}=1$、$\hat{\underline a}_0\hat{\underline a}_1^{\,*}+\hat{\underline a}_1\hat{\underline a}_0^{\,*}=0$），需构造 $\hat{\underline a}_2$ 使 $\bar a=\hat{\underline a}_0+\sigma\hat{\underline a}_1+\tfrac12\sigma^2\hat{\underline a}_2\in\mathcal U_2$，即满足 (3.6) 第三式。取
> $$
> \hat{\underline a}_2=-\hat{\underline a}_1\hat{\underline a}_1^{\,*}\hat{\underline a}_0 .
> \tag{3.10}
> $$
> 由 $\hat{\underline a}_0\hat{\underline a}_0^{\,*}=1$ 与 DQ 共轭的反自同构性，$\hat{\underline a}_0\hat{\underline a}_2^{\,*}=-\hat{\underline a}_0\hat{\underline a}_0^{\,*}\hat{\underline a}_1\hat{\underline a}_1^{\,*}=-\hat{\underline a}_1\hat{\underline a}_1^{\,*}$、$\hat{\underline a}_2\hat{\underline a}_0^{\,*}=-\hat{\underline a}_1\hat{\underline a}_1^{\,*}\hat{\underline a}_0\hat{\underline a}_0^{\,*}=-\hat{\underline a}_1\hat{\underline a}_1^{\,*}$，代入 (3.6) 第三式左端得 $-\hat{\underline a}_1\hat{\underline a}_1^{\,*}+2\hat{\underline a}_1\hat{\underline a}_1^{\,*}-\hat{\underline a}_1\hat{\underline a}_1^{\,*}=0$，故 $\bar a\in\mathcal U_2$。∎（提升不唯一：约束 (3.6) 第三式只固定 $\hat{\underline a}_2\hat{\underline a}_0^{\,*}$ 的自共轭部分，其纯部自由，对应 $\sigma^2$ 项的一族可调自由度；命题只断言存在性。）

(3.7) 解析上恒成立；数值积分会使其漂移。定义残差 $c_0=\|\hat{\underline x}\hat{\underline x}^*-1\|$，$c_1=\|\mathrm{Sc}(2\dot{\hat{\underline x}}\hat{\underline x}^*)\|$（$\mathrm{Sc}$ 取标量与对偶标量部）作为逐周期 $O(1)$ 监测量；超阈值触发重投影（0 阶归一化、1 阶按 $\dot{\hat{\underline x}}\mapsto\tfrac12\boldsymbol\xi_{\mathrm{proj}}\hat{\underline x}$ 重构）。前馈侧若使用 $\sigma^2$ 项可另监测

$$
c_2=\bigl\|\mathrm{Sc}(2\ddot{\hat{\underline x}}\hat{\underline x}^*)-\tfrac12\mathrm{Sc}(\boldsymbol\xi^2)\bigr\|
=\bigl\|\mathrm{Sc}(2\ddot{\hat{\underline x}}\hat{\underline x}^*)+\tfrac12\mathrm{Sc}(\boldsymbol\xi\boldsymbol\xi^*)\bigr\|
=\bigl\|\mathrm{Sc}(\dot{\boldsymbol\xi})\bigr\| ,
$$

其中第一个等号用 $\boldsymbol\xi\boldsymbol\xi^*=-\boldsymbol\xi^2$（$\boldsymbol\xi$ 为纯元），第二个等号即 (3.5) 第二式——(3.6) 的二阶约束经 $2\dot{\hat{\underline x}}\dot{\hat{\underline x}}^*=\tfrac12\boldsymbol\xi\boldsymbol\xi^*$ 化简后与之同一。沿真曲线 $\dot{\boldsymbol\xi}$ 为纯元故 $c_2$ 解析为零。超阈值时 2 阶按 $\ddot{\hat{\underline x}}\mapsto\tfrac12\bigl(\dot{\boldsymbol\xi}_{\mathrm{proj}}+\tfrac12\boldsymbol\xi_{\mathrm{proj}}^2\bigr)\hat{\underline x}$ 重构（由 (3.5) 第二式反解）。

---


## 6. 仿真验证

本章在 CoppeliaSim 物理仿真中检验第 3–5 章的理论主张，章节体例参照 [P2]（§5 Simulation results）与 [Ch20]（§4 Experimental Validation）：先给出平台与模型（§6.1）、信息流水线（§6.2），再描述 S3 抓取-搬运实验设计与公平对比协议（§6.3）、控制器与参数设置（§6.4），随后给出定量结果与分析（§6.5）并小结（§6.6）；针对 §5.3 注记 $\gamma_a$ 双分量推论的 γ 扫描协议作为后续实验设计列于 §6.7。全部数值摘自仿真运行导出的原始数据（`TNDQ_sim/results/grasp_metrics_summary.csv`），未做修饰。

### 6.1 平台与机器人模型

**平台**：CoppeliaSim（原 V-REP [Roh13]）中的 7 自由度 KUKA LBR4+ 轻量臂，末端装 RG2 二指夹爪；控制律以**力矩模式**直接下发关节力矩，控制/物理步长 dt = 5 ms（200 Hz，与 [Ch20] 的 Baxter 实验控制频率一致），单次实验 22.5 s、共 4500 个控制步。

**机器人模型**：LBR4+ 采用修正 DH 参数建模（S-R-S 构型，连杆偏置 $d=[0.251,\,0,\,0.4,\,0,\,0.39,\,0,\,0.078]$ m），动力学参数取自公开辨识结果 [Gaz14]，构成控制器内部的名义模型 $\hat M(\boldsymbol q),\hat C(\boldsymbol q,\dot{\boldsymbol q}),\hat g(\boldsymbol q)$。正运动学按 §3 的 TNDQ 链 (3.4) 实现：一次连乘同时产出位姿 $\hat{\underline x}$、twist $\boldsymbol\xi=\overline{\mathrm{vec}}_6(J\dot{\boldsymbol q})$ 与二阶读出 $\dot J\dot{\boldsymbol q}$（式 (3.5)，免于显式构造 Hessian 或数值差分），后者是控制律 (5.2) 前馈项的关键输入。

**负载对象**：被抓取对象为圆柱形水杯（质量 m = 0.25 kg），t = 2.5 s 闭爪后刚性附着于末端。关键设定：名义模型**不包含杯的动力学**，因此带载后 $\Delta M,\Delta g$ 构成真实的持续模型失配扰动，用以检验 §5.4 的静态刚度标度律（式 (5.9)）与定理 3(d) 的均方极限界（式 (5.7)）。

### 6.2 信息流水线（输入 → 计算 → 输出 → 反馈闭环）

```
输入层（每周期）
  ├─ 关节编码器 → q ∈ R⁷
  ├─ 速度估计（同一观测器）→ q̇ ∈ R⁷
  └─ 期望轨迹发生器（解析）→ x_d(t), ẋ_d(t), ξ_d(t), ξ̇_d(t)

正运动学层（TNDQ 链，O(n)）
  ├─ 实测链：x̄ = Π x̄_i(q,q̇,q̈=0) → 项读数 x, ẋ；(3.5) 免构造读出 J̇q̇
  ├─ HDQ 截断：取前两项 → x̆ = x + ε*ẋ（命题 2：无损）
  └─ 雅可比 J(q)（[P1] 前缀/后缀结构）；约束残差 c₀,c₁,c₂ 监测（§3.4，σ² 项在用故含 c₂）

误差层（HDQ 运算，定理 1/2）
  ├─ 一次 HDQ 乘法：x̆̃ = x̆·(x̆_d)* → x̃, dx̃/dt（3 次 DQ 乘）
  ├─ e_z = [O; T]（0 阶项，[P2] 原样）
  ├─ e_ξ = vec₆(2·dx̃/dt·x̃*)（几何一致 twist 误差）
  └─ A(x̃) 拼装（定理 2 的 3×3 块）

控制层（(5.2)，定理 3）
  ├─ 前馈：vec₆(Ad_x̃ ξ̇_d + ad_ξ̃ Ad_x̃ ξ_d)（引理 1；全为 DQ 乘法）
  ├─ 反馈：−K_d e_ξ − Aᵀ K_p e_z（K_p 在 Aᵀ 内侧，§5.2 说明 (c)）
  ├─ q̈_ref = J⁺(前馈 + 反馈 − J̇q̇)
  └─ τ = M̂ q̈_ref + Ĉ q̇ + ĝ（标称模型）

输出层
  └─ τ → CoppeliaSim 关节力矩接口（力矩模式）

反馈闭环
  └─ 仿真器推进一步 → 新的 q, q̇ 回到输入层；全部日志（e_z, e_ξ, V, c₀, c₁, τ, 运行时间）落盘
```

### 6.3 S3 抓取-搬运实验设计

**任务与目的**。S3 实验（抓取–搬运–圆周跟踪）在包含接触、负载突变与持续动态跟踪的物理交互场景中验证四点：(i) 控制律 (5.2) 的全相位闭环稳定性与误差收敛，并核验定理 3(b) 的水平集条件 (5.5b) 是否实际满足；(ii) 未建模负载这一持续扰动下定理 3(d) 的**均方极限界** (5.7) 与 §5.4 的**静态刚度标度律** (5.9)；(iii) 与两类代表性 DQ 基线在严格公平协议下的性能对比；(iv) H∞ 证书 (5.6a) 增益下界在含噪/高速/粗采样条件下的保守性。

**七相位时间线**。参考轨迹由七个相位以五次多项式平滑串接，工具姿态全程保持竖直向下：descend（下探至抓取位，[0, 2.0] s）→ hold（保持，t = 2.5 s 闭爪并刚性附着——负载突变，[2.0, 3.5] s）→ lift（垂直提升）→ retreat（水平后撤）→ transit（搬运至作业区上方）→ descend2（下探至圆心高度，至 9.5 s）→ circle（持载圆周跟踪 >1.5 圈，[9.5, 22.5] s）。圆周段半径 R = 0.06 m、角速度 ω = 1.0 rad/s（标准）/ 2.5 rad/s（高速条件），起始 2 s 内角速度按五次多项式平滑爬升；稳态统计窗（circle-ss）取 $t\ge12.5$ s。

**实验因子与运行清单**。因子为负载（noload / load）× 控制律（C1 / C2 / C3，§6.4）× 增益档（base / tuned / fast，仅对 C1）× 敏感条件（none / highspeed / fast-transit / noise / coarse-dt）。因子未作全叉乘（完全交叉为 2\times5\times3\times3=90 组），而是围绕三个对比目的取子集，实际完成 **43 组运行**（`grasp_metrics_summary.csv` 中的 43 个 $(\text{law},\text{gains},\text{mode},\text{condition})$ 组合）：

| 子集 | 组数 | 成分 | 用途 |
|---|---|---|---|
| C1 增益档扫描 | 3 | tndq × base × {noload, load}，none；tndq × fast × load，none | 验证 (5.9) 反比标度、1/4 折减与 (P2) 反演一致性 |
| 各律全条件对比 | 30 | {tndq-tuned (C1), dq-chandra (C2), dq-hinf (C3)} × {noload, load} × {none, highspeed, fast-transit, noise, coarse-dt} | 公平协议下的 10 组两两对比（5 条件 × 2 基线，带载）与空载基线校验 |
| C2-abl 消融档归档 | 10 | dq-ctc (C2-abl) × {noload, load} × 5 条件 | 仅仓库存档（`results/grasp_circle_dqctc_*`），不进入本章正文表格 |

（合计 3+30+10=43。）本章 §6.5 的定量结论只基于前两子集的 **33 组**（C1/C2/C3）；C2-abl 为 §6.4 定义的朴素消融律、非文献律，其测量数据存档备查，不作为对比基线报告。

**公平对比协议**。为使各律差异仅来自**误差几何与前馈构造**本身，强制四项共用机制：① 同一参考轨迹与初始条件（同一 $x_d,\boldsymbol\xi_d,\dot{\boldsymbol\xi}_d$ 序列）；② 同一力矩出口 $\boldsymbol\tau=\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+\hat C\dot{\boldsymbol q}+\hat g$，名义模型均不含杯——任何控制律不享有额外模型信息；③ 同一安全预算（阻尼伪逆、零空间治理器、加速度范数限幅 40 rad/s²、力矩饱和裁剪）；④ 同一量测噪声注入与同一指标计算脚本。各律仅切换 $\ddot{\boldsymbol q}_{\mathrm{ref}}$ 的计算分支——较 [P2] "同轨迹换控制律"的协议更进一步，把动力学出口也统一，排除运动学律与动力学律比较时的内环差异干扰。

**敏感条件**。标准工况下各律稳态差异极小（§6.5 的"准静态趋同"现象），为曝光结构性差异追加四个应力条件：highspeed（ω→2.5 rad/s，向心加速度前馈需求放大 6.25 倍，考验 $\dot{\boldsymbol\xi}_d$ 与 $\dot J\dot{\boldsymbol q}$ 的前馈质量）、fast-transit（搬运四段 lift/retreat/transit/descend2 时长 ×0.5，路标几何不变，考验快相位下的前馈精度）、noise（关节测量高斯噪声 $\sigma_q=5\times10^{-5}$ rad、$\sigma_{\dot q}=10^{-3}$ rad/s，考验差分类前馈（仅 C3 的桥接项）的噪声放大；C1/C2 均为解析前馈）、coarse-dt（控制更新 5→15 ms 降频 3 倍，考验离散化滞后敏感性）。

### 6.4 控制器与参数设置

**C1（本文，式 (5.2)）**：$e_\xi,e_z,A(\tilde x)$ 按定理 1/2（式 (4.1)–(4.5)）计算，$\dot J\dot{\boldsymbol q}$ 由 TNDQ 链解析读出（式 (3.5)）；位姿增益按附录 C.3 ③ 推广为对称正定矩阵 $K_p$。三档增益：

| 档位 | $K_d$ | $K_p=\mathrm{diag}(p_OI_3,p_TI_3)$ | 有效旋转刚度 $p_O/4$ | 设计极点（平移） | 备注 |
|---|---|---|---|---|---|
| base | $8I_6$ | $16I_6$（$p_O=p_T=16$） | **4**（与 $p_T=16$ 失配） | $\{-4,-4\}$ 临界 | 未整定对照档 |
| tuned | $24I_6$ | $p_O=320,\ p_T=80$ | 80（与 $p_T$ 配平） | $\{-4,-20\}$ | 主推档 |
| fast | $36I_6$ | $p_O=720,\ p_T=180$ | 180（与 $p_T$ 配平） | $\{-6,-30\}$ | 刚度上限档 |

增益由 §5.4 的极点分配规则生成（$K=(a+b)I$、$p_T=ab$、$p_O=4ab$）：旋转分量方程 (5.8) 含 $p_O/4$ 项（$A_0$ 引入的 **1/4 旋转刚度折减**），故 tuned 档取 $p_O=4p_T=320$ 使有效旋转刚度与平移刚度同为 80；base 档故意不补偿（$p_O=p_T=16$，有效旋转刚度仅 4），用以暴露折减缺陷（§6.5(2)）。三档的 $K_{p,T}$ 均为标量阵，满足定理 3(c-2) 分量拆分所需的各向同性条件。

**水平集条件的预核验**：定理 3(b) 要求 $c<c^*=\tfrac12\lambda_{\min}(K_{p,O})$。tuned 档 $c^*=160$、fast 档 $c^*=360$、base 档 $c^*=8$——即使 base 档也远大于实测的 $V$ 峰值（§6.5(5)），工作域假设在全部运行中成立。

**C2（忠实 [Ch20] resolved-acceleration 律，二阶基线）**：按原文式 (32)–(35) 与式 (2) 逐项移植——twist 误差取**经伴随搬运**的差 $\boldsymbol\omega_e=\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d-\boldsymbol\xi=-e_\xi$（式 (32)，与定理 1 的 $-e_\xi$ 同一）；加速度指令为
$$\boldsymbol a_{\mathrm{cmd}}=\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}(\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d)+K_v\boldsymbol\omega_e-K_P\,\mathrm{vec}_6(2\ln\tilde x),\qquad \boldsymbol u_{\mathrm{task}}=\boldsymbol a_{\mathrm{cmd}}-\dot J\dot{\boldsymbol q},$$
其中前两项即引理 1 的前馈（与 (5.2) 前馈**逐项相同**——[Ch20] 式 (33)/(34) 对 $\frac{d}{dt}\mathrm{Ad}$ 的展开与引理 1 同一），位姿反馈为原文式 (35) 的**螺旋对数整形** $K_P$ 作用于 $2\ln x_e$：原文的 $x_e$ 与本文右不变误差 $\tilde x$ 取向相反（原文约定下 $\frac{d}{dt}(2\ln x_e)=\boldsymbol\omega_e$，本文约定下 $\frac{d}{dt}\mathrm{vec}_6(2\ln\tilde x)=e_\xi=-\boldsymbol\omega_e$），故翻译到本文约定后符号取负（已由闭环消去 oracle 测试锁定：`tests/test_math_properties.py::test_chandra20_law_oracle`，取正号闭环发散、取负号收敛）。信息集按原文：$\dot{\boldsymbol\xi}_d$ 与 $\dot J\dot{\boldsymbol q}$ 均为**解析量**（期望链 $\sigma^2$ 项 / (3.5) 免构造读出），无差分。**与 C1 (5.2) 的唯一结构差异是位姿反馈形式**：螺旋对数整形 vs $A^\top$ 整形——前者近恒等时 $\mathrm{vec}_6(2\ln\tilde x)\to[-2\mathcal O;\mathcal T]$，但导数映射在 $\phi\to\pi$ 奇异（E4 大误差真判别项），且无对任意 $K_p$ 成立的精确耗散等式（无定理 3 证书）。

> **配平（忠实 C2）**：近恒等线性化逐分量为 $\ddot{\boldsymbol\ell}+K_v\dot{\boldsymbol\ell}+K_P\boldsymbol\ell=0$（$\boldsymbol\ell\equiv\mathrm{vec}_6(2\ln\tilde x)$；旋转分量 $\phi\boldsymbol n\approx2\,\mathrm{Im}\,\tilde r=-2\mathcal O$ 自带因子 2，**无** C1 的 $1/4$ 折减、也无下文 C2-abl 的 $1/2$ 折减）。与 C1-tuned 极点 $\{-4,-20\}$（特征多项式 $(s+4)(s+20)$）、DC 刚度 80 对齐得 $K_v=24I_6$、$K_P=80I_6$（代码 `config/params.py::CH20_K_V/CH20_K_P`）。

**C2-abl（朴素 twist 差消融基线，非文献律）**：twist 误差取坐标差 $\boldsymbol\xi_d-\boldsymbol\xi$（不经 $\mathrm{Ad}_{\tilde x}$ 搬运），前馈 $\dot{\boldsymbol\xi}_d$ 与 $\dot J\dot{\boldsymbol q}$ 由数值差分获得，无 $A^\top$ 几何整形项。须明示其文献地位：[Ch20] 式 (32) 与 [P2] 前馈均含 Ad 搬运，**朴素坐标差不对应任何已发表理论**，本档仅用于消融 C1 自身结构（§4.1 伪项、$A^\top$ 整形、解析前馈三项属性的代价量化）。本稿早期版本曾把该朴素律标注为"[Ch20] 类"并以其实测数据充当 C2 基线，与原文不符，现更正：[Ch20] 的忠实代表是上文的 C2（dq-chandra），本章正文的全部 C2 数值均采自 dq-chandra 运行（`results/grasp_circle_chandra_*.npz`）；本档更名 C2-abl（dq-ctc），其测量数据仅存档于仓库，不进入 §6.5 表格。

> **折减因子不同，必须分别配平（更正）**：C1 的旋转刚度折减是 $1/4$，来自两个独立的 $\tfrac12$——$\dot{\mathcal O}=-\tfrac12\tilde{\boldsymbol\omega}$（$A_0$ 第一行）与 $A_0^\top$ 作用在位姿反馈上的 $\tfrac12$（见 (5.8)）。C2-abl 没有 $A^\top$ 整形项，位姿反馈直接取 $[+p_O\mathcal O;-p_T\mathcal T]$，因此只剩前一个 $\tfrac12$，其近恒等线性化为
> $$\ddot{\mathcal O}+K_\omega\dot{\mathcal O}+\tfrac{p_O}2\mathcal O=-\tfrac12d_\omega,\qquad \ddot{\mathcal T}+K_v\dot{\mathcal T}+p_T\mathcal T=d_v .$$
> 折减因子是 $1/2$ 而非 $1/4$。故与 C1-tuned 配平到同一有效刚度 80、同一特征多项式 $(s+4)(s+20)$ 所需的 C2-abl 增益是 $K_d=24I_6$、$p_O=\mathbf{160}$、$p_T=80$（代码 `config/params.py::DQC_K_D/DQC_K_P` 即取此值）。本稿早期版本写作“增益按**同样的** 1/4 折减规则配平”，与实现不符（若真按 1/4 规则取 $p_O=320$，C2-abl 的有效旋转刚度将是 160，即两倍于 C1/C3，对比不再同预算），现更正。

配平后各律（C1-tuned / C2 / C2-abl / C3）的旋转/平移 DC 刚度同为 80、名义 $d\to(\mathcal O,\mathcal T)$ 传递函数逐分量相同，闭环极点均为 $\{-4,-20\}$。

> **数据版本说明**：本章全部数值采自忠实 C2（dq-chandra）补跑到位后的最新一批仿真（`grasp_metrics_summary.csv` 的全部 43 个 $(\text{law},\text{gains},\text{mode},\text{condition})$ 组合，含 `results/grasp_circle_chandra_*.npz`）；早期版本中以 C2-abl（dq-ctc）数据标注为"C2"的全部数值已删除或替换，不再出现在本章任何表格与分析中。

**C3（DQ-H∞ + 加速度桥接，一阶基线）**：忠实移植 [P2] 式 (12) 的 H∞ 运动学律（$k_O=\sqrt2/\gamma_O=8$、$k_T=\sqrt2/\gamma_T=4$），经内环速度伺服（$K_{\mathrm{servo}}=20$，含一拍差分）桥接到加速度级，等效级联极点亦为 $\{-4,-20\}$。至此各律 DC 刚度均为 80——标准工况线性化意义下增益完全配平，对比聚焦于结构差异。

**H∞ 证书参数（两种读法，不可混用）**。本稿早期版本写作“取 $\kappa=1.0,\gamma_a=0.5$，… tuned 档认证 $L_2$ 增益上界 $1/\lambda_{\min}(K_d)=1/24$”，这两句互不相容（前者认证的增益是 $\gamma_a\sqrt\kappa=0.5$，而非 $1/24$），现分列为两个独立的读法：

- **读法 A（可行性判定，$\theta=\tfrac12$ 成员）**：取 $\kappa=1.0$、$\gamma_a=0.5$，则 (5.6a) 要求 $\lambda_{\min}(K_d)\ge\tfrac12(\kappa^{-1}+\gamma_a^{-2})=\tfrac12(1+4)=2.5$，三档增益（8/24/36）均满足；此时被认证的 $L_2$ 能量增益是 $\gamma_a\sqrt\kappa=\mathbf{0.5}$。这一读法回答“给定的 $(\kappa,\gamma_a)$ 目标能否被现有增益认证”，但它并非族内最紧（因 $\kappa\ne\gamma_a^2$）。
- **读法 B（族内最紧增益，$\theta=\theta^*$）**：若目标是从给定 $K_d$ 反推**最小可证增益**，则应取 AM–GM 等号 $\kappa=\gamma_a^2$ 并使最紧条件 $\lambda_{\min}(K_d)\ge1/(\gamma_a\sqrt\kappa)=\gamma_a^{-2}$ 取等；tuned 档 $\lambda_{\min}(K_d)=24$ 对应 $\gamma_a=1/\sqrt{24}\approx\mathbf{0.204}$、$\kappa=\gamma_a^2=1/24\approx0.0417$，认证 $L_2$ 增益 $=1/\lambda_{\min}(K_d)=1/24\approx\mathbf{0.042}$。下文 §6.5(5) 引用的 0.042 指的是本读法。

两种读法均不影响闭环轨迹（$\kappa,\gamma_a$ 是分析参数，不进控制律），只影响“声称了什么”；代码侧 `control/gain_design.py` 的 `screen()` 按读法 A 计算可行性阀值 `level = 0.5*(1/kappa + 1/gamma_a**2)`，并另给 `l2_certified = 1.0/lam_min` 对应读法 B，两者在代码中已分开输出。

### 6.5 结果与分析

指标按相位统计：平移/姿态误差 $\|\mathcal T\|,\|\mathcal O\|$ 的 RMS、twist 误差 $\|e_\xi\|$ 的 RMS、关节力矩范数 $\tau_{\mathrm{rms}}$、Lyapunov 值 $V$。

> **$V$ 的权重口径（重要）**：日志中的 `V_ss`/`V_peak` 对**全部增益档**统一采用 **base 档权重** $K_p^{\mathrm{base}}=16I_6$ 记录，即 $V^{\mathrm{base}}=\tfrac12\|e_\xi\|^2+8\|e_z\|^2$（已由三档数据逐一复算核实，吻合到三位有效数字）。因此在把实测 $V$ 与定理 3(b) 的水平集阈值 $c^*=\tfrac12\lambda_{\min}(K_{p,O})$ 比较时**必须换算**：对 tuned 档（$K_p=\mathrm{diag}(320I_3,80I_3)$）有 $V^{\mathrm{tuned}}\le(320/16)\,V^{\mathrm{base}}=20\,V^{\mathrm{base}}$。

**(1) 空载基线：实现正确性与协议无偏**。空载（名义模型准确、无失配扰动）circle-ss 下三律误差均进入 $10^{-4}$ m / $10^{-5}$ rad 量级（C1-tuned：$\|\mathcal T\|_{\mathrm{rms}}=1.355\times10^{-4}$ m、$\|\mathcal O\|_{\mathrm{rms}}=8.70\times10^{-5}$），三律 $\|e_\xi\|_{\mathrm{rms}}$ 为 **1.532 / 1.532 / 1.502**（×10⁻⁴，C1-tuned / C2 / C3）。两个事实值得记录：其一，**C1 与忠实 C2 在五位有效数字内完全重合**（相对差 0.000%）——两律同为解析前馈 + Ad 搬运 twist 误差的二阶律，信息集相同，唯一差异是位姿反馈整形（$A^\top$ vs 螺旋对数），而后者在近恒等极限下只差高阶项，空载小误差工况的数值重合正是 §6.4 结构分析的定量印证；其二，**C1（与 C2）比 C3 高 1.98%**，即空载工况下 C1 并非最优。这与 §4–5 的机理分析并不冲突——空载时名义模型精确、$d_{\mathrm{ex}}\approx0$，残差由离散化与数值精度主导，二阶解析前馈链（$\mathrm{Ad}$/$\mathrm{ad}$ 与 TNDQ 二阶项，C1/C2 共有）比一阶桥接引入更多浮点运算，在扰动趋零的极限下其结构优势无从体现、舍入代价反而显露。C1 的优势只应在**存在真实扰动**时被主张（见 (3)(4)）。C1-base 低增益档（$\|\mathcal T\|_{\mathrm{rms}}=6.32\times10^{-4}$ m、$\|e_\xi\|_{\mathrm{rms}}=3.64\times10^{-4}$）亦稳定收敛，与定理 3(b) 的无扰渐近收敛/局部指数稳定一致。

**(2) 带载：静态刚度标度律的两级检验**。带载（0.25 kg 未建模杯，构成持续偏差型 $d_{\mathrm{ex}}$）C1 三档 circle-ss 稳态：

| 档位 | $\|\mathcal T\|_{\mathrm{rms}}$ (m) | $\|\mathcal O\|_{\mathrm{rms}}$ | $\tau_{\mathrm{rms}}$ (N·m) | $V_{ss}$ |
|---|---|---|---|---|
| base | $1.582\times10^{-2}$ | $5.262\times10^{-2}$ | 21.08 | $2.42\times10^{-2}$ |
| tuned | $4.859\times10^{-3}$ | $4.270\times10^{-3}$ | 19.20 | $3.36\times10^{-4}$ |
| fast | $2.201\times10^{-3}$ | $1.934\times10^{-3}$ | 19.19 | $6.89\times10^{-5}$ |

本节检验的对象是 §5.4 的**静态刚度标度律** (5.9)，而不是（已被撤回的）逐点 ISS 极限球。(5.9) 给出两个可伪造预言：

**(P1) 反比标度**（弱检验）：$\|\mathcal T\|_{\mathrm{ss}}\propto1/k_{p,T}$。tuned→fast 的刚度比 $80/180=0.4444$，实测残差比 $2.201/4.859=0.4530$，相符至 **1.9%**。此检验较弱，因为它只要求两点落在同一条反比线上，任何单调递减的刚度–误差关系都会给出定性正确的方向。

**(P2) 等效扰动的反演一致性**（强检验）：由 (5.9) 反解出的等效扰动幅值必须**在两档之间一致**，因为 $d_{\mathrm{ex}}$ 是物理量（未建模杯的重力/惯性效应），与控制增益无关。反演结果：

| 反演量 | tuned 档 | fast 档 | 相对偏差 |
|---|---|---|---|
| $\|d_v\|=k_{p,T}\|\mathcal T\|_{\mathrm{ss}}$ | $80\times4.859\times10^{-3}=0.3887$ | $180\times2.201\times10^{-3}=0.3962$ | **1.93%** |
| $\|d_\omega\|=\tfrac12\lambda(K_{p,O})\|\mathcal O\|_{\mathrm{ss}}$ | $\tfrac12\times320\times4.270\times10^{-3}=0.6832$ | $\tfrac12\times720\times1.934\times10^{-3}=0.6964$ | **1.93%** |

两个分量各自独立反演出的扰动幅值一致到 1.93%（与 (P1) 的偏差同值），且**两分量的偏差量恰好相同**——这正是 (5.8)/(5.9) 所预言的（两分量共用同一组极点比例，二阶残差以相同比例进入两式）。这是一个真正有伪造风险的检验：若 (5.9) 中的 1/4 旋转折减因子写错（例如漏掉 $\tfrac12$ 而用 $\lambda(K_{p,O})\|\mathcal O\|$），旋转分量反演值将变为 1.366/1.393，与平移分量的 0.389 相差 3.5 倍，量纲虽仍可辩（rad/s² vs m/s²）但两档一致性会被破坏；实测的两个分量同步一致构成对折减因子的独立确认。

**(3) base 档：折减缺陷的曝光与 (5.9) 适用域的边界**。base 档取 $p_O=p_T=16$（未补偿 1/4 折减，有效旋转刚度仅 **4**），带载姿态误差 $5.262\times10^{-2}$ 被放大至 tuned 档的 **12.3 倍**，且是 43 组运行中唯一触发零空间治理器达 3 步的组（另有 C3 与 C2-abl 的 load/fast-transit 组各触发 1 步，见 (7)）。按 (5.9) 反演 base 档得 $\|d_v\|=16\times1.582\times10^{-2}=0.253$、$\|d_\omega\|=\tfrac12\times16\times5.262\times10^{-2}=0.421$——与 tuned/fast 档的 0.389/0.683 相差 35%/38%，**明显不一致**。这不是 (5.9) 的反例，而是其适用域的界定：base 档误差已达 $\|\mathcal T\|\sim1.6$ cm、$\|\mathcal O\|\sim5\times10^{-2}$，近恒等假设 $\tilde x\approx1$（(5.8) 的推导前提）的一阶余项不再可忽略，且治理器触发意味着实际执行的加速度指令已被安全层修改、不再是 (5.2)。因此 (5.9) 的定量反演只在 tuned/fast 这类"小误差 + 无安全层干预"的档位有效；这一限制在 §5.4 已预先声明，此处得到实验确认。三档 $\tau_{\mathrm{rms}}$ 为 21.08/19.20/19.19 N·m，差 <10%（力矩主体为重力补偿，提高反馈刚度不显著增加控制 effort）。

**(4) 三律公平对比：准静态趋同与速度级分化**。标准工况带载 circle-ss，三律 $\|\mathcal T\|_{\mathrm{rms}}$ 为 4.859/4.859/4.861（×10⁻³ m，C1/C2/C3），位置级差异 <0.1%。原因：稳态残差由 (5.9) 的"静态刚度 × 恒定重力失配"主导，而三律 DC 刚度已刻意配平至 80；这一趋同本身是协议无偏的有力证据——若存在隐藏偏袒，配平后不可能三线重合。结构差异体现在速度级：$\|e_\xi\|_{\mathrm{rms}}$ 为 9.399/9.403/9.553（×10⁻⁴），C1 相对 C3 优 **1.61%**、相对忠实 C2 优 **0.04%**。后者已处于数值噪声量级，不构成独立的性能主张——它与 (1) 中 C1/C2 的重合是同一机理的两面：C1 与忠实 C2 共享同一信息集（解析 $\dot{\boldsymbol\xi}_d$、解析 $\dot J\dot{\boldsymbol q}$、Ad 搬运 twist 误差），在增益配平、小误差工况下只剩位姿整形的高阶差异。因此本章对忠实 C2 的主张不是"更好"，而是**性能等价 + 证书分化**：定理 3 的精确耗散等式、水平集不变性与均方界只对 (5.2) 的 $A^\top$ 整形成立，忠实 C2 的螺旋对数整形在 $\phi\to\pi$ 处导数奇异（E4 大误差工况无证书），且不存在对任意正定 $K_P$ 成立的同类耗散等式。

**(5) 敏感条件扫描**。四个应力条件下 circle-ss 的 $\|e_\xi\|_{\mathrm{rms}}$（×10⁻³，含标准工况共 5 条件）：

| 条件 | C1 tndq | C2 dq-chandra | C3 dq-hinf |
|---|---|---|---|
| none | 0.940 | 0.940 | 0.955 |
| highspeed (ω=2.5) | 2.314 | 2.314 | 2.361 |
| fast-transit (搬运 ×0.5) | 0.933 | 0.934 | 0.954 |
| noise | 3.164 | 3.165 | 3.234 |
| coarse-dt (15 ms) | 0.964 | 0.964 | 0.979 |

10 组两两对比（5 条件 × 2 基线）中 C1 零例外占优，两个基线的差距量级截然不同：**相对 C3 为 1.52%–2.15%**（none 1.61%、highspeed 2.00%、fast-transit 2.12%、noise 2.15%、coarse-dt 1.52%），**相对忠实 C2 仅 0.00%–0.05%**——五个条件下两律曲线在四位有效数字内重合。这是本章最重要的结构性观察：C1 与忠实 C2 的性能等价不是巧合，而是 §4–5 结构分析的定量确认——两律的前馈（$\mathrm{Ad}$ 搬运 + $\mathrm{ad}$ 输运修正，引理 1）与 twist 误差几何（$\omega_e=-e_\xi$，式 (32) 与 (4.4) 同一）**逐项相同**，增益配平后闭环线性化也逐分量相同，故任何工况下的差异只能来自位姿整形的高阶项；等价性同时意味着，本文相对 [Ch20] 的贡献**不能**表述为精度提升，而应表述为：同一性能水平下，(5.2) 额外携带定理 3 的三类证书（耗散等式、水平集不变性、均方界），且其 $A^\top$ 整形避开螺旋对数在 $\phi\to\pi$ 的导数奇异。

方向与 §4–5 的结构分析预测逐条一致：highspeed 下 C1/C2 的解析前馈对 C3 的领先扩大——一拍滞后桥接在高动态下损失精度；noise 下 C3 劣化最明显（3.234 vs 3.165/3.164），与其桥接项 $\Delta\dot q_{\mathrm{cmd}}/\mathrm{dt}$ 的噪声放大机制一致，而 C1 与 C2 在此条件下不可区分；coarse-dt 下 C3 的一拍滞后被放大 3 倍。位置级稳态残差始终被静态刚度锁定（三律差异 <0.2%），**结构差异集中体现在速度级误差 $e_\xi$**。诚实的强度评估：单组 1.5%–2% 的差距，在无重复实验（每组仅 1 次运行、无随机种子重复）的条件下不足以支撑统计显著性声明；本文主张的是**方向的一致性**（相对 C3 10/10 无例外）与**机理的可解释性**（C3 的桥接差分是唯一的信息损失来源，劣势随速度/噪声/采样粗化按机理扩大），而非幅度本身。

**(6) Lyapunov 收敛、水平集与均方界的核验**。空载 C1-tuned 从初始扰动 $V^{\mathrm{base}}_{\mathrm{peak}}=7.74\times10^{-5}$ 衰减 2.5 个数量级至 $V^{\mathrm{base}}_{ss}=2.20\times10^{-7}$（无扰渐近收敛，与定理 3(b)(iii)(iv) 一致）；带载在杯附着后 $V^{\mathrm{base}}_{\mathrm{peak}}=2.47\times10^{-2}$，随后在 $t_{\mathrm{conv}}=\mathbf{1.50}$ s 内回落至 $V^{\mathrm{base}}_{ss}=3.36\times10^{-4}$；$V_{ss}$ 随增益档单调递减（$2.42\times10^{-2}\to3.36\times10^{-4}\to6.89\times10^{-5}$）。三点定量核验：

- **水平集条件 (5.5b)**：按上文权重口径换算，$V^{\mathrm{tuned}}_{\mathrm{peak}}\le20\times2.47\times10^{-2}=0.494$，而 tuned 档 $c^*=\tfrac12\lambda_{\min}(K_{p,O})=160$，余度约 **2.5 个数量级**（本稿早期版本称"四个数量级"，系直接把 base 权重的 $V$ 与 tuned 权重的 $c^*$ 相比所致，现更正）。base 档 $c^*=8$ 对其自身 $V_{\mathrm{peak}}$ 亦有 2 个数量级余度。故定理 3(b)/3(d) 所需的工作域前提在全部 43 组运行中以充分余度成立——这是把定理 3(d) 的 $\Omega_c$ 前提"事后数值核验"（定理 3(d) 注记与附录 C.5）的具体落实。
- **均方极限界 (5.7) 的保守性**：由 (P2) 反演得 $\|d_{\mathrm{ex}}\|\approx\sqrt{0.389^2+0.683^2}=0.786$；带载时 $\alpha$ 主要来自 $\Delta M$（杯质量相对末端等效惯量），取 $\alpha\to0$ 的乐观极限，(5.7) 给出 $\mathrm{RMS}(e_\xi)\le0.786/\lambda_{\min}(K_d)=0.786/24=3.27\times10^{-2}$；实测 $\mathrm{RMS}(e_\xi)=9.399\times10^{-4}$，比值 **34.8**。即该界成立但**保守约 1.54 个数量级**。保守性的来源可以逐项归因：(5.7d) 中 $\|A\|_2\le1+\|\mathcal T\|$ 与 $\|A_{11}\|_2=\tfrac12$ 均按最坏方向取值，而实际 $d_{\mathrm{ex}}$ 与 $e_\xi$ 在圆周段近似正交（扰动主要沿重力方向、误差主要沿切向），故 $e_\xi^\top d$ 远小于 $\|e_\xi\|\|d\|$；Young 不等式的等号条件（$\|e_\xi\|=D/\lambda_{\mathrm{eff}}$）也远未达到。这一量级的保守性与 [P2] γ 扫描观察到的 H∞ 界保守性同源。
- **H∞ 证书 (5.6a)**：tuned 档按 §6.4 读法 B 认证 $L_2$ 增益上界 $1/\lambda_{\min}(K_d)=0.042$。须注意本章的带载工况扰动是**偏差型**（$d_{\mathrm{ex}}$ 近似常值，$\|d_{\mathrm{ex}}\|_{L_2}=\infty$），严格来说不落在定理 3(c) 的 $d\in L_2$ 前提内；能够核验的只有有限时窗上的能量比，实测该比值远小于 0.042。因此本文对 (5.6a) 的实验支持仅限于"未被违反"，**不构成对 $L_2$ 增益界紧性的检验**——后者需要 §6.7 的 γ 扫描协议（配合 $L_2$ 型注入扰动）才能完成。

**(7) 安全与计算审计**。力矩饱和步数：43 组运行**全部有记录且均为 0**（早期版本因导出字段缺失只能断言"17/19 组无饱和"，本批数据的 `sat_steps` 字段完整，可作全量断言）。零空间治理器：仅 C1-base 带载组触发 3 步；C3 与 C2-abl 的 load/fast-transit 组各触发 1 步（快搬运相位的高加速度需求），其余 40 组为 0。据此，除 base 档外的全部对比结果均在**远离安全边界**的线性工作区取得，不存在饱和掩盖差异的可能。计算开销方面，`runtime_mean_ms` 跨越 8.9–10.8 ms，但该字段**不能用于比较控制律的计算成本**：其数值远大于控制周期 $\Delta t=5$ ms，说明它是含仿真器 RPC 往返的墙钟时间；且 C1 自身在不同增益档间的跨度（9.1→10.8，19%）已超过任何组间差异。因此本文**撤回**早期版本"C1 的 TNDQ 链读出未带来可观测的额外开销"这一结论——该命题需要隔离控制器函数的专项计时（如 `perf_counter` 只包裹 `control_law` 调用）才能检验，本章数据不支持任何方向的结论。§3 关于式 (3.5) 复杂度的论述仍是**操作计数意义**上的（连乘一次给出三个项），与墙钟计时无关。

### 6.6 小结

(i) 控制律 (5.2) 在含接触与负载突变的全相位任务中稳定收敛，且定理 3(b) 所需的水平集条件 (5.5b) 在全部 43 组运行中以 ≥2 个数量级的余度成立（§6.5(6)）；(ii) §5.4 的**静态刚度标度律** (5.9) 通过了强形式检验——两个分量、两个增益档反演出的等效扰动幅值一致到 1.93%，其中旋转分量的 1/4 折减因子获得独立确认；(iii) 严格公平协议下，C1 在 10 组两两对比（5 条件 × 2 基线）的速度级指标上零例外占优：相对一阶桥接基线 C3 稳定优 1.52%–2.15%；相对忠实 [Ch20] 二阶基线 C2 数值等价（≤0.05%）——该等价是 §4–5 结构分析（两律同信息集、前馈与 twist 误差几何逐项相同）的定量确认，C1 对 C2 的分化体现在定理 3 证书与大误差几何（$A^\top$ 整形避开螺旋对数的 $\phi\to\pi$ 奇异），位置级与各基线持平；(iv) 均方极限界 (5.7) 与 H∞ 证书 (5.6a) 均未被违反，但前者保守约 1.54 个数量级。

**本章不支持的声明（诚实边界）**：(a) 空载工况下 C1（与 C2 重合）并不优于 C3（$\|e_\xi\|_{\mathrm{rms}}$ 反而高 1.98%），故不能声称无条件的精度优势；(b) 每组仅 1 次运行、无随机种子重复，1.5%–2% 的差距不具备统计显著性，只能主张方向一致性与机理可解释性；(c) 带载扰动为偏差型，不在定理 3(c) 的 $L_2$ 前提内，故 H∞ 界的**紧性**未获检验；(d) `runtime_mean_ms` 含仿真器 RPC，无法隔离控制器开销，计算代价无结论；(e) base 档落在 (5.9) 适用域之外，其反演不一致不应读作理论失效；(f) C1 与忠实 C2 的性能等价意味着本章**不主张**相对 [Ch20] 的精度提升，主张仅为同性能下的证书增益与几何鲁棒性。局限：仅覆盖 0.25 kg 单一负载与 2.5 rad/s 以下速度，三律位置级差异被刚度配平策略压缩，更高速度、柔性接触或增益不配平场景下的差异化验证，以及带重复种子的统计显著性实验，留待真机阶段。

### 6.7 γ 扫描协议（后续实验设计）

针对 §5.3 注记的 $\gamma_a$ 双分量推论，设计三组扫描（同对象/轨迹/扰动，每个 γ 点记录证书可行性、认证/实测 $L_2$ 增益与稳态误差 RMS）；**扫描必须注入 $L_2$ 型扰动**（如有限时长的脉冲/衰减扰力）而非 §6 的持续偏差型负载，否则不落在定理 3(c) 前提内（§6.5(6) 第三条）：**A 组**（证书扫描）固定增益扫 $\gamma_a$，预期测得列逐位不变——$\gamma_a$ 是分析参数，只移动证书可行域边界 $\gamma_a\ge[2\lambda_{\min}(K_d)-\kappa^{-1}]^{-1/2}$（注记 (i)）；**B 组**（综合模式）按 $\kappa=\gamma_a^2$、$K_d=\gamma_a^{-2}I$ 回写增益，预期误差随 $\gamma_a$ 单调下降、完整不等式 (5.6)（含 $2V(0)$ 项）逐点核验通过、认证增益 $=\gamma_a^2$（注记 (ii)）；**C 组**（对照）复刻 [P2] 的 $\gamma_O=\gamma_T=\gamma$ 综合参数扫描，预期与 B 组趋势同构但力矩接口下无证书可对照（"可调不可证"）。另建议补两项本章数据无法回答的对照：**D 组**（重复性）对 §6.5(5) 的 10 组两两对比取 ≥10 个噪声种子重复，以建立统计显著性；**E 组**（开销）用只包裹控制律调用的专项计时重测三律单步计算成本。

### 5.4 近恒等线性化模型与静态刚度标度律

定理 3 给出的是定性与能量层面的结论，不直接给出增益数值。本节在原点邻域把 (5.5) 线性化，得到一个**可直接用于增益整定且可实验证伪**的两分量二阶模型——它同时暴露了 $A_0$ 带来的一个容易被忽略的结构效应：旋转分量的刚度被折减四倍。

取 $K_d=\mathrm{diag}(K_\omega,K_v)$、$K_p=\mathrm{diag}(K_{p,O},k_{p,T}I_3)$，在 $\tilde x\to1$（$\tilde\eta\to1,\mathcal O\to0,\mathcal T\to0$）处 $A\to A_0=\mathrm{diag}(-\tfrac12I_3,I_3)$，故 $\dot{\mathcal O}=-\tfrac12\tilde\omega$、$\dot{\mathcal T}=\tilde v$。将其微分一次并代入 (5.5) 的第二式（注意 $(A_0^\top K_pe_z)_\omega=-\tfrac12K_{p,O}\mathcal O$、$(A_0^\top K_pe_z)_v=k_{p,T}\mathcal T$），消去 $e_\xi$ 得两条解耦的二阶方程：

$$
\boxed{\;
\ddot{\mathcal O}+K_\omega\dot{\mathcal O}+\tfrac14K_{p,O}\,\mathcal O=-\tfrac12\,d_\omega ,
\qquad
\ddot{\mathcal T}+K_v\dot{\mathcal T}+k_{p,T}\,\mathcal T=+\,d_v .\;}
\tag{5.8}
$$

**(i) 1/4 旋转刚度折减**：旋转分量的有效刚度是 $\tfrac14K_{p,O}$ 而不是 $K_{p,O}$，根源是 $A_0$ 的旋转块为 $-\tfrac12I_3$（$\mathcal O=-\mathrm{Im}\,\tilde r$ 与半角参数化共同贡献的因子），在位姿反馈与输出映射中各出现一次，故以平方形式 $(\tfrac12)^2$ 进入刚度。**工程含义**：若天真地取 $K_{p,O}=k_{p,T}I_3$（如 §6.3 的 base 档，$K_p=16I_6$），则旋转分量的实际刚度仅为平移分量的 1/4，两分量带宽严重失配；要使二者配平，应取 $K_{p,O}=4k_{p,T}I_3$（§6.3 tuned 档的 $p_O=320=4\times80$ 即此规则）。**(ii) 极点分配规则**：若各分量目标极点为 $\{-a,-b\}$（$a,b>0$），则

$$
K_\omega=K_v=(a+b)I_3,\qquad k_{p,T}=ab,\qquad K_{p,O}=4ab\,I_3 ,
$$

即 §6.3 三档增益的生成式（tuned $=\{-4,-20\}$、fast $=\{-6,-30\}$、base $=\{-4,-4\}$但未作 1/4 补偿）；离散实现另需极点与步长满足 $\max(a,b)\cdot\Delta t\lesssim0.2$。**(iii) 注意号差异**：旋转分量的扰动增益为 $-\tfrac12$、平移为 $+1$，同源于 $A_0$；该系数在下式的反演中必须保留。

令 (5.8) 中 $d_\omega,d_v$ 为准常量（低频成分主导，如未建模负载引起的 $\Delta M,\Delta\boldsymbol g$），取 $\ddot{(\cdot)}=\dot{(\cdot)}=0$ 得**静态刚度标度律**

$$
\boxed{\;
\|\mathcal T\|_{\mathrm{ss}}=\frac{\|d_v\|}{k_{p,T}} ,
\qquad
\|\mathcal O\|_{\mathrm{ss}}=\frac{2\,\|d_\omega\|}{\lambda(K_{p,O})} ,\;}
\tag{5.9}
$$

即稳态残差**只**由静态刚度决定、与阻尼无关（$K_d$ 只改变过渡过程）。(5.9) 给出两个可伪造的预言：**(P1) 反比标度**——刚度提高 $\rho$ 倍，稳态位姿残差降低至 $1/\rho$；**(P2) 等效扰动反演的一致性**——同一物理工况下用**不同增益档**的实测残差反演 $\|d_v\|=k_{p,T}\|\mathcal T\|_{\mathrm{ss}}$、$\|d_\omega\|=\tfrac12\lambda(K_{p,O})\|\mathcal O\|_{\mathrm{ss}}$，应得到**同一个**幅值。(P2) 比 (P1) 严苛得多（它要求两个独立档位的两个独立数字重合），是 §6.4 对本节模型的主检验（实测偏差 $\approx2\%$）。

(5.8)–(5.9) 为 $\tilde x\to1$ 的局部近似，大误差区耦合未计入；严格结论以定理 3 为准。

---