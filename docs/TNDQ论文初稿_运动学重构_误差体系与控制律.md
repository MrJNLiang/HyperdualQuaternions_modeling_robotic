# Trident Number Dual Quaternion (TNDQ) Modeling of Robot Kinematics with a Geometrically Consistent Error System and Mixed H∞/ISS Control

> **文稿性质**：论文初稿（第一版）。结构参照 Figueredo, Adorno & Ishihara, *Robust H∞ kinematic control of manipulator robots using dual quaternion algebra*, Automatica 132 (2021)（下称 [P2]）：摘要 → 引言（含贡献声明）→ 预备知识 → 主体理论（TNDQ 运动学 / 误差体系 / 控制律主定理）→ 模拟（本稿仅给出流程设计）→ 结论 → 附录（次要推导）。
>
> 理论内容取自项目文档体系（编号沿用）：扩展篇 `docs/HDQ动力学建模扩展_Jdot与Hessian.md`（(D-k)、(5.2′)）、误差篇 `docs/HDQ动力学误差体系重构_几何一致二阶误差方案.md`（(F-k)、TNDQ/HDQ 截断）。文献编号：[P1] = Cohen & Shoham MMT 2020；[P2] = Figueredo et al. Automatica 2021。
>
> **记号说明**：本稿采用统一装饰记号（§2 表 0）——$\hat a$ 单位四元数、$\hat{\underline a}$ 单位 DQ、$\breve a$ HDQ、$\bar a$ TNDQ。源文档中的算子记号 $T^1\boldsymbol x$、$T^2\boldsymbol x$、$\Pi_{\mathrm{HDQ}}$ 在本稿分别写作 $\breve x$、$\bar x$ 与"取前两通道截断"。
>
> **写作约定**：面向具备本科代数（环、商环）与常微分方程基础的数学系读者；机器人学专有概念（位姿、twist、雅可比）在首次出现处给出数学定义；工程细节（编码器、驱动器）只在模拟流程一节出现。

---

## 摘要

对偶四元数（DQ）代数为机械臂位姿运动学提供了紧凑的全局无奇异参数化，[P2] 在其上建立了运动学 H∞ 跟踪控制。然而 DQ 体系只携带位姿一阶信息：速度与加速度层的量必须另行构造，误差体系也只有位姿一层，进入动力学控制（力矩接口）时出现三个结构性缺口——速度误差无几何一致定义、加速度层扰动无入口、偏差型不确定性不满足 L₂ 假设。本文引入**三叉对偶四元数**（Trident Number Dual Quaternion, TNDQ）：以三个 DQ 通道（位姿/一阶导/二阶导）为元素的截断多项式代数 $\mathcal A_2=\widehat{\mathbb H}[\sigma]/(\sigma^3)$，证明位姿曲线的 TNDQ 表示 $\bar x$ 满足连乘法则 $\overline{xy}=\bar x\,\bar y$，从而串联链的 $\bar x$ 由各关节因子 $\bar x_i$ 一次连乘获得，位姿、twist、任务空间加速度与 $\dot J\dot{\boldsymbol q}$ 同批输出。在此基础上，本文证明取前两通道的 HDQ 截断 $\bar x\mapsto\breve x$ 与乘法相容（先乘后截 = 先截后乘），据此把**误差体系定义在 HDQ 截断上**：一次 HDQ 乘法同时生成右不变位姿误差与几何一致 twist 误差（定理 1），二者经输出映射闭合为严格级联误差运动学（定理 2）。针对动力学接口设计几何一致计算力矩律，证明闭环误差动态为级联标准形，并对噪声型（L₂）与偏差型（L∞）两类扰动分别给出 H∞ 增益界与输入-状态稳定（ISS）极限球界（定理 3）。加速度层不进入误差状态：期望加速度作为前馈、加速度层不确定性作为扰动，本文证明这一取舍不损失任何反馈信息。最后给出完整的仿真验证流程设计。

**关键词**：对偶四元数；超对偶四元数；截断多项式代数；几何一致误差；H∞ 控制；输入-状态稳定

---

## 1. 引言

### 1.1 背景与动机

刚体位姿（姿态 + 位置）的参数化是机器人控制的起点。单位对偶四元数将两者装入一个 8 维代数对象，运算全局无奇异，且乘法直接实现位姿复合；[P2] 在此参数化上给出了带 L₂ 扰动衰减保证的运动学跟踪控制器，是 DQ 控制的代表性结果。Cohen & Shoham [P1] 进一步引入超对偶四元数（HDQ）：给 DQ 增加一个幂零单位 $\varepsilon^*$（$\varepsilon^{*2}=0$），使一个 HDQ 元素同时携带位姿与其一阶时间导数，链式乘法自动执行微分（Leibniz 法则内化于乘法），从而正运动学一次传播同时输出位姿与 twist。

但 HDQ 到动力学层面仍差一阶：计算力矩控制需要任务空间加速度 $\dot{\boldsymbol\xi}$ 与雅可比导数项 $\dot J\dot{\boldsymbol q}$，而 [P1] 的 HDQ 结构中两个幂零单位（$\varepsilon$ 承载平移、$\varepsilon^*$ 承载一阶导）均已占用，$\varepsilon\varepsilon^*$ 通道携带的是"平移分量的一阶导数"，不含二阶时间导数。更重要的是**误差体系**：[P2] 的误差对象只有位姿一层，其扰动模型假设速度级加性扰动且属于 L₂——动力学环境下这三点都不再成立（§4.1 详述）。

### 1.2 本文贡献

1. **TNDQ 运动学重构**（§3）：定义三通道代数 $\mathcal A_2=\widehat{\mathbb H}[\sigma]/(\sigma^3)$（TNDQ），证明位姿曲线的 TNDQ 表示 $\bar x$ 满足连乘法则 $\overline{xy}=\bar x\,\bar y$（命题 1），据此正运动学链一次连乘给出 $(\hat{\underline x},\dot{\hat{\underline x}},\ddot{\hat{\underline x}})$ 及导出量 $\boldsymbol\xi,\dot{\boldsymbol\xi},\dot J\dot{\boldsymbol q}$，代价 $O(n)$。
2. **HDQ 截断与误差体系**（§4）：证明取前两通道的 HDQ 截断 $\bar a\mapsto\breve a$ 满足"先乘后截 = 先截后乘"（命题 2）；据此把误差对象定义为实测/期望链 HDQ 表示的一次乘积 $\breve{\tilde x}=\breve x\,(\breve x_d)^*$（定理 1），同时生成右不变位姿误差与几何一致 twist 误差，并证明输出误差满足闭式级联运动学 $\dot e_z=A(\tilde x)e_\xi$（定理 2）。加速度误差被证明在反馈中冗余，从误差状态中删除（§4.2）。
3. **几何一致控制律与混合性能保证**（§5）：给出几何一致计算力矩律（前馈经伴随搬运并补输运项、位姿反馈经 $A^\top$ 整形），证明闭环误差动态为级联标准形，Lyapunov 交叉项精确相消，对 L₂ 扰动给出 H∞ 增益界、对 L∞ 偏差给出 ISS 极限球界（定理 3，主定理）。
4. **模拟流程设计**（§6）：给出从关节测量到力矩指令的完整信息流水线与验证协议（本稿只含流程，不含数据）。

**与 [P2] 的关系**：本文不替代 [P2] 的运动学外环——0 阶通道的误差与控制在低速接口下退化回 [P2] 原样（向下兼容，§4.5），本文解决的是其向动力学接口延伸时的结构缺口。

**与 [P1] 的关系**：TNDQ 是 [P1] HDQ 思想（幂零单位承载导数）向二阶的最小扩展；HDQ 恰是 TNDQ 的两通道截断（§3.1 表 1），[P1] 的全部乘法机器在截断下原样保留。

### 1.3 论文结构

§2 预备知识（记号约定、四元数、DQ、twist、HDQ）；§3 TNDQ 代数与运动学重构；§4 误差体系；§5 控制律与主定理；§6 模拟流程设计；§7 结论。关键定理的证明放正文，较长的验证性推导放附录 A–C。

---

## 2. 预备知识

**记号约定**（全文统一）：同一条位姿曲线用同一核心字母（如 $x$），字母上方的装饰指明它所处的代数层：

| 记号 | 对象 | 说明 |
|---|---|---|
| $\hat a$ | 单位四元数（旋转） | $\hat a\in\mathrm{Spin}(3)$，§2.1 |
| $\hat{\underline a}$ | 单位对偶四元数（位姿） | $\hat{\underline a}\hat{\underline a}^*=1$，式 (2.1) |
| $\breve a$ | HDQ（超对偶四元数） | 两 DQ 通道；曲线的 HDQ 表示 $\breve x=\hat{\underline x}+\varepsilon^*\dot{\hat{\underline x}}$，§2.3 |
| $\bar a$ | TNDQ（三叉对偶四元数） | 三 DQ 通道；曲线的 TNDQ 表示 $\bar x=\hat{\underline x}+\sigma\dot{\hat{\underline x}}+\tfrac12\sigma^2\ddot{\hat{\underline x}}$，§3.2 |
| $\tilde{(\cdot)}$ | 误差量 | 只佩戴波浪号，不再叠加类型装饰；其类型由定义式指明（如 $\tilde x=\hat{\underline x}\hat{\underline x}_d^{\,*}$ 是单位 DQ，$\tilde r$ 是单位四元数） |
| 无装饰斜体 | 一般（未必单位）四元数 / DQ / 标量 | 所属代数在上下文声明 |
| 粗体 | 纯 DQ（twist 等）、向量与矩阵 | $\boldsymbol\xi,\boldsymbol q,J,K_d$ 等 |

（表 0：记号约定。标称模型矩阵 $\hat M,\hat C,\hat g$ 上的 hat 沿用控制文献"标称估计"的习惯用法，与四元数装饰无关；四元数虚单位 $\hat\imath,\hat\jmath,\hat k$ 为固定符号。）

### 2.1 四元数与对偶四元数

四元数代数 $\mathbb H=\{\eta+\mu_1\hat\imath+\mu_2\hat\jmath+\mu_3\hat k\}$，$\hat\imath^2=\hat\jmath^2=\hat k^2=\hat\imath\hat\jmath\hat k=-1$。共轭 $q^*=\eta-\mu$（$\mu$ 为向量部），范数 $\|q\|^2=qq^*$。单位四元数集 $\mathrm{Spin}(3)=\{\hat r\in\mathbb H:\|\hat r\|=1\}$ 双覆盖旋转群 $SO(3)$：绕单位轴 $n$ 转角 $\phi$ 对应 $\hat r=\cos\frac\phi2+n\sin\frac\phi2$。

对偶四元数（DQ）代数 $\widehat{\mathbb H}=\mathbb H\oplus\varepsilon\mathbb H$，$\varepsilon^2=0$（$\varepsilon$ 与四元数单位交换）。**单位 DQ**

$$
\hat{\underline x}=\hat r+\varepsilon\tfrac12\,p\,\hat r,\qquad \hat r\in\mathrm{Spin}(3),\ p\in\mathbb H_p\ (\text{纯四元数}\cong\mathbb R^3)
\tag{2.1}
$$

表示位姿（旋转 $\hat r$ + 平移 $p$），满足 $\hat{\underline x}\hat{\underline x}^*=1$；单位 DQ 全体构成群 $\mathrm{Spin}(3)\ltimes\mathbb R^3$，双覆盖 $SE(3)$，乘法即位姿复合。DQ 共轭 $\hat{\underline x}^*=\hat r^*+\varepsilon\tfrac12\hat r^*p^*$ 逐分量遵循四元数共轭。**纯 DQ**（标量部与对偶标量部为零者）构成 6 维实空间，记 $\mathrm{vec}_6$ 为取其两个向量部的坐标同构，$\overline{\mathrm{vec}}_6$ 为逆。

### 2.2 twist 与运动学

沿光滑单位 DQ 曲线 $\hat{\underline x}(t)$，**空间 twist** 定义为

$$
\boldsymbol\xi\triangleq 2\dot{\hat{\underline x}}\hat{\underline x}^{*}=\omega+\varepsilon v,\qquad v=\dot p+p\times\omega,
\tag{2.2}
$$

其中 $\omega$ 为角速度。$\boldsymbol\xi$ 是纯 DQ（附录 A.1），等价写法即左乘运动学 $\dot{\hat{\underline x}}=\tfrac12\boldsymbol\xi\hat{\underline x}$（[P2] 式(1) 约定）。对 $n$ 关节串联机械臂，正运动学 $\hat{\underline x}(\boldsymbol q)=\prod_{i=1}^n\hat{\underline x}_i(q_i)$（各关节单位 DQ 之积），微分给出 $\mathrm{vec}_6\,\boldsymbol\xi=J(\boldsymbol q)\dot{\boldsymbol q}$，$J\in\mathbb R^{6\times n}$ 称几何雅可比。

**伴随作用与李括号**：单位 DQ 对纯 DQ 的作用 $\mathrm{Ad}_{\hat{\underline x}}\boldsymbol a\triangleq\hat{\underline x}\boldsymbol a\hat{\underline x}^*$ 保持纯性与范数（它是 twist 在不同参考位姿间的搬运算子）；纯 DQ 上 $\mathrm{ad}_{\boldsymbol a}\boldsymbol b\triangleq\tfrac12(\boldsymbol{ab}-\boldsymbol{ba})$ 仍为纯 DQ（李括号的 DQ 形式）。

### 2.3 超对偶四元数（HDQ）

HDQ 代数 $\widehat{\mathbb H}[\varepsilon^*]/(\varepsilon^{*2})$：元素 $\breve q=q_0+\varepsilon^*q_1$（$q_0,q_1\in\widehat{\mathbb H}$），乘法

$$
(a_0+\varepsilon^*a_1)(b_0+\varepsilon^*b_1)=a_0b_0+\varepsilon^*(a_0b_1+a_1b_0).
\tag{2.3}
$$

(2.3) 的 $\varepsilon^*$ 通道正是 Leibniz 法则——这是 HDQ 自动微分能力的代数根源（[P1] 式(14)(25)）。**曲线的 HDQ 表示** $\breve x\triangleq\hat{\underline x}+\varepsilon^*\dot{\hat{\underline x}}$ 满足连乘法则 $\breve{xy}=\breve x\,\breve y$：对每个关节因子写出其 HDQ 表示后连乘，一次传播同时得到 $\hat{\underline x}$ 与 $\dot{\hat{\underline x}}$（进而 $\boldsymbol\xi$）。这里"$\breve{xy}$"指乘积曲线 $\hat{\underline x}(t)\hat{\underline y}(t)$ 的 HDQ 表示，是命题 1（§3.2）在两通道截断下的特例。

### 2.4 机械臂动力学（标准形）

关节空间动力学 $M(\boldsymbol q)\ddot{\boldsymbol q}+C(\boldsymbol q,\dot{\boldsymbol q})\dot{\boldsymbol q}+\boldsymbol g(\boldsymbol q)=\boldsymbol\tau$，$M$ 对称正定。计算力矩方案用标称模型 $\hat M,\hat C,\hat g$（此处 hat 表"标称估计"，见表 0）生成 $\boldsymbol\tau=\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+\hat C\dot{\boldsymbol q}+\hat g$，其中 $\ddot{\boldsymbol q}_{\mathrm{ref}}$ 是待设计的参考加速度——本文 §5 的控制量。

---

## 3. TNDQ：三叉对偶四元数与运动学重构

### 3.1 代数定义与截断塔

> **定义 1（TNDQ）**：三叉对偶四元数代数为截断多项式环
>
> $$
> \mathcal A_2\triangleq\widehat{\mathbb H}[\sigma]/(\sigma^3)
> =\Bigl\{\bar a=a_0+\sigma a_1+\tfrac12\sigma^2a_2:\ a_k\in\widehat{\mathbb H}\Bigr\},
> \tag{3.1}
> $$
>
> $\sigma$ 与全部四元数单位交换，$\sigma^3=0$。乘法由分配律与 $\sigma^3=0$ 唯一确定：
>
> $$
> \bar a\,\bar b=a_0b_0+\sigma(a_0b_1+a_1b_0)+\tfrac12\sigma^2\bigl(a_0b_2+2a_1b_1+a_2b_0\bigr).
> \tag{3.2}
> $$

"三叉"（trident）指其三个 DQ 通道：位姿 / 一阶导 / 二阶导；程序中一个 TNDQ 即三个并列的 8 维数组。与既有结构的截断关系：

| 结构 | 通道 | 元素 | 关系 |
|---|---|---|---|
| DQ | 1 | $a_0$ | TNDQ 的 $\sigma^0$ 通道 |
| HDQ（[P1]） | 2 | $a_0+\varepsilon^*a_1$ | TNDQ 的 $\sigma^0,\sigma^1$ 通道（$\sigma\leftrightarrow\varepsilon^*$） |
| **TNDQ** | 3 | $a_0+\sigma a_1+\tfrac12\sigma^2a_2$ | 全结构 |

（表 1：DQ–HDQ–TNDQ 截断塔）

数学上 $\mathcal A_2$ 就是"系数在 $\widehat{\mathbb H}$ 中的二阶 jet 代数"——沿时间曲线的二阶 Taylor 多项式在截断乘法下封闭。这一结构本身是标准对象；本文的内容在于它与 [P1] HDQ 的精确截断关系（表 1）、以下连乘/截断法则及其在误差/控制体系中的使用方式。

### 3.2 曲线的 TNDQ 表示与连乘法则

给定光滑单位 DQ 曲线 $\hat{\underline x}(t)$，其**TNDQ 表示**定义为把它与它的两阶导数装入三个通道：

$$
\bar x\triangleq\hat{\underline x}+\sigma\dot{\hat{\underline x}}+\tfrac12\sigma^2\ddot{\hat{\underline x}}\ \in\mathcal A_2 .
\tag{3.3a}
$$

> **命题 1（TNDQ 连乘法则）**：设 $\hat{\underline x}(t),\hat{\underline y}(t)$ 为两条光滑 DQ 曲线，$\hat{\underline z}(t)=\hat{\underline x}(t)\hat{\underline y}(t)$ 为其逐点乘积曲线。则三者的 TNDQ 表示满足
>
> $$
> \bar z=\bar x\,\bar y,\qquad\text{即}\qquad \overline{xy}=\bar x\,\bar y .
> \tag{3.3}
> $$
>
> 换言之：**"先把两条曲线相乘再取 TNDQ 表示"等于"分别取 TNDQ 表示后按 (3.2) 相乘"**。
>
> **证明**：逐通道对照 (3.2) 与乘积求导（Leibniz）的 0/1/2 阶结果——$\sigma^0$：$\hat{\underline x}\hat{\underline y}=\hat{\underline z}$；$\sigma^1$：$\dot{\hat{\underline x}}\hat{\underline y}+\hat{\underline x}\dot{\hat{\underline y}}=\dot{\hat{\underline z}}$；$\sigma^2$ 通道系数 $\tfrac12(\hat{\underline x}\ddot{\hat{\underline y}}+2\dot{\hat{\underline x}}\dot{\hat{\underline y}}+\ddot{\hat{\underline x}}\hat{\underline y})=\tfrac12\ddot{\hat{\underline z}}$。三通道正是 $\bar z$ 的三通道。∎

命题 1 是整个运动学重构的引擎：对串联臂 $\hat{\underline x}(\boldsymbol q)=\prod_i\hat{\underline x}_i(q_i(t))$，先写出每个关节因子的 TNDQ 表示 $\bar x_i$（单关节曲线的 $\dot{\hat{\underline x}}_i,\ddot{\hat{\underline x}}_i$ 有闭式，只依赖 $q_i,\dot q_i,\ddot q_i$；附录 B.1），再按 (3.2) 逐个连乘：

$$
\bar x=\bar x_1\,\bar x_2\cdots\bar x_n=\prod_{i=1}^{n}\bar x_i
\tag{3.4}
$$

一次 $O(n)$ 链连乘同时输出 $\hat{\underline x},\dot{\hat{\underline x}},\ddot{\hat{\underline x}}$（$\bar x$ 的三个通道）。导出量（附录 A）：

$$
\boldsymbol\xi=2\dot{\hat{\underline x}}\hat{\underline x}^*,\qquad
\dot{\boldsymbol\xi}=2\ddot{\hat{\underline x}}\hat{\underline x}^*-\tfrac12\boldsymbol\xi^2\ \text{（取纯部）},\qquad
\mathrm{vec}_6\dot{\boldsymbol\xi}=\dot J\dot{\boldsymbol q}+J\ddot{\boldsymbol q}.
\tag{3.5}
$$

特别地，令 $\ddot{\boldsymbol q}=0$ 连乘一次即单独读出 $\dot J\dot{\boldsymbol q}$——计算力矩律需要的正是这一项，无须显式构造 $\dot J$。

### 3.3 HDQ 截断与乘法相容性

把 TNDQ 表示 $\bar a=a_0+\sigma a_1+\tfrac12\sigma^2a_2$ **只保留前两个通道**（丢弃 $\sigma^2$ 通道，程序上即只取前两个数组），并把 $\sigma$ 记作 $\varepsilon^*$，得到它的 **HDQ 截断**，记作 $\bar a\big|_{\mathrm{HDQ}}$：

$$
\bar a\big|_{\mathrm{HDQ}}\triangleq a_0+\varepsilon^*a_1=\breve a .
\tag{3.6}
$$

对曲线的 TNDQ 表示 $\bar x$（式 (3.3a)）施行截断，恰得 §2.3 的 HDQ 表示：$\bar x\big|_{\mathrm{HDQ}}=\hat{\underline x}+\varepsilon^*\dot{\hat{\underline x}}=\breve x$。

> **命题 2（截断与乘法相容）**：对任意 $\bar a,\bar b\in\mathcal A_2$，
>
> $$
> \bigl(\bar a\,\bar b\bigr)\big|_{\mathrm{HDQ}}=\breve a\,\breve b,\qquad\text{即先按 (3.2) 相乘再截断 = 先截断再按 (2.3) 相乘。}
> \tag{3.7}
> $$
>
> 特别地，对曲线取截断与取表示可交换：$\overline{xy}\big|_{\mathrm{HDQ}}=\breve x\,\breve y$。
>
> **证明**：比较 (3.2) 与 (2.3)：乘积的 $\sigma^0,\sigma^1$ 两通道只依赖两因子的 $\sigma^0,\sigma^1$ 通道（截断多项式环的滤过性——$\sigma^2$ 通道无法向低阶"回流"），且其表达式与 (2.3) 逐字相同。故先乘后取前两通道，与先取前两通道再乘，结果一致。∎

命题 2 的实际意义：**丢弃 $\sigma^2$ 通道是无损操作**——只要后续运算只发生在前两个通道（程序上：只取前两个数组），结果与从未携带过第三通道完全一致。这为 §4 的结构性决策（误差体系定义在 HDQ 截断上）提供了严格依据。

### 3.4 单位性约束的提升

单位性 $\hat{\underline x}\hat{\underline x}^*=1$ 沿曲线逐阶求导给出约束族：

$$
\hat{\underline x}\hat{\underline x}^*=1,\qquad
\dot{\hat{\underline x}}\hat{\underline x}^*+\hat{\underline x}\dot{\hat{\underline x}}^*=0,\qquad
\ddot{\hat{\underline x}}\hat{\underline x}^*+2\dot{\hat{\underline x}}\dot{\hat{\underline x}}^*+\hat{\underline x}\ddot{\hat{\underline x}}^*=0 .
\tag{3.8}
$$

解析上恒成立；数值积分会使其漂移。定义残差 $c_0=\|\hat{\underline x}\hat{\underline x}^*-1\|$，$c_1=\|\mathrm{Sc}(2\dot{\hat{\underline x}}\hat{\underline x}^*)\|$（$\mathrm{Sc}$ 取标量与对偶标量部）作为逐周期 $O(1)$ 监测量；超阈值触发重投影（0 阶归一化、1 阶按 $\dot{\hat{\underline x}}\mapsto\tfrac12\boldsymbol\xi_{\mathrm{proj}}\hat{\underline x}$ 重构）。前馈侧若使用 $\sigma^2$ 通道可另监测

$$
c_2=\bigl\|\mathrm{Sc}(2\ddot{\hat{\underline x}}\hat{\underline x}^*)-\tfrac12\mathrm{Sc}(\boldsymbol\xi^2)\bigr\|
=\bigl\|\mathrm{Sc}(2\ddot{\hat{\underline x}}\hat{\underline x}^*)+\tfrac12\mathrm{Sc}(\boldsymbol\xi\boldsymbol\xi^*)\bigr\|
=\bigl\|\mathrm{Sc}(\dot{\boldsymbol\xi})\bigr\| ,
$$

其中第一个等号用 $\boldsymbol\xi\boldsymbol\xi^*=-\boldsymbol\xi^2$（$\boldsymbol\xi$ 为纯元），第二个等号即 (3.5) 第二式——(3.8) 的二阶约束经 $2\dot{\hat{\underline x}}\dot{\hat{\underline x}}^*=\tfrac12\boldsymbol\xi\boldsymbol\xi^*$ 化简后与之同一。沿真曲线 $\dot{\boldsymbol\xi}$ 为纯元故 $c_2$ 解析为零。超阈值时 2 阶按 $\ddot{\hat{\underline x}}\mapsto\tfrac12\bigl(\dot{\boldsymbol\xi}_{\mathrm{proj}}+\tfrac12\boldsymbol\xi_{\mathrm{proj}}^2\bigr)\hat{\underline x}$ 重构（由 (3.5) 第二式反解）。

---

## 4. 几何一致误差体系

### 4.1 DQ 误差体系在动力学环境的三个缺口

[P2] 的误差体系：右不变位姿误差 $\tilde x=\hat{\underline x}\hat{\underline x}_d^{\,*}$、误差函数 $\tilde z=1-\tilde x$、6 维输出 $e_z=[\mathcal O;\mathcal T]$（$\mathcal O=-\mathrm{Im}\,\tilde r$，$\mathcal T=\tilde p$）；扰动为速度级加性信号 $\boldsymbol v_w,\boldsymbol v_c\in L_2$。进入力矩接口时：

1. **速度误差无几何一致定义**：朴素差 $\boldsymbol\xi-\boldsymbol\xi_d$ 中两 twist 分别属于当前位姿与期望位姿处的切空间，直接相减混入伪项 $(\mathrm{Ad}_{\tilde x}-\mathrm{id})\boldsymbol\xi_d$，其范数 $\lesssim(2\|\mathcal O\|+\|\mathcal T\|\|\omega_d\|)\|\boldsymbol\xi_d\|$——随期望速度线性放大，高速工况下经 $K_d$ 直接进入力矩。
2. **加速度层扰动无入口**：动力学的主要不确定性（惯性参数偏差、力矩误差）作用在加速度层，[P2] 的速度级扰动通道无法表达。
3. **偏差型不确定性不满足 L₂**：$\Delta M,\Delta C,\Delta g$、摩擦残差是持续偏差，无限时域能量无穷，L₂ 增益指标对其空洞成立。

### 4.2 误差对象的正确阶数：为什么是 HDQ 而非 TNDQ

反馈需要几阶误差？计算力矩律的反馈项（§5）为 $-K_de_\xi-k_pA^\top e_z$——只消费位姿与速度两阶。若引入加速度误差 $e_a$ 及增益 $K_a$，闭环从二阶级联升为三阶系统：既无必要（定理 3 将证明两阶已指数收敛），又把高噪声的加速度估计（差分方差 $\propto\Delta t^{-4}$）直接引入反馈。加速度层信息的正确去向是两路：**期望加速度 $\dot{\boldsymbol\xi}_d$ 走前馈**（来自期望轨迹，确定量），**加速度层不确定性走扰动**（进入 $\dot e_\xi$ 方程的 $d(t)$，§5.2）。

因此：**正运动学与期望轨迹用 TNDQ 建模（前馈需要 $\sigma^2$ 通道），误差体系定义在 HDQ 截断上**。命题 2 保证这一截断在代数上无损。

### 4.3 定理 1：误差的 HDQ 提升

实测链取 HDQ 表示 $\breve x=\hat{\underline x}+\varepsilon^*\dot{\hat{\underline x}}$（即 $\bar x$ 的前两通道截断）；期望链取 $\breve x_d=\hat{\underline x}_d+\varepsilon^*\dot{\hat{\underline x}}_d$（程序中：各取前两个 DQ 数组）。

> **定理 1（误差的 HDQ 提升）**：定义 HDQ 误差元素
>
> $$
> \breve{\tilde x}\triangleq \breve x\cdot\bigl(\breve x_d\bigr)^{*} .
> \tag{4.1}
> $$
>
> 则
>
> $$
> \breve{\tilde x}=\tilde x+\varepsilon^*\dot{\tilde x},\qquad
> \tilde x=\hat{\underline x}\hat{\underline x}_d^{\,*},\quad
> \dot{\tilde x}=\dot{\hat{\underline x}}\hat{\underline x}_d^{\,*}+\hat{\underline x}\dot{\hat{\underline x}}_d^{\,*},
> \tag{4.2}
> $$
>
> 即一次 HDQ 乘法（3 次 DQ 乘）同时给出位姿误差 $\tilde x$ 与其导数；0 阶通道正是 [P2] 的 $\tilde x$。进一步定义**几何一致 twist 误差**
>
> $$
> \tilde{\boldsymbol\xi}\triangleq 2\dot{\tilde x}\tilde x^{*},\qquad
> e_\xi\triangleq\mathrm{vec}_6\tilde{\boldsymbol\xi},
> \tag{4.3}
> $$
>
> 则：(i) $\tilde{\boldsymbol\xi}$ 是纯 DQ，且对 unwinding 翻转 $\tilde x\to-\tilde x$ 不变；(ii) 误差满足与原系统同型的左乘运动学 $\dot{\tilde x}=\tfrac12\tilde{\boldsymbol\xi}\tilde x$；(iii) 无扰时
>
> $$
> \tilde{\boldsymbol\xi}=\boldsymbol\xi-\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d ,
> \tag{4.4}
> $$
>
> 即"实际 twist 减去**搬运到当前位姿处**的期望 twist"——与朴素差的偏离恰为 §4.1 的伪项。
>
> **证明**：(4.2)：将 (2.3) 用于 $a_0=\hat{\underline x},a_1=\dot{\hat{\underline x}},b_0=\hat{\underline x}_d^{\,*},b_1=\dot{\hat{\underline x}}_d^{\,*}$（共轭与求导交换保证 $(\breve x_d)^*$ 的 $\varepsilon^*$ 通道为 $\dot{\hat{\underline x}}_d^{\,*}$），$\varepsilon^*$ 通道即 Leibniz 展开的 $\frac{d}{dt}(\hat{\underline x}\hat{\underline x}_d^{\,*})$。(i)(ii)(iii) 的推导见附录 A.2——(iii) 的关键一步：$\dot{\tilde x}=\dot{\hat{\underline x}}\hat{\underline x}_d^{\,*}+\hat{\underline x}\dot{\hat{\underline x}}_d^{\,*}=\tfrac12\boldsymbol\xi\tilde x-\tfrac12\tilde x\boldsymbol\xi_d$（用 $\dot{\hat{\underline x}}_d^{\,*}=-\tfrac12\hat{\underline x}_d^{\,*}\boldsymbol\xi_d$），右乘 $2\tilde x^*$ 得 (4.4)。∎
>
> **注记（截断一致性）**：由命题 2，$\breve x\,(\breve x_d)^*$ 与"先在 TNDQ 上作误差乘法 $\bar x\,(\bar x_d)^*$ 再截断"给出同一 HDQ 对象——误差体系落在 HDQ 上不产生任何截断误差。
>
> **注记（与 [P2] 的一致性）**：[P2] 控制律 (P2-12) 的前馈项 $\mathrm{vec}_6(\tilde x\boldsymbol\xi_d\tilde x^*)$ 正是 $\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d$；其闭环化简之所以成立，正因为控制律隐式地把 (4.4) 而非朴素差驱为反馈量。定理 1 把这一隐含结构显式化为体系的一阶误差通道。

### 4.4 定理 2：输出误差的级联运动学

沿用 [P2] 输出 $e_z=[\mathcal O;\mathcal T]\in\mathbb R^6$，$\mathcal O=-\mathrm{Im}\,\tilde r$，$\mathcal T=\tilde p$（$\tilde r,\tilde p$ 为 $\tilde x$ 的旋转/平移分量，$\tilde r=\tilde\eta+\tilde\mu$）。

> **定理 2（输出误差运动学闭式）**：记 $\tilde{\boldsymbol\xi}=\tilde\omega+\varepsilon\tilde v$，则
>
> $$
> \dot e_z=A(\tilde x)\,e_\xi,
> \qquad
> A(\tilde x)=
> \begin{bmatrix}
> -\tfrac12\bigl(\tilde\eta I_3+[\mathcal O]_\times\bigr) & 0_3\\[2pt]
> -[\mathcal T]_\times & I_3
> \end{bmatrix},
> \tag{4.5}
> $$
>
> 且 $\tilde x\to1$ 时 $A\to A_0=\mathrm{diag}(-\tfrac12I_3,I_3)$，$\sigma_{\min}(A_0)=\tfrac12$。
>
> **证明**：旋转通道：由 $\dot{\tilde r}=\tfrac12\tilde\omega\tilde r$（定理 1(ii) 的四元数部），$\dot{\mathcal O}=-\mathrm{Im}(\tfrac12\tilde\omega\tilde r)=-\tfrac12(\tilde\eta I_3+[\mathcal O]_\times)\tilde\omega$（用 $\tilde\omega\times\tilde\mu=[\mathcal O]_\times\tilde\omega$）。平移通道：由 twist 约定 $\tilde v=\dot{\tilde p}+\tilde p\times\tilde\omega$ 得 $\dot{\mathcal T}=\tilde v-[\mathcal T]_\times\tilde\omega$。∎（一致性校验——代入 [P2] 理想闭环复现其指数稳定性——见附录 A.3。）

定理 1 + 定理 2 给出严格的两层级联：$e_z\xrightarrow{A}e_\xi$，$\dot e_z=Ae_\xi$；$\dot e_\xi$ 的动态由控制律与扰动决定（§5）。误差状态共 12 维——不含加速度层，这是 §4.2 论证的结构性取舍。

### 4.5 向下兼容

0 阶通道（$e_z$、$\tilde x$）与 [P2] 完全一致；速度接口下运行 [P2] 外环时，本体系只是把其隐含的 twist 误差显式化，不改变任何闭环性质。

---

## 5. 几何一致控制律与混合 H∞/ISS 性能

### 5.1 扰动通道：适定性与两类分解

内环以标称模型执行计算力矩，实际加速度 $\ddot{\boldsymbol q}=\ddot{\boldsymbol q}_{\mathrm{ref}}+\boldsymbol w_{\mathrm{dyn}}$，

$$
\boldsymbol w_{\mathrm{dyn}}=M^{-1}\bigl(\Delta M\ddot{\boldsymbol q}_{\mathrm{ref}}+\Delta C\dot{\boldsymbol q}+\Delta\boldsymbol g+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}}\bigr),
\tag{5.1}
$$

含与控制量成正比的乘性项。当 $\alpha\triangleq\sup_{\boldsymbol q}\|M^{-1}\Delta M\|_2<1$（经典计算力矩鲁棒性条件 [Spo92]）时反馈耦合可经 Neumann 级数解出，$\boldsymbol w_{\mathrm{dyn}}$ 整理为有界外生等效扰动，并按时间特性分解为 $\boldsymbol w_b\in L_\infty$（偏差型：参数误差、摩擦残差）与 $\boldsymbol w_{L_2}$（噪声型）。推导见附录 C.1。

### 5.2 控制律

取 $k_p>0$、$K_d$ 对称正定（若需定理 3(c-2) 的旋转/平移逐通道 H∞ 界，则取块对角 $K_d=\mathrm{diag}(K_\omega,K_v)$，$K_\omega,K_v\in\mathbb R^{3\times3}$ 对称正定），定义参考加速度（几何一致计算力矩律）：

$$
\ddot{\boldsymbol q}_{\mathrm{ref}}
=J^{+}\Bigl(\underbrace{\mathrm{vec}_6\bigl(\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d\bigr)}_{\text{前馈：搬运的期望加速度 + 输运修正}}
-K_d\,e_\xi-k_p\,A^{\top}(\tilde x)\,e_z-\dot J\dot{\boldsymbol q}\Bigr).
\tag{5.2}
$$

三点说明：(a) 前馈中 $\dot{\boldsymbol\xi}_d$ 来自期望轨迹（解析或期望链 TNDQ 表示 $\bar x_d$ 的 $\sigma^2$ 通道），$\mathrm{ad}$ 输运项由引理 1（下）决定——两者使前馈与误差动态中的非反馈项精确相消；(b) 位姿反馈经 $A^\top$ 整形，其目的将在定理 3 证明中显现（Lyapunov 交叉项精确相消）；(c) $\dot J\dot{\boldsymbol q}$ 由 TNDQ 链按 (3.5) 免构造获得。全部反馈量取自定理 1 的 HDQ 误差元素——不需要任何加速度误差的测量或估计。

> **引理 1（伴随输运公式）**：沿定理 1 的误差曲线，对任意光滑纯 DQ 曲线 $\boldsymbol a(t)$：
>
> $$
> \frac{d}{dt}\mathrm{Ad}_{\tilde x}\boldsymbol a=\mathrm{Ad}_{\tilde x}\dot{\boldsymbol a}+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\bigl(\mathrm{Ad}_{\tilde x}\boldsymbol a\bigr);
> \tag{5.3}
> $$
>
> 从而（含扰时右端加 $\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$）
>
> $$
> \dot{\tilde{\boldsymbol\xi}}=\dot{\boldsymbol\xi}-\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d-\mathrm{ad}_{\tilde{\boldsymbol\xi}}\bigl(\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d\bigr).
> \tag{5.4}
> $$
>
> **证明**：附录 A.4（由 $\dot{\tilde x}=\tfrac12\tilde{\boldsymbol\xi}\tilde x$ 与其共轭直接展开）。

### 5.3 主定理

本节四个结果（定理 3(a)–3(d)）共用如下设定：$J$ 行满秩（$JJ^+=I$）、$\alpha<1$，控制律取 (5.2)，扰动模型取 (5.1)；统一存储函数

$$
V=\tfrac12\|e_\xi\|^2+\tfrac{k_p}2\|e_z\|^2\;\ge0 .
$$

> **定理 3(a)（闭环误差动态）**：误差坐标 $(e_z,e_\xi)$ 满足级联标准形
>
> $$
> \dot e_z=A(\tilde x)\,e_\xi,\qquad
> \dot e_\xi=-K_d e_\xi-k_pA^{\top}(\tilde x)\,e_z+d(t),
> \tag{5.5}
> $$
>
> 其中 $d=J\boldsymbol w_{\mathrm{dyn}}+\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$ 汇集全部加速度层扰动。

> **证明**：
>
> 1. 由 (3.5) 与 $\ddot{\boldsymbol q}=\ddot{\boldsymbol q}_{\mathrm{ref}}+\boldsymbol w_{\mathrm{dyn}}$：$\mathrm{vec}_6\dot{\boldsymbol\xi}=J\ddot{\boldsymbol q}_{\mathrm{ref}}+\dot J\dot{\boldsymbol q}+J\boldsymbol w_{\mathrm{dyn}}$。
> 2. 代入 (5.2) 并用 $JJ^+=I$，$\dot J\dot{\boldsymbol q}$ 消去：
> $$
> \mathrm{vec}_6\dot{\boldsymbol\xi}=\mathrm{vec}_6\bigl(\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d\bigr)-K_de_\xi-k_pA^\top e_z+J\boldsymbol w_{\mathrm{dyn}} .
> $$
> 3. 与引理 1 的 (5.4)（含扰版）相减，前馈项与期望/输运项精确相消：$\dot e_\xi=\mathrm{vec}_6\dot{\tilde{\boldsymbol\xi}}=-K_de_\xi-k_pA^\top e_z+d$。
> 4. 配合定理 2 的 $\dot e_z=Ae_\xi$ 即得 (5.5)。∎

> **定理 3(b)（无扰指数稳定）**：$d\equiv0$ 时，$\dot V=-e_\xi^\top K_de_\xi\le0$，且在 $\tilde\eta>0$ 的工作域内 $(e_z,e_\xi)=(0,0)$ 指数稳定。

> **证明**：
>
> 1. 沿 (5.5)（$d\equiv0$）：$\dot V=-e_\xi^\top K_de_\xi-k_pe_\xi^\top A^\top e_z+k_pe_z^\top Ae_\xi=-e_\xi^\top K_de_\xi$，交叉项因 $A^\top$ 整形精确相消。
> 2. $\dot V\le0$ 给出 $(e_z,e_\xi)=(0,0)$ 的 Lyapunov 稳定性。
> 3. 指数收敛由级联结构与 $A$ 在 $\tilde\eta>0$ 域内的一致可逆性给出（附录 C.2，[Kha02] 形态）。∎

> **定理 3(c)（H∞ 通道：二次型/Schur 补判据与旋转/平移通道拆分）**：设 $d=d_{L_2}\in L_2$。
>
> **(c-1) 合并判据**（$K_d$ 任意对称正定）：性能目标 $\dot V\le-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$ 对一切 $(e_\xi,d)$ 成立**当且仅当**
>
> $$
> M\triangleq\begin{bmatrix}K_d-\tfrac1{2\kappa}I_6 & -\tfrac12 I_6\\[2pt] -\tfrac12 I_6 & \tfrac{\gamma_a^2}2 I_6\end{bmatrix}\succeq0
> \;\overset{\text{Schur}}{\Longleftrightarrow}\;
> K_d\succeq\tfrac12\bigl(\kappa^{-1}+\gamma_a^{-2}\bigr)I_6 ,
> \tag{5.6a}
> $$
>
> 此时
>
> $$
> \int_0^\infty\kappa^{-1}\|e_\xi\|^2dt\le\gamma_a^2\int_0^\infty\|d_{L_2}\|^2dt+2V(0),
> \tag{5.6}
> $$
>
> 即加速度层扰动到 twist 误差能量的 $L_2$ 增益 $\le\gamma_a\sqrt\kappa$（零初值 $V(0)=0$ 时退化为纯增益界；此界只约束 $e_\xi$，见注记 iv）。
>
> **(c-2) 通道拆分判据**（$K_d=\mathrm{diag}(K_\omega,K_v)$ 块对角）：记 $e_\xi=[\tilde\omega;\tilde v]$、$d=[d_\omega;d_v]$，$V_\omega\triangleq\tfrac12\|\tilde\omega\|^2+\tfrac{k_p}2\|\mathcal O\|^2$、$V_v\triangleq\tfrac12\|\tilde v\|^2+\tfrac{k_p}2\|\mathcal T\|^2$（$V=V_\omega+V_v$）。若
>
> $$
> K_\omega\succeq\tfrac12\bigl(\kappa_\omega^{-1}+\gamma_\omega^{-2}\bigr)I_3,
> \qquad
> K_v\succeq\tfrac12\bigl(\kappa_v^{-1}+\gamma_v^{-2}\bigr)I_3,
> \tag{5.6b}
> $$
>
> 则旋转/平移两通道**各自独立**满足
>
> $$
> \int_0^\infty\kappa_\omega^{-1}\|\tilde\omega\|^2dt\le\gamma_\omega^2\int_0^\infty\|d_\omega\|^2dt+2V_\omega(0),
> \qquad
> \int_0^\infty\kappa_v^{-1}\|\tilde v\|^2dt\le\gamma_v^2\int_0^\infty\|d_v\|^2dt+2V_v(0),
> \tag{5.6$'$}
> $$
>
> 且逐通道量纲齐次（不再混合 $(\mathrm{rad/s})^2$ 与 $(\mathrm{m/s})^2$），$\kappa_\omega,\gamma_\omega,\kappa_v,\gamma_v$ 可独立指定。
>
> **证明**：
>
> *第一步（$\dot V$ 精确式）*：含扰时定理 3(b) 证明第 1 步的交叉项相消与 $d$ 无关，故
> $$
> \dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d
> $$
> 精确成立（此式不含任何放缩；$e_\xi^\top d$ 可正可负）。
>
> *第二步（(c-1) 判据的等价性）*：性能目标 $\dot V\le-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$ 可以写成：
> $$-e_\xi^\top K_d e_\xi + e_\xi^\top d + \frac{1}{2\kappa}\|e_\xi\|^2 - \frac{\gamma_a^2}{2}\|d\|^2 \leq 0$$
> 等价于二次型不等式：
> $$
> \begin{bmatrix}e_\xi\\ d\end{bmatrix}^{\!\top}\!
> \begin{bmatrix}K_d-\tfrac1{2\kappa}I & -\tfrac12 I\\ -\tfrac12 I & \tfrac{\gamma_a^2}2 I\end{bmatrix}\!
> \begin{bmatrix}e_\xi\\ d\end{bmatrix}\ge0\quad\forall(e_\xi,d)\in\mathbb R^{12},
> $$
> 即 $M\succeq0$。右下块 $\tfrac{\gamma_a^2}2I\succ0$，取 Schur 补：$M\succeq0\iff K_d-\tfrac1{2\kappa}I-\tfrac14\cdot\tfrac{2}{\gamma_a^2}I\succeq0$，即 (5.6a)。不定号交叉项 $e_\xi^\top d$ 保留在二次型内整体判定，不经任何符号放缩（Schur 补、配方法与 Young 三条判定路径的等价性见附录 C.3）。
>
> *第三步（全局存在性）*：由 $M\succeq0$ 得 $\dot V\le\tfrac{\gamma_a^2}2\|d\|^2$，故 $V(t)\le V(0)+\tfrac{\gamma_a^2}2\|d_{L_2}\|_{L_2}^2<\infty$，$(e_z,e_\xi)$ 一致有界，解在 $[0,\infty)$ 上存在（无有限时间逃逸）。
>
> *第四步（积分收尾）*：在 $[0,T]$ 上积分 $\dot V\le-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$，弃去 $V(T)\ge0$，令 $T\to\infty$（单调收敛）即得 (5.6)。
>
> *第五步（(c-2) 通道解耦）*：块对角 $K_d$ 下两通道储能函数精确解耦，关键是两处三重积恒零。由 (4.5)，$A^\top=\begin{bmatrix}A_{11}^\top & [\mathcal T]_\times\\ 0 & I\end{bmatrix}$（$A_{11}=-\tfrac12(\tilde\eta I+[\mathcal O]_\times)$，$(-[\mathcal T]_\times)^\top=[\mathcal T]_\times$），故
> $$
> (A^\top e_z)_\omega=A_{11}^\top\mathcal O+\underbrace{[\mathcal T]_\times\mathcal T}_{=\,\mathcal T\times\mathcal T\,=\,0}=A_{11}^\top\mathcal O,
> \qquad
> (A^\top e_z)_v=\mathcal T,
> $$
> 故 $(A^\top e_z)_\omega$ 不含 $\mathcal T$；又 $\dot{\mathcal T}=-[\mathcal T]_\times\tilde\omega+\tilde v$ 中的耦合项做功为零：$\mathcal T^\top[\mathcal T]_\times\tilde\omega=\mathcal T\cdot(\mathcal T\times\tilde\omega)=0$（与附录 A.3 “叉积项与 $\mathcal T$ 正交”同机制）。于是
> $$
> \dot V_\omega=-\tilde\omega^\top K_\omega\tilde\omega
> \underbrace{-k_p\tilde\omega^\top A_{11}^\top\mathcal O+k_p\mathcal O^\top A_{11}\tilde\omega}_{=0}
> +\tilde\omega^\top d_\omega,
> \qquad
> \dot V_v=-\tilde v^\top K_v\tilde v
> \underbrace{-k_p\tilde v^\top\mathcal T+k_p\mathcal T^\top\tilde v}_{=0}
> +\tilde v^\top d_v ,
> $$
> 对每条通道重复第二至第四步的论证（$I_6\to I_3$）即得 (5.6b)⇒(5.6$'$)。两处恒零为代数恒等式，故 (c-1)/(c-2) 的全部结论均不依赖工作域 $\tilde\eta>0$（见注记 v）。∎

> **定理 3(d)（ISS 通道）**：$d=d_b\in L_\infty$ 时系统对 $d_b$ 输入-状态稳定，
>
> $$
> \limsup_{t\to\infty}\|e_\xi\|\le\frac{\|d_b\|_\infty}{\lambda_{\min}(K_d)},
> \tag{5.7}
> $$
>
> 并经级联传至 $e_z$ 的极限球。偏差型不确定性不破坏稳定性，只决定稳态误差球半径。

> **证明**：
>
> 1. 由定理 3(c) 证明第一步与 Cauchy–Schwarz：$\dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d_b\le-\lambda_{\min}(K_d)\|e_\xi\|^2+\|e_\xi\|\,\|d_b\|_\infty$。
> 2. 当 $\|e_\xi\|>\|d_b\|_\infty/\lambda_{\min}(K_d)$ 时 $\dot V<0$，轨迹进入并停留于该球（[Kha02] Thm 4.19 形态），即 (5.7)。
> 3. $e_z$ 的极限球经级联 $\dot e_z=Ae_\xi$ 与 $\sigma_{\min}(A_0)=\tfrac12$ 传递。∎

> **注记（Schur 判据 vs. Young 放缩、以及最紧可证增益）**：(i) 在各向同性罚权下二者恰好重合：(5.6a) 即 $\lambda_{\min}(K_d)\ge\tfrac12(\gamma_a^{-2}+\kappa^{-1})$，与 Young 路的标量条件相同——矩阵判据在**同一**供给率下不提供额外自由度；其真正收益是 (a) 从构造上免除符号放缩、(b) 罚权分块化后自然产出 (c-2) 的通道拆分（各向异性设计空间来自供给率加权的分块化，而非同一各向同性目标的改写）。(ii) 供给率整体缩放 $\theta>0$（目标 $\dot V\le-\theta\kappa^{-1}\|e_\xi\|^2+\theta\gamma_a^2\|d\|^2$，认证同一增益 $\gamma_a\sqrt\kappa$、偏置项 $V(0)/\theta$）给出条件族 $K_d\succeq(\theta\kappa^{-1}+\tfrac1{4\theta}\gamma_a^{-2})I$；对 $\theta$ 极小化（$\theta^*=\sqrt\kappa/2\gamma_a$）得**最紧可证条件**
> $$
> \lambda_{\min}(K_d)\ \ge\ \frac{1}{\gamma_a\sqrt\kappa}
> \qquad\Longleftrightarrow\qquad
> \text{认证 }L_2\text{ 增益 }\le\ \frac{1}{\lambda_{\min}(K_d)} .
> $$
> 由 AM–GM，$\tfrac12(\gamma_a^{-2}+\kappa^{-1})\ge(\gamma_a\sqrt\kappa)^{-1}$，等号当且仅当 $\kappa=\gamma_a^2$——(5.6a) 是该族在 $\theta=\tfrac12$ 处的成员，仅在 $\kappa=\gamma_a^2$ 时最紧（推导见附录 C.3）。最紧界与定理 3(d) 的 ISS 界 $\|d_b\|_\infty/\lambda_{\min}(K_d)$ 及线性极限 $\|(sI+K_d)^{-1}\|_{H_\infty}=1/\lambda_{\min}(K_d)$（$K_d$ 对称时）一致。
>
> **注记（诚实边界）**：(i) 奇异邻域内 $J^+$ 用阻尼伪逆时 $JJ^+\ne I$，残差归入 $d(t)$；(ii) 本定理是内环单层的严格结果 + 级联 ISS 结论，**不**声称与运动学外环级联后的整体 H∞ 界（开放问题）；(iii) Lyapunov/ISS 技术本身是标准的，本文的新内容在误差坐标的选择（HDQ 误差元素 + $A^\top$ 整形使两处相消都精确成立）与两类扰动的通道化归属；(iv) (5.6)/(5.6$'$) 只约束 $e_\xi$ 而非 $e_z$——扰动到 $e_z$ 的相对阶为 2，当前 $V$ 的导数中无 $-\|e_z\|^2$ 项；若需 $e_z$ 的直接 $L_2$ 界须在 $V$ 中加交叉项（strictification，形如 $\epsilon\,e_z^\top Ae_\xi$）并处理 $\dot A$，留作后续，$e_z$ 现经级联 $\dot e_z=Ae_\xi$ 获 ISS 型界；(v) 含扰时轨迹可能离开 $\tilde\eta>0$（unwinding 域边界），定理 3(c) 的 twist 误差界仍成立，但该域外不附带任何位姿收敛结论；(vi) 通道拆分 (c-2) 依赖 $K_d$ 块对角与 $[\mathcal T]_\times\mathcal T=0$、$\mathcal T^\top[\mathcal T]_\times\tilde\omega=0$ 两处恒零，一般正定 $K_d$ 退回合并判据 (c-1)。

### 5.4 与运动学外环的级联

外环照旧运行 [P2] 控制律（保持其 H∞ 保证），内环 (5.2) 以期望轨迹（或外环整形后的参考）为输入。内环 ISS + 外环对执行残差的 H∞ 鲁棒性 ⟹ 级联系统对两类扰动分别保持 L₂ 界与极限球界（标准级联 ISS 定理）。内环的效果等价于把外环感受到的速度级扰动变小。

---

## 6. 模拟验证流程设计（本稿仅流程，不含数据）

### 6.1 平台与对象

7R 串联机械臂（KUKA-like 构型，项目 `configs/kuka_like_7r.py`）；CoppeliaSim 力矩模式（或先以关节速度模式验证运动学级退化情形）；控制周期 2 ms（500 Hz）。

### 6.2 信息流水线（输入 → 计算 → 输出 → 反馈闭环）

```
输入层（每周期）
  ├─ 关节编码器 → q ∈ R⁷
  ├─ 速度估计（同一观测器）→ q̇ ∈ R⁷
  └─ 期望轨迹发生器（解析）→ x_d(t), ẋ_d(t), ξ_d(t), ξ̇_d(t)

正运动学层（TNDQ 链，O(n)）
  ├─ 实测链：x̄ = Π x̄_i(q,q̇,q̈=0) → 通道读数 x, ẋ；(3.5) 免构造读出 J̇q̇
  ├─ HDQ 截断：取前两通道 → x̆ = x + ε*ẋ（命题 2：无损）
  └─ 雅可比 J(q)（[P1] 前缀/后缀结构）；约束残差 c₀,c₁,c₂ 监测（§3.4，σ² 通道在用故含 c₂）

误差层（HDQ 运算，定理 1/2）
  ├─ 一次 HDQ 乘法：x̆̃ = x̆·(x̆_d)* → x̃, dx̃/dt（3 次 DQ 乘）
  ├─ e_z = [O; T]（0 阶通道，[P2] 原样）
  ├─ e_ξ = vec₆(2·dx̃/dt·x̃*)（几何一致 twist 误差）
  └─ A(x̃) 拼装（定理 2 的 3×3 块）

控制层（(5.2)，定理 3）
  ├─ 前馈：vec₆(Ad_x̃ ξ̇_d + ad_ξ̃ Ad_x̃ ξ_d)（引理 1；全为 DQ 乘法）
  ├─ 反馈：−K_d e_ξ − k_p Aᵀ e_z
  ├─ q̈_ref = J⁺(前馈 + 反馈 − J̇q̇)
  └─ τ = M̂ q̈_ref + Ĉ q̇ + ĝ（标称模型）

输出层
  └─ τ → CoppeliaSim 关节力矩接口（力矩模式）

反馈闭环
  └─ 仿真器推进一步 → 新的 q, q̇ 回到输入层；全部日志（e_z, e_ξ, V, c₀, c₁, τ, 运行时间）落盘
```

### 6.3 实验序列与每步的输入/输出/判据

| # | 实验 | 输入 | 输出 | 判据 / 反馈到哪 |
|---|---|---|---|---|
| E0 | 代数单元验证 | 随机 $(\boldsymbol q,\dot{\boldsymbol q},\ddot{\boldsymbol q})$ | TNDQ 通道值 vs. 数值差分残差 | 命题 1/2、定理 1 逐式到机器精度；不通过则修代数层 |
| E1 | 截断一致性 | 随机曲线对 | TNDQ 乘后截断 vs. 截断后 HDQ 乘 | 逐位相等（命题 2）；理论合法性的直接数值证据 |
| E2 | 误差运动学校验 | 无控自由运动轨迹 | $\dot e_z$ vs. $Ae_\xi$ 曲线 | 定理 2 残差 $O(\Delta t)$；偏离即相位错位签名（反馈到观测器对齐） |
| E3 | 无扰跟踪 | 直线/圆轨迹（`trajectory_line/circle`） | $e_z,e_\xi,V(t)$ | $\dot V\le-\lambda_{\min}(K_d)\|e_\xi\|^2$ 逐步成立（定理 3(b)）；增益整定反馈到 $K_d,k_p$ |
| E4 | L₂ 扰动 | 有限能量 twist 扰动注入 | 合并能量比 $\int\|e_\xi\|^2/\int\|d\|^2$；逐通道 $\int\|\tilde\omega\|^2/\int\|d_\omega\|^2$、$\int\|\tilde v\|^2/\int\|d_v\|^2$ | 实测增益 ≤ 理论 $\gamma_a^2\kappa$（定理 3(c-1)）与 $\gamma_\omega^2\kappa_\omega,\gamma_v^2\kappa_v$（定理 3(c-2)）；对照最紧能量界 $\lambda_{\min}(K_d)^{-2}$（注记） |
| E5 | 偏差扰动 | 惯性参数人为偏置 5–10%（$\alpha$ 已知） | 稳态 $\|e_\xi\|_\infty$ | 实测球半径 ≤ (5.7) 预算；反解 $K_d$ 设计流程闭环 |
| E6 | 对照组 | 同轨迹 | 朴素差 $\boldsymbol\xi-\boldsymbol\xi_d$ vs. $e_\xi$ 的伪项曲线；[P2] 纯运动学环 vs. 本文内外环级联 | 伪项随 $\|\boldsymbol\xi_d\|$ 线性增长的实证；高速工况收益量化 |
| E7 | 运行时间 | 各层计时 | TNDQ 链 / HDQ 误差 / 控制律逐层耗时 | 500 Hz 预算内；与 DQ 基线对比 |

### 6.4 预期呈现（论文最终版补充）

跟踪误差曲线（平移/姿态分量）、$V(t)$ 与理论包络、L₂ 增益实测/理论对比条形图、ISS 球半径 vs. $\alpha$ 曲线、伪项量级 vs. 期望速度、逐层运行时间栈状图。

---

## 7. 结论

本文以截断多项式代数 $\mathcal A_2$（TNDQ）重构机械臂运动学，核心是两条法则：连乘法则 $\overline{xy}=\bar x\,\bar y$（使位姿/速度/加速度一次链连乘同时得到）与截断相容性 $\breve x=$"$\bar x$ 的前两通道"（使误差体系可以无损地定义在两通道 HDQ 上）。误差体系由一次 HDQ 乘法生成（定理 1），经输出映射闭合为级联运动学（定理 2）；几何一致计算力矩律使闭环达到级联标准形，对噪声型与偏差型扰动分别给出 H∞（二次型/Schur 补判据，旋转/平移逐通道精确拆分）与 ISS 保证（定理 3）。加速度层被证明不需要误差通道：期望加速度走前馈、不确定性走扰动——这一结构性取舍同时简化了状态空间（12 维）与实现（误差层只用 DQ/HDQ 乘法）。

**局限与后续工作**：(i) 级联系统（内环 + 运动学外环）的整体 H∞ 界未建立；(ii) 变权存储函数（操作空间惯性 $\Lambda$ 加权）需处理 $\dot\Lambda$ 项，本文未展开；(iii) 模拟与实机验证（§6 流程）为下一版内容；(iv) 定理 1 的几何一致误差与 Adorno 学派 DQ 动力学控制文献的逐条查重在投稿前完成。

---

## 附录 A：DQ/HDQ 层的验证性推导

### A.1 twist 的纯性

对 $\hat{\underline x}\hat{\underline x}^*=1$ 求导：$\dot{\hat{\underline x}}\hat{\underline x}^*+\hat{\underline x}\dot{\hat{\underline x}}^*=0$，而 $(\dot{\hat{\underline x}}\hat{\underline x}^*)^*=\hat{\underline x}\dot{\hat{\underline x}}^*$，故 $\boldsymbol\xi=2\dot{\hat{\underline x}}\hat{\underline x}^*$ 反自共轭（标量部与对偶标量部为零），即纯 DQ。同理适用于误差曲线 $\tilde x(t)$。

### A.2 定理 1 的 (i)(ii)(iii)

(ii)：由 (4.3) 直接右乘 $\tilde x$。(i)：纯性由 A.1；翻转不变性：$2(-\dot{\tilde x})(-\tilde x^*)=2\dot{\tilde x}\tilde x^*$。(iii)：$\dot{\tilde x}=\dot{\hat{\underline x}}\hat{\underline x}_d^{\,*}+\hat{\underline x}\dot{\hat{\underline x}}_d^{\,*}$；用 $\dot{\hat{\underline x}}=\tfrac12\boldsymbol\xi\hat{\underline x}$ 与 $\dot{\hat{\underline x}}_d^{\,*}=-\tfrac12\hat{\underline x}_d^{\,*}\boldsymbol\xi_d$（后者由 $\dot{\hat{\underline x}}_d=\tfrac12\boldsymbol\xi_d\hat{\underline x}_d$ 取共轭）：
$\dot{\tilde x}=\tfrac12\boldsymbol\xi\tilde x-\tfrac12\hat{\underline x}\hat{\underline x}_d^{\,*}\boldsymbol\xi_d=\tfrac12\boldsymbol\xi\tilde x-\tfrac12\tilde x\boldsymbol\xi_d$。右乘 $2\tilde x^*$：$\tilde{\boldsymbol\xi}=\boldsymbol\xi-\tilde x\boldsymbol\xi_d\tilde x^*=\boldsymbol\xi-\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d$。含速度级扰动 $\boldsymbol v_w,\boldsymbol v_c$（[P2] 模型）时右端加 $\boldsymbol v_w+\boldsymbol v_c$。

### A.3 定理 2 与 [P2] 稳定性的一致性校验

代入 [P2] 理想闭环 $\tilde\omega=\kappa_{\mathcal O}\mathcal O$、$\tilde v=-\kappa_{\mathcal T}\mathcal T$：旋转通道 $\dot{\mathcal O}=-\tfrac12\kappa_{\mathcal O}\tilde\eta\mathcal O$（$[\mathcal O]_\times\mathcal O=0$），$\tilde\eta>0$ 时指数稳定——恰为 [P2] Remark 1 的 unwinding 条件；平移通道 $\tfrac{d}{dt}\|\mathcal T\|^2=2\mathcal T^\top(-[\mathcal T]_\times\tilde\omega-\kappa_{\mathcal T}\mathcal T)=-2\kappa_{\mathcal T}\|\mathcal T\|^2$（叉积项与 $\mathcal T$ 正交）。定理 2 复现 [P2] 的稳定性结论，确认 $A(\tilde x)$ 的正确性。

### A.4 引理 1 的证明

$\tfrac{d}{dt}(\tilde x\boldsymbol a\tilde x^*)
=\dot{\tilde x}\boldsymbol a\tilde x^*+\tilde x\dot{\boldsymbol a}\tilde x^*+\tilde x\boldsymbol a\dot{\tilde x}^*$。代入 $\dot{\tilde x}=\tfrac12\tilde{\boldsymbol\xi}\tilde x$、$\dot{\tilde x}^*=-\tfrac12\tilde x^*\tilde{\boldsymbol\xi}$：
$=\tfrac12\tilde{\boldsymbol\xi}(\mathrm{Ad}_{\tilde x}\boldsymbol a)+\mathrm{Ad}_{\tilde x}\dot{\boldsymbol a}-\tfrac12(\mathrm{Ad}_{\tilde x}\boldsymbol a)\tilde{\boldsymbol\xi}
=\mathrm{Ad}_{\tilde x}\dot{\boldsymbol a}+\mathrm{ad}_{\tilde{\boldsymbol\xi}}(\mathrm{Ad}_{\tilde x}\boldsymbol a)$。(5.4)：对 (4.4) 逐项求导并取 $\boldsymbol a=\boldsymbol\xi_d$。∎

## 附录 B：TNDQ 链的构件

### B.1 单关节因子的 TNDQ 表示

旋转关节 $i$ 的 DQ 因子 $\hat{\underline x}_i(q_i)$ 沿单位螺旋轴 $\boldsymbol s_i$（纯 DQ）满足 $\partial\hat{\underline x}_i/\partial q_i=\tfrac12\boldsymbol s_i\hat{\underline x}_i$，故沿时间曲线
$\dot{\hat{\underline x}}_i=\tfrac12\dot q_i\boldsymbol s_i\hat{\underline x}_i$，$\ddot{\hat{\underline x}}_i=\tfrac12\ddot q_i\boldsymbol s_i\hat{\underline x}_i+\tfrac14\dot q_i^2\boldsymbol s_i^2\hat{\underline x}_i$——三个通道均为闭式，只依赖 $(q_i,\dot q_i,\ddot q_i)$。把 $\bar x_i=\hat{\underline x}_i+\sigma\dot{\hat{\underline x}}_i+\tfrac12\sigma^2\ddot{\hat{\underline x}}_i$ 依 (3.4) 连乘即得全链。

### B.2 (3.5) 第二式的推导

对 $\boldsymbol\xi=2\dot{\hat{\underline x}}\hat{\underline x}^*$ 求导：$\dot{\boldsymbol\xi}=2\ddot{\hat{\underline x}}\hat{\underline x}^*+2\dot{\hat{\underline x}}\dot{\hat{\underline x}}^*$。由 $\dot{\hat{\underline x}}=\tfrac12\boldsymbol\xi\hat{\underline x}$、$\dot{\hat{\underline x}}^*=-\tfrac12\hat{\underline x}^*\boldsymbol\xi$：$2\dot{\hat{\underline x}}\dot{\hat{\underline x}}^*=-\tfrac12\boldsymbol\xi^2$。$\boldsymbol\xi^2$ 的纯部为零（纯 DQ 的平方是"对偶标量"型），故取纯部即得 $\dot{\boldsymbol\xi}$；$\mathrm{vec}_6$ 层直接 $\mathrm{vec}_6\dot{\boldsymbol\xi}=\dot J\dot{\boldsymbol q}+J\ddot{\boldsymbol q}$（对 $\mathrm{vec}_6\boldsymbol\xi=J\dot{\boldsymbol q}$ 求导）。

## 附录 C：控制层的补充推导

### C.1 扰动通道适定性（(5.1) 与 α 条件）

$\boldsymbol\tau=\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+\hat C\dot{\boldsymbol q}+\hat g$ 代入真实动力学并解出 $\ddot{\boldsymbol q}$：
$\ddot{\boldsymbol q}=M^{-1}\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+M^{-1}(\Delta C\dot{\boldsymbol q}+\Delta\boldsymbol g+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}})$，而 $M^{-1}\hat M=I+M^{-1}\Delta M$，即 (5.1)。$\ddot{\boldsymbol q}_{\mathrm{ref}}$ 本身含反馈项，故 (5.1) 是隐式方程；$\alpha<1$ 时把乘性项移项，$(I+M^{-1}\Delta M)$ 可逆（Neumann 级数），解出的等效扰动增益 $\le\alpha/(1-\alpha)$ 倍反馈量 + 有界外生项，闭环适定。分解 $\boldsymbol w_b/\boldsymbol w_{L_2}$ 按各源时间特性归类：参数误差与摩擦残差持续存在（$L_\infty$），测量噪声能量有限或可白化（$L_2$ 类）。

### C.2 定理 3(b) 的指数收敛细节

$\dot V=-e_\xi^\top K_de_\xi$ 只是半负定（$e_\xi=0$ 面上 $\dot V=0$）；用级联论证补足：在 $\tilde\eta\ge\eta_0>0$ 的工作域内 $\sigma_{\min}(A)\ge c(\eta_0)>0$，$e_\xi\to0$ 经 $\dot e_\xi$ 方程强制 $A^\top e_z\to0$ 即 $e_z\to0$（LaSalle / 级联 ISS 均可闭合，[Kha02]）；线性化在 $(0,0)$ 处的谱在开左半平面，给出局部指数率。域限制 $\tilde\eta>0$ 与 [P2] Remark 1 的 unwinding 条件一致。

### C.3 定理 3(c) 的二次型/Schur 补细节与最紧增益族

**(c-1) 三条等价路径。** 记供给率 $s(e_\xi,d)\triangleq-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$。性能目标 $\dot V\le s$（对一切 $(e_\xi,d)$ 逐点成立）等价于 $-e_\xi^\top K_de_\xi+e_\xi^\top d-s(e_\xi,d)\le0\ \forall(e_\xi,d)$，可经三条路径判定：

1. **二次型/Schur 补**（正文路径）：整理为 $-[e_\xi;d]^\top M[e_\xi;d]\le0$，其中 $M$ 为 (5.6a) 的分块矩阵。$M\succeq0$ 且右下块 $\tfrac{\gamma_a^2}2I\succ0$，取 Schur 补：$M\succeq0\iff K_d-\tfrac1{2\kappa}I-\tfrac12(\tfrac{\gamma_a^2}2)^{-1}\!\cdot\tfrac14 I\succeq0\iff K_d\succeq\tfrac12(\kappa^{-1}+\gamma_a^{-2})I$。这是**当且仅当**判据：不定号交叉项 $e_\xi^\top d$ 保留在二次型内整体判定，无任何符号放缩。
2. **配方法**：对 $\dot V$ 直接完成平方，
$$
\dot V=-\Bigl\|K_d^{1/2}e_\xi-\tfrac12K_d^{-1/2}d\Bigr\|^2+\tfrac14 d^\top K_d^{-1}d\ \le\ \tfrac14 d^\top K_d^{-1}d ,
$$
放缩残差全部集中在唯一正定项 $\tfrac14 d^\top K_d^{-1}d$（交叉项符号被完全平方吸收，透明可见）。要求 $\dot V\le s$，即把 $-\tfrac1{2\kappa}\|e_\xi\|^2$ 从 $-e_\xi^\top K_de_\xi$ 中先行剥出后对余下部分完成平方，条件化简后与路径 1 相同。
3. **Young 放缩**：$e_\xi^\top d\le|e_\xi^\top d|\le\|e_\xi\|\|d\|\le\tfrac1{2\gamma_a^2}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$（绝对值一步不可省——Cauchy–Schwarz 后的 AM–GM 对双向符号有效，但书写上必须经过 $|e_\xi^\top d|$），再要求 $K_d$ 吸收 $\tfrac1{2\gamma_a^2}\|e_\xi\|^2$ 并余出 $\tfrac1{2\kappa}\|e_\xi\|^2$。它一般只给充分条件，但在此处**各向同性罚权**（$\|e_\xi\|^2,\|d\|^2$ 取欧氏范数、系数各带 $\tfrac12$）下与路径 1 的当且仅当条件重合——Young/AM–GM 在最优参数下对二元各向同性二次型是紧的。

**关于"矩阵判据更少保守"的澄清。** 在**同一**各向同性供给率下，$M\succeq0$ 与标量条件 $\lambda_{\min}(K_d)\ge\tfrac12(\kappa^{-1}+\gamma_a^{-2})$ 完全等价（对 $M$ 左右夹 $[u;u']$ 型向量即可见每个特征方向须独立满足同一标量不等式），矩阵形式**不**提供额外的各向异性自由度。真正的各向异性设计空间来自**供给率本身的分块加权**——每通道取各自的 $(\kappa_i,\gamma_i)$，此时判据变为分块 Schur 条件，(c-2) 正是其按 $\tilde\omega/\tilde v$ 二分块的实现。

**θ 缩放族与最紧可证条件。** 供给率整体缩放 $\theta>0$：目标 $\dot V\le-\theta\kappa^{-1}\|e_\xi\|^2+\theta\gamma_a^2\|d\|^2$ 积分后两边除以 $\theta$，认证的仍是同一增益 $\gamma_a\sqrt\kappa$（偏置项变为 $2V(0)/\theta$，零初值下无差别）。对每个 $\theta$，路径 1 的 Schur 判据给出
$$
K_d\succeq f(\theta)\,I,\qquad f(\theta)=\theta\kappa^{-1}+\tfrac1{4\theta}\gamma_a^{-2} .
$$
$\theta=\tfrac12$ 即正文 (5.6a)（两边各带 $\tfrac12$ 的标准供给率）；$\theta=1$ 给出变体 $K_d\succeq(\kappa^{-1}+\tfrac14\gamma_a^{-2})I$——两者是同一族的两个成员，在不同 $(\kappa,\gamma_a)$ 区域各有松紧，谈不上孰更严谨。对 $\theta$ 极小化：$f'(\theta)=\kappa^{-1}-\tfrac1{4\theta^2}\gamma_a^{-2}=0\Rightarrow\theta^*=\tfrac{\sqrt\kappa}{2\gamma_a}$，$f(\theta^*)=\tfrac1{\gamma_a\sqrt\kappa}$，故本路径的**最紧可证条件**为
$$
\lambda_{\min}(K_d)\ \ge\ \frac1{\gamma_a\sqrt\kappa}
\qquad\Longleftrightarrow\qquad
\text{认证 }L_2\text{ 增益}\ \le\ \frac1{\lambda_{\min}(K_d)} .
$$
由 AM–GM，$\tfrac12(\gamma_a^{-2}+\kappa^{-1})\ge(\gamma_a\sqrt\kappa)^{-1}$，等号当且仅当 $\kappa=\gamma_a^2$——(5.6a) 仅在 $\kappa=\gamma_a^2$ 时达到族内最紧。最紧界与 (d) 的 ISS 界 $\|d_b\|_\infty/\lambda_{\min}(K_d)$、以及 $e_\xi$ 子系统线性极限的 $\|(sI+K_d)^{-1}\|_{H_\infty}=1/\lambda_{\min}(K_d)$（$K_d$ 对称时）三者一致，说明 $1/\lambda_{\min}(K_d)$ 是这条 Lyapunov 路径可证增益的天花板。

**(c-2) 两处恒零的几何含义与失效条件。** 拆分精确成立依赖两条叉积混合积恒等式：(i) $(A^\top e_z)_\omega$ 中 $[\mathcal T]_\times\mathcal T=\mathcal T\times\mathcal T=0$——平移误差对旋转反馈通道的耦合以"作用在自身上的叉积"形式进入，恒零；(ii) $\dot{\mathcal T}=-[\mathcal T]_\times\tilde\omega+\tilde v$ 中耦合项对 $\tfrac12\|\mathcal T\|^2$ 不做功，$\mathcal T^\top(\mathcal T\times\tilde\omega)=0$——与附录 A.3 中 [P2] 平移通道"叉积项与 $\mathcal T$ 正交"同一机制。二者是代数恒等式，不依赖工作域，故 (c-2) 与 (c-1) 一样是全局（$\tilde\eta$ 无关）结论。若 $K_d$ 非块对角，$-e_\xi^\top K_de_\xi$ 含 $\tilde\omega^\top K_{\omega v}\tilde v$ 型交叉项，两通道能量不再分离，退回合并判据 (c-1)。

---

## 参考文献

1. **[P1]** A. Cohen, M. Shoham, *Hyper Dual Quaternions representation of rigid bodies kinematics*, Mechanism and Machine Theory 150 (2020) 103861.
2. **[P2]** L.F.C. Figueredo, B.V. Adorno, J.Y. Ishihara, *Robust H∞ kinematic control of manipulator robots using dual quaternion algebra*, Automatica 132 (2021) 109817.
3. J.A. Fike, J.J. Alonso, *The Development of Hyper-Dual Numbers for Exact Second-Derivative Calculations*, AIAA 2011-886.
4. K.M. Lynch, F.C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017.
5. O. Khatib, *A unified approach for motion and force control of robot manipulators: The operational space formulation*, IEEE J. Robotics and Automation 3(1), 1987.
6. **[Spo92]** M.W. Spong, *On the robust control of robot manipulators*, IEEE Trans. Automatic Control 37(11), 1992.
7. **[Kha02]** H.K. Khalil, *Nonlinear Systems*, 3rd ed., Prentice Hall, 2002.
8. 项目文档：主文档 `docs/数学理论与代码实现详解.md`；扩展篇 `docs/HDQ动力学建模扩展_Jdot与Hessian.md`；误差篇 `docs/HDQ动力学误差体系重构_几何一致二阶误差方案.md`。
