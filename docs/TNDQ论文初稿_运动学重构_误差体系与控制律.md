# Trident Number Dual Quaternion (TNDQ) Modeling of Robot Kinematics with a Geometrically Consistent Error System and Mixed H∞/ISS Control

> **文稿性质**：论文初稿（第一版）。结构参照 Figueredo, Adorno & Ishihara, *Robust H∞ kinematic control of manipulator robots using dual quaternion algebra*, Automatica 132 (2021)（下称 [P2]）：摘要 → 引言（含贡献声明）→ 预备知识 → 主体理论（TNDQ 运动学 / 误差体系 / 控制律主定理）→ 仿真验证 → 结论 → 附录（次要推导）。
>
> 理论内容取自项目文档体系（编号沿用）：扩展篇 `docs/HDQ动力学建模扩展_Jdot与Hessian.md`（(D-k)、(5.2′)）、误差篇 `docs/HDQ动力学误差体系重构_几何一致二阶误差方案.md`（(F-k)、TNDQ/HDQ 截断）。文献编号：[P1] = Cohen & Shoham MMT 2020；[P2] = Figueredo et al. Automatica 2021。
>
> **记号说明**：本稿采用统一装饰记号（§2 表 0）——$\hat a$ 单位四元数、$\hat{\underline a}$ 单位 DQ、$\breve a$ HDQ、$\bar a$ TNDQ。源文档中的算子记号 $T^1\boldsymbol x$、$T^2\boldsymbol x$、$\Pi_{\mathrm{HDQ}}$ 在本稿分别写作 $\breve x$、$\bar x$ 与"取前两通道截断"。
>
> **写作约定**：面向具备本科代数（环、商环）与常微分方程基础的数学系读者；机器人学专有概念（位姿、twist、雅可比）在首次出现处给出数学定义；工程细节（驱动接口、采样与限幅）只在仿真验证一节出现。

---

## 摘要

对偶四元数（DQ）代数为机械臂位姿运动学提供了紧凑的全局无奇异参数化，[P2] 在其上建立了运动学 H∞ 跟踪控制。然而 DQ 体系只携带位姿一阶信息：速度与加速度层的量必须另行构造，误差体系也只有位姿一层，进入动力学控制（力矩接口）时出现三个结构性缺口——位姿与速度误差相互分离（[Ch20] 类律的位姿反馈取螺旋对数，其导数映射在大误差处奇异且无对任意正定增益成立的耗散等式结构）、加速度层扰动无入口、偏差型不确定性不满足 L₂ 假设。本文引入**三叉对偶四元数**（Trident Number Dual Quaternion, TNDQ）：以三个 DQ 通道（位姿/一阶导/二阶导）为元素的截断多项式代数 $\mathcal A_2=\widehat{\mathbb H}[\sigma]/(\sigma^3)$，证明位姿曲线的 TNDQ 表示 $\bar x$ 满足连乘法则 $\overline{xy}=\bar x\,\bar y$，从而串联链的 $\bar x$ 由各关节因子 $\bar x_i$ 一次连乘获得，位姿、twist、任务空间加速度与 $\dot J\dot{\boldsymbol q}$ 同批输出。在此基础上，本文证明取前两通道的 HDQ 截断 $\bar x\mapsto\breve x$ 与乘法相容（先乘后截 = 先截后乘），据此把**误差体系定义在 HDQ 截断上**：一次 HDQ 乘法同时生成右不变位姿误差与几何一致 twist 误差（定理 1），二者经输出映射闭合为严格级联误差运动学（定理 2）。针对动力学接口设计几何一致计算力矩律，证明闭环误差动态为级联标准形且统一存储函数满足**精确耗散等式** $\dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d$（无任何放缩），据此：无扰时给出工作域水平集的正向不变性、渐近收敛与局部指数稳定；对噪声型（$L_2$）扰动给出 H∞ 增益的 Schur 补**充要**判据（含旋转/平移逐通道拆分）；对偏差型（$L_\infty$）扰动给出 twist 误差的**均方（RMS）极限界**（定理 3）。扰动通道 $d(t)$ 中与控制量成正比的乘性分量被显式分离并以小增益参数 $\alpha$ 刻画，不再默认为外生信号。误差状态不含加速度层：期望加速度作为前馈、加速度层不确定性作为扰动——本文给出这一取舍的结构性论证（反馈只消费两阶误差、闭环两阶已足够），不声称其为可证的信息无损性。最后在 CoppeliaSim/KUKA LBR4+ 力矩模式仿真中以抓取-搬运-圆周跟踪任务完成验证：近恒等线性化通道的静态刚度标度律以 2% 精度获得证实（两个独立增益档由稳态残差反演出同一等效扰动幅值），ISS 均方界成立且保守约 1.5 个数量级，所提控制律在带载敏感条件（高速/快搬运/噪声/粗采样）的 10 组两两对比中速度级指标全部占优：相对一阶桥接基线稳定优 1.5%–2.2%，相对忠实 [Ch20] 二阶基线数值等价（≤0.05%，两律同信息集的结构分析由此获得定量确认，本文的差异化主张为同性能下的耗散等式证书与大误差几何鲁棒性）。

**关键词**：对偶四元数；超对偶四元数；三叉对偶四元数；截断多项式代数；几何一致误差；H∞ 控制；输入-状态稳定

---

## 1. 引言

### 1.1 背景与动机

刚体位姿（姿态 + 位置）的参数化是机器人控制的起点。单位对偶四元数将两者装入一个 8 维代数对象，运算全局无奇异，且乘法直接实现位姿复合；[P2] 在此参数化上给出了带 L₂ 扰动衰减保证的运动学跟踪控制器，是 DQ 控制的代表性结果。Cohen & Shoham [P1] 进一步引入超对偶四元数（HDQ）：给 DQ 增加一个幂零单位 $\varepsilon^*$（$\varepsilon^{*2}=0$），使一个 HDQ 元素同时携带位姿与其一阶时间导数，链式乘法自动执行微分（Leibniz 法则内化于乘法），从而正运动学一次传播同时输出位姿与 twist。

但 HDQ 到动力学层面仍差一阶：计算力矩控制需要任务空间加速度 $\dot{\boldsymbol\xi}$ 与雅可比导数项 $\dot J\dot{\boldsymbol q}$，而 [P1] 的 HDQ 结构中两个幂零单位（$\varepsilon$ 承载平移、$\varepsilon^*$ 承载一阶导）均已占用，$\varepsilon\varepsilon^*$ 通道携带的是"平移分量的一阶导数"，不含二阶时间导数。更重要的是**误差体系**：[P2] 的误差对象只有位姿一层，其扰动模型假设速度级加性扰动且属于 L₂——动力学环境下这三点都不再成立（§4.1 详述）。

**与相邻文献的分工**。用幂零元承载导数的思想源自 Fike & Alonso 的超对偶数 [3]（面向精确二阶导数计算），[P1] 将其四元数化并用于刚体运动学；本文的 $\mathcal A_2$ 是同一思想在**二阶截断多项式环**上的最小实现，因此 [3] 的"二阶导数无截断误差"性质在此以商环同态的形式重述（命题 1/2）。DQ 动力学控制方面，Adorno 学派的对偶四元数动力学与任务空间控制、以及 [Ch20] 的 resolved-acceleration DQ 律，都已在加速度层工作，且 [Ch20] 的 twist 误差已经伴随映射搬运（其式 (32)：$\boldsymbol\omega_e=\mathrm{Ad}\,\boldsymbol\xi_d-\boldsymbol\xi$，几何一致）；与它们相比，本文的差别不在"能否算加速度"或"是否搬运 twist"，而在**误差对象的代数层级与反馈整形**：现有文献的位姿误差与 twist 误差分别定义、互不生成（[P2] 甚至只有位姿一层，速度误差仅隐式出现在前馈中），[Ch20] 的位姿反馈取螺旋对数 $2\ln x_e$（导数映射在 $\phi\to\pi$ 奇异）；本文把误差提升到 HDQ 使位姿误差与其导数由同一次乘法生成，从而 twist 误差自动几何一致（定理 1），并以 $A^\top$ 整形的位姿反馈获得对任意对称正定增益成立的精确耗散等式（定理 3）。操作空间方法 [5] 的加权度量（惯性矩阵 $\Lambda$ 加权）是本文存储函数的自然推广方向，因需处理 $\dot\Lambda$ 项，本文未展开（§7 局限 ii）。

### 1.2 本文贡献

1. **TNDQ 运动学重构**（§3）：定义三通道代数 $\mathcal A_2=\widehat{\mathbb H}[\sigma]/(\sigma^3)$（TNDQ），证明位姿曲线的 TNDQ 表示 $\bar x$ 满足连乘法则 $\overline{xy}=\bar x\,\bar y$（命题 1），据此正运动学链一次连乘给出 $(\hat{\underline x},\dot{\hat{\underline x}},\ddot{\hat{\underline x}})$ 及导出量 $\boldsymbol\xi,\dot{\boldsymbol\xi},\dot J\dot{\boldsymbol q}$，代价 $O(n)$。
2. **HDQ 截断与误差体系**（§4）：证明取前两通道的 HDQ 截断 $\bar a\mapsto\breve a$ 满足"先乘后截 = 先截后乘"（命题 2）；据此把误差对象定义为实测/期望链 HDQ 表示的一次乘积 $\breve{\tilde x}=\breve x\,(\breve x_d)^*$（定理 1），同时生成右不变位姿误差与几何一致 twist 误差，并证明输出误差满足闭式级联运动学 $\dot e_z=A(\tilde x)e_\xi$（定理 2）。加速度误差在反馈中并非必要，从误差状态中删除（§4.2 的结构性论证）。
3. **几何一致控制律与混合性能保证**（§5）：给出几何一致计算力矩律（前馈经伴随搬运并补输运项、位姿反馈经 $A^\top$ 整形），证明闭环误差动态为级联标准形且存储函数满足精确耗散等式（Lyapunov 交叉项因 $A^\top$ 整形精确相消，对任意对称正定位姿增益矩阵 $K_p$ 成立）；由此对 $L_2$ 扰动给出 H∞ 增益的 Schur 补充要判据（含旋转/平移逐通道拆分）、对 $L_\infty$ 偏差给出 twist 误差的均方极限界，并给出近恒等线性化通道模型与稳态刚度标度律（定理 3、式 (5.8)–(5.9)）。扰动通道 $d(t)$ 中与控制量成正比的乘性分量（$\propto\Delta M$）被显式分离，其效应由小增益参数 $\alpha$ 刻画（§5.1），不再默认 $d$ 为外生信号。
4. **仿真验证**（§6）：在 CoppeliaSim/KUKA LBR4+ 力矩模式下以 S3 抓取-搬运-圆周任务、严格公平协议对比两类文献 DQ 基线（忠实 [Ch20] resolved-acceleration 律 C2、[P2] H∞ 律 + 加速度桥接 C3；朴素 twist 差消融律 C2-abl 仅作为实现存档、不报告数值），定量核验理论结论——无扰收敛、静态刚度标度律 (5.9) 以 2% 精度吻合、ISS 均方界 (5.7) 成立且保守约 1.5 个数量级、H∞ 证书 (5.6a) 的可行性判定与增益回写规则自洽。

**与 [P2] 的关系**：本文不替代 [P2] 的运动学外环——0 阶通道的误差与控制在低速接口下退化回 [P2] 原样（向下兼容，§4.5），本文解决的是其向动力学接口延伸时的结构缺口。

**与 [P1] 的关系**：TNDQ 是 [P1] HDQ 思想（幂零单位承载导数）向二阶的最小扩展；HDQ 恰是 TNDQ 的两通道截断（§3.1 表 1），[P1] 的全部乘法机器在截断下原样保留。

### 1.3 论文结构

§2 预备知识（记号约定、四元数、DQ、twist、HDQ）；§3 TNDQ 代数与运动学重构；§4 误差体系；§5 控制律与主定理（§5.1 不确定性模型、§5.2 控制律、§5.3 定理 3、§5.4 近恒等线性化通道、§5.5 与运动学外环的级联）；§6 仿真验证；§7 结论。关键定理的证明放正文，较长的验证性推导放附录 A（DQ/HDQ 层）、B（TNDQ 链构件）、C（控制层：C.1 扰动通道、C.2 收敛细节与 $A$ 的可逆性、C.3 二次型判据族、C.4 强化存储函数路线、C.5 定理 3(d) 的界形态）。

---

## 2. 预备知识

**记号约定**（全文统一）：同一条位姿曲线用同一核心字母（如 $x$），字母上方的装饰指明它所处的代数层：

| 记号 | 对象 | 说明 |
|---|---|---|
| $\hat a$ | 四元数 | $\hat a\in\mathrm{Spin}(3)$，§2.1 |
| $\hat{\underline a}$ | 对偶四元数 | $\hat{\underline a}\hat{\underline a}^*=1$，式 (2.1) |
| $\breve a$ | HDQ（超对偶四元数） | 两 DQ 通道；曲线的 HDQ 表示 $\breve x=\hat{\underline x}+\varepsilon^*\dot{\hat{\underline x}}$，§2.3 |
| $\bar a$ | TNDQ（三叉对偶四元数） | 三 DQ 通道；曲线的 TNDQ 表示 $\bar x=\hat{\underline x}+\sigma\dot{\hat{\underline x}}+\tfrac12\sigma^2\ddot{\hat{\underline x}}$，§3.2 |
| $\tilde{(\cdot)}$ | 误差量 | 只佩戴波浪号，不再叠加类型装饰；其类型由定义式指明（如 $\tilde x=\hat{\underline x}\hat{\underline x}_d^{\,*}$ 是单位 DQ，$\tilde r$ 是单位四元数） |
| 无装饰斜体 | 一般（未必单位）四元数 / DQ / 标量 | 所属代数在上下文声明 |
| 粗体 | 纯 DQ（twist 等）、向量与矩阵 | $\boldsymbol\xi,\boldsymbol q,J,K_d$ 等 |

（表 0：记号约定。标称模型矩阵 $\hat M,\hat C,\hat g$ 上的 hat 沿用控制文献"标称估计"的习惯用法，与四元数装饰无关；四元数虚单位 $\hat\imath,\hat\jmath,\hat k$ 为固定符号；对偶四元数代数记号 $\widehat{\mathbb H}$（§2.1）的宽帽是代数名称的固定部分，不表示单位化——$\widehat{\mathbb H}$ 的元素是一般（未必单位）对偶四元数；带下标的通道记号 $\hat{\underline a}_k,\hat{\underline b}_k$（§2.3、定义 1）表示一般对偶四元数通道，未必单位——如曲线表示的导数通道 $\dot{\hat{\underline x}}$。）

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

HDQ 代数 $\widehat{\mathbb H}[\varepsilon^*]/(\varepsilon^{*2})$：元素 $\breve a=\hat{\underline a}_0+\varepsilon^*\hat{\underline a}_1$，两个通道 $\hat{\underline a}_0,\hat{\underline a}_1\in\widehat{\mathbb H}$ 均为**对偶四元数**（§2.1），乘法

$$
(\hat{\underline a}_0+\varepsilon^*\hat{\underline a}_1)(\hat{\underline b}_0+\varepsilon^*\hat{\underline b}_1)=\hat{\underline a}_0\hat{\underline b}_0+\varepsilon^*(\hat{\underline a}_0\hat{\underline b}_1+\hat{\underline a}_1\hat{\underline b}_0).
\tag{2.3}
$$

(2.3) 的 $\varepsilon^*$ 通道正是 Leibniz 法则——这是 HDQ 自动微分能力的代数根源（[P1] 式(14)(25)）。**曲线的 HDQ 表示** $\breve x\triangleq\hat{\underline x}+\varepsilon^*\dot{\hat{\underline x}}$ 满足连乘法则 $\breve{xy}=\breve x\,\breve y$：对每个关节因子写出其 HDQ 表示后连乘，一次传播同时得到 $\hat{\underline x}$ 与 $\dot{\hat{\underline x}}$（进而 $\boldsymbol\xi$）。这里"$\breve{xy}$"指乘积曲线 $\hat{\underline x}(t)\hat{\underline y}(t)$ 的 HDQ 表示，是命题 1（§3.2）在两通道截断下的特例。

**HDQ 共轭**（定理 1 中 $(\breve x_d)^*$ 的含义）定义为**逐通道的 DQ 共轭**：

$$
\breve a^{\,*}\triangleq\hat{\underline a}_0^{\,*}+\varepsilon^*\hat{\underline a}_1^{\,*}.
\tag{2.4}
$$

定义 (2.4) 使共轭与求导可交换：对曲线的 HDQ 表示有 $(\breve x)^*=\hat{\underline x}^*+\varepsilon^*\dot{\hat{\underline x}}^*=\breve{(x^*)}$，即"曲线 $\hat{\underline x}^*(t)$ 的 HDQ 表示"。又因 DQ 共轭是 $\widehat{\mathbb H}$ 上的反自同构且 $\varepsilon^*$ 与一切交换，(2.4) 仍是反自同构：逐通道展开得 $(\breve a\breve b)^*=\breve b^{\,*}\breve a^{\,*}$（定理 1 与命题 2 均用到这一点）。

### 2.4 机械臂动力学（标准形）

关节空间动力学 $M(\boldsymbol q)\ddot{\boldsymbol q}+C(\boldsymbol q,\dot{\boldsymbol q})\dot{\boldsymbol q}+\boldsymbol g(\boldsymbol q)=\boldsymbol\tau$，$M$ 对称正定。计算力矩方案用标称模型 $\hat M,\hat C,\hat g$（此处 hat 表"标称估计"，见表 0）生成 $\boldsymbol\tau=\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+\hat C\dot{\boldsymbol q}+\hat g$，其中 $\ddot{\boldsymbol q}_{\mathrm{ref}}$ 是待设计的参考加速度——本文 §5 的控制量。

---

## 3. TNDQ：三叉对偶四元数与运动学重构

### 3.1 代数定义与截断塔

> **定义 1（TNDQ）**：三叉对偶四元数代数为截断多项式环
>
> $$
> \mathcal A_2\triangleq\widehat{\mathbb H}[\sigma]/(\sigma^3)
> =\Bigl\{\bar a=\hat{\underline a}_0+\sigma\hat{\underline a}_1+\tfrac12\sigma^2\hat{\underline a}_2:\ \hat{\underline a}_k\in\widehat{\mathbb H}\Bigr\},
> \tag{3.1}
> $$
>
> $\sigma$ 与全部四元数单位交换，$\sigma^3=0$。乘法由分配律与 $\sigma^3=0$ 唯一确定：
>
> $$
> \bar a\,\bar b=\hat{\underline a}_0\hat{\underline b}_0+\sigma(\hat{\underline a}_0\hat{\underline b}_1+\hat{\underline a}_1\hat{\underline b}_0)+\tfrac12\sigma^2\bigl(\hat{\underline a}_0\hat{\underline b}_2+2\hat{\underline a}_1\hat{\underline b}_1+\hat{\underline a}_2\hat{\underline b}_0\bigr).
> \tag{3.2}
> $$
>
> **TNDQ 共轭**定义为逐通道 DQ 共轭（与 (2.4) 同一约定）：$\bar a^{\,*}\triangleq\hat{\underline a}_0^{\,*}+\sigma\hat{\underline a}_1^{\,*}+\tfrac12\sigma^2\hat{\underline a}_2^{\,*}$。它是 $\mathcal A_2$ 上的反自同构（$(\bar a\bar b)^*=\bar b^{\,*}\bar a^{\,*}$：在单项式基 $\{\sigma^m\}$ 下乘积系数为 $\sum_{k+l=m}\hat{\underline a}_k\hat{\underline b}_l$，逐项取 DQ 共轭即得 $\sum_{k+l=m}\hat{\underline b}_l^{\,*}\hat{\underline a}_k^{\,*}$），且与求导交换：曲线表示满足 $(\bar x)^*=\overline{(x^*)}$。

"三叉"（trident）指其三个 DQ 通道：位姿 / 一阶导 / 二阶导；程序中一个 TNDQ 即三个并列的 8 维数组。与既有结构的截断关系：

| 结构 | 通道 | 元素 | 关系 |
|---|---|---|---|
| DQ | 1 | $\hat{\underline a}_0$ | TNDQ 的 $\sigma^0$ 通道 |
| HDQ（[P1]） | 2 | $\hat{\underline a}_0+\varepsilon^*\hat{\underline a}_1$ | TNDQ 的 $\sigma^0,\sigma^1$ 通道（$\sigma\leftrightarrow\varepsilon^*$） |
| **TNDQ** | 3 | $\hat{\underline a}_0+\sigma\hat{\underline a}_1+\tfrac12\sigma^2\hat{\underline a}_2$ | 全结构 |

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

把 TNDQ 元素 $\bar a=\hat{\underline a}_0+\sigma\hat{\underline a}_1+\tfrac12\sigma^2\hat{\underline a}_2$（$\hat{\underline a}_k\in\widehat{\mathbb H}$，定义 1）**只保留前两个通道**（丢弃 $\sigma^2$ 通道，程序上即只取前两个数组），并把 $\sigma$ 记作 $\varepsilon^*$，得到它的 **HDQ 截断**，记作 $\bar a\big|_{\mathrm{HDQ}}$：

$$
\bar a\big|_{\mathrm{HDQ}}\triangleq \hat{\underline a}_0+\varepsilon^*\hat{\underline a}_1=\breve a .
\tag{3.6}
$$

式 (3.6) 中 $\hat{\underline a}_0,\hat{\underline a}_1\in\widehat{\mathbb H}$ 仍为对偶四元数通道，与 §2.3 HDQ 元素的通道类型一致（若通道取为四元数，右端便不是 §2.3 意义下的 HDQ）。对曲线的 TNDQ 表示 $\bar x$（式 (3.3a)）施行截断，恰得 §2.3 的 HDQ 表示：$\bar x\big|_{\mathrm{HDQ}}=\hat{\underline x}+\varepsilon^*\dot{\hat{\underline x}}=\breve x$。

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

三式并非三个独立条件，而是同一个代数等式的三个通道：由命题 1（取 $\hat{\underline y}=\hat{\underline x}^*$）与 TNDQ 共轭的逐通道定义（定义 1 后）有 $\bar x\,\bar x^{\,*}=\bar x\,\overline{(x^*)}=\overline{x x^*}$，其三通道依次为 (3.8) 的三个左端。故

$$
\text{(3.8) 三式同时成立}\iff \bar x\,\bar x^{\,*}=1\ \text{在}\ \mathcal A_2\ \text{中},
\tag{3.9}
$$

即位姿曲线的 TNDQ 表示落在单位群

$$
\mathcal U_2\triangleq\{\bar a\in\mathcal A_2:\bar a\bar a^{\,*}=1\}
$$

上。$\mathcal U_2$ 确实是群：由共轭的反自同构性（定义 1 后），$\bar a,\bar b\in\mathcal U_2\Rightarrow(\bar a\bar b)(\bar a\bar b)^*=\bar a\bar b\bar b^{\,*}\bar a^{\,*}=\bar a\bar a^{\,*}=1$。这与命题 1 互相印证：各关节因子的 TNDQ 表示逐个属于 $\mathcal U_2$，按 (3.4) 连乘得到的链 TNDQ 自动属于 $\mathcal U_2$，即 (3.8) 三式沿链自动保持。同理，$\mathcal U_2$ 在 HDQ 截断下的像就是单位 HDQ 群（命题 2）。

(3.9) 解析上恒成立；数值积分会使其漂移。定义残差 $c_0=\|\hat{\underline x}\hat{\underline x}^*-1\|$，$c_1=\|\mathrm{Sc}(2\dot{\hat{\underline x}}\hat{\underline x}^*)\|$（$\mathrm{Sc}$ 取标量与对偶标量部）作为逐周期 $O(1)$ 监测量；超阈值触发重投影（0 阶归一化、1 阶按 $\dot{\hat{\underline x}}\mapsto\tfrac12\boldsymbol\xi_{\mathrm{proj}}\hat{\underline x}$ 重构）。前馈侧若使用 $\sigma^2$ 通道可另监测

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

1. **位姿与速度误差相互分离，位姿反馈无证书结构**：须先澄清一个常见误读——[Ch20] 的 twist 误差**已经伴随映射搬运**（其式 (32) $\boldsymbol\omega_e=\mathrm{Ad}\,\boldsymbol\xi_d-\boldsymbol\xi$，与本文 (4.4) 的 $-e_\xi$ 同一），故"速度误差无几何一致定义"不是对 [Ch20] 的指控；真正的缺口有二：(a) 其位姿反馈取螺旋对数 $2\ln x_e$，该映射的导数在旋转角 $\phi\to\pi$ 处奇异（大误差工况失效），且此整形方式不能给出定理 3 所依赖的、对**任意**对称正定 $K_p$ 成立的精确耗散等式；(b) [P2] 只有位姿一层，速度误差仅隐式存在于前馈中，无独立误差通道。两者共同的结构性事实是：位姿误差与 twist 误差**分别定义、互不生成**——而它们由同一次误差乘法生成，恰是定理 2 把输出误差闭合为与原系统同型的级联运动学 $\dot e_z=A(\tilde x)e_\xi$ 的前提。另注：朴素坐标差 $\boldsymbol\xi-\boldsymbol\xi_d$（两 twist 分属不同位姿的切空间，直接相减混入伪项 $(\mathrm{Ad}_{\tilde x}-\mathrm{id})\boldsymbol\xi_d$，其范数 $\lesssim2\|\mathcal O\|\,\|\boldsymbol\xi_d\|+\|\mathcal T\|\,\|\omega_d\|$）在 [P2]、[Ch20] 中均**未被采用**，本文 §6.4 将其实现为消融档（C2-abl，仅供代码层面的结构消融，测量数据存档不报告）以量化该伪项的代价。
2. **加速度层扰动无入口**：动力学的主要不确定性（惯性参数偏差、力矩误差）作用在加速度层，[P2] 的速度级扰动通道无法表达。
3. **偏差型不确定性不满足 L₂**：$\Delta M,\Delta C,\Delta g$、摩擦残差是持续偏差，无限时域能量无穷，L₂ 增益指标对其空洞成立。

### 4.2 误差对象的正确阶数：为什么是 HDQ 而非 TNDQ

反馈需要几阶误差？计算力矩律的反馈项（§5）为 $-K_de_\xi-A^\top K_pe_z$——只消费位姿与速度两阶。若引入加速度误差 $e_a$ 及增益 $K_a$，闭环从二阶级联升为三阶系统：既无必要（定理 3 将证明两阶已足以给出工作域内的渐近收敛与原点邻域的指数收敛），又把高噪声的加速度估计（差分方差 $\propto\Delta t^{-4}$）直接引入反馈。加速度层信息的正确去向是两路：**期望加速度 $\dot{\boldsymbol\xi}_d$ 走前馈**（来自期望轨迹，确定量），**加速度层不确定性走扰动**（进入 $\dot e_\xi$ 方程的 $d(t)$，§5.2）。

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
> **证明**：(4.2)：将 (2.3) 用于 $\hat{\underline a}_0=\hat{\underline x},\hat{\underline a}_1=\dot{\hat{\underline x}},\hat{\underline b}_0=\hat{\underline x}_d^{\,*},\hat{\underline b}_1=\dot{\hat{\underline x}}_d^{\,*}$（共轭与求导交换保证 $(\breve x_d)^*$ 的 $\varepsilon^*$ 通道为 $\dot{\hat{\underline x}}_d^{\,*}$），$\varepsilon^*$ 通道即 Leibniz 展开的 $\frac{d}{dt}(\hat{\underline x}\hat{\underline x}_d^{\,*})$。(i)(ii)(iii) 的推导见附录 A.2——(iii) 的关键一步：$\dot{\tilde x}=\dot{\hat{\underline x}}\hat{\underline x}_d^{\,*}+\hat{\underline x}\dot{\hat{\underline x}}_d^{\,*}=\tfrac12\boldsymbol\xi\tilde x-\tfrac12\tilde x\boldsymbol\xi_d$（用 $\dot{\hat{\underline x}}_d^{\,*}=-\tfrac12\hat{\underline x}_d^{\,*}\boldsymbol\xi_d$），右乘 $2\tilde x^*$ 得 (4.4)。∎
>
> **注记（截断一致性）**：由命题 2，$\breve x\,(\breve x_d)^*$ 与"先在 TNDQ 上作误差乘法 $\bar x\,(\bar x_d)^*$ 再截断"给出同一 HDQ 对象——误差体系落在 HDQ 上不产生任何截断误差。
>
> **注记（与既有文献的一致性）**：本文的几何一致 twist 误差并非新创，而是把两条既有文献路线中**隐式存在**的结构显式化为体系的一阶误差通道：[P2] 控制律 (P2-12) 的前馈项 $\mathrm{vec}_6(\tilde x\boldsymbol\xi_d\tilde x^*)$ 括号内正是 $\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d$，其闭环化简之所以成立，正因为控制律隐式地把 (4.4) 而非朴素差驱为反馈量；[Ch20] 更进一步，其式 (32) 直接以搬运后的差 $\boldsymbol\omega_e=\mathrm{Ad}\,\boldsymbol\xi_d-\boldsymbol\xi=-e_\xi$ 作为反馈量。定理 1 与这两者的差异不在 twist 误差的定义（已同一），而在生成方式：$\breve{\tilde x}$ 的一次乘法使位姿误差与 twist 误差成为同一代数对象的两个通道，从而 (4.5) 的级联闭合与 §5 的 $A^\top$ 整形证书成为可能。

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

### 5.1 扰动通道：显式解算、乘法项分离与假设集

> **本节定位**：本节的不确定性模型本身是标准的——(5.1a)(5.1b) 为经典刚体动力学与计算力矩／resolved-acceleration 律 [4, LWP80, Spo92]，(5.1d) 的"乘法 + 加性"分解是鲁棒控制的常规范式 [ZDG96, Abd91, Sag99]。本节不主张新的扰动模型，只主张**在 TNDQ 误差坐标下对它的精确记账**：$u_{\mathrm{fb}}$ 的 DQ 结构使乘法分量分裂为关于误差次数不同的两项，在定理 3 中分别进入有效阻尼与等效扰动幅值——该分裂是误差坐标特有的，一阶 DQ 律中不出现。逐项代数与适定性讨论见附录 C.1。

**真实对象与标称模型**。真实关节空间动力学（含摩擦与外力）

$$
M(\boldsymbol q)\ddot{\boldsymbol q}+C(\boldsymbol q,\dot{\boldsymbol q})\dot{\boldsymbol q}+\boldsymbol g(\boldsymbol q)+\boldsymbol f(\boldsymbol q,\dot{\boldsymbol q})=\boldsymbol\tau+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}},
\tag{5.1a}
$$

内环以标称模型执行计算力矩

$$
\boldsymbol\tau=\hat M(\boldsymbol q)\ddot{\boldsymbol q}_{\mathrm{ref}}+\hat C(\boldsymbol q,\dot{\boldsymbol q})\dot{\boldsymbol q}+\hat{\boldsymbol g}(\boldsymbol q)+\hat{\boldsymbol f}(\boldsymbol q,\dot{\boldsymbol q}).
\tag{5.1b}
$$

其中 $\delta\boldsymbol\tau$ 为**执行器力矩实现误差**（量化、传输延迟、饱和与死区残差），$\boldsymbol\tau_{\mathrm{ext}}$ 为环境接触投影到关节的广义力，$\boldsymbol f$ 为摩擦；失配量统一取"标称减真实"：$\Delta M\triangleq\hat M-M$，$\Delta C,\Delta\boldsymbol g,\Delta\boldsymbol f$ 同理。将 (5.1b) 代入 (5.1a) 并左乘 $M^{-1}$，得 $\ddot{\boldsymbol q}=\ddot{\boldsymbol q}_{\mathrm{ref}}+\boldsymbol w_{\mathrm{dyn}}$，其中

$$
\boldsymbol w_{\mathrm{dyn}}=M^{-1}\bigl(\Delta M\,\ddot{\boldsymbol q}_{\mathrm{ref}}+\Delta C\dot{\boldsymbol q}+\Delta\boldsymbol g+\Delta\boldsymbol f+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}}\bigr).
\tag{5.1}
$$

由于 $\ddot{\boldsymbol q}_{\mathrm{ref}}$ 由 (5.2) 只依赖 $(\boldsymbol q,\dot{\boldsymbol q},t)$ 而**不依赖 $\ddot{\boldsymbol q}$**，(5.1) 是对 $\ddot{\boldsymbol q}$ 的**显式赋值**而非隐式方程，仅需 $M(\boldsymbol q)\succ0$；(A3) 的 $\alpha$ 条件服务于闭合乘法回路的 Lyapunov 证书，与方程可解性无关。

**乘法分量与外生分量的分离**。记 (5.2) 的任务空间指令为 $\ddot{\boldsymbol q}_{\mathrm{ref}}=J^{+}\bigl(u_{\mathrm{ff}}+u_{\mathrm{fb}}-\dot J\dot{\boldsymbol q}\bigr)$，其中

$$
u_{\mathrm{ff}}=\mathrm{vec}_6\bigl(\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d\bigr),
\qquad
u_{\mathrm{fb}}=-K_de_\xi-A^{\top}(\tilde x)K_pe_z .
$$

令 $d\triangleq J\boldsymbol w_{\mathrm{dyn}}+\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$（$\boldsymbol v_w,\boldsymbol v_c$ 是 [P2] 的速度级测量/通信扰动，其时间导数进入加速度层），则把 $\ddot{\boldsymbol q}_{\mathrm{ref}}$ 代入 (5.1) 后可**精确**拆为“与反馈成正比”与“不含反馈”两部分：

$$
\boxed{\;d=\Theta(\boldsymbol q)\,u_{\mathrm{fb}}+d_{\mathrm{ex}},\qquad
\Theta\triangleq J M^{-1}\Delta M\,J^{+}\in\mathbb R^{6\times6},\;}
\tag{5.1d}
$$

其中 $d_{\mathrm{ex}}\triangleq\Theta(u_{\mathrm{ff}}-\dot J\dot{\boldsymbol q})+JM^{-1}(\Delta C\dot{\boldsymbol q}+\Delta\boldsymbol g+\Delta\boldsymbol f+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}})+\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$ 只依赖 $(\boldsymbol q,\dot{\boldsymbol q},t)$ 与外生信号、**不含**反馈量 $(e_\xi,e_z)$（逐项核对见附录 C.1）。$\Theta u_{\mathrm{fb}}$ 是唯一的回路内乘法分量——这是与 [P2] 仅含外生速度级扰动的关键区别；$\Theta$ 把关节空间失配映到任务空间，与 operational-space 惯量对模型误差的敏感性同源 [5, Nak08]。

全文“扰动”指 (5.1d) 的 $d$、“外生扰动”指 $d_{\mathrm{ex}}$，两者在定理 3 中角色不同，不得混用。$d_{\mathrm{ex}}$ 再按时间特性分解为 $d_{\mathrm{ex}}=d_{L_2}+d_b$：$d_{L_2}\in L_2$（噪声型：测量噪声导数、接触冲击）由定理 3(c) 处理，$d_b\in L_\infty$（偏差型：参数误差、未建模负载、摩擦残差、重力偏差）由定理 3(d) 处理。

**假设集**（定理 3 全文沉默使用，在此一次性列出）：

- **(A1) 雅可比正则**：存在工作集 $\mathcal Q$ 使 $J(\boldsymbol q)$ 在 $\mathcal Q$ 上行满秩且 $\sigma_{\min}(J)\ge\underline\sigma>0$，从而 $JJ^{+}=I_6$（奇异邻域取阻尼伪逆 [Nak86] 时的残差处理见定理 3 诚实边界 (i)）。
- **(A2) 惯量与失配有界**：$M(\boldsymbol q)\succeq \underline m I_n$（$\underline m>0$），且 $\Delta M,\Delta C,\Delta\boldsymbol g,\Delta\boldsymbol f$ 在 $\mathcal Q\times\{\|\dot{\boldsymbol q}\|\le\bar v\}$ 上有界。
- **(A3) 乘法小增益条件**：
  $$
  \alpha\triangleq\sup_{\boldsymbol q\in\mathcal Q}\bigl\|\Theta(\boldsymbol q)\bigr\|_2
  \;<\;\frac{\lambda_{\min}(K_d)}{\lambda_{\max}(K_d)}=\frac1{\mathrm{cond}_2(K_d)}\ \le 1 .
  \tag{5.1f}
  $$
  各向同性阻尼 $K_d=k_dI_6$ 时 (5.1f) 退化为经典计算力矩鲁棒性条件 $\alpha<1$ [Spo92]；一般情形由 $\|\Theta\|_2\le\|J\|\,\|J^+\|\,\|M^{-1}\Delta M\|_2$ 可见它是该条件的任务空间（雅可比条件数修正）版本。
- **(A4) 期望轨迹与外生信号**：$\hat{\underline x}_d(t)$ 为 $C^2$ 且 $\|\boldsymbol\xi_d\|,\|\dot{\boldsymbol\xi}_d\|$ 有界；$d_{L_2}\in L_2$、$d_b\in L_\infty$，且 $D_{\mathrm{ex}}\triangleq\|d_{\mathrm{ex}}\|_{L_\infty}<\infty$。其中对 $\dot{\boldsymbol v}_w$ 的有界性要求偏强（噪声型信号的直接微分会放大高频），实现上取滤波导数或观测器估计 [Ber93]，滤波带宽的影响并入 $D_{\mathrm{ex}}$——此为本节假设集的主要保守之处（§7）。

### 5.2 控制律

取 $K_p$ 对称正定（典型取块对角 $K_p=\mathrm{diag}(p_O I_3,p_T I_3)$；取 $K_p=k_pI_6$ 即恢复单标量增益情形）、$K_d$ 对称正定（若需定理 3(c-2) 的旋转/平移逐通道 H∞ 界，则取块对角 $K_d=\mathrm{diag}(K_\omega,K_v)$，$K_\omega,K_v\in\mathbb R^{3\times3}$ 对称正定），定义参考加速度（几何一致计算力矩律）：

$$
\ddot{\boldsymbol q}_{\mathrm{ref}}
=J^{+}\Bigl(\underbrace{\mathrm{vec}_6\bigl(\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d\bigr)}_{\text{前馈：搬运的期望加速度 + 输运修正}}
-K_d\,e_\xi-A^{\top}(\tilde x)\,K_p\,e_z-\dot J\dot{\boldsymbol q}\Bigr).
\tag{5.2}
$$

四点说明：(a) 前馈中 $\dot{\boldsymbol\xi}_d$ 来自期望轨迹（解析或期望链 TNDQ 表示 $\bar x_d$ 的 $\sigma^2$ 通道），$\mathrm{ad}$ 输运项由引理 1（下）决定——两者使前馈与误差动态中的非反馈项精确相消；(b) 位姿反馈经 $A^\top$ 整形，其目的将在定理 3 证明中显现（Lyapunov 交叉项精确相消）；(c) $\dot J\dot{\boldsymbol q}$ 由 TNDQ 链按 (3.5) 免构造获得。全部反馈量取自定理 1 的 HDQ 误差元素——不需要任何加速度误差的测量或估计。

符号约定（与代码实现对齐）：$K_d,K_p\succ0$ 为**正**定矩阵，反馈项前的负号已显式写在 (5.2) 中；即任务空间反馈为 $u_{\mathrm{fb}}=-K_de_\xi-A^\top K_pe_z$。

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

本节四个结果（定理 3(a)–3(d)）共用假设 (A1)–(A4)（§5.1）、控制律 (5.2) 与扰动模型 (5.1)/(5.1d)；统一存储函数

$$
V(e_z,e_\xi)=\tfrac12\|e_\xi\|^2+\tfrac12\,e_z^{\top}K_pe_z\;\ge0 ,
\tag{5.4a}
$$

$K_p\succ0$ 对称（取 $K_p=k_pI_6$ 即回到标量形式 $\tfrac{k_p}2\|e_z\|^2$）。两点结构性观察决定了下文结论的强弱：(i) $V$ 关于 $e_\xi$ 径向无界，但 $e_z$ 的取值集本身有界（$\|\mathcal O\|\le1$，因 $\tilde r$ 为单位四元数），且 $\tilde\eta=0$ 处 $A(\tilde x)$ 奇异，故**不可能**有全局结论，全部收敛声明都是工作域（水平集）内的局部结论；(ii) $\dot V$ 只含 $-\|e_\xi\|^2$ 型负项而无 $-\|e_z\|^2$ 项（扰动到 $e_z$ 的相对阶为 2），故 $V$ 单独不能给出指数衰减率也不能给出 $e_\xi$ 的逐点极限球（见定理 3(d) 注记与附录 C.5）。

> **定理 3(a)（闭环误差动态）**：误差坐标 $(e_z,e_\xi)$ 满足级联标准形
>
> $$
> \dot e_z=A(\tilde x)\,e_\xi,\qquad
> \dot e_\xi=-K_d e_\xi-A^{\top}(\tilde x)\,K_p\,e_z+d(t),
> \tag{5.5}
> $$
>
> 其中 $d=J\boldsymbol w_{\mathrm{dyn}}+\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$ 汇集全部加速度层扰动，并按 (5.1d) 精确分解为 $d=\Theta u_{\mathrm{fb}}+d_{\mathrm{ex}}$。**术语约定**：下文“$d\in L_2/L_\infty$”均指**总扰动通道** $d$（定理 3(c) 的认证对象）；涉及**外生**扰动 $d_{\mathrm{ex}}$ 的结论须先用 (A3) 把 $\Theta u_{\mathrm{fb}}$ 回收到阻尼项与交叉项中（定理 3(d)），两者不得混用。

> **证明**：
>
> 1. 由 (3.5) 与 $\ddot{\boldsymbol q}=\ddot{\boldsymbol q}_{\mathrm{ref}}+\boldsymbol w_{\mathrm{dyn}}$：$\mathrm{vec}_6\dot{\boldsymbol\xi}=J\ddot{\boldsymbol q}_{\mathrm{ref}}+\dot J\dot{\boldsymbol q}+J\boldsymbol w_{\mathrm{dyn}}$。
> 2. 代入 (5.2) 并用 $JJ^+=I$（假设 (A1)），$\dot J\dot{\boldsymbol q}$ 消去：
> $$
> \mathrm{vec}_6\dot{\boldsymbol\xi}=\mathrm{vec}_6\bigl(\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d\bigr)-K_de_\xi-A^\top K_pe_z+J\boldsymbol w_{\mathrm{dyn}} .
> $$
> 3. 与引理 1 的 (5.4)（含扰版）相减，前馈项与期望/输运项精确相消：$\dot e_\xi=\mathrm{vec}_6\dot{\tilde{\boldsymbol\xi}}=-K_de_\xi-A^\top K_pe_z+d$。
> 4. 配合定理 2 的 $\dot e_z=Ae_\xi$ 即得 (5.5)。∎

> **定理 3(b)（无扰：水平集不变性、渐近收敛与局部指数稳定）**：设 $d\equiv0$，$K_p=\mathrm{diag}(K_{p,O},K_{p,T})$ 对称正定（旋转/平移块）。取
>
> $$
> 0<c<c^{*}\triangleq\tfrac12\lambda_{\min}(K_{p,O}),
> \qquad
> \Omega_c\triangleq\{(e_z,e_\xi):V\le c\}.
> \tag{5.5b}
> $$
>
> 则：**(i)** $\dot V=-e_\xi^\top K_de_\xi\le0$，故 $\Omega_c$ 紧且正向不变；**(ii)** 若初值取 $\tilde\eta(0)>0$ 分支，则沿全程
> $$
> \tilde\eta(t)\ \ge\ \eta_0\triangleq\sqrt{1-\frac{2c}{\lambda_{\min}(K_{p,O})}}>0 ,
> \tag{5.5c}
> $$
> 从而 $A(\tilde x)$ 在 $\Omega_c$ 上一致可逆（$\det A=-\tfrac18\tilde\eta\le-\tfrac18\eta_0<0$）；**(iii)** $\Omega_c$ 内一切轨迹满足 $(e_z,e_\xi)\to(0,0)$；**(iv)** $(0,0)$ 是**局部指数稳定**的。

> **证明**：
>
> 1. *交叉项精确相消*：沿 (5.5)（$d\equiv0$），
> $$
> \dot V=e_\xi^\top\dot e_\xi+e_z^\top K_p\dot e_z
> =e_\xi^\top\bigl(-K_de_\xi-A^\top K_pe_z\bigr)+e_z^\top K_pAe_\xi
> =-e_\xi^\top K_de_\xi ,
> $$
> 因 $e_z^\top K_pAe_\xi=(A^\top K_p^\top e_z)^\top e_\xi=(A^\top K_pe_z)^\top e_\xi$——这里**只**用到 $K_p$ 对称与 $K_p$ 写在 $A^\top$ 内侧（§5.2 说明 (c)），不需要 $K_p$ 为标量。故 $\dot V\le0$，$\Omega_c$ 正向不变；又 $V\le c$ 给出 $\|e_\xi\|\le\sqrt{2c}$、$\|e_z\|\le\sqrt{2c/\lambda_{\min}(K_p)}$，故 $\Omega_c$ 紧。
> 2. *工作域保号 (5.5c)*：$V\le c$ 蕴含 $\tfrac12\mathcal O^\top K_{p,O}\mathcal O\le c$，即 $\|\mathcal O\|^2\le2c/\lambda_{\min}(K_{p,O})<1$（由 $c<c^*$）。由 $\mathcal O=-\mathrm{Im}\,\tilde r$ 与 $\|\tilde r\|=1$ 得 $\tilde\eta^2+\|\mathcal O\|^2=1$，故 $|\tilde\eta|\ge\eta_0>0$；$\tilde\eta(t)$ 连续且恒不为零，符号不可突变，由 $\tilde\eta(0)>0$ 得 (5.5c)。
> 3. *$A$ 的行列式*：$A$ 块下三角（(4.5)），故
> $$
> \det A=\det\bigl(-\tfrac12(\tilde\eta I_3+[\mathcal O]_\times)\bigr)\cdot\det I_3
> =\bigl(-\tfrac12\bigr)^3\tilde\eta\bigl(\tilde\eta^2+\|\mathcal O\|^2\bigr)=-\tfrac18\tilde\eta ,
> $$
> 用到 $\det(aI_3+[b]_\times)=a(a^2+\|b\|^2)$ 与 $\tilde\eta^2+\|\mathcal O\|^2=1$。定量奇异值下界 $\sigma_{\min}(A)\ge\bigl[2(1+\|\mathcal T\|)/\tilde\eta+1\bigr]^{-1}$ 见附录 C.2。
> 4. *LaSalle*：$\Omega_c$ 紧且不变，$E\triangleq\{\dot V=0\}\cap\Omega_c=\{e_\xi=0\}\cap\Omega_c$。若轨迹全程留在 $E$ 内：$e_\xi\equiv0\Rightarrow\dot e_\xi\equiv0\Rightarrow A^\top K_pe_z\equiv0$；由第 3 步 $A$ 可逆、$K_p\succ0$ 得 $e_z\equiv0$。故 $E$ 内最大不变集为 $\{(0,0)\}$，由 LaSalle 不变集定理（[Kha02] Thm 4.4）得 (iii)。
> 5. *局部指数稳定*：在 $(0,0)$ 处 $\tilde x\to1$，$A\to A_0=\mathrm{diag}(-\tfrac12I_3,I_3)$，(5.5) 的雅可比为
> $$
> F=\begin{bmatrix}0_6 & A_0\\ -A_0^\top K_p & -K_d\end{bmatrix}.
> $$
> 对该 LTI 系统同一个 $V$ 仍给出 $\dot V=-e_\xi^\top K_de_\xi\le0$，且 $A_0$ 可逆，重复第 4 步得 LTI 系统渐近稳定，故 $F$ 为 Hurwitz；再由 Lyapunov 线化定理（[Kha02] Thm 4.7）得非线性系统在原点邻域指数稳定。块对角 $K_d,K_p$ 下 $F$ 逐通道解耦，其两个二阶多项式与极点由 (5.8) 显式给出。█

> **注记（结论为何只能是局部的）**：三条障碍使全局指数稳定不可得：(i) 单位四元数对 $SO(3)$ 的双覆盖使 $\tilde x=-1$（$\tilde\eta=-1,\mathcal O=0$）也是 (5.5) 的平衡点，任何连续反馈都不可能使 $\tilde x=1$ 全局吸引（unwinding，拓扑障碍）；(ii) $\tilde\eta=0$ 处 $A$ 奇异，$e_z$ 对位姿的参数化在此退化；(iii) $\dot V$ 不含 $-\|e_z\|^2$ 项，指数性必须由线化获得，因而只在原点邻域成立。水平集条件 (5.5b) 是可数值核验的：§6 tuned 档取 $p_O=320$，故 $c^{*}=160$，而换算到同一权重口径后实测 $V^{\mathrm{tuned}}_{\mathrm{peak}}\le0.494\ll c^{*}$，余度约 **2.5 个数量级**（§6.5(6)）。工程实现上，当 $\tilde\eta<0$ 时将 HDQ 误差整体翻转符号（$\breve{\tilde x}\to-\breve{\tilde x}$，由定理 1(i) 不改变 $\tilde{\boldsymbol\xi}$）即可强制 $\tilde\eta\ge0$，这正是证明第 2 步“取 $\tilde\eta>0$ 分支”的落实（§6.2）。收敛论证的定量细节（$\sigma_{\min}(A)$ 的一致下界与 LaSalle 步骤的严格收尾）见附录 C.2。∎

> **定理 3(c)（H∞ 通道：二次型/Schur 补判据与旋转/平移通道拆分）**：设 $d=d_{L_2}\in L_2$。
>
> **证书参数**：$\kappa>0$ 是**误差罚权的倒数**（供给率中 $\|e_\xi\|^2$ 项的系数取 $\tfrac1{2\kappa}$，量纲 s），$\gamma_a>0$ 是**待认证的加速度层扰动衰减水平**（量纲 s$^{1/2}$），被认证量是 $d\to e_\xi$ 的 $L_2$ 能量增益 $\gamma_a\sqrt\kappa$（无量纲）。二者均为**分析参数**：不出现在控制律 (5.2) 中，只出现在证书 (5.6a) 中；通道拆分版本中 $(\kappa_\omega,\gamma_\omega)$ 与 $(\kappa_v,\gamma_v)$ 可独立指定。
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
> 即加速度层扰动到 twist 误差能量的 $L_2$ 增益 $\le\gamma_a\sqrt\kappa$（零初值时退化为纯增益界；此界只约束 $e_\xi$，见诚实边界 (iv)）。
>
> **(c-2) 通道拆分判据**（$K_d=\mathrm{diag}(K_\omega,K_v)$ 块对角，且 $K_p=\mathrm{diag}(K_{p,O},\,k_{p,T}I_3)$——平移刚度块须**各向同性**，必要性见附录 C.3）：记 $e_\xi=[\tilde\omega;\tilde v]$、$d=[d_\omega;d_v]$，$V_\omega\triangleq\tfrac12\|\tilde\omega\|^2+\tfrac12\mathcal O^\top K_{p,O}\mathcal O$、$V_v\triangleq\tfrac12\|\tilde v\|^2+\tfrac{k_{p,T}}2\|\mathcal T\|^2$（$V=V_\omega+V_v$）。若
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
> *第二步（(c-1) 判据的等价性）*：性能目标即 $-e_\xi^\top K_de_\xi+e_\xi^\top d+\tfrac1{2\kappa}\|e_\xi\|^2-\tfrac{\gamma_a^2}2\|d\|^2\le0$，等价于二次型不等式
> $$
> \begin{bmatrix}e_\xi\\ d\end{bmatrix}^{\!\top}\!M\!
> \begin{bmatrix}e_\xi\\ d\end{bmatrix}\ge0\quad\forall(e_\xi,d)\in\mathbb R^{12},
> $$
> 即 $M\succeq0$。右下块 $\tfrac{\gamma_a^2}2I\succ0$，取 Schur 补得 $K_d-\tfrac1{2\kappa}I-\tfrac1{2\gamma_a^2}I\succeq0$，即 (5.6a)。关键在于不定号交叉项 $e_\xi^\top d$ 保留在二次型内整体判定，不经任何符号放缩（Schur 补、配方法与 Young 三条路径的等价性见附录 C.3）。
>
> *第三步（全局存在性）*：由 $M\succeq0$ 得 $\dot V\le\tfrac{\gamma_a^2}2\|d\|^2$，故 $V(t)\le V(0)+\tfrac{\gamma_a^2}2\|d_{L_2}\|_{L_2}^2<\infty$，$(e_z,e_\xi)$ 一致有界，解在 $[0,\infty)$ 上存在（无有限时间逃逸）。
>
> *第四步（积分收尾）*：在 $[0,T]$ 上积分 $\dot V\le-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$，弃去 $V(T)\ge0$，令 $T\to\infty$（单调收敛）即得 (5.6)。
>
> *第五步（(c-2) 通道解耦）*：块对角 $K_d$ 与各向同性平移刚度 $K_{p,T}=k_{p,T}I_3$ 下两通道储能精确解耦，关键是两处混合积恒零：由 (4.5)，$(A^\top K_pe_z)_\omega=A_{11}^\top K_{p,O}\mathcal O+k_{p,T}[\mathcal T]_\times\mathcal T=A_{11}^\top K_{p,O}\mathcal O$（$\mathcal T\times\mathcal T=0$，故旋转反馈不含 $\mathcal T$）、$(A^\top K_pe_z)_v=k_{p,T}\mathcal T$；又 $\dot{\mathcal T}=-[\mathcal T]_\times\tilde\omega+\tilde v$ 中的耦合项做功为零（$\mathcal T\cdot(\mathcal T\times\tilde\omega)=0$，与附录 A.3 同机制）。于是位姿交叉项在两通道内分别由 $K_{p,O}$ 对称与 $k_{p,T}$ 为标量而精确相消，
> $$
> \dot V_\omega=-\tilde\omega^\top K_\omega\tilde\omega+\tilde\omega^\top d_\omega,
> \qquad
> \dot V_v=-\tilde v^\top K_v\tilde v+\tilde v^\top d_v ,
> $$
> 对每条通道重复第二至第四步的论证（$I_6\to I_3$）即得 (5.6b)⇒(5.6$'$)。两处恒零为代数恒等式，故 (c-1)/(c-2) 的全部结论均不依赖工作域 $\tilde\eta>0$（见诚实边界 (v)）。逐项代数、$K_{p,T}$ 各向同性的必要性与失效条件见附录 C.3。∎

> **注记（最紧可证增益与 $\gamma_a$ 的角色）**：(i) 各向同性罚权下 (5.6a) 与 Young 路的标量条件重合；矩阵判据的收益不在放宽条件，而在从构造上免除符号放缩、并使罚权分块化后自然产出 (c-2)。(ii) 供给率整体缩放 $\theta>0$ 给出条件族 $K_d\succeq(\theta\kappa^{-1}+\tfrac1{4\theta}\gamma_a^{-2})I$，对 $\theta$ 极小化（$\theta^*=\sqrt\kappa/2\gamma_a$）得**最紧可证条件**
> $$
> \lambda_{\min}(K_d)\ \ge\ \frac{1}{\gamma_a\sqrt\kappa}
> \qquad\Longleftrightarrow\qquad
> \text{认证 }L_2\text{ 增益 }\le\ \frac{1}{\lambda_{\min}(K_d)} ;
> $$
> (5.6a) 是该族在 $\theta=\tfrac12$ 处的成员，仅在 $\kappa=\gamma_a^2$ 时最紧（AM–GM 等号）。该值与定理 3(d) 均方界的 $\alpha\to0$ 极限、以及线性极限 $\|(sI+K_d)^{-1}\|_{H_\infty}=1/\lambda_{\min}(K_d)$ 三者一致——同一个一阶耗散通道 $\dot e_\xi=-K_de_\xi+d$ 在三种度量下的同一增益，也是这条 Lyapunov 路线可证增益的**天花板**（附录 C.3）。(iii) 与 [P2] 中 $\gamma$ 直接决定增益（$k=\sqrt2/\gamma$）的**综合参数**角色不同，$\gamma_a$ 是**分析参数**，由此得两条可实验证伪的推论：固定 $K_d$ 扫 $\gamma_a$ 时闭环轨迹与误差**严格不变**，变的只是证书可行域边界 $\gamma_a\ge[2\lambda_{\min}(K_d)-\kappa^{-1}]^{-1/2}$；要让 $\gamma_a$ 影响误差必须经 $\gamma$-$\kappa$ 规则**回写增益**（$\kappa^*=\gamma_a^2$、$\lambda_{\min}(K_d)=\gamma_a^{-2}$），此时认证增益 $=\gamma_a^2$，而 $\gamma_a$ 的可达下界由指令峰值与离散化余量决定。两条通道的实验化见 §6.7，设计规则见附录 C.3。

> **定理 3(d)（$L_\infty$ 通道：乘法分量分离与 twist 误差的均方极限界）**：设 (A1)–(A4) 成立，扰动通道按 (5.1d) 分解为 $d=\Theta u_{\mathrm{fb}}+d_{\mathrm{ex}}$，$\alpha\triangleq\sup_t\|\Theta\|_2$ 满足小增益条件 (5.1f)，$D_{\mathrm{ex}}\triangleq\|d_{\mathrm{ex}}\|_{L_\infty}<\infty$。记**有效阻尼**
>
> $$
> \lambda_{\mathrm{eff}}\triangleq\lambda_{\min}(K_d)-\alpha\,\lambda_{\max}(K_d)\;>\;0
> \tag{5.7a}
> $$
>
> （正性由 (5.1f) 保证）。设轨迹在所考察时段内留在水平集 $\Omega_c$ 内（(5.5b)；该前提仅用于界定 $\|e_z\|$，需事后数值核验，见下方注记），并记**等效扰动幅值**
>
> $$
> D\triangleq D_{\mathrm{ex}}+\alpha\,\lambda_{\max}(K_p)\Bigl(1+\sqrt{\tfrac{2c}{\lambda_{\min}(K_{p,T})}}\Bigr)\sqrt{\tfrac{2c}{\lambda_{\min}(K_p)}} .
> \tag{5.7b}
> $$
>
> 则 twist 误差满足有限时段均方界
>
> $$
> \frac1T\int_0^T\|e_\xi(t)\|^2\,dt\;\le\;\frac{D^2}{\lambda_{\mathrm{eff}}^2}+\frac{2V(0)}{\lambda_{\mathrm{eff}}\,T},
> \qquad\forall T>0,
> \tag{5.7c}
> $$
>
> 从而
>
> $$
> \limsup_{T\to\infty}\ \mathrm{RMS}_{[0,T]}(e_\xi)
> \;\triangleq\;\limsup_{T\to\infty}\Bigl(\frac1T\int_0^T\|e_\xi\|^2dt\Bigr)^{1/2}
> \;\le\;\frac{D}{\lambda_{\mathrm{eff}}}
> \;\xrightarrow[\ \alpha\to0\ ]{}\;\frac{\|d_{\mathrm{ex}}\|_{L_\infty}}{\lambda_{\min}(K_d)} .
> \tag{5.7}
> $$
>
> 即：偏差型（$L_\infty$）不确定性不破坏有界性，只按 $D/\lambda_{\mathrm{eff}}$ 抬高 twist 误差的均方稳态水平。乘法分量 $\Theta u_{\mathrm{fb}}$ 的作用有二——以 $\alpha\lambda_{\max}(K_d)$ 折减有效阻尼（分母）、以 $\alpha\lambda_{\max}(K_p)\|e_z\|$ 抬高等效扰动幅值（分子）——二者均在 $\alpha\to0$ 时消失。位姿误差 $e_z$ 不在本定理结论之内，其稳态量级由近恒等线化通道 (5.9) 给出准静态估计。

> **证明**：
>
> 1. *精确耗散等式与乘法项展开*：由定理 3(c) 证明第一步，$\dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d$ 精确成立（无任何放缩）。代入 (5.1d) 与 (5.2) 的 $u_{\mathrm{fb}}=-K_de_\xi-A^\top K_pe_z$：
> $$
> \dot V=-e_\xi^\top K_de_\xi\;\underbrace{-\,e_\xi^\top\Theta K_de_\xi}_{\text{与阻尼同类}}\;\underbrace{-\,e_\xi^\top\Theta A^\top K_pe_z}_{\text{与扰动同类}}\;+\;e_\xi^\top d_{\mathrm{ex}} .
> $$
> 2. *两类乘法项的分别回收*：$|e_\xi^\top\Theta K_de_\xi|\le\alpha\lambda_{\max}(K_d)\|e_\xi\|^2$（回收进阻尼，得 (5.7a) 的 $\lambda_{\mathrm{eff}}$；由 (A3)/(5.1f) 即 $\alpha<\lambda_{\min}(K_d)/\lambda_{\max}(K_d)$ 得 $\lambda_{\mathrm{eff}}>0$）；$|e_\xi^\top\Theta A^\top K_pe_z|\le\alpha\|A\|_2\lambda_{\max}(K_p)\|e_z\|\,\|e_\xi\|$（回收进等效扰动幅值 $D$）。
> 3. *$A$ 的谱范数界*：由 $A_{11}^\top A_{11}=\tfrac14(I_3-\mathcal O\mathcal O^\top)$ 得 $\|A_{11}\|_2=\tfrac12$（**精确值**，与 $\tilde x$ 无关；奇异值计算见附录 C.2），再由 $A$ 的块下三角结构得 $\|A\|_2\le\max\{\|A_{11}\|_2,1\}+\|[\mathcal T]_\times\|_2=1+\|\mathcal T\|$。在 $\Omega_c$ 上 $\|\mathcal T\|\le\sqrt{2c/\lambda_{\min}(K_{p,T})}$、$\|e_z\|\le\sqrt{2c/\lambda_{\min}(K_p)}$，代入第 2 步即得 (5.7b) 与
> $$
> \dot V\ \le\ -\lambda_{\mathrm{eff}}\|e_\xi\|^2+D\,\|e_\xi\| .
> \tag{5.7d}
> $$
> 4. *Young 与积分收尾*：$D\|e_\xi\|\le\tfrac{\lambda_{\mathrm{eff}}}2\|e_\xi\|^2+\tfrac{D^2}{2\lambda_{\mathrm{eff}}}$，故 $\dot V\le-\tfrac{\lambda_{\mathrm{eff}}}2\|e_\xi\|^2+\tfrac{D^2}{2\lambda_{\mathrm{eff}}}$。在 $[0,T]$ 上积分并弃去 $V(T)\ge0$：
> $$
> \tfrac{\lambda_{\mathrm{eff}}}2\int_0^T\|e_\xi\|^2dt\ \le\ V(0)+\tfrac{D^2}{2\lambda_{\mathrm{eff}}}\,T ,
> $$
> 两端除以 $\tfrac{\lambda_{\mathrm{eff}}}2T$ 即 (5.7c)；令 $T\to\infty$ 得 (5.7)。$\alpha\to0$ 时 $D\to D_{\mathrm{ex}}$、$\lambda_{\mathrm{eff}}\to\lambda_{\min}(K_d)$，退化为经典形式。█

> **注记（界为何是均方而非逐点）**：(5.7d) 只给出“$\|e_\xi\|>D/\lambda_{\mathrm{eff}}\Rightarrow\dot V<0$”。$\{\|e_\xi\|\le r\}$ 在 $(e_z,e_\xi)$ 空间中沿 $e_z$ 方向无界、并非 $V$ 的水平集，故“$V$ 在其外单调下降”不蕴含轨迹被它捕获；根源是 $\dot V$ 的负项只含 $-\|e_\xi\|^2$（扰动到 $e_z$ 的相对阶为 2），$V$ 只是 $e_\xi$ 方向的耗散证书而非全状态 ISS-Lyapunov 函数。因此本定理不自称 ISS，结论取积分（均方）形式——ISS 极限球的积分类比物，其被界定的量恰为 §6.5 实际统计的稳态窗口 RMS。此外，$\Omega_c$ 前提与 (5.7b) 的 $D$ 相互依赖，构成需事后核验的循环（§6.5(6)：实测余度约 2.5 个数量级）。三点的完整论证见附录 C.5，彻底解除循环的 strictification 路线见附录 C.4 与 §7。∎

> **注记（诚实边界）**：(i) 奇异邻域内取阻尼伪逆时 $JJ^+\ne I$，残差归入 $d_{\mathrm{ex}}$（不正比于 $\Delta M$，故不属 $\Theta u_{\mathrm{fb}}$）；(ii) 本定理是内环单层的严格结果（$L_2$ 通道充要判据 + $L_\infty$ 通道均方界），**不**声称全状态 ISS，也**不**声称与运动学外环级联后的整体 H∞ 界（开放问题，§7）；(iii) Lyapunov 与耗散技术本身是标准的，本文的新内容在误差坐标的选择（HDQ 误差元素 + $A^\top$ 整形使两处相消都精确成立）与两类扰动的通道化归属；(iv) (5.6)/(5.6$'$)/(5.7) 均只约束 $e_\xi$ 而非 $e_z$——$V$ 在 $e_z$ 方向无耗散项；本稿范围内 $e_z$ 的稳态量级**只**由近恒等线化通道 (5.9) 给出准静态估计（$\|\mathcal T\|_{\mathrm{ss}}\approx\|d_v\|/p_T$、$\|\mathcal O\|_{\mathrm{ss}}\approx2\|d_\omega\|/p_O$），属工程指标而非严格上界，严格化需 strictification（附录 C.4）；(v) 含扰时轨迹可能离开 $\tilde\eta>0$（unwinding 域边界），定理 3(c) 的 twist 误差界仍成立（其推导不用到 $A$ 可逆），但域外不附带任何位姿收敛结论；(vi) 通道拆分 (c-2) 依赖 $K_d$ 块对角与 $K_{p,T}$ 各向同性，否则退回合并判据 (c-1)。

### 5.4 近恒等线性化通道与静态刚度标度律

定理 3 给出的是定性与能量层面的结论，不直接给出增益数值。本节在原点邻域把 (5.5) 线性化，得到一个**可直接用于增益整定且可实验证伪**的两通道二阶模型——它同时暴露了 $A_0$ 带来的一个容易被忽略的结构效应：旋转通道的刚度被折减四倍。

取 $K_d=\mathrm{diag}(K_\omega,K_v)$、$K_p=\mathrm{diag}(K_{p,O},k_{p,T}I_3)$，在 $\tilde x\to1$（$\tilde\eta\to1,\mathcal O\to0,\mathcal T\to0$）处 $A\to A_0=\mathrm{diag}(-\tfrac12I_3,I_3)$，故 $\dot{\mathcal O}=-\tfrac12\tilde\omega$、$\dot{\mathcal T}=\tilde v$。将其微分一次并代入 (5.5) 的第二式（注意 $(A_0^\top K_pe_z)_\omega=-\tfrac12K_{p,O}\mathcal O$、$(A_0^\top K_pe_z)_v=k_{p,T}\mathcal T$），消去 $e_\xi$ 得两条解耦的二阶方程：

$$
\boxed{\;
\ddot{\mathcal O}+K_\omega\dot{\mathcal O}+\tfrac14K_{p,O}\,\mathcal O=-\tfrac12\,d_\omega ,
\qquad
\ddot{\mathcal T}+K_v\dot{\mathcal T}+k_{p,T}\,\mathcal T=+\,d_v .\;}
\tag{5.8}
$$

三点读数。**(i) 1/4 旋转刚度折减**：旋转通道的有效刚度是 $\tfrac14K_{p,O}$ 而不是 $K_{p,O}$，根源是 $A_0$ 的旋转块为 $-\tfrac12I_3$（$\mathcal O=-\mathrm{Im}\,\tilde r$ 与半角参数化共同贡献的因子），在位姿反馈与输出映射中各出现一次，故以平方形式 $(\tfrac12)^2$ 进入刚度。**工程含义**：若天真地取 $K_{p,O}=k_{p,T}I_3$（如 §6.4 的 base 档，$K_p=16I_6$），则旋转通道的实际刚度仅为平移通道的 1/4，两通道带宽严重失配；要使二者配平，应取 $K_{p,O}=4k_{p,T}I_3$（§6.4 tuned 档的 $p_O=320=4\times80$ 即此规则）。**(ii) 极点分配规则**：若各通道目标极点为 $\{-a,-b\}$（$a,b>0$），则

$$
K_\omega=K_v=(a+b)I_3,\qquad k_{p,T}=ab,\qquad K_{p,O}=4ab\,I_3 ,
$$

即 §6.4 三档增益的生成式（tuned$=\{-4,-20\}$、fast$=\{-6,-30\}$、base$=\{-4,-4\}$但未作 1/4 补偿）；离散实现另需极点与步长满足 $\max(a,b)\cdot\Delta t\lesssim0.2$。**(iii) 注意号差异**：旋转通道的扰动增益为 $-\tfrac12$、平移为 $+1$，同源于 $A_0$；该系数在下式的反演中必须保留。

令 (5.8) 中 $d_\omega,d_v$ 为准常量（低频成分主导，如未建模负载引起的 $\Delta M,\Delta\boldsymbol g$），取 $\ddot{(\cdot)}=\dot{(\cdot)}=0$ 得**静态刚度标度律**

$$
\boxed{\;
\|\mathcal T\|_{\mathrm{ss}}=\frac{\|d_v\|}{k_{p,T}} ,
\qquad
\|\mathcal O\|_{\mathrm{ss}}=\frac{2\,\|d_\omega\|}{\lambda(K_{p,O})} ,\;}
\tag{5.9}
$$

即稳态残差**只**由静态刚度决定、与阻尼无关（$K_d$ 只改变过渡过程）。(5.9) 给出两个可伪造的预言：**(P1) 反比标度**——刚度提高 $\rho$ 倍，稳态位姿残差降低至 $1/\rho$；**(P2) 等效扰动反演的一致性**——同一物理工况下用**不同增益档**的实测残差反演 $\|d_v\|=k_{p,T}\|\mathcal T\|_{\mathrm{ss}}$、$\|d_\omega\|=\tfrac12\lambda(K_{p,O})\|\mathcal O\|_{\mathrm{ss}}$，应得到**同一个**幅值。(P2) 比 (P1) 严苛得多（它要求两个独立档位的两个独立数字重合），是 §6.5 对本节模型的主检验（实测偏差 $\approx2\%$）。代码侧 `control/gain_design.py` 的 `c1_channels()` 逐字实现 (5.8)（旋转通道传入 `p_O/4` 与扰动增益 $-0.5$，平移通道传入 `p_T` 与 $+1.0$），可作为本节与实现一致性的交叉校验点。

**适用边界**：(5.8)–(5.9) 是 $\tilde x\to1$ 的一阶近似，与定理 3(b)(iv) 的局部指数稳定共用同一个线性化雅可比矩阵 $F$（因而也共用其适用域）；它不是严格上界，大误差区的 $[\mathcal T]_\times$ 耦合与 $\tilde\eta<1$ 导致的刚度变异均未计入。严格结论仍以定理 3 为准；(5.9) 的定位是**增益整定与扰动反演的工程模型**，同时承担定理 3(d) 诚实边界 (iv) 中 $e_z$ 稳态量级估计的职能。

### 5.5 与运动学外环的级联

外环照旧运行 [P2] 控制律（保持其 H∞ 保证），内环 (5.2) 以期望轨迹（或外环整形后的参考）为输入。内环的 $L_2$ 证书 (5.6) 与外环对执行残差的 H∞ 鲁棒性结合，在**小增益条件成立**时可给出级联系统的 $L_2$ 界；内环的效果等价于把外环感受到的速度级扰动变小。**诚实陈述**：本稿早期版本写作“内环 ISS + 外环 H∞ ‹⇒› 级联系统分别保持 $L_2$ 界与极限球界（标准级联 ISS 定理）”，这一断言并未在本稿证明：其一，定理 3(d) 只给出 $e_\xi$ 的均方界而非全状态 ISS（见该定理后的注记与附录 C.5），标准级联 ISS 定理的前提不满足；其二，两环共用同一执行器与同一量测，其互连不是单向级联。因此本节只作**结构性陈述与设计指引**，整体级联的定量保证列为开放问题（§7 局限 i）。

---

## 6. 仿真验证

本章在 CoppeliaSim 物理仿真中检验第 3–5 章的理论主张，章节体例参照 [P2]（§5 Simulation results）与 [Ch20]（§4 Experimental Validation）：先给出平台与模型（§6.1）、信息流水线（§6.2），再描述 S3 抓取-搬运实验设计与公平对比协议（§6.3）、控制器与参数设置（§6.4），随后给出定量结果与分析（§6.5）并小结（§6.6）；针对 §5.3 注记 $\gamma_a$ 双通道推论的 γ 扫描协议作为后续实验设计列于 §6.7。全部数值摘自仿真运行导出的原始数据（`TNDQ_sim/results/grasp_metrics_summary.csv`），未做修饰。

### 6.1 平台与机器人模型

**平台**：CoppeliaSim（原 V-REP [Roh13]）中的 7 自由度 KUKA LBR4+ 轻量臂，末端装 RG2 二指夹爪；控制律以**力矩模式**直接下发关节力矩，控制/物理步长 dt = 5 ms（200 Hz，与 [Ch20] 的 Baxter 实验控制频率一致），单次实验 22.5 s、共 4500 个控制步。

**机器人模型**：LBR4+ 采用修正 DH 参数建模（S-R-S 构型，连杆偏置 $d=[0.251,\,0,\,0.4,\,0,\,0.39,\,0,\,0.078]$ m），动力学参数取自公开辨识结果 [Gaz14]，构成控制器内部的名义模型 $\hat M(\boldsymbol q),\hat C(\boldsymbol q,\dot{\boldsymbol q}),\hat g(\boldsymbol q)$。正运动学按 §3 的 TNDQ 链 (3.4) 实现：一次连乘同时产出位姿 $\hat{\underline x}$、twist $\boldsymbol\xi=\overline{\mathrm{vec}}_6(J\dot{\boldsymbol q})$ 与二阶读出 $\dot J\dot{\boldsymbol q}$（式 (3.5)，免于显式构造 Hessian 或数值差分），后者是控制律 (5.2) 前馈通道的关键输入。

**负载对象**：被抓取对象为圆柱形水杯（质量 m = 0.25 kg），t = 2.5 s 闭爪后刚性附着于末端。关键设定：名义模型**不包含杯的动力学**，因此带载后 $\Delta M,\Delta g$ 构成真实的持续模型失配扰动，用以检验 §5.4 的静态刚度标度律（式 (5.9)）与定理 3(d) 的均方极限界（式 (5.7)）。

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

**敏感条件**。标准工况下各律稳态差异极小（§6.5 的"准静态趋同"现象），为曝光结构性差异追加四个应力条件：highspeed（ω→2.5 rad/s，向心加速度前馈需求放大 6.25 倍，考验 $\dot{\boldsymbol\xi}_d$ 与 $\dot J\dot{\boldsymbol q}$ 通道质量）、fast-transit（搬运四段 lift/retreat/transit/descend2 时长 ×0.5，路标几何不变，考验快相位下的前馈精度）、noise（关节测量高斯噪声 $\sigma_q=5\times10^{-5}$ rad、$\sigma_{\dot q}=10^{-3}$ rad/s，考验差分类前馈（仅 C3 的桥接项）的噪声放大；C1/C2 均为解析前馈）、coarse-dt（控制更新 5→15 ms 降频 3 倍，考验离散化滞后敏感性）。

### 6.4 控制器与参数设置

**C1（本文，式 (5.2)）**：$e_\xi,e_z,A(\tilde x)$ 按定理 1/2（式 (4.1)–(4.5)）计算，$\dot J\dot{\boldsymbol q}$ 由 TNDQ 链解析读出（式 (3.5)）；位姿增益按附录 C.3 ③ 推广为对称正定矩阵 $K_p$。三档增益：

| 档位 | $K_d$ | $K_p=\mathrm{diag}(p_OI_3,p_TI_3)$ | 有效旋转刚度 $p_O/4$ | 设计极点（平移） | 备注 |
|---|---|---|---|---|---|
| base | $8I_6$ | $16I_6$（$p_O=p_T=16$） | **4**（与 $p_T=16$ 失配） | $\{-4,-4\}$ 临界 | 未整定对照档 |
| tuned | $24I_6$ | $p_O=320,\ p_T=80$ | 80（与 $p_T$ 配平） | $\{-4,-20\}$ | 主推档 |
| fast | $36I_6$ | $p_O=720,\ p_T=180$ | 180（与 $p_T$ 配平） | $\{-6,-30\}$ | 刚度上限档 |

增益由 §5.4 的极点分配规则生成（$K=(a+b)I$、$p_T=ab$、$p_O=4ab$）：旋转通道方程 (5.8) 含 $p_O/4$ 项（$A_0$ 引入的 **1/4 旋转刚度折减**），故 tuned 档取 $p_O=4p_T=320$ 使有效旋转刚度与平移刚度同为 80；base 档故意不补偿（$p_O=p_T=16$，有效旋转刚度仅 4），用以暴露折减缺陷（§6.5(2)）。三档的 $K_{p,T}$ 均为标量阵，满足定理 3(c-2) 通道拆分所需的各向同性条件。

**水平集条件的预核验**：定理 3(b) 要求 $c<c^*=\tfrac12\lambda_{\min}(K_{p,O})$。tuned 档 $c^*=160$、fast 档 $c^*=360$、base 档 $c^*=8$——即使 base 档也远大于实测的 $V$ 峰值（§6.5(5)），工作域假设在全部运行中成立。

**C2（忠实 [Ch20] resolved-acceleration 律，二阶基线）**：按原文式 (32)–(35) 与式 (2) 逐项移植——twist 误差取**经伴随搬运**的差 $\boldsymbol\omega_e=\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d-\boldsymbol\xi=-e_\xi$（式 (32)，与定理 1 的 $-e_\xi$ 同一）；加速度指令为
$$\boldsymbol a_{\mathrm{cmd}}=\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}(\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d)+K_v\boldsymbol\omega_e-K_P\,\mathrm{vec}_6(2\ln\tilde x),\qquad \boldsymbol u_{\mathrm{task}}=\boldsymbol a_{\mathrm{cmd}}-\dot J\dot{\boldsymbol q},$$
其中前两项即引理 1 的前馈（与 (5.2) 前馈**逐项相同**——[Ch20] 式 (33)/(34) 对 $\frac{d}{dt}\mathrm{Ad}$ 的展开与引理 1 同一），位姿反馈为原文式 (35) 的**螺旋对数整形** $K_P$ 作用于 $2\ln x_e$：原文的 $x_e$ 与本文右不变误差 $\tilde x$ 取向相反（原文约定下 $\frac{d}{dt}(2\ln x_e)=\boldsymbol\omega_e$，本文约定下 $\frac{d}{dt}\mathrm{vec}_6(2\ln\tilde x)=e_\xi=-\boldsymbol\omega_e$），故翻译到本文约定后符号取负（已由闭环消去 oracle 测试锁定：`tests/test_math_properties.py::test_chandra20_law_oracle`，取正号闭环发散、取负号收敛）。信息集按原文：$\dot{\boldsymbol\xi}_d$ 与 $\dot J\dot{\boldsymbol q}$ 均为**解析量**（期望链 $\sigma^2$ 通道 / (3.5) 免构造读出），无差分。**与 C1 (5.2) 的唯一结构差异是位姿反馈形式**：螺旋对数整形 vs $A^\top$ 整形——前者近恒等时 $\mathrm{vec}_6(2\ln\tilde x)\to[-2\mathcal O;\mathcal T]$，但导数映射在 $\phi\to\pi$ 奇异（E4 大误差真判别项），且无对任意 $K_p$ 成立的精确耗散等式（无定理 3 证书）。

> **配平（忠实 C2）**：近恒等线性化逐通道为 $\ddot{\boldsymbol\ell}+K_v\dot{\boldsymbol\ell}+K_P\boldsymbol\ell=0$（$\boldsymbol\ell\equiv\mathrm{vec}_6(2\ln\tilde x)$；旋转通道 $\phi\boldsymbol n\approx2\,\mathrm{Im}\,\tilde r=-2\mathcal O$ 自带因子 2，**无** C1 的 $1/4$ 折减、也无下文 C2-abl 的 $1/2$ 折减）。与 C1-tuned 极点 $\{-4,-20\}$（特征多项式 $(s+4)(s+20)$）、DC 刚度 80 对齐得 $K_v=24I_6$、$K_P=80I_6$（代码 `config/params.py::CH20_K_V/CH20_K_P`）。

**C2-abl（朴素 twist 差消融基线，非文献律）**：twist 误差取坐标差 $\boldsymbol\xi_d-\boldsymbol\xi$（不经 $\mathrm{Ad}_{\tilde x}$ 搬运），前馈 $\dot{\boldsymbol\xi}_d$ 与 $\dot J\dot{\boldsymbol q}$ 由数值差分获得，无 $A^\top$ 几何整形项。须明示其文献地位：[Ch20] 式 (32) 与 [P2] 前馈均含 Ad 搬运，**朴素坐标差不对应任何已发表理论**，本档仅用于消融 C1 自身结构（§4.1 伪项、$A^\top$ 整形、解析前馈三项属性的代价量化）。本稿早期版本曾把该朴素律标注为"[Ch20] 类"并以其实测数据充当 C2 基线，与原文不符，现更正：[Ch20] 的忠实代表是上文的 C2（dq-chandra），本章正文的全部 C2 数值均采自 dq-chandra 运行（`results/grasp_circle_chandra_*.npz`）；本档更名 C2-abl（dq-ctc），其测量数据仅存档于仓库，不进入 §6.5 表格。

> **折减因子不同，必须分别配平（更正）**：C1 的旋转刚度折减是 $1/4$，来自两个独立的 $\tfrac12$——$\dot{\mathcal O}=-\tfrac12\tilde{\boldsymbol\omega}$（$A_0$ 第一行）与 $A_0^\top$ 作用在位姿反馈上的 $\tfrac12$（见 (5.8)）。C2-abl 没有 $A^\top$ 整形项，位姿反馈直接取 $[+p_O\mathcal O;-p_T\mathcal T]$，因此只剩前一个 $\tfrac12$，其近恒等线性化为
> $$\ddot{\mathcal O}+K_\omega\dot{\mathcal O}+\tfrac{p_O}2\mathcal O=-\tfrac12d_\omega,\qquad \ddot{\mathcal T}+K_v\dot{\mathcal T}+p_T\mathcal T=d_v .$$
> 折减因子是 $1/2$ 而非 $1/4$。故与 C1-tuned 配平到同一有效刚度 80、同一特征多项式 $(s+4)(s+20)$ 所需的 C2-abl 增益是 $K_d=24I_6$、$p_O=\mathbf{160}$、$p_T=80$（代码 `config/params.py::DQC_K_D/DQC_K_P` 即取此值）。本稿早期版本写作“增益按**同样的** 1/4 折减规则配平”，与实现不符（若真按 1/4 规则取 $p_O=320$，C2-abl 的有效旋转刚度将是 160，即两倍于 C1/C3，对比不再同预算），现更正。

配平后各律（C1-tuned / C2 / C2-abl / C3）的旋转/平移 DC 刚度同为 80、名义 $d\to(\mathcal O,\mathcal T)$ 传递函数逐通道恒等，闭环极点均为 $\{-4,-20\}$。

> **数据版本说明**：本章全部数值采自忠实 C2（dq-chandra）补跑到位后的最新一批仿真（`grasp_metrics_summary.csv` 的全部 43 个 $(\text{law},\text{gains},\text{mode},\text{condition})$ 组合，含 `results/grasp_circle_chandra_*.npz`）；早期版本中以 C2-abl（dq-ctc）数据标注为"C2"的全部数值已删除或替换，不再出现在本章任何表格与分析中。

**C3（DQ-H∞ + 加速度桥接，一阶基线）**：忠实移植 [P2] 式 (12) 的 H∞ 运动学律（$k_O=\sqrt2/\gamma_O=8$、$k_T=\sqrt2/\gamma_T=4$），经内环速度伺服（$K_{\mathrm{servo}}=20$，含一拍差分）桥接到加速度级，等效级联极点亦为 $\{-4,-20\}$。至此各律 DC 刚度均为 80——标准工况线性化意义下增益完全配平，对比聚焦于结构差异。

**H∞ 证书参数（两种读法，不可混用）**。本稿早期版本写作“取 $\kappa=1.0,\gamma_a=0.5$，… tuned 档认证 $L_2$ 增益上界 $1/\lambda_{\min}(K_d)=1/24$”，这两句互不相容（前者认证的增益是 $\gamma_a\sqrt\kappa=0.5$，而非 $1/24$），现分列为两个独立的读法：

- **读法 A（可行性判定，$\theta=\tfrac12$ 成员）**：取 $\kappa=1.0$、$\gamma_a=0.5$，则 (5.6a) 要求 $\lambda_{\min}(K_d)\ge\tfrac12(\kappa^{-1}+\gamma_a^{-2})=\tfrac12(1+4)=2.5$，三档增益（8/24/36）均满足；此时被认证的 $L_2$ 能量增益是 $\gamma_a\sqrt\kappa=\mathbf{0.5}$。这一读法回答“给定的 $(\kappa,\gamma_a)$ 目标能否被现有增益认证”，但它并非族内最紧（因 $\kappa\ne\gamma_a^2$）。
- **读法 B（族内最紧增益，$\theta=\theta^*$）**：若目标是从给定 $K_d$ 反推**最小可证增益**，则应取 AM–GM 等号 $\kappa=\gamma_a^2$ 并使最紧条件 $\lambda_{\min}(K_d)\ge1/(\gamma_a\sqrt\kappa)=\gamma_a^{-2}$ 取等；tuned 档 $\lambda_{\min}(K_d)=24$ 对应 $\gamma_a=1/\sqrt{24}\approx\mathbf{0.204}$、$\kappa=\gamma_a^2=1/24\approx0.0417$，认证 $L_2$ 增益 $=1/\lambda_{\min}(K_d)=1/24\approx\mathbf{0.042}$。下文 §6.5(5) 引用的 0.042 指的是本读法。

两种读法均不影响闭环轨迹（$\kappa,\gamma_a$ 是分析参数，不进控制律），只影响“声称了什么”；代码侧 `control/gain_design.py` 的 `screen()` 按读法 A 计算可行性阀值 `level = 0.5*(1/kappa + 1/gamma_a**2)`，并另给 `l2_certified = 1.0/lam_min` 对应读法 B，两者在代码中已分开输出。

### 6.5 结果与分析

指标按相位统计：平移/姿态误差 $\|\mathcal T\|,\|\mathcal O\|$ 的 RMS、twist 误差 $\|e_\xi\|$ 的 RMS、关节力矩范数 $\tau_{\mathrm{rms}}$、Lyapunov 值 $V$。

> **$V$ 的权重口径（重要）**：日志中的 `V_ss`/`V_peak` 对**全部增益档**统一采用 **base 档权重** $K_p^{\mathrm{base}}=16I_6$ 记录，即 $V^{\mathrm{base}}=\tfrac12\|e_\xi\|^2+8\|e_z\|^2$（已由三档数据逐一复算核实，吻合到三位有效数字）。因此在把实测 $V$ 与定理 3(b) 的水平集阈值 $c^*=\tfrac12\lambda_{\min}(K_{p,O})$ 比较时**必须换算**：对 tuned 档（$K_p=\mathrm{diag}(320I_3,80I_3)$）有 $V^{\mathrm{tuned}}\le(320/16)\,V^{\mathrm{base}}=20\,V^{\mathrm{base}}$。

**(1) 空载基线：实现正确性与协议无偏**。空载（名义模型准确、无失配扰动）circle-ss 下三律误差均进入 $10^{-4}$ m / $10^{-5}$ rad 量级（C1-tuned：$\|\mathcal T\|_{\mathrm{rms}}=1.355\times10^{-4}$ m、$\|\mathcal O\|_{\mathrm{rms}}=8.70\times10^{-5}$），三律 $\|e_\xi\|_{\mathrm{rms}}$ 为 **1.532 / 1.532 / 1.502**（×10⁻⁴，C1-tuned / C2 / C3）。两个事实值得记录：其一，**C1 与忠实 C2 在五位有效数字内完全重合**（相对差 0.000%）——两律同为解析前馈 + Ad 搬运 twist 误差的二阶律，信息集相同，唯一差异是位姿反馈整形（$A^\top$ vs 螺旋对数），而后者在近恒等极限下只差高阶项，空载小误差工况的数值重合正是 §6.4 结构分析的定量印证；其二，**C1（与 C2）比 C3 高 1.98%**，即空载工况下 C1 并非最优。这与 §4–5 的机理分析并不冲突——空载时名义模型精确、$d_{\mathrm{ex}}\approx0$，残差由离散化与数值精度主导，二阶解析前馈链（$\mathrm{Ad}$/$\mathrm{ad}$ 与 TNDQ 二阶通道，C1/C2 共有）比一阶桥接引入更多浮点运算，在扰动趋零的极限下其结构优势无从体现、舍入代价反而显露。C1 的优势只应在**存在真实扰动**时被主张（见 (3)(4)）。C1-base 低增益档（$\|\mathcal T\|_{\mathrm{rms}}=6.32\times10^{-4}$ m、$\|e_\xi\|_{\mathrm{rms}}=3.64\times10^{-4}$）亦稳定收敛，与定理 3(b) 的无扰渐近收敛/局部指数稳定一致。

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

两个通道各自独立反演出的扰动幅值一致到 1.93%（与 (P1) 的偏差同值），且**两通道的偏差量恰好相同**——这正是 (5.8)/(5.9) 所预言的（两通道共用同一组极点比例，二阶残差以相同比例进入两式）。这是一个真正有伪造风险的检验：若 (5.9) 中的 1/4 旋转折减因子写错（例如漏掉 $\tfrac12$ 而用 $\lambda(K_{p,O})\|\mathcal O\|$），旋转通道反演值将变为 1.366/1.393，与平移通道的 0.389 相差 3.5 倍，量纲虽仍可辩（rad/s² vs m/s²）但两档一致性会被破坏；实测的双通道同步一致构成对折减因子的独立确认。

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

10 组两两对比（5 条件 × 2 基线）中 C1 零例外占优，两个基线的差距量级截然不同：**相对 C3 为 1.52%–2.15%**（none 1.61%、highspeed 2.00%、fast-transit 2.12%、noise 2.15%、coarse-dt 1.52%），**相对忠实 C2 仅 0.00%–0.05%**——五个条件下两律曲线在四位有效数字内重合。这是本章最重要的结构性观察：C1 与忠实 C2 的性能等价不是巧合，而是 §4–5 结构分析的定量确认——两律的前馈（$\mathrm{Ad}$ 搬运 + $\mathrm{ad}$ 输运修正，引理 1）与 twist 误差几何（$\omega_e=-e_\xi$，式 (32) 与 (4.4) 同一）**逐项相同**，增益配平后闭环线性化也逐通道恒等，故任何工况下的差异只能来自位姿整形的高阶项；等价性同时意味着，本文相对 [Ch20] 的贡献**不能**表述为精度提升，而应表述为：同一性能水平下，(5.2) 额外携带定理 3 的三类证书（耗散等式、水平集不变性、均方界），且其 $A^\top$ 整形避开螺旋对数在 $\phi\to\pi$ 的导数奇异。

方向与 §4–5 的结构分析预测逐条一致：highspeed 下 C1/C2 的解析前馈对 C3 的领先扩大——一拍滞后桥接在高动态下损失精度；noise 下 C3 劣化最明显（3.234 vs 3.165/3.164），与其桥接项 $\Delta\dot q_{\mathrm{cmd}}/\mathrm{dt}$ 的噪声放大机制一致，而 C1 与 C2 在此条件下不可区分；coarse-dt 下 C3 的一拍滞后被放大 3 倍。位置级稳态残差始终被静态刚度锁定（三律差异 <0.2%），**结构差异集中体现在速度级误差 $e_\xi$**。诚实的强度评估：单组 1.5%–2% 的差距，在无重复实验（每组仅 1 次运行、无随机种子重复）的条件下不足以支撑统计显著性声明；本文主张的是**方向的一致性**（相对 C3 10/10 无例外）与**机理的可解释性**（C3 的桥接差分是唯一的信息损失来源，劣势随速度/噪声/采样粗化按机理扩大），而非幅度本身。

**(6) Lyapunov 收敛、水平集与均方界的核验**。空载 C1-tuned 从初始扰动 $V^{\mathrm{base}}_{\mathrm{peak}}=7.74\times10^{-5}$ 衰减 2.5 个数量级至 $V^{\mathrm{base}}_{ss}=2.20\times10^{-7}$（无扰渐近收敛，与定理 3(b)(iii)(iv) 一致）；带载在杯附着后 $V^{\mathrm{base}}_{\mathrm{peak}}=2.47\times10^{-2}$，随后在 $t_{\mathrm{conv}}=\mathbf{1.50}$ s 内回落至 $V^{\mathrm{base}}_{ss}=3.36\times10^{-4}$；$V_{ss}$ 随增益档单调递减（$2.42\times10^{-2}\to3.36\times10^{-4}\to6.89\times10^{-5}$）。三点定量核验：

- **水平集条件 (5.5b)**：按上文权重口径换算，$V^{\mathrm{tuned}}_{\mathrm{peak}}\le20\times2.47\times10^{-2}=0.494$，而 tuned 档 $c^*=\tfrac12\lambda_{\min}(K_{p,O})=160$，余度约 **2.5 个数量级**（本稿早期版本称"四个数量级"，系直接把 base 权重的 $V$ 与 tuned 权重的 $c^*$ 相比所致，现更正）。base 档 $c^*=8$ 对其自身 $V_{\mathrm{peak}}$ 亦有 2 个数量级余度。故定理 3(b)/3(d) 所需的工作域前提在全部 43 组运行中以充分余度成立——这是把定理 3(d) 的 $\Omega_c$ 前提"事后数值核验"（定理 3(d) 注记与附录 C.5）的具体落实。
- **均方极限界 (5.7) 的保守性**：由 (P2) 反演得 $\|d_{\mathrm{ex}}\|\approx\sqrt{0.389^2+0.683^2}=0.786$；带载时 $\alpha$ 主要来自 $\Delta M$（杯质量相对末端等效惯量），取 $\alpha\to0$ 的乐观极限，(5.7) 给出 $\mathrm{RMS}(e_\xi)\le0.786/\lambda_{\min}(K_d)=0.786/24=3.27\times10^{-2}$；实测 $\mathrm{RMS}(e_\xi)=9.399\times10^{-4}$，比值 **34.8**。即该界成立但**保守约 1.54 个数量级**。保守性的来源可以逐项归因：(5.7d) 中 $\|A\|_2\le1+\|\mathcal T\|$ 与 $\|A_{11}\|_2=\tfrac12$ 均按最坏方向取值，而实际 $d_{\mathrm{ex}}$ 与 $e_\xi$ 在圆周段近似正交（扰动主要沿重力方向、误差主要沿切向），故 $e_\xi^\top d$ 远小于 $\|e_\xi\|\|d\|$；Young 不等式的等号条件（$\|e_\xi\|=D/\lambda_{\mathrm{eff}}$）也远未达到。这一量级的保守性与 [P2] γ 扫描观察到的 H∞ 界保守性同源。
- **H∞ 证书 (5.6a)**：tuned 档按 §6.4 读法 B 认证 $L_2$ 增益上界 $1/\lambda_{\min}(K_d)=0.042$。须注意本章的带载工况扰动是**偏差型**（$d_{\mathrm{ex}}$ 近似常值，$\|d_{\mathrm{ex}}\|_{L_2}=\infty$），严格来说不落在定理 3(c) 的 $d\in L_2$ 前提内；能够核验的只有有限时窗上的能量比，实测该比值远小于 0.042。因此本文对 (5.6a) 的实验支持仅限于"未被违反"，**不构成对 $L_2$ 增益界紧性的检验**——后者需要 §6.7 的 γ 扫描协议（配合 $L_2$ 型注入扰动）才能完成。

**(7) 安全与计算审计**。力矩饱和步数：43 组运行**全部有记录且均为 0**（早期版本因导出字段缺失只能断言"17/19 组无饱和"，本批数据的 `sat_steps` 字段完整，可作全量断言）。零空间治理器：仅 C1-base 带载组触发 3 步；C3 与 C2-abl 的 load/fast-transit 组各触发 1 步（快搬运相位的高加速度需求），其余 40 组为 0。据此，除 base 档外的全部对比结果均在**远离安全边界**的线性工作区取得，不存在饱和掩盖差异的可能。计算开销方面，`runtime_mean_ms` 跨越 8.9–10.8 ms，但该字段**不能用于比较控制律的计算成本**：其数值远大于控制周期 $\Delta t=5$ ms，说明它是含仿真器 RPC 往返的墙钟时间；且 C1 自身在不同增益档间的跨度（9.1→10.8，19%）已超过任何组间差异。因此本文**撤回**早期版本"C1 的 TNDQ 链读出未带来可观测的额外开销"这一结论——该命题需要隔离控制器函数的专项计时（如 `perf_counter` 只包裹 `control_law` 调用）才能检验，本章数据不支持任何方向的结论。§3 关于式 (3.5) 复杂度的论述仍是**操作计数意义**上的（连乘一次给出三通道），与墙钟计时无关。

### 6.6 小结

(i) 控制律 (5.2) 在含接触与负载突变的全相位任务中稳定收敛，且定理 3(b) 所需的水平集条件 (5.5b) 在全部 43 组运行中以 ≥2 个数量级的余度成立（§6.5(6)）；(ii) §5.4 的**静态刚度标度律** (5.9) 通过了强形式检验——两个通道、两个增益档反演出的等效扰动幅值一致到 1.93%，其中旋转通道的 1/4 折减因子获得独立确认；(iii) 严格公平协议下，C1 在 10 组两两对比（5 条件 × 2 基线）的速度级指标上零例外占优：相对一阶桥接基线 C3 稳定优 1.52%–2.15%；相对忠实 [Ch20] 二阶基线 C2 数值等价（≤0.05%）——该等价是 §4–5 结构分析（两律同信息集、前馈与 twist 误差几何逐项相同）的定量确认，C1 对 C2 的分化体现在定理 3 证书与大误差几何（$A^\top$ 整形避开螺旋对数的 $\phi\to\pi$ 奇异），位置级与各基线持平；(iv) 均方极限界 (5.7) 与 H∞ 证书 (5.6a) 均未被违反，但前者保守约 1.54 个数量级。

**本章不支持的声明（诚实边界）**：(a) 空载工况下 C1（与 C2 重合）并不优于 C3（$\|e_\xi\|_{\mathrm{rms}}$ 反而高 1.98%），故不能声称无条件的精度优势；(b) 每组仅 1 次运行、无随机种子重复，1.5%–2% 的差距不具备统计显著性，只能主张方向一致性与机理可解释性；(c) 带载扰动为偏差型，不在定理 3(c) 的 $L_2$ 前提内，故 H∞ 界的**紧性**未获检验；(d) `runtime_mean_ms` 含仿真器 RPC，无法隔离控制器开销，计算代价无结论；(e) base 档落在 (5.9) 适用域之外，其反演不一致不应读作理论失效；(f) C1 与忠实 C2 的性能等价意味着本章**不主张**相对 [Ch20] 的精度提升，主张仅为同性能下的证书增益与几何鲁棒性。局限：仅覆盖 0.25 kg 单一负载与 2.5 rad/s 以下速度，三律位置级差异被刚度配平策略压缩，更高速度、柔性接触或增益不配平场景下的差异化验证，以及带重复种子的统计显著性实验，留待真机阶段。

### 6.7 γ 扫描协议（后续实验设计）

针对 §5.3 注记的 $\gamma_a$ 双通道推论，设计三组扫描（同对象/轨迹/扰动，每个 γ 点记录证书可行性、认证/实测 $L_2$ 增益与稳态误差 RMS）；**扫描必须注入 $L_2$ 型扰动**（如有限时长的脉冲/衰减扰力）而非 §6 的持续偏差型负载，否则不落在定理 3(c) 前提内（§6.5(6) 第三条）：**A 组**（证书扫描）固定增益扫 $\gamma_a$，预期测得列逐位不变——$\gamma_a$ 是分析参数，只移动证书可行域边界 $\gamma_a\ge[2\lambda_{\min}(K_d)-\kappa^{-1}]^{-1/2}$（注记 (i)）；**B 组**（综合模式）按 $\kappa=\gamma_a^2$、$K_d=\gamma_a^{-2}I$ 回写增益，预期误差随 $\gamma_a$ 单调下降、完整不等式 (5.6)（含 $2V(0)$ 项）逐点核验通过、认证增益 $=\gamma_a^2$（注记 (ii)）；**C 组**（对照）复刻 [P2] 的 $\gamma_O=\gamma_T=\gamma$ 综合参数扫描，预期与 B 组趋势同构但力矩接口下无证书可对照（"可调不可证"）。另建议补两项本章数据无法回答的对照：**D 组**（重复性）对 §6.5(5) 的 10 组两两对比取 ≥10 个噪声种子重复，以建立统计显著性；**E 组**（开销）用只包裹控制律调用的专项计时重测三律单步计算成本。

---

## 7. 结论

本文以截断多项式代数 $\mathcal A_2$（TNDQ）重构机械臂运动学，核心是两条法则：连乘法则 $\overline{xy}=\bar x\,\bar y$（使位姿/速度/加速度一次链连乘同时得到）与截断相容性 $\breve x=$"$\bar x$ 的前两通道"（使误差体系可以无损地定义在两通道 HDQ 上）。误差体系由一次 HDQ 乘法生成（定理 1），经输出映射闭合为级联运动学（定理 2）；几何一致计算力矩律使闭环达到级联标准形，并在一个共同的存储函数上得到三类证书：无扰时的**水平集不变性 + 渐近收敛 + 局部指数稳定**（定理 3(b)）、$L_2$ 扰动下的 **H∞ 二次型/Schur 补当且仅当判据**与旋转/平移通道的精确拆分（定理 3(c)）、$L_\infty$ 扰动下 twist 误差的**均方（RMS）极限界**（定理 3(d)）。加速度层被证明不需要误差通道：期望加速度走前馈、不确定性走扰动——这一结构性取舍同时简化了状态空间（12 维）与实现（误差层只用 DQ/HDQ 乘法）。近恒等线性化通道 (5.8) 进一步揭示了一个容易被忽略的实现陷阱：$A_0$ 的旋转块 $-\tfrac12I_3$ 使旋转刚度受 **1/4 折减**，不补偿则两通道带宽严重失配（§6.5(3) 以 12.3 倍的姿态误差放大从反面验证）。CoppeliaSim/KUKA LBR4+ 力矩模式仿真（§6，43 组运行）定量核验了上述主张：静态刚度标度律 (5.9) 的**等效扰动反演一致性**在两通道、两增益档上同步符合至 1.93%；所提控制律在全部带载敏感条件的速度级指标上以 10/10 无例外的方向一致性优于增益配平后的一阶桥接基线 C3（1.52%–2.15%）；与忠实 [Ch20] 二阶基线 C2 数值等价（≤0.05%）——该等价定量确认了两律同信息集的结构分析，本文相对 [Ch20] 的主张为同性能下的证书增益（定理 3 的耗散等式/水平集不变性/均方界）与大误差几何鲁棒性，而非精度提升；水平集条件与两类证书均未被违反。

**局限与后续工作**（按严重程度排序）：(i) **定理 3(d) 的 $\Omega_c$ 前提尚未自洽闭合**——含扰时 $\dot V$ 可正，水平集未必不变，而等效扰动幅值 $D$ 反过来依赖于该水平集，形成循环；彻底解除需附录 C.4 的 strictification（变权存储函数 $W=V+\epsilon e_z^\top K_pAe_\xi$）以获得全状态负定项，这是得到真正局部 ISS 结论的必经之路，本文仅给出路线而未完成；(ii) 本文的稳定性结论均为**工作域局部**——双覆盖引起的 unwinding 与 $\tilde\eta=0$ 处 $A$ 的奇异是拓扑障碍，任何连续反馈均不可能给出全局结果；(iii) 级联系统（内环 + 运动学外环）的整体 H∞ 界未建立，本文已撤回早期版本"内环 ISS + 外环 H∞ 蕴含级联 ISS"的断言；(iv) 变权存储函数（操作空间惯量 $\Lambda$ 加权）需处理 $\dot\Lambda$ 项，本文未展开；(v) 仿真验证仅覆盖单一负载与中低速工况、每组无重复，真机实验与 §6.7 的 A–E 五组扫描协议（尤其是重复性 D 组与开销 E 组）为后续内容；(vi) 定理 1 的几何一致误差与 Adorno 学派 DQ 动力学控制文献的逐条查重在投稿前完成。

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

### C.1 扰动通道的适定性（(5.1) 为何是显式等式，以及 $\alpha$ 条件的真正作用）

$\boldsymbol\tau=\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+\hat C\dot{\boldsymbol q}+\hat g$ 代入真实动力学 $M\ddot{\boldsymbol q}+C\dot{\boldsymbol q}+\boldsymbol g=\boldsymbol\tau+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}}$ 并解出 $\ddot{\boldsymbol q}$：
$$
\ddot{\boldsymbol q}=M^{-1}\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+M^{-1}\bigl(\Delta C\dot{\boldsymbol q}+\Delta\boldsymbol g+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}}\bigr),
\qquad M^{-1}\hat M=I+M^{-1}\Delta M ,
$$
即 (5.1)。注意 $\ddot{\boldsymbol q}_{\mathrm{ref}}$ 由 (5.2) 完全由**当前可测量** $(\boldsymbol q,\dot{\boldsymbol q},\hat{\underline x}_d,\boldsymbol\xi_d,\dot{\boldsymbol\xi}_d)$ 给出，**不依赖 $\ddot{\boldsymbol q}$**，故 (5.1) 对 $\ddot{\boldsymbol q}$ 是**显式赋值**：不存在隐式代数环，既不需要移项、也不需要 Neumann 级数，更不会产生 $\alpha/(1-\alpha)$ 型的等效扰动增益因子。

由此可见 $\alpha<1/\mathrm{cond}_2(K_d)$（(5.1f)）与适定性无关，而是为了使定理 3(d) 中的**有效阻尼保持正定**：乘性分量 $\Theta u_{\mathrm{fb}}$ 中与 $-K_de_\xi$ 同向的部分会削弱阻尼，由 $|e_\xi^\top\Theta(-K_de_\xi)|\le\alpha\lambda_{\max}(K_d)\|e_\xi\|^2$ 得 $\lambda_{\mathrm{eff}}=\lambda_{\min}(K_d)-\alpha\lambda_{\max}(K_d)$，正性恰好等价于 (5.1f)。换句话说，$\alpha$ 条件是一个**小增益型的证书条件**，而不是微分方程适定性条件；若 $\alpha$ 过大，闭环仍然定义良好，但本文的 Lyapunov 证书失效（并不意味着失稳，只是不可证）。

**(5.1d) 与 $d_{\mathrm{ex}}$ 的逐项核对。** 将 $\ddot{\boldsymbol q}_{\mathrm{ref}}=J^{+}(u_{\mathrm{ff}}+u_{\mathrm{fb}}-\dot J\dot{\boldsymbol q})$ 代入 $d=J\boldsymbol w_{\mathrm{dyn}}+\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$，只有 $JM^{-1}\Delta M\,\ddot{\boldsymbol q}_{\mathrm{ref}}$ 一项含控制量；用 (A1) 的 $JJ^{+}=I_6$ 得
$$
JM^{-1}\Delta M\,\ddot{\boldsymbol q}_{\mathrm{ref}}=\underbrace{JM^{-1}\Delta MJ^{+}}_{=\,\Theta}\bigl(u_{\mathrm{ff}}+u_{\mathrm{fb}}-\dot J\dot{\boldsymbol q}\bigr)=\Theta u_{\mathrm{fb}}+\Theta\bigl(u_{\mathrm{ff}}-\dot J\dot{\boldsymbol q}\bigr).
$$
右端第一项即 (5.1d) 的乘法分量；第二项与剩余的 $JM^{-1}(\Delta C\dot{\boldsymbol q}+\Delta\boldsymbol g+\Delta\boldsymbol f+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}})+\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$ 合成 $d_{\mathrm{ex}}$。关键记账点：$u_{\mathrm{ff}}$ 虽由控制律生成，但只依赖期望轨迹与位姿误差的**前馈**结构、不包含 $(K_d,K_p)$ 反馈增益，故归入外生部分；若将其也计入乘法通道，定理 3(d) 中的 $\lambda_{\mathrm{eff}}$ 不变而 $D$ 会被重复计数。

分解 $\boldsymbol w_b/\boldsymbol w_{L_2}$ 按各源时间特性归类：参数误差与摩擦残差持续存在（$L_\infty$），测量噪声能量有限或可白化（$L_2$ 类）。注意此处的分类是按**时间特性**（$L_2$ vs $L_\infty$），与 (5.1d) 按**依赖关系**的分解（乘性 $\Theta u_{\mathrm{fb}}$ vs 外生 $d_{\mathrm{ex}}$）是两个正交的划分维度，不得混用（定理 3(a) 的术语约定）。

### C.2 定理 3(b) 的定量奇异值下界与收敛论证细节

$\dot V=-e_\xi^\top K_de_\xi$ 只是关于全状态的**半**负定（$e_\xi=0$ 面上 $\dot V=0$），故不能直接给出收敛，必须补 LaSalle 步骤（定理 3(b) 证明第 4 步），而 LaSalle 步骤需要 $A$ 在工作域内可逆。定量化如下。

由 (4.5)，$A=\begin{bmatrix}A_{11}&0\\ -[\mathcal T]_\times& I_3\end{bmatrix}$，$A_{11}=-\tfrac12(\tilde\eta I+[\mathcal O]_\times)$。其逆为
$$
A^{-1}=\begin{bmatrix}A_{11}^{-1}&0\\ [\mathcal T]_\times A_{11}^{-1}& I_3\end{bmatrix},
\qquad
A_{11}^{-1}=-2\,\frac{\tilde\eta^2I+\tilde\eta[\mathcal O]_\times^{\top}+\mathcal O\mathcal O^\top}{\tilde\eta(\tilde\eta^2+\|\mathcal O\|^2)}
=-\frac2{\tilde\eta}\bigl(\tilde\eta^2I-\tilde\eta[\mathcal O]_\times+\mathcal O\mathcal O^\top\bigr),
$$
用到 $\tilde\eta^2+\|\mathcal O\|^2=1$。$\|A_{11}^{-1}\|_2$ 无需放缩即可精确算出：记 $B\triangleq\tilde\eta I+[\mathcal O]_\times$（故 $A_{11}=-\tfrac12B$），则
$$
B^\top B=(\tilde\eta I-[\mathcal O]_\times)(\tilde\eta I+[\mathcal O]_\times)=\tilde\eta^2I-[\mathcal O]_\times^2=(\tilde\eta^2+\|\mathcal O\|^2)I-\mathcal O\mathcal O^\top=I-\mathcal O\mathcal O^\top ,
$$
其特征值为 $\{\,\tilde\eta^2,\,1,\,1\,\}$（沿 $\mathcal O$ 方向为 $1-\|\mathcal O\|^2=\tilde\eta^2$，两个正交方向为 1）。于是 $B$ 的奇异值为 $\{\tilde\eta,1,1\}$，同时给出两个**精确**等式
$$
\|A_{11}\|_2=\tfrac12\sigma_{\max}(B)=\tfrac12 ,
\qquad
\|A_{11}^{-1}\|_2=\frac2{\sigma_{\min}(B)}=\frac2{\tilde\eta} ,
$$
前者正是 §5.2 与定理 3(b) 反复使用的精确值。再由分块下三角结构 $\|A^{-1}\|_2\le(1+\|\mathcal T\|)\|A_{11}^{-1}\|_2+1$，
$$
\sigma_{\min}(A)=\frac1{\|A^{-1}\|_2}\;\ge\;\Bigl[\frac{2(1+\|\mathcal T\|)}{\tilde\eta}+1\Bigr]^{-1} .
$$
在 $\Omega_c$ 内 $\tilde\eta\ge\eta_0$（(5.5c)）且 $\|\mathcal T\|\le\sqrt{2c/\lambda_{\min}(K_{p,T})}$，故 $\sigma_{\min}(A)\ge c_A(\eta_0,c)>0$ 一致成立。于是 LaSalle 第 4 步中由 $A^\top K_pe_z\equiv0$ 推出 $\|e_z\|\le\|K_p^{-1}\|_2\|A^{-\top}\|_2\cdot0=0$ 严格成立。局部指数率由线化矩阵 $F$ 的谱给出，逐通道具体极点见 (5.8)。域限制 $\tilde\eta>0$ 与 [P2] Remark 1 的 unwinding 条件一致。

> 注：无扰情形下 LaSalle 路线自身是完整的，但**不能**改用级联 ISS 论证闭合——后者需要 $e_\xi\to e_z$ 通道的 ISS 增益，而 $V$ 在 $e_z$ 方向无耗散项，无法直接提供该增益（参见附录 C.5 与 §5.5）。

### C.3 定理 3(c) 的二次型/Schur 补细节与最紧增益族

**(c-1) 三条等价路径。** 记供给率 $s(e_\xi,d)\triangleq-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$。性能目标 $\dot V\le s$（对一切 $(e_\xi,d)$ 逐点成立）等价于 $-e_\xi^\top K_de_\xi+e_\xi^\top d-s(e_\xi,d)\le0\ \forall(e_\xi,d)$，可经三条路径判定：

1. **二次型/Schur 补**（正文路径）：整理为 $-[e_\xi;d]^\top M[e_\xi;d]\le0$，其中 $M$ 为 (5.6a) 的分块矩阵。$M\succeq0$ 且右下块 $\tfrac{\gamma_a^2}2I\succ0$，取 Schur 补（对称分块 $\begin{bmatrix}P&Q\\ Q^\top&R\end{bmatrix}\succeq0\iff R\succ0$ 且 $P-QR^{-1}Q^\top\succeq0$，此处 $Q=-\tfrac12I$、$QR^{-1}Q^\top=\tfrac14\cdot\tfrac2{\gamma_a^2}I=\tfrac1{2\gamma_a^2}I$）：
$$
M\succeq0\iff K_d-\tfrac1{2\kappa}I-\tfrac1{2\gamma_a^2}I\succeq0\iff K_d\succeq\tfrac12(\kappa^{-1}+\gamma_a^{-2})I .
$$
这是**当且仅当**判据：不定号交叉项 $e_\xi^\top d$ 保留在二次型内整体判定，无任何符号放缩。
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
由 AM–GM，$\tfrac12(\gamma_a^{-2}+\kappa^{-1})\ge(\gamma_a\sqrt\kappa)^{-1}$，等号当且仅当 $\kappa=\gamma_a^2$——(5.6a) 仅在 $\kappa=\gamma_a^2$ 时达到族内最紧。这一最紧界与两个独立来源的 $1/\lambda_{\min}(K_d)$ 一致：(d) 的均方极限界在 $\alpha\to0$ 极限下为 $\|d_{\mathrm{ex}}\|_{L_\infty}/\lambda_{\min}(K_d)$（(5.7)），以及 $e_\xi$ 子系统线性极限的 $\|(sI+K_d)^{-1}\|_{H_\infty}=1/\lambda_{\min}(K_d)$（$K_d$ 对称时）。三者同为 $1/\lambda_{\min}(K_d)$ 并非巧合：三条路线均仅使用了精确耗散等式 $\dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d$ 中的同一个阻尼项，而对 $e_z$ 方向完全未加利用（$V$ 在该方向无耗散）；因此 $1/\lambda_{\min}(K_d)$ 是这条 Lyapunov 路线可证增益的**天花板**，要突破它必须换存储函数（附录 C.4）。

**(c-2) 两处恒零的几何含义与失效条件。** 拆分精确成立依赖两条叉积混合积恒等式：(i) $(A^\top e_z)_\omega$ 中 $[\mathcal T]_\times\mathcal T=\mathcal T\times\mathcal T=0$——平移误差对旋转反馈通道的耦合以"作用在自身上的叉积"形式进入，恒零；(ii) $\dot{\mathcal T}=-[\mathcal T]_\times\tilde\omega+\tilde v$ 中耦合项对 $\tfrac12\|\mathcal T\|^2$ 不做功，$\mathcal T^\top(\mathcal T\times\tilde\omega)=0$——与附录 A.3 中 [P2] 平移通道"叉积项与 $\mathcal T$ 正交"同一机制。二者是代数恒等式，不依赖工作域，故 (c-2) 与 (c-1) 一样是全局（$\tilde\eta$ 无关）结论。若 $K_d$ 非块对角，$-e_\xi^\top K_de_\xi$ 含 $\tilde\omega^\top K_{\omega v}\tilde v$ 型交叉项，两通道能量不再分离，退回合并判据 (c-1)。

**$K_{p,T}$ 各向同性的必要性。** 将平移刚度写作一般对称正定 $K_{p,T}$ 时，上述两处恒零均失效：(i) $(A^\top K_pe_z)_\omega=A_{11}^\top K_{p,O}\mathcal O+[\mathcal T]_\times K_{p,T}\mathcal T$，而 $[\mathcal T]_\times K_{p,T}\mathcal T=\mathcal T\times(K_{p,T}\mathcal T)\ne0$ 除非 $\mathcal T$ 是 $K_{p,T}$ 的特征向量，故旋转反馈重新含 $\mathcal T$；(ii) 平移储能变为 $\tfrac12\mathcal T^\top K_{p,T}\mathcal T$，其导数中的耦合项 $-\mathcal T^\top K_{p,T}[\mathcal T]_\times\tilde\omega$ 不再为零（仅当 $K_{p,T}=k_{p,T}I_3$ 时由 $\mathcal T^\top[\mathcal T]_\times=0$ 而消失）。两项残留均是 $\tilde\omega$–$\mathcal T$ 型耦合，使 $V_\omega,V_v$ 不再各自闭合，因而同样退回 (c-1)。这与 §5.2 说明 (c) 中“$K_p$ 写在 $A^\top$ 内侧”的要求是同一代数机制的两个侧面。

**$\gamma$-$\kappa$ 设计规则（实验化，对应 §6.7）。** 上述 θ 族给出从目标增益到增益矩阵的完整综合流程：给定目标 $\gamma_a$，① 取 $\kappa^*=\gamma_a^2$（AM–GM 等号，(5.6a) 在族内最紧）；② 取 $\lambda_{\min}(K_d)=1/(\gamma_a\sqrt{\kappa^*})=\gamma_a^{-2}$（最紧条件取等号，证书恰紧，对应 $\theta^*=\sqrt{\kappa^*}/2\gamma_a=\tfrac12$）；③ 位姿增益另行配置动态品质：正文自 (5.2) 起已统一采用**对称正定矩阵** $K_p$（$V$ 中为 $\tfrac12e_z^\top K_pe_z$、(5.2) 中为 $A^\top K_pe_z$，$K_p$ 必须写在 $A^\top$ **内侧**，否则对各向异构或带耦合块的 $K_p$，定理 3(b) 第 1 步的交叉项不再相消（块各向同性情形下两种写法恰巧等价，见 §5.2 说明 (c)）；标量情形 $K_p=k_pI_6$ 是其特例。取 $K_p=\mathrm{diag}(p_OI_3,p_TI_3)$，当 $K_\omega=K_v=k_dI_3$ 时临界阻尼可取 $p_T=(k_d/2)^2$、$p_O=4p_T$（**1/4 旋转折减**的补偿，见 (5.8)），两线性化通道双重极点。此时认证 $L_2$ 能量增益 $=1/\lambda_{\min}(K_d)=\gamma_a^2$，$\gamma_a$ 成为单参数性能旋钮。可达下界由工程约束给出：离散化余量 $\max_i|p_i|\Delta t\le c_{\mathrm{disc}}$（显式积分稳定裕度）与指令峰值预算 $\lambda_{\max}(K_d)\|e_\xi\|+\tfrac12\lambda_{\max}(K_p)\|e_z\|\le\ddot q_{\max}$ 均随 $\gamma_a^{-2}$ 增长，两者中先达界者决定 $\gamma_a^{\min}$。反之，若固定 $K_d$ 不回写（仅分析已有设计），则 $\gamma_a$ 扫描只移动证书可行域边界 $\gamma_a\ge[2\lambda_{\min}(K_d)-\kappa^{-1}]^{-1/2}$，对闭环轨迹无任何影响——这是"分析参数"与 [P2] "综合参数"（$k=\sqrt2/\gamma$ 直接进控制律）的可观测判别。

### C.4 待完成的一步：strictification 与真正的局部 ISS（未来工作）

定理 3(d) 的三条缺口（不是 ISS、依赖 $\Omega_c$ 前提、只约束 $e_\xi$）共同的根源只有一个：$\dot V$ 中没有 $-\|e_z\|^2$ 型负项。标准的补救手段是 **strictification**（亦称交叉项注入）：取
$$
W\;\triangleq\;V+\epsilon\,e_z^\top K_pA(\tilde x)\,e_\xi ,\qquad \epsilon>0\ \text{待定} .
$$
则在 $\Omega_c$ 内（$\|A\|_2$ 与 $\|A^{-1}\|_2$ 已由 C.2 一致有界），取 $\epsilon$ 足够小可使 $W$ 与 $V$ 等价（$\tfrac12V\le W\le\tfrac32V$），而求导后新增的主项为
$$
\epsilon\,e_\xi^\top A^\top K_pA e_\xi-\epsilon\,e_z^\top K_pA\bigl(K_de_\xi+A^\top K_pe_z\bigr)+\epsilon\,e_z^\top K_p\dot A e_\xi+\epsilon\,e_z^\top K_pA\,d ,
$$
其中 $-\epsilon\,e_z^\top K_pAA^\top K_pe_z\le-\epsilon\,\sigma_{\min}(A)^2\lambda_{\min}(K_p)^2\,\|e_z\|^2$ 提供了所需的 $e_z$ 方向负定项（其严格正性正是 C.2 的 $\sigma_{\min}(A)\ge c_A>0$ 的用处）。若能验证剩余交叉项（含 $\dot A$ 项，需用 $\dot A$ 关于 $e_\xi$ 的仿射界）可被两个负定项吸收，则得到形式
$$
\dot W\le-c_1\|(e_z,e_\xi)\|^2+c_2\|d\|\cdot\|(e_z,e_\xi)\| \quad\text{在 }\Omega_c\text{ 内},
$$
这才是一个真正的（局部）ISS-Lyapunov 函数：它同时给出全状态的逐点极限球、$\Omega_c$ 的受限不变性（从而解除定理 3(d) 中“$D$ 依赖 $\Omega_c$ 而 $\Omega_c$ 又未必不变”的循环）、以及 $e_z$ 的指数收敛率。代价是：$c_1,c_2,\epsilon$ 的可行域会变得保守（依赖 $c,\eta_0,\mathrm{cond}(K_p)$），且不再保持定理 3(c) 的“当且仅当”品质。本文**没有**完成这一验证（关键难点是 $\dot A$ 项的一致界与 $\epsilon$ 的可行区间非空性），故定理 3(d) 保守地只声明均方界。此项列为 §7 局限 (i)。

### C.5 定理 3(d) 的界形态：为何不是逐点 ISS 极限球

对一阶耗散系统，“$\|e\|$ 较大时 $\dot V<0$”通常直接给出逐点极限球 $\limsup_t\|e(t)\|\le r$（[Kha02] Thm 4.19）。本文的 (5.7d) 不能这样收尾，原因有三条，它们共同决定了定理 3(d) 只能取均方（积分）形式。

**(i) “球”不是水平集。** (5.7d) 给出的是“$\|e_\xi\|>D/\lambda_{\mathrm{eff}}\Rightarrow\dot V<0$”。要由此断言轨迹进入并**停留**于 $\{\|e_\xi\|\le r\}$，需该集合是 $V$ 的水平集或可被水平集夹逼。但 $V=\tfrac12\|e_\xi\|^2+\tfrac12e_z^\top K_pe_z$ 不是 $\|e_\xi\|$ 的函数，$\{\|e_\xi\|\le r\}$ 是 $(e_z,e_\xi)$ 空间中沿 $e_z$ 方向无界的“板状”集，$V$ 在其上无上界；因此“$V$ 在板外单调下降”不蕴含轨迹被板捕获。

**(ii) ISS-Lyapunov 函数的前提不满足。** [Kha02] Thm 4.19 要求存在 $\mathcal K_\infty$ 函数对**全状态**范数作夹逼 $\alpha_1(\|x\|)\le V\le\alpha_2(\|x\|)$，且 $\dot V\le-W(\|x\|)+\rho(\|u\|)$ 的负定项含全状态。此处 $x=(e_z,e_\xi)$，而 $\dot V$ 的负项只含 $-\|e_\xi\|^2$——扰动到 $e_z$ 的相对阶为 2，$e_z$ 方向没有耗散。故 $V$ 不是全状态的 ISS-Lyapunov 函数，只能提供 $e_\xi$ 方向的耗散信息。相应地，定理 3(d) 不自称 “ISS”，而称为“$L_\infty$ 通道的均方极限界”——它是 ISS 极限球的积分类比物。同理，级联 ISS 路线也无法由 $V$ 直接闭合（§5.5、附录 C.2）。

**(iii) $\Omega_c$ 前提与 $D$ 的循环。** 定理 3(b)(i) 的水平集不变性依赖 $d\equiv0$；含扰时 $\dot V$ 可正，$\Omega_c$ 未必不变，而 (5.7b) 的 $D$ 又依赖 $\Omega_c$ 上的 $\|e_z\|$ 界。本稿的处理是把“轨迹留在 $\Omega_c$”列为定理 3(d) 的**显式前提**并事后数值核验（§6.5(6)：实测 $V^{\mathrm{base}}_{\mathrm{peak}}=2.47\times10^{-2}$，换算到 tuned 权重后 $\le0.494$，而 tuned 档 $c^*=160$，余度约 2.5 个数量级）。彻底解除该循环需强化存储函数使 $\dot W$ 含 $-\|e_z\|^2$ 项（附录 C.4），届时可得真正的全状态局部 ISS 与逐点极限球；本稿将其列为开放问题（§7 局限 i）。

**数值与语义。** (5.7) 的界值 $D/\lambda_{\mathrm{eff}}$ 在 $\alpha\to0$ 极限下为 $\|d_{\mathrm{ex}}\|_{L_\infty}/\lambda_{\min}(K_d)$，与逐点形态的常数一致（Young 放缩并未抬高常数），故 §6.5 的核验数值与保守性结论不受影响；受影响的是被核验对象的**语义**——不是“任意时刻误差的上限”，而是“稳态窗口内误差 RMS 的上限”。后者恰好就是 §6.5 实际统计的量（代码侧 `exi_rms`），理论与实测的口径因此一致。

---

## 参考文献

1. **[P1]** A. Cohen, M. Shoham, *Hyper Dual Quaternions representation of rigid bodies kinematics*, Mechanism and Machine Theory 150 (2020) 103861.
2. **[P2]** L.F.C. Figueredo, B.V. Adorno, J.Y. Ishihara, *Robust H∞ kinematic control of manipulator robots using dual quaternion algebra*, Automatica 132 (2021) 109817.
3. J.A. Fike, J.J. Alonso, *The Development of Hyper-Dual Numbers for Exact Second-Derivative Calculations*, AIAA 2011-886.
4. K.M. Lynch, F.C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017.
5. O. Khatib, *A unified approach for motion and force control of robot manipulators: The operational space formulation*, IEEE J. Robotics and Automation 3(1), 1987.
6. **[Spo92]** M.W. Spong, *On the robust control of robot manipulators*, IEEE Trans. Automatic Control 37(11), 1992.
7. **[Kha02]** H.K. Khalil, *Nonlinear Systems*, 3rd ed., Prentice Hall, 2002.
8. **[Ch20]** A. Chandra, J.A. Corrales-Ramon, Y. Mezouar, *Resolved-acceleration control of serial robotic manipulators using unit dual quaternions*, IFAC-PapersOnLine 53(2) (2020) 8500–8505.
9. **[Gaz14]** C. Gaz, F. Flacco, A. De Luca, *Identifying the dynamic model used by the KUKA LWR: A reverse engineering approach*, Proc. IEEE Int. Conf. Robotics and Automation (ICRA), 2014, 1386–1392.
10. **[Roh13]** E. Rohmer, S.P.N. Singh, M. Freese, *V-REP: A versatile and scalable robot simulation framework*, Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS), 2013, 1321–1326.
11. **[LWP80]** J.Y.S. Luh, M.W. Walker, R.P.C. Paul, *Resolved-acceleration control of mechanical manipulators*, IEEE Trans. Automatic Control 25(3), 1980.
12. **[Abd91]** C. Abdallah, D. Dawson, P. Dorato, M. Jamshidi, *Survey of robust control for rigid robots*, IEEE Control Systems Magazine 11(2), 1991.
13. **[Sag99]** H.G. Sage, M.F. De Mathelin, E. Ostertag, *Robust control of robot manipulators: a survey*, International Journal of Control 72(16), 1999.
14. **[ZDG96]** K. Zhou, J.C. Doyle, K. Glover, *Robust and Optimal Control*, Prentice Hall, 1996.
15. **[Nak86]** Y. Nakamura, H. Hanafusa, *Inverse kinematic solutions with singularity robustness for robot manipulator control*, ASME J. Dynamic Systems, Measurement, and Control 108(3), 1986.
16. **[Nak08]** J. Nakanishi, R. Cory, M. Mistry, J. Peters, S. Schaal, *Operational space control: A theoretical and empirical comparison*, International Journal of Robotics Research 27(6), 2008.
17. **[Ber93]** H. Berghuis, H. Nijmeijer, *A passivity approach to controller–observer design for robots*, IEEE Trans. Robotics and Automation 9(6), 1993.
18. 项目文档：主文档 `docs/数学理论与代码实现详解.md`；扩展篇 `docs/HDQ动力学建模扩展_Jdot与Hessian.md`；误差篇 `docs/HDQ动力学误差体系重构_几何一致二阶误差方案.md`；仿真篇 `docs/TNDQ论文_仿真验证章节.md`（含完整逐相位数据表、图位预留与理论–代码一致性核对表）。
