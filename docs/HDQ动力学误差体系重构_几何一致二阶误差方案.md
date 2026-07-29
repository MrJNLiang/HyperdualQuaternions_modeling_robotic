# HDQ 动力学误差体系重构——TNDQ 运动学结构下几何一致的误差定义、传播机制与混合 H∞/ISS 集成

> **文档定位与阅读路线**
>
> 本文是项目理论体系的第四层：
>
> | 层 | 文档 | 内容 |
> |---|---|---|
> | 第一层 | `docs/数学理论与代码实现详解.md`（**主文档**） | DQ/HDQ 一阶运动学、H∞ 控制、实验数据 |
> | 第二层 | `docs/HDQ动力学建模扩展_Jdot与Hessian.md`（**扩展篇**） | 二阶时间链 $\mathcal A_2$、Hessian/$\dot J$ 闭式、电机反馈层级，(D-0)–(D-9) |
> | 第三层 | `docs/HDQ高阶结构动力学创新应用分析.md`（**高阶篇**） | $k$ 阶节代数、$M,C$ 的 HDQ 解析装配、(E-1)–(E-7) |
> | 第四层 | 本文 | **误差体系重构**：分析 [P2] 式(10) DQ 误差体系在动力学环境下的局限，在 **TNDQ**（Trident Number Dual Quaternion，见 §0）运动学结构下设计**几何一致误差体系**——误差对象取实测链/期望链 TNDQ 输出的 **HDQ 截断**（舍弃 $\sigma^2$ 通道，因加速度误差在反馈计算中冗余，见 §2.2），给出误差传播机制、闭环误差动态方程与混合 H∞/ISS 性能框架，新推导 **(F-1)–(F-8)** |
>
> **动机（对应扩展篇 §5.1 的遗留问题）**：扩展篇式 (5.2) 的计算力矩律写作
> $\ddot{\boldsymbol q}_{\mathrm{ref}}=J^{+}(\dot{\boldsymbol\xi}_d+K_d\delta\boldsymbol\xi+K_p\delta\boldsymbol z-\dot J\dot{\boldsymbol q})$，
> 其中 $\delta\boldsymbol z,\delta\boldsymbol\xi$ 暂时"沿用 [P2] 式(10) 的 DQ 误差体系"。但 HDQ 正运动学如今是**多通道对象**（$\boldsymbol x,\dot{\boldsymbol x},\ddot{\boldsymbol x}$ 三通道 + Hessian + $\dot J$），且动力学引入了 DQ 运动学时代不存在的误差源（惯性参数偏差、力矩换算误差、加速度估计误差）。仅有位姿级的 $\tilde{\boldsymbol z}=1-\tilde{\boldsymbol x}$ 不足以覆盖这套结构。本文给出与之匹配的误差体系。
>
> **公式来源标注约定**（与前三层一致，总表见 §8）：
>
> - `[P1] 式(k)` / `[P2] 式(k)`：Cohen & Shoham MMT 2020 / Figueredo et al. Automatica 2021；
> - `主文档 (k)` / `(D-k)` / `(E-k)`：项目已有推导（第一/二/三层）；
> - `[LP17]` / `[Kha87]` / `[Spo92]` / `[Kha02]`：教科书/经典文献标准结果（见 §9）；
> - **(F-k)**：**本文新推导**（附证明或证明思路）。
>
> **记号**：沿用主文档。位姿 DQ $\boldsymbol x\in\mathrm{Spin}(3)\ltimes\mathbb R^3$，左乘约定 $\dot{\boldsymbol x}=\tfrac12\boldsymbol\xi\boldsymbol x$（[P2] 式(1)），空间 twist $\boldsymbol\xi=2\dot{\boldsymbol x}\boldsymbol x^*=\omega+\varepsilon v$，$v=\dot p+p\times\omega$。伴随作用记 $\mathrm{Ad}_{\boldsymbol x}(\boldsymbol a)\triangleq\boldsymbol x\boldsymbol a\boldsymbol x^*$（单位 DQ 对纯 DQ 的保范作用，[P2] §2 标准运算）。纯 DQ 上 $\tfrac12[\boldsymbol a,\boldsymbol b]=\tfrac12(\boldsymbol{ab}-\boldsymbol{ba})=\mathrm{ad}_{\boldsymbol a}\boldsymbol b$（扩展篇 (D-8) 已用）。$T^2$ 为二阶提升算子（扩展篇 §3.2），$T^1$ 为一阶（HDQ）提升算子。

---

## 0. TNDQ 命名约定与 HDQ 截断

扩展篇 §3.1 引入的二阶节代数 $\mathcal A_2=\widehat{\mathbb H}[\sigma]/(\sigma^3)$ 自本文起正式命名为 **TNDQ（Trident Number Dual Quaternion，三叉对偶四元数）**：其元素

$$
\breve a = a_0+\sigma a_1+\tfrac12\sigma^2 a_2,\qquad a_0,a_1,a_2\in\widehat{\mathbb H},
$$

由**三个 DQ 通道**（三叉：位姿 / 一阶导 / 二阶导）构成，程序中以三个 8 维向量（数组）并列存储；乘法为扩展篇 (3.1)。TNDQ 与项目已有代数塔的关系：

| 结构 | 通道数 | 通道内容 | 截断关系 |
|---|---|---|---|
| DQ $\widehat{\mathbb H}$ | 1 | $a_0$ | TNDQ 取 $\sigma^0$ 通道 |
| HDQ（[P1] 式(25)） | 2 | $a_0+\varepsilon^*a_1$ | TNDQ 取 $\sigma^0,\sigma^1$ 通道（$\sigma\leftrightarrow\varepsilon^*$） |
| **TNDQ**（$\mathcal A_2$） | 3 | $a_0+\sigma a_1+\tfrac12\sigma^2a_2$ | 全结构 |

**HDQ 截断算子**：$\Pi_{\mathrm{HDQ}}:\mathcal A_2\to\widehat{\mathbb H}[\varepsilon^*]/(\varepsilon^{*2})$，

$$
\Pi_{\mathrm{HDQ}}\bigl(a_0+\sigma a_1+\tfrac12\sigma^2a_2\bigr)\triangleq a_0+\varepsilon^*a_1 .
$$

因 $\sigma^2$ 通道在 TNDQ 乘法 (3.1) 中**不反馈**到 $\sigma^0,\sigma^1$ 通道（截断多项式环的滤过性：低阶通道的乘积结果只依赖低阶通道），$\Pi_{\mathrm{HDQ}}$ 是**代数同态**：$\Pi_{\mathrm{HDQ}}(\breve a\,\breve b)=\Pi_{\mathrm{HDQ}}(\breve a)\,\Pi_{\mathrm{HDQ}}(\breve b)$。程序实现即"只取前两个数组、按 HDQ 乘法运算"，零额外代价。

**本文的结构性决策**（动机见 §2.2）：正运动学与期望轨迹以 TNDQ 建模（动力学前馈需要 $\sigma^2$ 通道的 $\dot{\boldsymbol\xi}$、$\dot J\dot{\boldsymbol q}$、$\dot{\boldsymbol\xi}_d$）；**误差体系只在 HDQ 截断上定义**——反馈仅消费位姿误差与 twist 误差两阶，加速度误差在反馈中冗余，舍弃 $\sigma^2$ 部分既不损失控制性能，又使误差运算严格落在 [P1] 的 HDQ 代数内。

---

## 目录

1. [现有 DQ 误差体系回顾](#1-现有-dq-误差体系回顾)
2. [局限性分析：DQ 误差体系在动力学环境中的六个不足](#2-局限性分析dq-误差体系在动力学环境中的六个不足)
3. [新误差体系的设计原则与总体结构](#3-新误差体系的设计原则与总体结构)
4. [数学推导：误差定义与误差运动学](#4-数学推导误差定义与误差运动学)
5. [动力学误差源与闭环误差动态方程](#5-动力学误差源与闭环误差动态方程)
6. [误差传播机制与现实量级预算](#6-误差传播机制与现实量级预算)
7. [与现有控制框架和代码的集成](#7-与现有控制框架和代码的集成)
8. [公式来源总表](#8-公式来源总表)
9. [参考文献](#9-参考文献)

---

## 1. 现有 DQ 误差体系回顾

[P2] §3.1（主文档 §7.2 已完整复现）的误差体系由四个对象构成：

| 对象 | 定义 | 编号 |
|---|---|---|
| 右不变空间误差 | $\tilde{\boldsymbol x}=\boldsymbol x\boldsymbol x_d^{*}=\tilde r+\varepsilon\tfrac12\tilde p\tilde r$ | (P2-8) |
| 误差运动学 | $\dot{\tilde{\boldsymbol x}}=\tfrac12(\overline{\mathrm{vec}}_6(J\dot{\boldsymbol q})+\boldsymbol v_w+\boldsymbol v_c)\tilde{\boldsymbol x}-\tfrac12\tilde{\boldsymbol x}\boldsymbol\xi_d$ | (P2-9) |
| 误差函数 | $\tilde{\boldsymbol z}=1-\tilde{\boldsymbol x}$ | (P2-10) |
| 被控输出 | $\mathcal O(\tilde{\boldsymbol z})=-\mathrm{Im}(\tilde r)$，$\mathcal T(\tilde{\boldsymbol z})=\tilde p$ | (P2-11) |

代码实现为 `core/errors.py::pose_error`（主文档 §7.2 逐行对应）。扰动模型为 twist 级加性扰动 $\boldsymbol v_w,\boldsymbol v_c\in L_2$（P2-4/6），性能指标为 $L_2$ 能量增益 $\gamma$（[P2] Definition 1）。

这套体系是**位姿级、运动学级**的：状态是 $\tilde{\boldsymbol x}$（0 阶），输出是 6 维位姿误差，扰动全部作用在速度通道，控制量是 $\dot{\boldsymbol q}$。在其设计工况（速度接口、低速、扰动为外生有界信号）内它是完备且被项目实验验证的（主文档 §9）。

---

## 2. 局限性分析：DQ 误差体系在动力学环境中的六个不足

### 2.1 (L1) 速度误差无几何一致定义——朴素差 $\boldsymbol\xi-\boldsymbol\xi_d$ 不是右不变量

[P2] 体系只显式定义了位姿误差；扩展篇 (5.2) 中的 $\delta\boldsymbol\xi$ 若按最直接的 $\boldsymbol\xi-\boldsymbol\xi_d$ 取，则两个 twist 分别是**当前位形**与**期望位形**处的空间速度，属于不同点的切空间，直接相减在几何上不一致。定量地，与几何一致定义（§4.2 的 $\tilde{\boldsymbol\xi}$）之差为

$$
(\boldsymbol\xi-\boldsymbol\xi_d)-\tilde{\boldsymbol\xi}
=\bigl(\mathrm{Ad}_{\tilde{\boldsymbol x}}-\mathrm{id}\bigr)\boldsymbol\xi_d,
\qquad
\bigl\|(\mathrm{Ad}_{\tilde{\boldsymbol x}}-\mathrm{id})\boldsymbol\xi_d\bigr\|
\lesssim\bigl(2\|\mathcal O\|+\|\mathcal T\|\,\|\omega_d\|\bigr)\cdot\|\boldsymbol\xi_d\|,
$$

即混入一个与**位姿误差 × 期望速度**成正比的伪项。运动学环里这项被 (P2-12) 的前馈 $\mathrm{vec}_6(\tilde{\boldsymbol x}\boldsymbol\xi_d\tilde{\boldsymbol x}^*)$ 悄悄吸收了（见 §4.2 注记）；但在动力学内环里 $\delta\boldsymbol\xi$ 直接乘 $K_d$ 进入力矩，伪项随 $\|\boldsymbol\xi_d\|$ 线性放大——高速工况（恰是动力学控制的目标工况，高阶篇 §3.4）下不可忽略。

### 2.2 (L2) 加速度前馈缺失 vs. 加速度误差冗余——误差对象的正确阶数

TNDQ 链已把 $\ddot{\boldsymbol x},\dot{\boldsymbol\xi}$ 变成一次传播的通道读数（(D-1)(D-4)），计算力矩律 (5.2) 也显式消费期望加速度 $\dot{\boldsymbol\xi}_d$。这里必须区分两件事：

- **期望加速度前馈 $\dot{\boldsymbol\xi}_d$ 是必需的**：它来自**期望轨迹自身**（轨迹发生器解析给出，或期望链 TNDQ 的 $\sigma^2$ 通道），是前馈量而非误差量；
- **加速度误差 $e_a=\mathrm{vec}_6(\dot{\tilde{\boldsymbol\xi}})$ 在反馈中冗余**：计算力矩律（§5.3 的 (5.2$'$)）的反馈项为 $-K_d e_\xi-k_p A^\top e_z$，只用到**位姿误差与 twist 误差两阶**；若再引入 $e_a$ 并乘一个增益 $K_a$，则 $\dot e_\xi=-K_d e_\xi-\cdots$ 变为含 $\ddot e_\xi$ 的三阶系统，既无必要（二阶级联 $e_z\to e_\xi$ 已能指数收敛，见 (F-7)），又把差分/力矩换算的高噪声加速度估计直接引入反馈（噪声放大，扩展篇 §1.2）。

**结论**：误差对象的正确阶数是 **HDQ（位姿 + 速度两通道）**。[P2] 体系的真正缺口不是"缺加速度误差"，而是**缺几何一致的速度误差通道**（(L1)）与**无法把加速度层扰动表达为误差动态中的量**：加速度层的估计误差与模型偏差应作为**扰动 $d(t)$ 进入 $e_\xi$ 的动态方程**（(F-7a) 的 $\dot e_\xi=\cdots+d$），而非单独的反馈状态。这正是 §0 舍弃 $\sigma^2$ 通道的依据。

### 2.3 (L3) 扰动模型不容纳偏差型（非 $L_2$）误差

[P2] 假设 $\boldsymbol v_w,\boldsymbol v_c\in L_2$（能量有限、渐近消失）。动力学的主要误差源却是**持续偏差型**：惯性参数误差 $\Delta M,\Delta C,\Delta g$、摩擦模型残差、力矩常数偏差——它们在无限时域上能量无穷，$L_2$ 增益指标对其**空洞成立**（右端 $=\infty$）。必须引入 $L_\infty$/ISS 型通道与之匹配（§5.3）。

### 2.4 (L4) 扰动的外生性假设被破坏

[P2] 的 $\boldsymbol v_w,\boldsymbol v_c$ 是外生信号（不依赖控制量）。动力学中 $\Delta M\ddot{\boldsymbol q}_{\mathrm{ref}}$ 项**与控制量成正比**——扰动经反馈耦合，属乘性不确定性。这要求误差体系显式给出适定性条件（经典计算力矩鲁棒性条件 $\|M^{-1}\Delta M\|<1$，[Spo92]），而非把它当外生噪声（§5.2）。

### 2.5 (L5) 无内部一致性（约束残差）通道——HDQ 多通道结构特有

DQ 时代唯一的代数约束是 $\|\tilde r\|=1$（主文档的归一化投影处理）。$\mathcal A_2$ 链输出必须满足**提升后的整族约束**：对 $\boldsymbol x\boldsymbol x^*=1$ 逐阶求导，

$$
\boldsymbol x\boldsymbol x^*=1,\qquad
\dot{\boldsymbol x}\boldsymbol x^*+\boldsymbol x\dot{\boldsymbol x}^*=0,\qquad
\ddot{\boldsymbol x}\boldsymbol x^*+2\dot{\boldsymbol x}\dot{\boldsymbol x}^*+\boldsymbol x\ddot{\boldsymbol x}^*=0 .
$$

数值积分/漂移会让高阶通道**先于** 0 阶通道violate这些恒等式（高阶篇 §7.4 已指出该空白），而现有误差体系对此完全不可见。误差体系应把约束残差作为可监测通道（§4.5）。

### 2.6 (L6) 量纲混合与跨通道相位错位无表达

- $\mathcal O$（无量纲/弧度级）与 $\mathcal T$（米）直接拼 6 维范数 `pose_error_norm`，隐含"1 rad ≍ 1 m"的任意折算；动力学提供了**自然度量**——操作空间惯性 $\Lambda$（动能内积），应据此加权（§4.6）。
- 扩展篇 §1.4 的"一致性要求"指出：$q,\dot q,\ddot q$ 若来自不同延迟的滤波器，$\varepsilon^*$ 与 $\sigma^2$ 通道会相位错位并被链放大为系统性 $\dot{\boldsymbol\xi}$ 偏差。这是**跨通道误差**，位姿级体系无处安放（§6.2 给出其量级模型）。

**小结**：六个不足中，(L1) 是几何缺口（速度误差无一致定义），(L2) 是误差对象阶数与扰动入口的定位问题，(L3)(L4) 是扰动模型缺口，(L5)(L6) 是 TNDQ 多通道与物理量纲缺口。下面的设计逐一对应。

---

## 3. 新误差体系的设计原则与总体结构

### 3.1 四条设计原则

| 原则 | 内容 | 针对 |
|---|---|---|
| **P-i 结构同构（HDQ 阶）** | 误差对象应与反馈所需的运动学输出同构：反馈只消费位姿 + 速度两阶（§2.2），故误差对象取实测链/期望链 TNDQ 输出的 **HDQ 截断** $\Pi_{\mathrm{HDQ}}$（§0），由**一次 HDQ 乘法**生成，复用 [P1] 式(14) 与 (D-3) 的全部机器；$\sigma^2$ 通道不进入误差对象，只供前馈（$\dot{\boldsymbol\xi}_d$）与 $\dot J\dot{\boldsymbol q}$ | (L1)(L2) |
| **P-ii 几何一致** | 所有速度级比较必须先经 $\mathrm{Ad}_{\tilde{\boldsymbol x}}$ 搬运到同一切空间；误差量对 unwinding 符号翻转 $\tilde{\boldsymbol x}\to-\tilde{\boldsymbol x}$ 不变（位姿输出除外） | (L1) |
| **P-iii 误差源可分离** | 每个物理误差源（噪声型/偏差型/乘性/相位/漂移）有专属通道与专属性能指标，不混装进同一个 $\boldsymbol v_w$；加速度层误差源一律作为扰动进入 $e_\xi$ 动态（§2.2） | (L2)(L3)(L4)(L5)(L6) |
| **P-iv 向下兼容** | 0 阶通道退化为 [P2] 式(10)(11) 原样；运动学外环 (P2-12) 及其 H∞ 保证不被破坏 | 集成 |

### 3.2 总体结构：两层误差 + 前馈量 + 两类扰动 + 一组监测量

```
                          ┌──────── 误差状态（几何层，HDQ 元素）────────┐
  实测链 TNDQ ─Π_HDQ──┐   │  X̃ = T¹x · (T¹x_d)*  =  x̃ + ε* dx̃/dt  (F-1)│
  期望链 TNDQ ─Π_HDQ──┴─▶ │  0阶: e_z = [O;T]   位姿误差 ([P2]式(10)(11))│
                          │  1阶: e_ξ = vec₆ ξ̃   twist误差 (F-2)        │
                          └─────────────────────────────────────────┘
  前馈量（非误差；期望链 σ² 通道/解析轨迹）: ξ_d, dξ_d/dt → (5.2') 几何一致前馈项
  扰动输入（动力学层，进入 e_ξ 动态）:  w_L2   噪声型: 测量噪声、v_w,v_c 及其导数 → H∞ 通道
                                    w_bias 偏差型: ΔM,ΔC,Δg,δτ_f,δK_t   → ISS 通道 (F-5)
  内部监测量（代数层）:   c₀,c₁  HDQ 提升约束残差                      (F-6)
  度量:                 ‖e_ξ‖_Λ（误差动能）、‖e_z‖_K（误差势能）       (§4.6)
```

**与旧版（三层误差）方案的差异**：旧版把加速度误差 $e_a$ 列为第三层误差状态；本版依 §2.2 将其删除——$e_a$ 在反馈中冗余，加速度层信息拆为两路：期望加速度走**前馈**（$\dot{\boldsymbol\xi}_d$，确定量，来自期望轨迹而非误差对象），加速度层不确定性走**扰动**（$d(t)$ 进入 $\dot e_\xi$ 方程，§5.3）。误差状态从三层（$e_z,e_\xi,e_a$）降为两层（$e_z,e_\xi$，共 12 维），闭环分析（§5.3）反而更干净；程序层面，TNDQ 的三个 DQ 都以向量/数组形式存储，HDQ 截断即"只取前两个数组"，实现零成本。

---

## 4. 数学推导：误差定义与误差运动学

### 4.1 误差的 HDQ 提升：一次 HDQ 乘法生成位姿与速度两阶误差

实测链与期望链先各自作 HDQ 截断（§0，程序中即各取前两个 DQ 数组）：实测侧 $\Pi_{\mathrm{HDQ}}(T^2\boldsymbol x)=T^1\boldsymbol x\triangleq\boldsymbol x+\varepsilon^*\dot{\boldsymbol x}$；期望轨迹由轨迹发生器解析给出 $\boldsymbol x_d(t)$ 及其导数（`trajectory_line/circle` 的直线/圆轨迹均有闭式 $\dot{\boldsymbol x}_d,\ddot{\boldsymbol x}_d$，其中 $\ddot{\boldsymbol x}_d$ 只供 §5.3 的前馈、不进入误差对象），截断为 $T^1\boldsymbol x_d=\boldsymbol x_d+\varepsilon^*\dot{\boldsymbol x}_d$。DQ 共轭是线性映射且与求导交换，故逐通道取共轭即得 $(T^1\boldsymbol x_d)^*=T^1(\boldsymbol x_d^*)$。

> **定理 (F-1)（误差的 HDQ 提升）**：定义 HDQ 误差元素
>
> $$
> \breve{\mathfrak X}\triangleq T^1\boldsymbol x\cdot\bigl(T^1\boldsymbol x_d\bigr)^{*}\in\widehat{\mathbb H}[\varepsilon^*]/(\varepsilon^{*2}) .
> $$
>
> 则由 HDQ 乘法的 Leibniz 结构（[P1] 式(14)），
>
> $$
> \breve{\mathfrak X}=T^1\bigl(\boldsymbol x\boldsymbol x_d^*\bigr)
> =\tilde{\boldsymbol x}+\varepsilon^*\,\dot{\tilde{\boldsymbol x}},
> \qquad
> \dot{\tilde{\boldsymbol x}}=\dot{\boldsymbol x}\boldsymbol x_d^*+\boldsymbol x\dot{\boldsymbol x}_d^{\,*},
> $$
>
> 即**一次 HDQ 乘法（3 次 DQ 乘）同时给出误差位姿与误差速度两个通道**，且 0 阶通道正是 [P2] 式(8) 的 $\tilde{\boldsymbol x}$。
>
> **证明**：HDQ 乘法 $(\hat a_0+\varepsilon^*\hat a_1)(\hat b_0+\varepsilon^*\hat b_1)=\hat a_0\hat b_0+\varepsilon^*(\hat a_0\hat b_1+\hat a_1\hat b_0)$（[P1] 式(14)）在结构上就是乘积求导的 Leibniz 法则。取 $\hat a_0=\boldsymbol x,\ \hat a_1=\dot{\boldsymbol x},\ \hat b_0=\boldsymbol x_d^*,\ \hat b_1=\dot{\boldsymbol x}_d^{\,*}$（共轭与求导的交换性保证 $(T^1\boldsymbol x_d)^*=T^1(\boldsymbol x_d^*)$）：0 阶通道给出 $\boldsymbol x\boldsymbol x_d^*=\tilde{\boldsymbol x}$，$\varepsilon^*$ 通道给出 $\boldsymbol x\dot{\boldsymbol x}_d^{\,*}+\dot{\boldsymbol x}\boldsymbol x_d^*=\tfrac{d}{dt}(\boldsymbol x\boldsymbol x_d^*)=\dot{\tilde{\boldsymbol x}}$。∎
>
> **注记（与 TNDQ 链的同态一致性）**：若先在 TNDQ 上作误差乘法、再截断，由 $\Pi_{\mathrm{HDQ}}$ 的同态性（§0）与共轭的逐通道性：
>
> $$
> \Pi_{\mathrm{HDQ}}\bigl(T^2\boldsymbol x\,(T^2\boldsymbol x_d)^*\bigr)
> =\Pi_{\mathrm{HDQ}}(T^2\boldsymbol x)\cdot\Pi_{\mathrm{HDQ}}\bigl(T^2\boldsymbol x_d\bigr)^{*}
> =T^1\boldsymbol x\cdot(T^1\boldsymbol x_d)^*=\breve{\mathfrak X},
> $$
>
> 即**先乘后截 = 先截后乘**：舍弃 $\sigma^2$ 通道不会在位姿/速度两阶产生任何截断误差。这是 §0 结构性决策（误差体系定义在 HDQ 截断上）数学合法性的精确表述。

这落实了设计原则 P-i：误差不再是"位姿误差 + 若干临时定义的差"，而是与 FK 输出（HDQ 截断后）同类型的代数对象；(D-3) 的提取机器**对误差曲线 $\tilde{\boldsymbol x}(t)$ 原样适用**（它同样是单位 DQ 曲线）。

### 4.2 一阶误差通道：几何一致 twist 误差

> **定义/命题 (F-2)（误差 twist）**：
>
> $$
> \tilde{\boldsymbol\xi}\triangleq 2\,\dot{\tilde{\boldsymbol x}}\,\tilde{\boldsymbol x}^{*},
> \qquad
> e_\xi\triangleq\mathrm{vec}_6\,\tilde{\boldsymbol\xi}\in\mathbb R^6 .
> $$
>
> 沿受扰误差运动学 (P2-9) 展开得其物理分解：
>
> $$
> \tilde{\boldsymbol\xi}
> =\boldsymbol\xi-\underbrace{\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol\xi_d}_{\text{搬运后的期望 twist}}
> +\boldsymbol v_w+\boldsymbol v_c ,
> $$
>
> 并且：
> 1. $\tilde{\boldsymbol\xi}$ 是纯 DQ（twist 型），且对 unwinding 翻转 $\tilde{\boldsymbol x}\to-\tilde{\boldsymbol x}$ **不变**；
> 2. $\tilde{\boldsymbol x}$ 的运动学写作 $\dot{\tilde{\boldsymbol x}}=\tfrac12\tilde{\boldsymbol\xi}\tilde{\boldsymbol x}$——误差本身满足与 (P2-1) 同型的左乘运动学；
> 3. 与朴素差的偏离为 $(\mathrm{Ad}_{\tilde{\boldsymbol x}}-\mathrm{id})\boldsymbol\xi_d$（§2.1 的 (L1) 伪项），$\tilde{\boldsymbol x}\to1$ 时消失。
>
> **证明**：由 (P2-9)，$2\dot{\tilde{\boldsymbol x}}\tilde{\boldsymbol x}^*=(\overline{\mathrm{vec}}_6(J\dot{\boldsymbol q})+\boldsymbol v_w+\boldsymbol v_c)-\tilde{\boldsymbol x}\boldsymbol\xi_d\tilde{\boldsymbol x}^*$，第一括号即 $\boldsymbol\xi+\boldsymbol v_w+\boldsymbol v_c$。纯性：$\tilde{\boldsymbol x}$ 单位，(D-3) 的论证给出 $\dot{\tilde{\boldsymbol x}}\tilde{\boldsymbol x}^*$ 反自共轭。翻转不变性：$2(-\dot{\tilde{\boldsymbol x}})(-\tilde{\boldsymbol x}^*)=2\dot{\tilde{\boldsymbol x}}\tilde{\boldsymbol x}^*$。∎

> **注记（与 [P2] 前馈的关系，设计原则 P-iv 的证据）**：(P2-12) 的前馈项 $\mathrm{vec}_6(\tilde{\boldsymbol x}\boldsymbol\xi_d\tilde{\boldsymbol x}^*)$ 正是 $\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol\xi_d$；[P2] 闭环化简 (P2-13) 之所以成立，正因为控制律隐式地把 $\tilde{\boldsymbol\xi}$ 而非 $\boldsymbol\xi-\boldsymbol\xi_d$ 驱为反馈量。**(F-2) 只是把 [P2] 结构里隐含的正确速度误差显式化并升格为体系的一阶通道**——因此与运动学外环零冲突。

### 4.3 伴随输运引理——为什么不需要加速度误差通道

旧版方案曾在此处定义第三层误差状态 $e_a=\mathrm{vec}_6\dot{\tilde{\boldsymbol\xi}}$；依 §2.2 的分析，本版将其删除——反馈只消费 $e_z,e_\xi$ 两阶。但闭环推导（§5.3 的 (F-7)）仍需要知道 $\tilde{\boldsymbol\xi}$ 的时间导数**作为中间量**怎么展开（它出现在 $\dot e_\xi$ 方程的左端，但不作为反馈状态、不需要在线测量）。所需的全部工具是下面的输运公式：

> **引理 (F-3)（伴随输运公式与误差 twist 的导数）**：设 $\tilde{\boldsymbol x}(t)$ 为单位 DQ 曲线，$\tilde{\boldsymbol\xi}=2\dot{\tilde{\boldsymbol x}}\tilde{\boldsymbol x}^*$。则对任意光滑纯 DQ 曲线 $\boldsymbol a(t)$：
>
> **(i) 输运公式**：
>
> $$
> \frac{d}{dt}\,\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol a
> =\mathrm{Ad}_{\tilde{\boldsymbol x}}\dot{\boldsymbol a}
> +\mathrm{ad}_{\tilde{\boldsymbol\xi}}\bigl(\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol a\bigr);
> $$
>
> **(ii) 误差 twist 的导数**（对 (F-2) 的分解求导，无扰情形）：
>
> $$
> \dot{\tilde{\boldsymbol\xi}}
> =\dot{\boldsymbol\xi}
> -\mathrm{Ad}_{\tilde{\boldsymbol x}}\dot{\boldsymbol\xi}_d
> -\mathrm{ad}_{\tilde{\boldsymbol\xi}}\bigl(\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol\xi_d\bigr),
> $$
>
> 三项分别是：实际任务空间加速度（(D-4) 的测量侧）、搬运后的期望加速度、**搬运本身随误差运动产生的"输运/科氏修正"**。含扰动时右端追加 $\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$。
>
> **证明**：(i) 由 $\dot{\tilde{\boldsymbol x}}=\tfrac12\tilde{\boldsymbol\xi}\tilde{\boldsymbol x}$、$\dot{\tilde{\boldsymbol x}}^*=-\tfrac12\tilde{\boldsymbol x}^*\tilde{\boldsymbol\xi}$（对 $\tilde{\boldsymbol x}\tilde{\boldsymbol x}^*=1$ 求导）直接展开：
> $\tfrac{d}{dt}(\tilde{\boldsymbol x}\boldsymbol a\tilde{\boldsymbol x}^*)=\tfrac12\tilde{\boldsymbol\xi}\tilde{\boldsymbol x}\boldsymbol a\tilde{\boldsymbol x}^*+\tilde{\boldsymbol x}\dot{\boldsymbol a}\tilde{\boldsymbol x}^*-\tfrac12\tilde{\boldsymbol x}\boldsymbol a\tilde{\boldsymbol x}^*\tilde{\boldsymbol\xi}=\mathrm{Ad}_{\tilde{\boldsymbol x}}\dot{\boldsymbol a}+\tfrac12[\tilde{\boldsymbol\xi},\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol a]$，纯 DQ 上 $\tfrac12[\cdot,\cdot]=\mathrm{ad}$。(ii) 对 $\tilde{\boldsymbol\xi}=\boldsymbol\xi-\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol\xi_d$（(F-2) 无扰形式）逐项求导，对第二项用 (i) 取 $\boldsymbol a=\boldsymbol\xi_d$。∎

**物理意义与用途**：(ii) 中的 $\mathrm{ad}$ 项与 (D-8) 的 Lie 括号同源——它是"在误差流形上比较加速度"必然出现的联络项，量级 $\sim\|e_\xi\|\,\|\boldsymbol\xi_d\|$。(F-3) 在本文中的唯一消费者是控制律：(5.2$'$) 的几何一致前馈项 $\mathrm{vec}_6(\mathrm{Ad}_{\tilde{\boldsymbol x}}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol\xi_d)$ 正是为了在闭环中精确消去 (ii) 右端的后两项而设（见 (F-7) 证明）。注意其中 $\dot{\boldsymbol\xi}_d$ 来自期望轨迹（解析或期望链 TNDQ 的 $\sigma^2$ 通道），$\tilde{\boldsymbol x},\tilde{\boldsymbol\xi}$ 来自 (F-1)(F-2)——**全部是已有量，不需要测量或估计任何加速度误差**；这从推导层面坐实了 §2.2 的结论。

### 4.4 输出误差的精确运动学：连接 $e_z$ 与 $e_\xi$ 的闭式映射

被控输出沿用 (P2-11)：$e_z\triangleq[\mathcal O;\mathcal T]\in\mathbb R^6$，$\mathcal O=-\mathrm{Im}\,\tilde r=-\tilde\mu$（记 $\tilde r=\tilde\eta+\tilde\mu$），$\mathcal T=\tilde p$。

> **命题 (F-4)（输出误差运动学闭式）**：记 $\tilde{\boldsymbol\xi}=\tilde\omega+\varepsilon\tilde v$，则
>
> $$
> \dot e_z=A(\tilde{\boldsymbol x})\,e_\xi,
> \qquad
> A(\tilde{\boldsymbol x})=
> \begin{bmatrix}
> -\tfrac12\bigl(\tilde\eta I_3+[\mathcal O]_\times\bigr) & 0_3\\[2pt]
> -[\mathcal T]_\times & I_3
> \end{bmatrix},
> $$
>
> 且 $\tilde{\boldsymbol x}\to1$ 时 $A\to A_0=\mathrm{diag}(-\tfrac12I_3,\;I_3)$，$\sigma_{\min}(A_0)=\tfrac12$。
>
> **证明**：实部通道：$\dot{\tilde r}=\tfrac12\tilde\omega\tilde r$（$\dot{\tilde{\boldsymbol x}}=\tfrac12\tilde{\boldsymbol\xi}\tilde{\boldsymbol x}$ 的四元数部），故
> $\dot{\mathcal O}=-\mathrm{Im}(\tfrac12\tilde\omega\tilde r)=-\tfrac12(\tilde\eta\tilde\omega+\tilde\omega\times\tilde\mu)=-\tfrac12(\tilde\eta I+[\mathcal O]_\times)\tilde\omega$
> （用 $\tilde\omega\times\tilde\mu=-[\tilde\mu]_\times\tilde\omega=[\mathcal O]_\times\tilde\omega$）。平移通道：由空间 twist 约定 $\tilde v=\dot{\tilde p}+\tilde p\times\tilde\omega$（主文档 §3 的 $v=\dot p+p\times\omega$ 对误差 DQ 原样成立），$\dot{\mathcal T}=\tilde v-[\mathcal T]_\times\tilde\omega$。∎
>
> **一致性校验**（(F-4) 复现 [P2] 稳定性）：把运动学律 (P2-12) 的理想闭环 $\tilde\omega=\kappa_{\mathcal O}\mathcal O,\ \tilde v=-\kappa_{\mathcal T}\mathcal T$ 代入：
> $\dot{\mathcal O}=-\tfrac12\kappa_{\mathcal O}\tilde\eta\,\mathcal O$（$[\mathcal O]_\times\mathcal O=0$），$\tilde\eta>0$ 时指数稳定——恰是 [P2] Remark 1 的 unwinding 条件；
> $\tfrac{d}{dt}\|\mathcal T\|^2=2\mathcal T^\top(-[\mathcal T]_\times\tilde\omega-\kappa_{\mathcal T}\mathcal T)=-2\kappa_{\mathcal T}\|\mathcal T\|^2$（叉积项正交）✓。**(F-4)**

(F-4) 使误差体系闭合为严格的两层级联链 $e_z\xrightarrow{A}e_\xi$：$\dot e_z=Ae_\xi$，而 $\dot e_\xi$ 的动态由控制律与扰动决定（§5.3 的 (F-7a)）。这是写内环误差动态方程（§5）的运动学骨架，也是 [P2] 原文未显式需要（其 Lyapunov 论证只用符号结构）、动力学扩展中必须补上的一块。

### 4.5 代数层监测通道：提升约束残差

> **定义 (F-6)（提升约束残差）**：对 HDQ 截断后的链输出 $(\boldsymbol x,\dot{\boldsymbol x})$ 定义
>
> $$
> c_0\triangleq\bigl\|\boldsymbol x\boldsymbol x^*-1\bigr\|,\qquad
> c_1\triangleq\bigl\|\mathrm{Sc}\bigl(2\dot{\boldsymbol x}\boldsymbol x^*\bigr)\bigr\|,
> $$
>
> 其中 $\mathrm{Sc}(\cdot)$ 取 DQ 的标量部与对偶标量部（2 维）。解析上两者恒为零：$c_0$ 是单位性；$c_1=0$ ⇔ $\boldsymbol\xi$ 纯（(D-3)）。数值上它们以 $O(1)$ 代价逐周期计算，是误差体系两个通道数值健康度的充分监测量：任一 $c_k$ 超阈值即说明对应阶通道的积分漂移/归一化缺失已污染输出，应触发重投影（0 阶归一化 + 1 阶按 $\dot{\boldsymbol x}\mapsto\tfrac12\boldsymbol\xi_{\text{proj}}\boldsymbol x$ 重构）。对误差元素 $\breve{\mathfrak X}$ 同样定义 $\tilde c_0,\tilde c_1$。若前馈侧使用 TNDQ 链的 $\sigma^2$ 通道（供 $\dot{\boldsymbol\xi}_d,\dot J\dot{\boldsymbol q}$），可另按同法监测 $c_2\triangleq\|\mathrm{Sc}(2\ddot{\boldsymbol x}\boldsymbol x^*)+\tfrac12\mathrm{Sc}(\boldsymbol\xi^2)\|$——但它属于**前馈健康度**，不属于误差体系。**(F-6)**

这填补 (L5)：DQ 体系只有 $c_0$ 一个约束可查，HDQ 两通道需要两个；(F-6) 给出的正是"提升约束"的逐阶残差，且全部复用已算出的量（$\boldsymbol\xi$ 为 (D-3) 副产品）。

### 4.6 度量层：动能/势能加权范数取代"1 rad ≍ 1 m"

针对 (L6) 的量纲问题，采用动力学自然度量：

$$
\|e_\xi\|_{\Lambda}^2\triangleq e_\xi^{\top}\Lambda(\boldsymbol q)\,e_\xi
\quad[\mathrm J]\ (\text{误差运动的动能}),
\qquad
\|e_z\|_{K}^2\triangleq e_z^{\top}\!\begin{bmatrix}K_{\mathcal O}&0\\0&K_{\mathcal T}\end{bmatrix}\!e_z
\quad[\mathrm J]\ (\text{虚拟弹性势能}),
$$

其中 $\Lambda=(JM^{-1}J^\top)^{-1}$ 是操作空间惯性（(5.3)，其几何构件全部由 HDQ 供给，高阶篇 §2.5），$K_{\mathcal O}\,[\mathrm{N\,m/rad}],K_{\mathcal T}\,[\mathrm{N/m}]$ 是设计的任务刚度。两个范数同量纲（焦耳），相加有物理意义（误差总机械能）——这正是阻抗控制的能量函数，也是 §5.4 Lyapunov/存储函数的物理原型。低速或无惯性参数时退化取 $\Lambda=I$ 加人工权重，即回到现状的 `pose_error_norm` 加权版。

---

## 5. 动力学误差源与闭环误差动态方程

### 5.1 误差源清单与通道归属（针对 P-iii）

| # | 误差源 | 数学形式 | 类型 | 归属通道 |
|---|---|---|---|---|
| S1 | 编码器量化/噪声 $\delta q$ | $e_z$ 经 (D-0)、$e_\xi$ 经 Hessian 耦合（§6.1） | 噪声型 $L_2$/$L_\infty$ | $w_{L_2}$ |
| S2 | 速度估计噪声/滞后 $\delta\dot q$ | $J\delta\dot q$ 进 $e_\xi$ | 噪声型 | $w_{L_2}$ |
| S3 | 加速度估计误差 $\delta\ddot q$ | 路线相关：差分=方差型，力矩换算=偏差型（扩展篇 §1.2） | 双属性 | $w_{L_2}$ 或 $w_b$ |
| S4 | 惯性参数误差 $\Delta M,\Delta C,\Delta g$ | 经 (5.1) 进入 $\ddot q$：见 (F-5) | **偏差型、乘性** | $w_b$ |
| S5 | 力矩误差 $\delta\tau$（摩擦残差、$K_t$ 偏差、齿隙） | 加性进 (5.1) 右端 | 偏差型为主 | $w_b$ |
| S6 | 外部 twist 扰动 $\boldsymbol v_w,\boldsymbol v_c$ 及导数 | (P2-4/6) 原通道 + $\dot{\boldsymbol v}$ 进 $d(t)$（(F-7a)） | 噪声型 | $w_{L_2}$ |
| S7 | 跨通道相位错位 $\tau_d$ | $e_{\text{phase}}\approx\tau_d\,\|\dot{(\cdot)}\|$（§6.2） | 系统偏差 | $w_b$ |
| S8 | 数值漂移 | 约束残差 $c_0,c_1$（前馈侧可加 $c_2$） | 监测量 | (F-6) |

### 5.2 动力学扰动通道的定义与适定性

内环用标称模型 $\hat M,\hat C,\hat g$ 执行计算力矩 (5.2)：$\boldsymbol\tau=\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+\hat C\dot{\boldsymbol q}+\hat g$。真实动力学 (5.1) 含摩擦/力矩误差 $\delta\boldsymbol\tau$ 与外力矩 $\boldsymbol\tau_{\mathrm{ext}}$。记 $\Delta M=\hat M-M$ 等。

> **命题 (F-5)（动力学扰动通道与适定性条件）**：实际关节加速度满足
>
> $$
> \ddot{\boldsymbol q}=\ddot{\boldsymbol q}_{\mathrm{ref}}+\boldsymbol w_{\mathrm{dyn}},
> \qquad
> \boldsymbol w_{\mathrm{dyn}}
> = M^{-1}\bigl(\Delta M\,\ddot{\boldsymbol q}_{\mathrm{ref}}+\Delta C\,\dot{\boldsymbol q}+\Delta\boldsymbol g+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}}\bigr).
> $$
>
> $\boldsymbol w_{\mathrm{dyn}}$ 含与控制量成正比的乘性项 $M^{-1}\Delta M\ddot{\boldsymbol q}_{\mathrm{ref}}$；当
>
> $$
> \alpha\triangleq\sup_{\boldsymbol q}\bigl\|M^{-1}(\boldsymbol q)\Delta M(\boldsymbol q)\bigr\|_2<1
> $$
>
> 时（[Spo92] 经典条件；$\hat M$ 取得越准 $\alpha$ 越小，且 $\hat M,M$ 均正定时可保证），反馈耦合可解出，$\boldsymbol w_{\mathrm{dyn}}$ 可整理为"有界外生等效扰动 + 增益 $\le\alpha/(1-\alpha)$ 的误差反馈项"，闭环适定。分解
>
> $$
> \boldsymbol w_{\mathrm{dyn}}=\underbrace{\boldsymbol w_b}_{\text{偏差型：}\Delta M,\Delta C,\Delta g,\delta\tau_f\ (\in L_\infty)}+\underbrace{\boldsymbol w_{L_2}}_{\text{噪声型：电流噪声、}\delta\ddot q\ \text{差分噪声等}} .
> $$
>
> **证明思路**：把 $\ddot{\boldsymbol q}_{\mathrm{ref}}$（含反馈项）代回 $\boldsymbol w_{\mathrm{dyn}}$ 得隐式方程，$\alpha<1$ 时 Neumann 级数收敛给出显式解；分解按各源的时间特性归类（S4/S5 持续存在故 $L_\infty$，S1–S3 噪声部分能量有限或可白化故归 $L_2$ 类）。∎ **(F-5)**

这同时解决 (L3)（偏差型有了 $w_b$ 通道）与 (L4)（乘性耦合有了显式适定条件，而非伪装成外生噪声）。

### 5.3 闭环误差动态方程

在新误差坐标下重写内环控制律（替换扩展篇 (5.2) 中的临时误差量；$K_p$ 取标量 $k_p$、$K_d$ 对称正定——若需 (F-7c) 的旋转/平移逐通道 H∞ 界，则取块对角 $K_d=\mathrm{diag}(K_\omega,K_v)$，$K_\omega,K_v\in\mathbb R^{3\times3}$ 对称正定）：

$$
\ddot{\boldsymbol q}_{\mathrm{ref}}
=J^{+}\Bigl(\underbrace{\mathrm{vec}_6\bigl(\mathrm{Ad}_{\tilde{\boldsymbol x}}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol\xi_d\bigr)}_{\text{几何一致前馈（(F-3)(ii) 的搬运项）}}
-K_d\,e_\xi-k_p\,A^{\top}(\tilde{\boldsymbol x})\,e_z-\dot J\dot{\boldsymbol q}\Bigr).
\tag{5.2$'$}
$$

与 (5.2) 的差异：期望加速度经 $\mathrm{Ad}$ 搬运并补 $\mathrm{ad}$ 输运项（使其在闭环中与 (F-3)(ii) 的后两项精确相消）；位姿反馈经 $A^\top$ 整形（使 Lyapunov 交叉项精确相消，见下）；$\dot J\dot{\boldsymbol q}$ 仍由 (D-5) 的 $O(n)$ 链给出。反馈只用 $e_z,e_\xi$ 两阶（§2.2），全部取自 (F-1) 的 HDQ 误差元素。

> **定理 (F-7)（闭环误差动态与混合 H∞/ISS 性能）**：设 $J$ 行满秩（$JJ^+=I$），则在控制律 (5.2$'$) 与扰动模型 (F-5) 下，误差坐标 $(e_z,e_\xi)$ 满足**闭环误差动态方程**（级联标准形）：
>
> $$
> \boxed{\;
> \begin{aligned}
> \dot e_z&=A(\tilde{\boldsymbol x})\,e_\xi,\\
> \dot e_\xi&=-K_d\,e_\xi-k_p\,A^{\top}(\tilde{\boldsymbol x})\,e_z
> +\underbrace{J\boldsymbol w_{\mathrm{dyn}}+\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c}_{\triangleq\,d(t)\ \text{（总扰动进入加速度层）}} .
> \end{aligned}\;}
> \tag{F-7a}
> $$
>
> 取存储函数（§4.6 的误差机械能，此处用常权版本以获闭式）
>
> $$
> V=\tfrac12\|e_\xi\|^2+\tfrac{k_p}2\,\|e_z\|^2\;\ge0,
> $$
>
> 则沿 (F-7a)：交叉项精确相消（$k_p e_z^\top A e_\xi-k_p e_\xi^\top A^\top e_z=0$），**精确地**（此式不含任何放缩；$e_\xi^\top d$ 可正可负）
>
> $$
> \dot V=-e_\xi^{\top}K_de_\xi+e_\xi^{\top}d ,
> $$
>
> 从而对 $d=d_{L_2}+d_b$ 的两类分量分别得到：
>
> 1. **H∞ 通道**（$d_{L_2}\in L_2$，$d_b=0$；二次型/Schur 补判据 + 旋转/平移通道拆分）：
>    - **(1a) 合并判据**（$K_d$ 任意对称正定）：性能目标 $\dot V\le-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$ 对一切 $(e_\xi,d)$ 成立**当且仅当**
>    $$
>    M\triangleq\begin{bmatrix}K_d-\tfrac1{2\kappa}I_6 & -\tfrac12 I_6\\[2pt] -\tfrac12 I_6 & \tfrac{\gamma_a^2}2 I_6\end{bmatrix}\succeq0
>    \;\overset{\text{Schur}}{\Longleftrightarrow}\;
>    K_d\succeq\tfrac12\bigl(\kappa^{-1}+\gamma_a^{-2}\bigr)I_6 ,
>    \tag{F-7b}
>    $$
>    此时 $\int_0^\infty\kappa^{-1}\|e_\xi\|^2dt\le\gamma_a^2\int_0^\infty\|d_{L_2}\|^2dt+2V(0)$——
>    **加速度层扰动到 twist 误差能量的 $L_2$ 增益 $\le\gamma_a\sqrt\kappa$**，形式与 [P2] Definition 1 平行，但扰动入口从速度层升到加速度层；
>    - **(1b) 通道拆分判据**（$K_d=\mathrm{diag}(K_\omega,K_v)$ 块对角）：记 $e_\xi=[\tilde\omega;\tilde v]$、$d=[d_\omega;d_v]$，$V_\omega\triangleq\tfrac12\|\tilde\omega\|^2+\tfrac{k_p}2\|\mathcal O\|^2$、$V_v\triangleq\tfrac12\|\tilde v\|^2+\tfrac{k_p}2\|\mathcal T\|^2$（$V=V_\omega+V_v$）。若
>    $$
>    K_\omega\succeq\tfrac12\bigl(\kappa_\omega^{-1}+\gamma_\omega^{-2}\bigr)I_3,
>    \qquad
>    K_v\succeq\tfrac12\bigl(\kappa_v^{-1}+\gamma_v^{-2}\bigr)I_3,
>    \tag{F-7c}
>    $$
>    则旋转/平移两通道**各自独立**满足
>    $\int_0^\infty\kappa_\omega^{-1}\|\tilde\omega\|^2dt\le\gamma_\omega^2\int_0^\infty\|d_\omega\|^2dt+2V_\omega(0)$ 与
>    $\int_0^\infty\kappa_v^{-1}\|\tilde v\|^2dt\le\gamma_v^2\int_0^\infty\|d_v\|^2dt+2V_v(0)$——
>    逐通道量纲齐次（不再混合 $(\mathrm{rad/s})^2$ 与 $(\mathrm{m/s})^2$）、$\gamma_\omega,\gamma_v$ 可独立指定，恢复 [P2] Definition 1 把 $\mathcal O/\mathcal T$ 分开的双通道结构（在本体系中对应把 $e_\xi$ 按 $\tilde\omega/\tilde v$ 拆分）；
> 2. **ISS 通道**（$d_b\in L_\infty$）：$e=(e_z,e_\xi)$ 对 $d_b$ 输入-状态稳定，极限球半径
> $\limsup_{t\to\infty}\|e_\xi\|\le\|d_b\|_\infty/\lambda_{\min}(K_d)$（再经 $\dot e_z=Ae_\xi$ 与 (F-4) 的 $\sigma_{\min}(A_0)=\tfrac12$ 传至 $e_z$ 的极限球）。**偏差型误差源（S4/S5/S7）不会破坏稳定性，只决定稳态误差球大小**——这正是它们与 $L_2$ 噪声在性能表上的本质区别，也是 (L3) 的解决方式。
>
> **证明**：分四步。
>
> *第一步（得出 (F-7a)）*：由 (D-5) 与 (F-5)（$\ddot{\boldsymbol q}=\ddot{\boldsymbol q}_{\mathrm{ref}}+\boldsymbol w_{\mathrm{dyn}}$），
> $\mathrm{vec}_6\dot{\boldsymbol\xi}=J\ddot{\boldsymbol q}+\dot J\dot{\boldsymbol q}=J\ddot{\boldsymbol q}_{\mathrm{ref}}+\dot J\dot{\boldsymbol q}+J\boldsymbol w_{\mathrm{dyn}}$；代入 (5.2$'$) 并用 $JJ^+=I$，$\dot J\dot{\boldsymbol q}$ 相消：
> $\mathrm{vec}_6\dot{\boldsymbol\xi}=\mathrm{vec}_6\bigl(\mathrm{Ad}_{\tilde{\boldsymbol x}}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol\xi_d\bigr)-K_de_\xi-k_pA^\top e_z+J\boldsymbol w_{\mathrm{dyn}}$。
> 另一方面，由引理 (F-3)(ii)（含扰版）：$\dot{\tilde{\boldsymbol\xi}}=\dot{\boldsymbol\xi}-\mathrm{Ad}_{\tilde{\boldsymbol x}}\dot{\boldsymbol\xi}_d-\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol\xi_d+\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$。两式相减，前馈项与 (F-3)(ii) 的期望/输运项**精确相消**（机制与 [P2] (P2-13) 的前馈相消完全平行）：
> $\dot e_\xi=\mathrm{vec}_6\dot{\tilde{\boldsymbol\xi}}=-K_de_\xi-k_pA^\top e_z+d(t)$。配合 (F-4) 的 $\dot e_z=Ae_\xi$ 即得 (F-7a)。
>
> *第二步（$\dot V$ 精确式）*：沿 (F-7a)，$\dot V=e_\xi^\top\dot e_\xi+k_pe_z^\top\dot e_z=-e_\xi^\top K_de_\xi-k_pe_\xi^\top A^\top e_z+e_\xi^\top d+k_pe_z^\top Ae_\xi$，交叉项精确相消（$A^\top$ 整形的设计目的），得 $\dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d$。此式不做任何放缩——不定号交叉项 $e_\xi^\top d$ 留待下一步在二次型内整体判定（若走 Young 路径则须先取绝对值：$e_\xi^\top d\le|e_\xi^\top d|\le\|e_\xi\|\|d\|\le\tfrac1{2\rho}\|e_\xi\|^2+\tfrac\rho2\|d\|^2$，绝对值一步不可省；二次型路径从构造上免除符号处理）。
>
> *第三步（H∞ 通道）*：**(1a)** 性能目标 $\dot V\le-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$ 等价于二次型不等式 $[e_\xi;d]^\top M[e_\xi;d]\ge0\ \forall(e_\xi,d)\in\mathbb R^{12}$，即 $M\succeq0$；右下块 $\tfrac{\gamma_a^2}2I\succ0$，取 Schur 补即得 (F-7b)（当且仅当）。Young 路径给出同一条件的充分形式，二者在各向同性罚权下重合（TNDQ 论文初稿附录 C.3）。*全局存在性*：由 $M\succeq0$ 得 $\dot V\le\tfrac{\gamma_a^2}2\|d\|^2$，故 $V(t)\le V(0)+\tfrac{\gamma_a^2}2\|d_{L_2}\|_{L_2}^2<\infty$，$(e_z,e_\xi)$ 一致有界，解在 $[0,\infty)$ 上存在（无有限时间逃逸）。*积分收尾*：在 $[0,T]$ 上积分 $\dot V\le-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$，弃去 $V(T)\ge0$，令 $T\to\infty$（单调收敛）。**(1b)** 块对角 $K_d$ 下两通道储能函数精确解耦，关键是两处三重积恒零。由 (F-4)，$A^\top=\begin{bmatrix}A_{11}^\top & [\mathcal T]_\times\\ 0 & I\end{bmatrix}$（$A_{11}=-\tfrac12(\tilde\eta I+[\mathcal O]_\times)$，$(-[\mathcal T]_\times)^\top=[\mathcal T]_\times$），故 $(A^\top e_z)_\omega=A_{11}^\top\mathcal O+[\mathcal T]_\times\mathcal T=A_{11}^\top\mathcal O$（$\mathcal T\times\mathcal T=0$，**旋转反馈通道天然不含平移误差**）、$(A^\top e_z)_v=\mathcal T$；又 $\dot{\mathcal T}=-[\mathcal T]_\times\tilde\omega+\tilde v$ 中耦合项做功为零：$\mathcal T^\top[\mathcal T]_\times\tilde\omega=\mathcal T\cdot(\mathcal T\times\tilde\omega)=0$（与 [P2] 平移通道"叉积项与 $\mathcal T$ 正交"同一机制）。于是 $\dot V_\omega=-\tilde\omega^\top K_\omega\tilde\omega+\tilde\omega^\top d_\omega$、$\dot V_v=-\tilde v^\top K_v\tilde v+\tilde v^\top d_v$（各自的 $k_p$ 交叉项分别相消），对每条通道重复 (1a) 的二次型/Schur 论证（$I_6\to I_3$）即得 (F-7c) 的两条独立积分不等式。两处恒零是代数恒等式，不依赖工作域（$\tilde\eta$ 无关）。
>
> *第四步（ISS 通道）*：标准二次型论证（[Kha02] Thm 4.19 形态）；$e_z$ 有界性用级联结构与 $A$ 的有界性（(F-4) 的 $A$ 各块由单位四元数分量与 $\mathcal T$ 构成，在 $\|\mathcal T\|$ 有界的工作域内有界）。∎ **(F-7)**

> **注记（Schur 判据 vs. Young 放缩、最紧可证增益）**：(i) 在同一各向同性供给率下，(F-7b) 与标量条件 $\lambda_{\min}(K_d)\ge\tfrac12(\gamma_a^{-2}+\kappa^{-1})$ 完全等价——矩阵判据的收益不是"同一目标下更小的下界"，而是 (a) 从构造上免除符号放缩、(b) 供给率分块加权后自然产出 (F-7c) 的通道拆分（各向异性设计空间来自供给率加权的分块化，而非同一各向同性目标的改写）。(ii) 供给率整体缩放 $\theta>0$（认证同一增益 $\gamma_a\sqrt\kappa$）给出条件族 $K_d\succeq(\theta\kappa^{-1}+\tfrac1{4\theta}\gamma_a^{-2})I$，$\theta=\tfrac12$ 即 (F-7b)；对 $\theta$ 极小化（$\theta^*=\sqrt\kappa/2\gamma_a$）得**最紧可证条件** $\lambda_{\min}(K_d)\ge1/(\gamma_a\sqrt\kappa)$，即认证 $L_2$ 增益 $\le1/\lambda_{\min}(K_d)$——与第四步的 ISS 界及 $K_d$ 对称时的线性极限 $\|(sI+K_d)^{-1}\|_{H_\infty}=1/\lambda_{\min}(K_d)$ 一致（完整推导见 TNDQ 论文初稿附录 C.3）。

> **注记（诚实边界）**：(F-7) 在 $J$ 行满秩与 $\alpha<1$（(F-5)）下成立；奇异邻域内 $J^+$ 用阻尼伪逆时，$JJ^+\ne I$ 的残差按 [P2] 脚注 6/14 的同一豁免机制归入 $d(t)$。含 $\Lambda(\boldsymbol q)$ 加权的变权存储函数需要额外处理 $\dot\Lambda$ 项（利用 $\dot\Lambda-2\Lambda\mu$-型斜对称性），本文不展开——高阶篇 §7.5 已把"级联整体 $L_2$ 界"列为开放问题，(F-7) 给出的是内环单层的严格结果 + 级联 ISS 结论，未声称整体 H∞ 界。此外：(i) (F-7b)/(F-7c) 只约束 $e_\xi$ 而非 $e_z$——扰动到 $e_z$ 的相对阶为 2，当前 $V$ 的导数中无 $-\|e_z\|^2$ 项；若需 $e_z$ 的直接 $L_2$ 界须在 $V$ 中加交叉项（strictification，形如 $\epsilon\,e_z^\top Ae_\xi$）并处理 $\dot A$，留作后续，$e_z$ 现经级联 $\dot e_z=Ae_\xi$ 获 ISS 型界；(ii) 含扰时轨迹可能离开 $\tilde\eta>0$（unwinding 域边界），H∞ 通道的 twist 误差界仍成立（其证明不依赖 $\tilde\eta$），但该域外不附带任何位姿收敛结论；(iii) 通道拆分 (F-7c) 依赖 $K_d$ 块对角与 $[\mathcal T]_\times\mathcal T=0$、$\mathcal T^\top[\mathcal T]_\times\tilde\omega=0$ 两处恒零，一般正定 $K_d$ 退回合并判据 (F-7b)。

### 5.4 与 [P2] 运动学 H∞ 外环的级联相容性

外环照旧运行 (P2-12)（保持其 H∞ 保证，P-iv），输出 $\dot{\boldsymbol q}_{\mathrm{cmd}}$ 与对应的 $\boldsymbol\xi_{\mathrm{ref}}=\overline{\mathrm{vec}}_6(J\dot{\boldsymbol q}_{\mathrm{cmd}})$；内环 (5.2$'$) 以 $(\boldsymbol x_d,\boldsymbol\xi_d,\dot{\boldsymbol\xi}_d)$ 或外环整形后的参考为输入。级联论证：内环 ISS（(F-7) 第 2 条）+ 外环对"执行残差"的 H∞ 鲁棒性（[P2] 把未实现的 $\dot q$ 残差归入 $\boldsymbol v_w$）⟹ 级联系统对两类扰动分别保持 $L_2$ 界与极限球界（标准级联 ISS 定理 [Kha02]）。内环存在的效果是把外环感受到的 $\|\boldsymbol v_w\|$ **变小**（高阶篇 §3.4 第 2 点"扰动整形"的严格版）。

---

## 6. 误差传播机制与现实量级预算

### 6.1 从关节层到任务层的完整传播矩阵

综合 (D-0)、§1.4（扩展篇）与 (F-2)，各层测量误差到两个误差通道的一阶传播为：

$$
\begin{bmatrix}\delta e_z\\ \delta e_\xi\end{bmatrix}
\approx
\begin{bmatrix}
A\,J & 0\\[2pt]
\Bigl(\sum_j h_{\cdot j}\dot q_j\Bigr) & J
\end{bmatrix}
\begin{bmatrix}\delta\boldsymbol q\\ \delta\dot{\boldsymbol q}\end{bmatrix},
\qquad
\delta d\approx J\,\boldsymbol w_{\mathrm{dyn}}+\bigl(\partial_q[\dot J\dot q+J\ddot q]\bigr)\delta\boldsymbol q+\bigl(2\dot J+\text{ad 项}\bigr)\delta\dot{\boldsymbol q}+J\,\delta\ddot{\boldsymbol q}.
\tag{F-8}
$$

**结构解读**：
- 误差状态块（左式，分块下三角）：对角块全为 $J$（乘 $A$）——每层测量误差以 $\|J\|$ 增益进入同层误差，(D-0) 的推广；次对角块是 **Hessian 列组合 $\sum_j h_{ij}\dot q_j$**：位置测量误差向速度误差通道的"泄漏"增益随运动速度线性增长。这把扩展篇 §1.4 的观察（"一阶误差预算需要二阶几何量"）升格为完整传播矩阵：**误差预算矩阵的全部非平凡块都由 HDQ Hessian 层 (D-6)(D-7)(D-9) 解析供给**，无需差分估计；
- 扰动块（右式）：加速度层的全部误差（动力学扰动 $\boldsymbol w_{\mathrm{dyn}}$、位置/速度误差向加速度层的泄漏、$\delta\ddot q$）不再对应一个误差状态，而是整体汇入 (F-7a) 的扰动 $d(t)$——与 §2.2 的定位（加速度层不确定性走扰动入口）完全一致；其各块同样由 Hessian 层解析供给，供先验估计 $\|d\|_\infty$ 与 (F-7) 的 ISS 球半径。

### 6.2 现实量级预算（7R 臂典型参数，工程依据）

取项目实验同级的典型值：编码器 $\sigma_q\!\sim\!5\times10^{-5}$ rad（19 位绝对编码器 + 减速器背隙折算）、$\Delta t=2$ ms（500 Hz 内环）、$\|J\|\!\sim\!1$ m/rad、$\|\dot q\|\le0.8$（低速）或 $\sim3$ rad/s（高速）、工业臂惯性参数出厂标定误差典型 $5$–$10\%$（负载未知时 $M$ 末端行误差可达 $30\%$）：

| 误差源 | 传播路径 | 低速量级 | 高速量级 | 类型 |
|---|---|---|---|---|
| S1 $\delta q$ → $e_z$ | $\|J\|\sigma_q$ | $\sim5\times10^{-5}$ m | 同 | 噪声 |
| S1 → $e_\xi$（经 Hessian 块） | $\|h\|\|\dot q\|\sigma_q$ | $\sim4\times10^{-5}$ | $\sim1.5\times10^{-4}$ m/s | 噪声 |
| S2 $\delta\dot q$（差分）→ $e_\xi$ | $\|J\|\sqrt{2}\sigma_q/\Delta t$ | $\sim3.5\times10^{-2}$ m/s | 同 | 噪声（观测器可降 ~10×） |
| S3 $\delta\ddot q$ 差分路线 → $d(t)$ | $\|J\|\sqrt{6}\sigma_q/\Delta t^2$ | $\sim30$ m/s²（**不可用**，印证扩展篇 §1.2） | 同 | 噪声 |
| S3 力矩换算路线 → $d(t)$ | $\|J\|\,\|M^{-1}\|(\|\Delta M\|\|\ddot q\|+\ldots)$ | $\sim10^{-2}$ m/s² | $\sim0.5$ m/s² | **偏差** |
| S4 $\Delta M$（10%）经 $w_{\mathrm{dyn}}$ | $\alpha\|\ddot q_{\mathrm{ref}}\|$ | $\alpha\sim0.1$，可忽略 | 主导项 | 偏差（ISS 球） |
| S7 相位错位 $\tau_d\sim1$ 周期 | $d^{\text{phase}}\sim\tau_d\|\ddot\xi\|$ | 小 | $\sim\tau_d\|\dot q\|\|\ddot q\|$ 级 | 系统偏差 |

**三条工程结论**（数量级层面，与前三层文档判断一致并加细）：

1. **加速度层信息只宜以偏差型路线进入扰动 $d(t)$ 预算**：差分路线 30 m/s² 的噪声底说明任何消费加速度估计的反馈都毫无意义——这从量级层面再次印证 §2.2 删除加速度误差通道的决策；力矩换算路线的噪声可用，但其偏差属 $w_b$，由 (F-7) ISS 球吸收——这复证了扩展篇"路线选择决定误差性质"的论断，并给出定量归属；
2. **低速工况新体系收益趋零**：表中低速列各项均小于运动学环现有误差水平（项目实测跟踪误差 ~mm 级），与高阶篇 §3.4 的诚实评估一致——新误差体系的价值兑现区在中高速/大惯量/接触工况；
3. **稳态误差球可先验预算**：由 (F-7) ISS 界，$\|e_\xi\|_\infty\lesssim\|J\|\,\alpha\,\|\ddot q\|_{\max}/\lambda_{\min}(K_d)$——设计 $K_d$ 时可直接由目标精度与已知 $\alpha$（参数标定报告）反解，误差体系闭环到设计流程。

### 6.3 跨通道相位一致性的处理（S7）

对 (L6) 后半：要求 $q,\dot q$（及前馈侧的 $\ddot q$ 或 $\tau$）经**同组延迟对齐**后再进 TNDQ 因子构造（同一 Kalman 观测器输出三个状态即自动满足）；残余错位 $\tau_d$ 按 S7 行归入 $w_b$。监测手段：比较 $e_z$ 的差分变化率与 $A\,e_\xi$（(F-4) 应恒成立）——两者系统性偏离即相位错位签名，量级 $\approx\tau_d\|\dot e_\xi\|$。

---

## 7. 与现有控制框架和代码的集成

### 7.1 分层集成路线（不破坏现状）

| 阶段 | 内容 | 依赖 | 对现有实验的改动 |
|---|---|---|---|
| 0（现状） | (P2-12) 速度环 + `pose_error` | 无 | — |
| 1 | 误差侧升级：$e_\xi$ 按 (F-2) 计算并记录（对比 $\boldsymbol\xi-\boldsymbol\xi_d$ 的伪项曲线）；(F-6) 残差监测上线 | 仅 DQ 运算 | 零（只加日志） |
| 2 | HDQ 误差元素 $\breve{\mathfrak X}$：一次 HDQ 乘法（(F-1)，已有 `hdq_math` 乘法即可）；前馈侧如需 $\dot{\boldsymbol\xi}_d,\dot J\dot{\boldsymbol q}$ 再上 TNDQ 链（扩展篇 §6 衔接表） | HDQ 乘法（现成）/ TNDQ 乘法（前馈） | 零（离线分析 + (F-8) 误差预算） |
| 3 | 内环 (5.2$'$) + (F-7)：CoppeliaSim 切力矩模式或实机 | 惯性参数 $\hat M,\hat C,\hat g$ | 新实验脚本 |

### 7.2 代码衔接点（未实现，供参考；风格对齐扩展篇 §6）

| 推导 | 衔接现有代码 | 改动性质 |
|---|---|---|
| (F-1) 误差提升 | `errors.py::pose_error`（`x_tilde = dq_mul(x, dq_conj(xd))`） | 升级为 HDQ 版 `pose_error_hdq`：各取实测/期望链前两个 DQ 数组，一次 HDQ 乘法（3 次 DQ 乘，[P1] 式(14)）；0 阶输出保持 `O, T, x_tilde` 签名不变（P-iv） |
| (F-2) $e_\xi$ | `spatial_twist_from_hdq`（`hdq_math.py` L126–138） | 对误差元素调用同一函数：`vec6(2*x̃̇*x̃*)` |
| (F-3) 输运引理 | 无需单独实现（不是反馈量） | 仅用于 (5.2$'$) 前馈项：`Ad(x_tilde, xi_d_dot) + ad(xi_tilde, Ad(x_tilde, xi_d))`，全部是 DQ 乘法 |
| (F-4) $A(\tilde{\boldsymbol x})$ | 新增小函数（3×3 块拼装，输入 `O, T, eta`） | 供 (5.2$'$) 与仿真验证 (F-4) 校验式 |
| (F-5)(F-7) 内环 | `controllers.py`（现 H∞ 律处）新增 `computed_torque_hdq` | 消费 (D-5) 的 $\dot J\dot q$、(F-1)(F-2) 误差、(F-3) 前馈项、$\hat M,\hat C,\hat g$ |
| (F-6) 残差 | 新增 `constraint_residuals(x, xdot)` | 每周期 $O(1)$，超阈值触发重投影 |
| (F-8) 预算 | 分析脚本（复用 (D-6)(D-7) Hessian） | 离线误差预算报告 |

**验证策略**（沿用主文档 §6.5 互证方法学）：
(i) (F-1) 的 $\varepsilon^*$ 通道值 vs. 对 $\tilde{\boldsymbol x}$ 数值差分——残差应 $O(\epsilon^2)$ vs. 机器精度；
(ii) (F-3)(ii) 两侧（对 $\tilde{\boldsymbol\xi}$ 差分 vs. 右端三项解析式）在随机 $(\boldsymbol q,\dot{\boldsymbol q},\ddot{\boldsymbol q})$ 下互证至离散化阶；
(iii) (F-4) 的 $\dot e_z=Ae_\xi$ 用轨迹积分数值校验；
(iv) (F-7a) 在无扰仿真下检查 $\dot V\le-\lambda_{\min}(K_d)\|e_\xi\|^2$ 逐步成立；
(v) 同态一致性（(F-1) 注记）：TNDQ 乘法后截断 vs. 截断后 HDQ 乘法，两者应逐位相等（机器精度）；
(vi) $L_2$ 扰动注入下的能量比：合并比 $\int\|e_\xi\|^2/\int\|d\|^2$ 对照 (F-7b) 的 $\gamma_a^2\kappa$，逐通道比 $\int\|\tilde\omega\|^2/\int\|d_\omega\|^2$、$\int\|\tilde v\|^2/\int\|d_v\|^2$ 对照 (F-7c) 的 $\gamma_\omega^2\kappa_\omega,\gamma_v^2\kappa_v$（并对照最紧能量界 $\lambda_{\min}(K_d)^{-2}$，见 §5.3 注记）。

---

## 8. 公式来源总表

| 编号 | 内容 | 来源 |
|---|---|---|
| (P2-8)(P2-9)(P2-10)(P2-11)(P2-12)(P2-13) | 右不变误差、误差运动学、H∞ 律 | [P2]（主文档 §7 已复现） |
| (D-1)(D-3)(D-4)(D-5)(D-6)(D-7)(D-8)(D-9) | $\mathcal A_2$ 同态、共轭导数、$\dot\xi$ 提取、$\dot J\dot q$、Hessian | 扩展篇（本项目） |
| (E-1)–(E-6)、(5.3) 构件 | $k$ 阶同态、$M,C$ 装配、$\Lambda$ | 高阶篇（本项目）/[Kha87][LP17] |
| (5.1)(5.2) | 关节动力学、计算力矩 | [LP17] 标准结果 |
| $\alpha<1$ 适定条件 | 计算力矩乘性不确定性经典条件 | [Spo92]（综述级标准结果） |
| ISS/级联定理形态 | 二次型 ISS 论证、级联稳定性 | [Kha02] 教科书标准结果 |
| **(F-1)** | 误差的 HDQ 提升 $\breve{\mathfrak X}=T^1\boldsymbol x(T^1\boldsymbol x_d)^*$，一次 HDQ 乘法得位姿/速度两阶误差；同态一致性（先乘后截=先截后乘） | **新推导**（[P1] 式(14) 应用于误差曲线 + §0 截断同态） |
| **(F-2)** | 几何一致误差 twist $\tilde{\boldsymbol\xi}=2\dot{\tilde{\boldsymbol x}}\tilde{\boldsymbol x}^*$ 及物理分解、unwinding 不变性 | **新推导**（显式化 [P2] 前馈的隐含结构） |
| **(F-3)** | 伴随输运引理 $\tfrac{d}{dt}\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol a=\mathrm{Ad}_{\tilde{\boldsymbol x}}\dot{\boldsymbol a}+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol a$ 与误差 twist 导数展开（仅供 (F-7) 推导，不是误差状态；旧版加速度误差通道已删，见 §2.2/§4.3） | **新推导**（李群标准输运公式在 DQ 左乘约定下的显式版） |
| **(F-4)** | 输出误差运动学闭式 $\dot e_z=A(\tilde{\boldsymbol x})e_\xi$ 及 [P2] 稳定性复现校验 | **新推导** |
| **(F-5)** | 动力学扰动通道 $\boldsymbol w_{\mathrm{dyn}}$ 分解与 $\alpha<1$ 适定性 | **新整理**（构件为 [Spo92] 条件） |
| **(F-6)** | 提升约束残差 $c_0,c_1$（前馈侧可选 $c_2$）与重投影触发 | **新推导**（填补高阶篇 §7.4 空白） |
| **(F-7)** | 闭环误差动态 (F-7a)、交叉项相消、混合 H∞($L_2$)/ISS($L_\infty$) 双通道性能；H∞ 判据的二次型/Schur 补形式 (F-7b)（当且仅当，无符号放缩）与旋转/平移逐通道拆分 (F-7c)（由 $[\mathcal T]_\times\mathcal T=0$、$\mathcal T^\top[\mathcal T]_\times\tilde\omega=0$ 两处恒零精确成立）（完整四步证明见 §5.3） | **新推导**（Lyapunov/ISS 论证为标准技术，误差坐标、相消结构与逐通道拆分为本文；Schur 补/配方法为标准工具） |
| **(F-8)** | 两阶误差传播矩阵（非平凡块 = HDQ Hessian 层输出）+ 加速度层汇入扰动 $d(t)$ 的预算式 | **新推导**（(D-0)+扩展篇 §1.4 的矩阵化完备版） |

> **诚实性声明**：(F-2) 的"用 $\mathrm{Ad}$ 搬运后作差"在李群控制文献（含 DQ 文献如 Adorno 系）中有等价思想，本文贡献是其在左乘约定 + HDQ 截断结构下的显式定理化与和 [P2] 前馈项的等同性证明；(F-3) 的输运公式是李群标准结果在 DQ 记号下的重写；(F-7) 的 Lyapunov/ISS 技术是标准的，新的是误差坐标选择（$A^\top$ 整形反馈使交叉项精确相消）与两类扰动的通道化归属；(F-7) **不**声称级联系统的整体 H∞ 界（该问题仍开放，见高阶篇 §7.5）。TNDQ 为本项目对 $\mathcal A_2=\widehat{\mathbb H}[\sigma]/(\sigma^3)$ 的命名（§0），其代数结构（截断多项式环/二阶 jet）在数学上是标准对象，新的是其与 [P1] HDQ 的截断关系定位及在误差体系中的工程化使用方式。§6.2 的量级表为工程估算（参数取典型值），非实测。用于学术发表前建议对 (F-2)(F-3) 与 Adorno 学派 DQ 动力学控制文献做逐条查重。

---

## 9. 参考文献

1. **[P1]** A. Cohen, M. Shoham, *Hyper Dual Quaternions representation of rigid bodies kinematics*, Mechanism and Machine Theory 150 (2020) 103861.
2. **[P2]** L.F.C. Figueredo, B.V. Adorno, J.Y. Ishihara, *Robust H∞ kinematic control of manipulator robots using dual quaternion algebra*, Automatica 132 (2021) 109817.
3. **[Kha87]** O. Khatib, *A unified approach for motion and force control of robot manipulators: The operational space formulation*, IEEE J. Robotics and Automation 3(1), 1987.
4. **[LP17]** K.M. Lynch, F.C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017.
5. **[Spo92]** M.W. Spong, *On the robust control of robot manipulators*, IEEE Trans. Automatic Control 37(11), 1992.（计算力矩鲁棒性与 $\|M^{-1}\Delta M\|<1$ 型条件的经典出处）
6. **[Kha02]** H.K. Khalil, *Nonlinear Systems*, 3rd ed., Prentice Hall, 2002.（ISS 与级联稳定性标准定理）
7. 主文档：`docs/数学理论与代码实现详解.md`；扩展篇：`docs/HDQ动力学建模扩展_Jdot与Hessian.md`；高阶篇：`docs/HDQ高阶结构动力学创新应用分析.md`（本项目）。
