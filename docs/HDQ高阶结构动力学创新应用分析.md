# HDQ 与更高阶四元数结构在机械臂动力学-运动学一体化中的创新应用分析

> **文档定位与阅读路线**
>
> 本文是项目理论体系的第三层：
>
> | 层 | 文档 | 内容 |
> |---|---|---|
> | 第一层 | `docs/数学理论与代码实现详解.md`（**主文档**） | DQ/HDQ 一阶运动学、H∞ 控制、实验数据 |
> | 第二层 | `docs/HDQ动力学建模扩展_Jdot与Hessian.md`（**扩展篇**） | 二阶时间链、Hessian/$\dot J$ 闭式、电机反馈层级，新推导 (D-0)–(D-9) |
> | 第三层 | 本文 | **更高阶结构**（$k$ 阶节代数）、动力学系数矩阵 $M,C$ 与 HDQ 输出的解析关联、物理模型、工程流程与创新对比，新推导 (E-1)–(E-7) |
>
> 扩展篇的 (D-k) 结果在本文中直接引用、不再重复证明。
>
> **公式来源标注约定**（与主文档、扩展篇一致，总表见 §6）：
>
> - `[P1] 式(k)`：Cohen & Shoham, *Hyper Dual Quaternions representation of rigid bodies kinematics*, MMT 150 (2020)；
> - `[P2] 式(k)`：Figueredo, Adorno, Ishihara, *Robust H∞ kinematic control…*, Automatica 132 (2021)；
> - `主文档 (k)` / `(D-k)`：项目已有推导 / 扩展篇新推导；
> - `[F&A]`：Fike & Alonso, AIAA 2011-886（超对偶数二阶精确微分）；
> - `[C&S16]`：Cohen & Shoham, *Application of Hyper-Dual Numbers to Multibody Kinematics*, ASME JMR 8(1), 2016；
> - `[C&S17]`：Cohen & Shoham, *Application of hyper-dual numbers to rigid bodies equations of motion*, MMT 111 (2017) 76–84 —— HDN 刚体动力学方程（超对偶雅可比、超对偶速度）；
> - `[SQA22]`：Silva, Quiroz-Omaña, Adorno, *Dynamics of Mobile Manipulators Using Dual Quaternion Algebra*, ASME JMR 14(6), 2022 —— DQ twist/wrench 递推牛顿-欧拉与 Gauss 最小约束原理两条路线；
> - `[Kha87]` / `[LP17]`：Khatib 操作空间动力学 / Lynch & Park《Modern Robotics》标准结果；
> - **(E-k)**：**本文新推导**（附证明或证明思路）。
>
> **记号**：沿用主文档。位姿 DQ $\boldsymbol x\in\mathrm{Spin}(3)\ltimes\mathbb R^3$，左乘约定 $\dot{\boldsymbol x}=\tfrac12\boldsymbol\xi\boldsymbol x$（[P2] 式(1)），$\bar S_i=\overline{\mathrm{vec}}_6(S_i)$ 为关节螺旋轴纯 DQ，$P_k=X_1\cdots X_k$ 为前缀积。$\widehat{\mathbb H}$ 记 DQ 代数。

---

## 目录

1. [代数塔的完整图景：从 ℍ 到 k 阶节代数](#1-代数塔的完整图景从-ℍ-到-k-阶节代数)
2. [数学推导：动力学所需全部几何量的高阶四元数表示](#2-数学推导动力学所需全部几何量的高阶四元数表示)
3. [物理模型：HDQ 输出量与动力学系数矩阵的关联](#3-物理模型hdq-输出量与动力学系数矩阵的关联)
4. [工程实现流程（简化）](#4-工程实现流程简化)
5. [创新与优化对比](#5-创新与优化对比)
6. [公式来源总表](#6-公式来源总表)
7. [局限性与研究空白（诚实声明）](#7-局限性与研究空白诚实声明)
8. [参考文献](#8-参考文献)

---

## 1. 代数塔的完整图景：从 ℍ 到 k 阶节代数

### 1.1 已有层级（主文档 §1–§4、扩展篇 §2）

$$
\mathbb R\;\subset\;\mathbb H\;\subset\;\underbrace{\mathbb H[\varepsilon]/(\varepsilon^2)}_{\text{DQ }\widehat{\mathbb H}}\;\subset\;\underbrace{\widehat{\mathbb H}[\varepsilon^*]/(\varepsilon^{*2})}_{\text{HDQ（[P1] 式(25)）}}\;\subset\;\underbrace{\widehat{\mathbb H}[\sigma]/(\sigma^3)}_{\mathcal A_2\text{（扩展篇 §3.1）}}\;\subset\;\cdots
$$

各幂零单位的**通道分工**（关键结构事实，扩展篇 §2.2）：$\varepsilon$ 承载平移（几何），$\varepsilon^*$（即 $\mathcal A_2$ 中 $\sigma$ 的一阶截断）承载一阶时间导数，$\sigma^2$ 承载二阶导数。**每升一阶导数需求，就在系数环上再乘一个幂零方向或提高截断阶**——这是"更高阶四元数结构"的准确含义：不是新的四元数，而是**同一 DQ 系数上更深的截断多项式环（节代数，jet algebra）**。

### 1.2 k 阶节代数与提升同态

**定义**：$\mathcal A_k\triangleq\widehat{\mathbb H}[\sigma]/(\sigma^{k+1})$，元素

$$
\breve a=\sum_{m=0}^{k}\frac{\sigma^m}{m!}\,a_m,\qquad a_m\in\widehat{\mathbb H},
$$

乘法由 DQ 乘法双线性延拓并截断 $\sigma^{k+1}=0$：

$$
\breve a\,\breve b=\sum_{m=0}^{k}\frac{\sigma^m}{m!}\Bigl(\sum_{r=0}^{m}\binom{m}{r}a_r\,b_{m-r}\Bigr).
\tag{1.1}
$$

> **定理 (E-1)（$k$ 阶提升同态）**：对光滑 DQ 曲线 $a(t)$ 定义 $T^k a\triangleq\sum_{m=0}^k\frac{\sigma^m}{m!}a^{(m)}$，则对任意光滑曲线 $a,b$：
>
> $$
> T^k(ab)=T^ka\cdot T^kb .
> $$
>
> **证明**：DQ 乘法双线性 ⟹ 广义 Leibniz 公式 $(ab)^{(m)}=\sum_{r}\binom{m}{r}a^{(r)}b^{(m-r)}$ 成立（不需交换性）；与 (1.1) 的 $\sigma^m$ 通道逐项相同。∎
>
> $k=1$ 退化为 [P1] 式(14)（HDQ 乘法即 Leibniz 法则，主文档 §4.2）；$k=2$ 即扩展篇 (D-1)。**(E-1)**

**含义**：$n$ 个关节因子的一次 $\mathcal A_k$ 链乘，同时输出 $\boldsymbol x,\dot{\boldsymbol x},\ldots,\boldsymbol x^{(k)}$，代价 $O(n)$ 次 $\mathcal A_k$ 乘（每次 $\binom{k+2}{2}$ 次 DQ 乘）。$k$ 阶广义 Leibniz 多重求和全部由乘法规则隐式完成——主文档 §6.4 "求和结构由乘法法则隐式完成"的观察对任意阶成立。

### 1.3 各阶通道的工程语义

| 截断阶 $k$ | 新增输出 | 服务对象 |
|---|---|---|
| 0（DQ） | $\boldsymbol x$ | 位姿反馈（[P2] 式(8)(10)，项目现状） |
| 1（HDQ，[P1]） | $\dot{\boldsymbol x},\boldsymbol\xi$ | 速度前馈、交叉验证（主文档 §6） |
| 2（$\mathcal A_2$，扩展篇） | $\ddot{\boldsymbol x},\dot{\boldsymbol\xi},\dot J\dot q$ | 计算力矩 (5.2)、操作空间动力学 (5.3) |
| 3（$\mathcal A_3$，本文 §2.2） | $\boldsymbol x^{(3)},\ddot{\boldsymbol\xi}$（jerk 级） | 轨迹光滑度约束、柔性/振动抑制、微分平坦性输出 |
| 多参数 $\sigma_1,\sigma_2$ | 混合偏导 $H_{ij}$ | Hessian 验证器（扩展篇 §4.4，[F&A] 思想） |

$k\ge3$ 的边际收益递减：jerk 反馈几乎不可测，仅前馈规划用；本文推导止于 $k=3$。

---

## 2. 数学推导：动力学所需全部几何量的高阶四元数表示

### 2.1 二阶量清单（扩展篇结果引用）

动力学建模需要的二阶几何量已在扩展篇完成，罗列备查：

| 量 | 公式 | 编号 | 复杂度 |
|---|---|---|---|
| $\ddot{\boldsymbol x}$ | $\mathcal A_2$ 链 $\sigma^2$ 通道；单关节因子 $\ddot x_i=\tfrac12\ddot\theta_i\bar S_ix_i+\tfrac14\dot\theta_i^2\bar S_i^2x_i$，转动关节 $\bar S_i^2=-1$ | (D-1)(D-2) | $O(n)$ |
| $\dot{\boldsymbol\xi}$ | $\dot{\boldsymbol\xi}=2\ddot{\boldsymbol x}\boldsymbol x^*-\tfrac12\boldsymbol\xi^2$，vec6 层修正项消失 | (D-4) | $O(1)$ 后处理 |
| $\dot J\dot{\boldsymbol q}$ | 二阶链播种 $\ddot q=0$，免构造 $\dot J$ | (D-5) | $O(n)$ |
| $H_{ij}=\partial^2\boldsymbol x/\partial q_i\partial q_j$ | $P_{i-1}X_i'(P_i^*P_{j-1})X_j'S_{j+1}$ | (D-6) | $O(n^2)$ 全部 |
| $h_{ij}=\partial\boldsymbol\jmath_i/\partial q_j$ | $2H_{ij}\boldsymbol x^*-\tfrac12\boldsymbol\jmath_i\boldsymbol\jmath_j$ | (D-7) | $O(n^2)$ |
| $\dot J$ 全矩阵 | $\dot{\boldsymbol\jmath}_i=2(\sum_jH_{ij}\dot q_j)\boldsymbol x^*-\tfrac12\boldsymbol\jmath_i\boldsymbol\xi$ | (D-9) | $O(n^2)$ |

### 2.2 三阶量：jerk 级的提取公式

$\mathcal A_3$ 链给出四个通道 $\boldsymbol x,\dot{\boldsymbol x},\ddot{\boldsymbol x},\boldsymbol x^{(3)}$（单关节因子的 $\sigma^3$ 通道由 (D-2) 再求导：$x_i^{(3)}=\tfrac12\theta_i^{(3)}\bar S_ix_i+\tfrac34\ddot\theta_i\dot\theta_i\bar S_i^2x_i+\tfrac18\dot\theta_i^3\bar S_i^3x_i$，转动关节 $\bar S_i^3=-\bar S_i$）。twist 的各阶导数直接按 Leibniz 展开（共轭是线性映射，与求导可交换）：

> **命题 (E-2)（twist 高阶导数的通道表示）**：
>
> $$
> \boldsymbol\xi=2\dot{\boldsymbol x}\boldsymbol x^*,\qquad
> \dot{\boldsymbol\xi}=2\bigl(\ddot{\boldsymbol x}\boldsymbol x^*+\dot{\boldsymbol x}\dot{\boldsymbol x}^*\bigr),\qquad
> \ddot{\boldsymbol\xi}=2\bigl(\boldsymbol x^{(3)}\boldsymbol x^*+2\ddot{\boldsymbol x}\dot{\boldsymbol x}^*+\dot{\boldsymbol x}\ddot{\boldsymbol x}^*\bigr),
> $$
>
> 其中 $\dot{\boldsymbol x}^*,\ddot{\boldsymbol x}^*$ 就是链输出通道的共轭（无需额外计算）。$\dot{\boldsymbol\xi}$ 的表达式与 (D-4) 等价——由 (D-3) 有 $2\dot{\boldsymbol x}\dot{\boldsymbol x}^*=-\tfrac12\boldsymbol\xi^2$。**(E-2)**
>
> **证明**：对 $\boldsymbol\xi=2\dot{\boldsymbol x}\boldsymbol x^*$ 逐次求导并用共轭线性性。∎

工程上 (E-2) 优于反复化简：所有右端项都是链通道的 $O(1)$ 组合，jerk 级 $\ddot{\boldsymbol\xi}$ 用于 jerk 受限轨迹规划（$\mathrm{vec}_6\ddot{\boldsymbol\xi}=J q^{(3)}+2\dot J\ddot q+\ddot J\dot q$ 的任务空间侧免构造获取）。

### 2.3 惯性矩阵 $M(\boldsymbol q)$ 的前缀积装配

设连杆 $i$ 的质心系空间惯性 $\bar M_i=\begin{bmatrix}I_i&0\\0&m_iE_3\end{bmatrix}\in\mathbb R^{6\times6}$（质心系表示，$I_i$ 转动惯量、$m_i$ 质量）。关节空间惯性矩阵的标准装配式（[LP17] 式(8.57) 的等价形式）：

$$
M(\boldsymbol q)=\sum_{i=1}^{n}J_{c_i}^{T}\,\bar M_i^{\,s}(\boldsymbol q)\,J_{c_i},
\tag{2.1}
$$

其中 $J_{c_i}\in\mathbb R^{6\times n}$ 是连杆 $i$ 质心的**部分雅可比**（第 $j>i$ 列为零），$\bar M_i^{\,s}$ 是 $\bar M_i$ 经伴随变换搬到基座系的结果。

> **命题 (E-3)（部分雅可比的前缀积复用）**：连杆 $i$ 的位姿链为 $\boldsymbol x_{c_i}=P_i\,\boldsymbol c_i$（$\boldsymbol c_i$ 为关节 $i$ 帧到质心帧的常值 DQ）。其部分雅可比的第 $j\le i$ 列为
>
> $$
> \boldsymbol\jmath^{(i)}_j=2\,P_{j-1}X_j'\,\bigl(P_j^*P_i\bigr)\boldsymbol c_i\;\boldsymbol x_{c_i}^{*}
> \;=\;\boldsymbol\jmath_j\qquad(j\le i),
> $$
>
> 即**部分雅可比的非零列与全链雅可比的对应列相同**，全部 $J_{c_i}\ (i=1..n)$ 无需任何新链传播——一次 `hdq_fast` 型前缀预计算（主文档 §5.3(iv)）同时供给全部 $n$ 个连杆。
>
> **证明**：$\boldsymbol\jmath^{(i)}_j=2(\partial_j\boldsymbol x_{c_i})\boldsymbol x_{c_i}^*$，按主文档 (5.1) 的引理展开：$\partial_j\boldsymbol x_{c_i}=P_{j-1}X_j'(X_{j+1}\cdots X_i)\boldsymbol c_i$，右乘 $\boldsymbol x_{c_i}^*$ 后中段与 $\boldsymbol c_i$ 全部消去（单位 DQ 共轭即逆），只剩 $2P_{j-1}X_j'P_{j-1}^*\cdot(P_{j-1}X_jP_j^*)\cdots$ 化简至与全链 $\boldsymbol\jmath_j$ 相同的表达式——这正是空间雅可比列只依赖关节 $j$ 之前几何的几何事实的代数重述。∎
>
> **(E-3)** 的意义：(2.1) 的装配代价 $O(n^3)$（矩阵乘）中，**几何部分（全部 $J_{c_i}$ 与各 $\mathrm{Ad}_{P_i}$）只需一次 $O(n)$ 前缀传播**；HDQ/DQ 链把"每根连杆单独跑一次 FK"的朴素 $O(n^2)$ 几何开销压缩掉了。

### 2.4 Coriolis 矩阵：Hessian 的第一个动力学刚需

Coriolis 矩阵的 Christoffel 装配（标准结果，[LP17] 式(8.51)）：

$$
C_{ij}(\boldsymbol q,\dot{\boldsymbol q})=\sum_{k=1}^{n}\frac12\Bigl(\frac{\partial M_{ij}}{\partial q_k}+\frac{\partial M_{ik}}{\partial q_j}-\frac{\partial M_{jk}}{\partial q_i}\Bigr)\dot q_k .
\tag{2.2}
$$

(2.2) 需要 $\partial M/\partial q_k$——由 (2.1) 求导：

> **命题 (E-4)（惯性矩阵导数的 Hessian 表示）**：
>
> $$
> \frac{\partial M}{\partial q_k}
> =\sum_{i=1}^{n}\Bigl[(\partial_kJ_{c_i})^{T}\bar M_i^{s}J_{c_i}
> +J_{c_i}^{T}(\partial_k\bar M_i^{s})J_{c_i}
> +J_{c_i}^{T}\bar M_i^{s}(\partial_kJ_{c_i})\Bigr],
> $$
>
> 其中：
> - $(\partial_kJ_{c_i})$ 的第 $j$ 列 $=\mathrm{vec}_6\,h_{jk}$（$j,k\le i$，否则为零）——**恰为扩展篇任务空间 Hessian (D-7)**；
> - $\partial_k\bar M_i^{s}=[\mathrm{ad}_{\mathrm{vec}_6\jmath_k},\bar M_i^{s}]$（伴随搬运的导数即 $\mathrm{ad}$ 括号，$k\le i$），$\mathrm{ad}$ 为 $se(3)$ 伴随小代数算子（[LP17] §8.2 标准算子）。
>
> **(E-4)**（组合方式为新整理；两个构件分别是 (D-7) 与教科书 $\mathrm{ad}$ 公式。）

**数学系读者的几何解读**：$M(\boldsymbol q)$ 是位形流形上的黎曼度量（动能 $=\tfrac12\dot{\boldsymbol q}^TM\dot{\boldsymbol q}$），(2.2) 的 $C$ 正是该度量的 Christoffel 记号缩并。**度量的导数需要浸入映射（FK）的二阶导数——这就是 Hessian (D-6)(D-7) 在动力学中"结构性不可绕过"的几何原因**：无扰动力学 $M\ddot q+C\dot q=0$ 就是度量测地线方程，而测地线方程天然含二阶几何量。

### 2.5 操作空间动力学各量的 HDQ 供给清单

Khatib 操作空间方程（[Kha87]，扩展篇 (5.3)）：

$$
\Lambda(\boldsymbol q)\dot{\boldsymbol\xi}+\mu(\boldsymbol q,\dot{\boldsymbol q})+\boldsymbol p(\boldsymbol q)=\boldsymbol F,
\qquad
\Lambda=(JM^{-1}J^{T})^{-1},\quad
\mu=\Lambda\bigl(JM^{-1}C\dot{\boldsymbol q}-\dot J\dot{\boldsymbol q}\bigr),\quad
\boldsymbol p=\Lambda JM^{-1}\boldsymbol g .
$$

| 方程构件 | HDQ/DQ 来源 | 公式 |
|---|---|---|
| $J$ | `hdq_fast` 前缀/后缀 | 主文档 §5.3(iv)，[P2] 式(3) |
| $\dot J\dot{\boldsymbol q}$ | 二阶链播种 | (D-5) |
| $\dot J$（若需全矩阵） | Hessian 列和 | (D-9) |
| $\dot{\boldsymbol\xi}$（测量侧） | $\mathcal A_2$ 链 + 提取 | (D-4)/(E-2) |
| $M$ | 前缀积装配 | (2.1)+(E-3) |
| $C\dot{\boldsymbol q}$ | Christoffel + Hessian | (2.2)+(E-4) |
| $\boldsymbol g$ | $\boldsymbol g_i=\sum_k m_k\,\partial_i(\text{质心高度})$，由部分链 0 阶通道 + 一阶列 | (E-3) 的 $J_{c_i}$ 平移行 |

**与 [C&S17]、[SQA22] 的关系定位**：[C&S17] 用超对偶数写出单刚体动力学方程（超对偶雅可比/速度），[SQA22] 用 DQ twist/wrench 做递推牛顿-欧拉——两者都证明"四元数系代数能承载动力学"。本文路线的差异点：**把导数获取本身代数化**（节代数通道），使 (2.1)(2.2) 的全部几何系数从同一条链的不同通道读出，而非逐条手工递推。

### 2.6 递推牛顿-欧拉的前向传播 ≡ $\mathcal A_2$ 链

> **命题 (E-5)（RNE 前向传播的节代数重述）**：递推牛顿-欧拉算法的前向阶段（连杆速度/加速度传播，[LP17] 算法 8.1；DQ 形式见 [SQA22]）
>
> $$
> \xi_i=\mathrm{Ad}_{i,i-1}\,\xi_{i-1}+\bar S_i\dot q_i,\qquad
> \dot\xi_i=\mathrm{Ad}_{i,i-1}\,\dot\xi_{i-1}+\mathrm{ad}_{\xi_i}\bar S_i\dot q_i+\bar S_i\ddot q_i
> $$
>
> 与"$\mathcal A_2$ 链在前缀 $P_i$ 处的 $\sigma,\sigma^2$ 通道 + (D-4)/(E-2) 提取"给出的 $\{\xi_i,\dot\xi_i\}$ **逐连杆相等**。
>
> **证明思路**：对前缀 $P_i=P_{i-1}X_i$ 应用 (E-1) 的同态性与 (D-4)：$\xi_i=2\dot P_iP_i^*$ 展开即伴随传播项加新关节项；再求导一次给出 $\dot\xi_i$ 的三项，其中 $\mathrm{ad}$ 项来自伴随的导数（同 (E-4) 第二构件）。逐项对应即得。∎
>
> **物理意义**：$\mathcal A_2$ 链**就是** RNE 前向传播的代数打包——每个前缀积的三个通道分别是"该连杆的位姿、twist、加速度"。RNE 的后向力传播（wrench 回代）不属几何层，需惯性参数介入，接 [SQA22] 的 DQ wrench 递推即可。**(E-5)**

### 2.7 可操作度梯度：Hessian 的控制级应用

Yoshikawa 可操作度 $w(\boldsymbol q)=\sqrt{\det(JJ^T)}$ 的梯度（避奇异势场、零空间优化必需）：

$$
\frac{\partial w}{\partial q_k}=w\cdot\mathrm{tr}\Bigl((JJ^{T})^{-1}\,\frac{\partial J}{\partial q_k}J^{T}\Bigr),
\tag{2.3}
$$

（(2.3) 为矩阵微积分标准结果。）其中 $\partial J/\partial q_k$ 的第 $j$ 列 $=\mathrm{vec}_6\,h_{jk}$——又是 (D-7)。

> **推论 (E-6)（精确避奇异梯度）**：零空间梯度投影律
>
> $$
> \dot{\boldsymbol q}=J^{+}\boldsymbol u_{\text{task}}+\lambda\bigl(E_n-J^{+}J\bigr)\nabla_{\boldsymbol q}w
> $$
>
> 中的 $\nabla w$ 可由 (D-7)+(2.3) **解析精确**给出，替代常用的逐关节差分（$n$ 次额外全雅可比计算 + $O(\epsilon)$ 误差）。这是 Hessian 在**不引入任何动力学参数**的前提下就能落地的控制优化——对项目现有 7R 冗余臂（$n=7>6$）立即适用。**(E-6)**

---

## 3. 物理模型：HDQ 输出量与动力学系数矩阵的关联

### 3.1 关节空间与操作空间的二阶运动学关系

$$
\underbrace{\mathrm{vec}_6\,\boldsymbol\xi=J\dot{\boldsymbol q}}_{\text{一阶（[P2] 式(2)}}
\qquad\Longrightarrow\qquad
\underbrace{\mathrm{vec}_6\,\dot{\boldsymbol\xi}=J\ddot{\boldsymbol q}+\dot J\dot{\boldsymbol q}}_{\text{二阶（D-5）}} .
$$

物理解读：$J\ddot q$ 是"关节加速度直接贡献"，$\dot J\dot q$ 是**运动状态本身引起的几何弯曲贡献**（哪怕 $\ddot q=0$，末端仍有加速度——圆周运动的向心加速度是其最简单例子）。速度越高 $\dot J\dot q$ 越大（对 $\dot q$ 二次齐次），这决定了它是高速运动控制中不可忽略、又最容易被纯运动学控制器丢掉的一项。

### 3.2 系数矩阵的物理/几何身份与 HDQ 供给

| 矩阵 | 物理身份 | 几何身份 | HDQ 供给的部分 | 非几何输入 |
|---|---|---|---|---|
| $M(\boldsymbol q)$ | 广义质量（动能二次型） | 位形流形黎曼度量 | 全部 $J_{c_i}$、$\mathrm{Ad}_{P_i}$（(E-3)，一次前缀传播） | 惯性参数 $m_i,I_i$ |
| $C(\boldsymbol q,\dot{\boldsymbol q})$ | 科里奥利/离心力 | 度量的 Christoffel 记号 | $\partial J_{c_i}/\partial q_k$ = Hessian 列（(E-4)+(D-7)） | 同上 |
| $\boldsymbol g(\boldsymbol q)$ | 重力广义力 | 势函数梯度 | 质心链 0 阶通道 + $J_{c_i}$ 平移行 | $m_i$、重力方向 |
| $\Lambda,\mu$ | 操作空间等效惯性/偏置力 | 度量在 $J$ 下的推前 | $J,\dot J\dot q$（(D-5)(D-9)） | $M,C$ |

**核心物理图像**：动力学系数矩阵 = "惯性参数 ⊗ 几何量"的双线性组合。HDQ 节代数把其中**全部几何量**（各阶导数）变成一条链的不同通道读数；惯性参数是且仅是额外输入。因此"HDQ 动力学建模"的准确表述是：**HDQ 承包动力学的微分几何层，动力学参数层照旧**。

### 3.3 传感器信息层级对建模精度的影响

扩展篇 §1 已给出完整分析（编码器量化、差分噪声方差 $2\sigma_q^2/\Delta t^2$、$6\sigma_q^2/\Delta t^4$、力矩⇄加速度换算），此处只补充动力学视角的增量结论：

| 反馈层 | 新解锁的动力学能力 | 精度瓶颈 |
|---|---|---|
| $q$ | $M(\boldsymbol q),\boldsymbol g(\boldsymbol q)$ 可实时装配（只依赖位形） | 惯性参数误差 $\Delta m,\Delta I$ |
| $+\dot q$ | $C\dot q$、$\dot J\dot q$、$\Lambda,\mu$ 全部可算 ⟹ **计算力矩前馈 (5.2) 齐备** | 速度估计噪声进入 $C\dot q$（对 $\dot q$ 二次） |
| $+\ddot q$ 或 $\tau$ | 正动力学校验、外力估计 $\hat F_{\mathrm{ext}}=\tau-M\ddot q-C\dot q-g$ | 二选一：差分方差爆炸 vs 模型偏差（扩展篇 §1.2 路线 (a)(c)） |

注意一个常被忽略的事实：**计算力矩控制并不需要测量 $\ddot q$**——(5.2) 中的 $\ddot q_{\mathrm{ref}}$ 是控制器算出来的期望值，测量侧只需 $q,\dot q$。$\ddot q$ 测量只在模型辨识与外力估计中才是刚需。这降低了二阶控制律落地的传感门槛。

### 3.4 反馈控制律能否实际获益？——分速度域的诚实评估

设跟踪误差动力学。纯运动学控制（项目现状，[P2] 式(12)）把 $\dot q_{\mathrm{cmd}}$ 直接发给关节速度环，隐含假设"速度环无穷快、$\dot J\dot q$ 可忽略"。该假设的破坏程度随速度增长：

- **低速域**（项目实验工况：$|\dot q|\le0.8$ rad/s、50 Hz）：$\dot J\dot q\sim\|h\|\|\dot q\|^2$ 量级小，厂商速度环带宽（通常 ≳ 200 Hz）远高于任务环。**结论：二阶成果无实质收益**，这与主文档 §10、扩展篇 §5.3 的判断一致；
- **中高速/大惯量域**：$\dot J\dot q$ 与 $C\dot q$ 变为一阶误差源。级联结构（H∞ 运动学外环给 $\boldsymbol\xi_{\mathrm{ref}}$，计算力矩内环 (5.2) 消化 $M,C,g,\dot J\dot q$）可获得的具体改进：
  1. **前馈消偏**：$-\dot J\dot q$ 项在 (5.2) 中显式补偿，消除随速度平方增长的系统性跟踪偏差；
  2. **扰动整形**：[P2] 的 H∞ 界继续对残余扰动成立——模型补偿把"结构性偏差"从 $v_w$ 中挪走，等效缩小了 $\|v_w\|_{L_2}$，从而按 Definition 1 的不等式**收紧同一 $\gamma$ 下的误差能量上界**（$\gamma$ 本身不变，是输入能量变小）；
  3. **无动力学参数也可得的收益**：(E-6) 精确避奇异梯度只用几何层，7R 冗余臂立即适用。
- **接触/交互域**：外力估计与阻抗控制必需 $M,C,g$ + 力矩反馈，HDQ 供给几何层。

---

## 4. 工程实现流程（简化）

### 4.1 流水线总图

```
输入（每控制周期）:  q ∈ ℝⁿ (编码器)    q̇ ∈ ℝⁿ (观测器)    [q̈_ref 或 τ (电流)]
                     │
  ┌──────────────────┴────────────────────────────────────────────┐
  │ 步骤1  关节因子构造:  X_i, X_i′ (dq_standard_dh_and_derivative)│  [P2]式(3)分解
  │        A₂因子:  x_i + σ(½q̇ᵢS̄ᵢxᵢ) + ½σ²(½q̈ᵢS̄ᵢxᵢ+¼q̇ᵢ²S̄ᵢ²xᵢ)    │  (D-2)
  ├────────────────────────────────────────────────────────────────┤
  │ 步骤2  一次 A₂ 链乘 + 前缀/后缀缓存 P_k, S_k                    │  (E-1)(3.1)
  │        通道读出:  x, ẋ, ẍ                                      │  O(n)
  ├────────────────────────────────────────────────────────────────┤
  │ 步骤3  一阶提取:  ξ = vec₆(2ẋx*)                               │  主文档(6.1)
  │        二阶提取:  ξ̇ = vec₆(2ẍx*)                               │  (D-4)
  │        J:  prefix/suffix 逐列                                  │  主文档§5.3(iv)
  │        J̇q̇:  播种 q̈=0 的 σ² 通道                                │  (D-5)
  │        [可选] H_{ij}, J̇, ∇w:  双重循环                          │  (D-6)(D-9)(E-6), O(n²)
  ├────────────────────────────────────────────────────────────────┤
  │ 步骤4  [动力学层, 需惯性参数]  M: (2.1)+(E-3)   C: (2.2)+(E-4)  │  O(n²)~O(n³)
  │        g: 质心链平移行                                          │
  ├────────────────────────────────────────────────────────────────┤
  │ 步骤5  控制律:                                                  │
  │   低速: q̇_cmd = J⁺(κ·err + 前馈)          ← 项目现状            │  [P2]式(12)
  │   高速: τ = M·J⁺(ξ̇_d + K_d·δξ + K_p·δz − J̇q̇) + Cq̇ + g          │  (5.2)
  │   冗余: + λ(E−J⁺J)∇w                                           │  (E-6)
  └────────────────────────────────────────────────────────────────┘
输出:  x (位姿DQ)   ξ (twist)   ξ̇ (任务加速度)   J, J̇q̇ [, H, J̇, M, C, g, τ]
```

### 4.2 各环节输入/输出/公式对照表

| 环节 | 输入 | 输出 | 调用公式 | 复杂度 |
|---|---|---|---|---|
| 因子构造 | $q,\dot q[,\ddot q]$ | $\{X_i,X_i',\breve X_i\}$ | (D-2)、主文档 (6.2) | $O(n)$ |
| 链传播 | 因子序列 | $\boldsymbol x,\dot{\boldsymbol x},\ddot{\boldsymbol x}$；$P_k,S_k$ | (E-1)(3.1)、[P1] 式(28) 升阶 | $O(n)$ |
| 一阶提取 | 通道 | $\boldsymbol\xi$ | 主文档 (6.1)、[P1] 式(33) 左乘版 | $O(1)$ |
| 二阶提取 | 通道 | $\dot{\boldsymbol\xi}$、$\dot J\dot q$ | (D-4)(D-5)(E-2) | $O(1)$ |
| 雅可比 | $P,S,X_i'$ | $J$ | 主文档 (5.1)、[P2] 式(3) | $O(n)$ |
| Hessian 层 | $P,S,X_i',X_i''$ | $H,\dot J,\nabla w$ | (D-6)(D-7)(D-9)(2.3)(E-6) | $O(n^2)$ |
| 动力学装配 | 几何层 + $m_i,I_i$ | $M,C,\boldsymbol g$ | (2.1)(2.2)(E-3)(E-4) | $O(n^2)$–$O(n^3)$ |
| 控制律 | 上述全部 + 误差 | $\dot q_{\mathrm{cmd}}$ 或 $\boldsymbol\tau$ | [P2] 式(12) / (5.2) / (E-6) | $O(n^2)$（伪逆） |

### 4.3 性能指标

**实测部分**（主文档 §9.2/§9.3，7-DOF，Python）：$J$ 各方法 0.138–6.739 ms；一阶 HDQ 链每周期 0.743 ms vs DQ 0.330 ms；两路线 twist 差 $\sim10^{-18}$。

**理论估计部分**（明确标注：未实测）：

| 量 | 估算依据 | 预期每周期耗时（Python，n=7） |
|---|---|---|
| $\mathcal A_2$ 链（$\boldsymbol x,\dot{\boldsymbol x},\ddot{\boldsymbol x}$） | 6 次 DQ 乘/因子，一阶链（2 次/因子）的 3 倍 | $\approx2.2$ ms |
| 全 Hessian + $\dot J$ | $\tfrac{n(n+1)}2=28$ 项 × 每项 ~5 次 DQ 乘 ≈ `hdq_fast` 的 3 倍 | $\approx2.7$ ms |
| $M,C$ 装配 | numpy 矩阵运算主导 | $\approx1$ ms |

Python 合计 ~6 ms/周期，可支撑 100 Hz 力矩外环原型验证；1 kHz 工业力矩环需编译实现（DQ 乘法为固定 24-乘-16-加核，C/JIT 化后上表全部量预计 < 0.1 ms——定性判断，依据是同类 DQ 库的常见量级，未实测）。

---

## 5. 创新与优化对比

### 5.1 与数值微分（差分）对比

| 维度 | 差分路线 | 节代数路线 |
|---|---|---|
| $J$ | $2n$ 次 FK，$O(\epsilon^2)$ 截断（主文档 §5.3(ii)，实测 0.743 ms） | 精确，$O(n)$（`hdq_fast` 0.892 ms，实测） |
| $\dot J$ | $J$ 的差分：截断+噪声双重放大，步长两难（$\epsilon$ 大→截断，$\epsilon$ 小→舍入） | (D-9) 精确闭式 |
| $\ddot q$ 依赖量 | 双重差分方差 $6\sigma_q^2/\Delta t^4$（扩展篇 §1.2，不可用） | 不差分：$\ddot q$ 由力矩+模型换算或仅用于前馈（§3.3） |
| Hessian | $O(n^2)$ 次 FK 的二阶差分，条件数灾难 | (D-6) 精确，$O(n^2)$ 次 DQ 乘 |

**本质区别**：差分的误差随微分阶数按 $\epsilon^{-k}$ 恶化；节代数的各阶通道全部精确到机器精度（幂零截断是恒等式而非近似，[F&A] 的核心论点在任意阶成立，(E-1)）。

### 5.2 与符号微分对比

符号微分 $\partial^2\boldsymbol x/\partial q_i\partial q_j$ 面临表达式膨胀：$n$ 连杆三角函数积的二阶导项数随 $n$ 组合爆炸，需离线代码生成且参数改动即重新生成。节代数路线**无表达式对象**——所有导数是运行时数值通道，DH/POE 参数改动零成本；代价换来的是每周期 $O(n^2)$ 在线计算（对 $n\le10$ 可忽略）。

### 5.3 与 DQ 方法在二阶量上的对比

| 能力 | 纯 DQ | HDQ/节代数 |
|---|---|---|
| 位姿 $\boldsymbol x$ | ✅ | ✅（0 阶通道） |
| $\boldsymbol\xi$ | 需先构造 $J$ 再 $J\dot q$ | 链通道直接输出（免 $J$，主文档 §6） |
| $\dot{\boldsymbol\xi},\dot J\dot q$ | 无代数通道，只能差分或手推递推（[SQA22] 路线：手工推导 DQ-RNE 递推式） | $\sigma^2$ 通道 + (D-4)(D-5)，**递推式由乘法规则自动生成**（(E-5)：$\mathcal A_2$ 链 ≡ RNE 前向传播） |
| Hessian | 无 | (D-6)，且是 `hdq_fast` 前缀数组的零成本复用 |

即：DQ 是位姿代数，HDQ/节代数是"位姿 + 全部导数"的代数。**一次链传播多通道输出**是 DQ 结构上给不出的能力——它的 $\varepsilon$ 已被平移占用（扩展篇 §2.2）。

### 5.4 实时动力学控制中的实际性能提升（分级诚实结论）

1. **已被项目实测支持的**：一阶层面 HDQ 无速度优势（0.743 vs 0.330 ms），价值在交叉验证与 `hdq_fast`（主文档 §9.3/§10）；
2. **有严格数学保证、待实验的**：$\dot J\dot q$ 免构造 $O(n)$ 获取 (D-5)、$C$ 矩阵解析装配 (E-4)、避奇异精确梯度 (E-6)——相对差分路线的精度提升是定理级的（机器精度 vs $O(\epsilon)$），相对速度是复杂度级的（$O(n)$ vs $O(n^2)$ 次 FK）；
3. **依赖工况的**：闭环跟踪精度改善只在中高速/大惯量/接触场景兑现（§3.4）；项目当前 50 Hz 低速工况下收益趋零；
4. **当前不可验证的**：CoppeliaSim 实验走关节速度接口，力矩级控制律 (5.2) 需切换力矩接口或实机，属后续工作。

---

## 6. 公式来源总表

| 编号 | 内容 | 来源 |
|---|---|---|
| [P1] 式(14)(25)(28)(29)(33) | HDQ 乘法/结构/链式法则/twist 提取 | Cohen & Shoham 2020 |
| [P2] 式(1)(2)(3)(8)(10)(12) | 左乘运动学、雅可比列、误差、H∞ 律 | Figueredo et al. 2021 |
| 主文档 (5.1)(6.1)(6.2)、§5.3(iv)、§9 | 雅可比引理、twist 提取、`hdq_fast`、实测数据 | 本项目 |
| (D-1)–(D-9) | 二阶链、$\dot\xi$、$\dot J\dot q$、Hessian、$\dot J$ | 扩展篇（本项目） |
| (1.1) | 截断多项式环乘法（$k$ 阶） | 教科书级（jet 代数） |
| **(E-1)** | $k$ 阶提升同态 $T^k(ab)=T^ka\,T^kb$ | **新推导**（(D-1) 的任意阶推广；一元幂零 AD 原理见 [F&A]） |
| **(E-2)** | $\dot{\boldsymbol\xi},\ddot{\boldsymbol\xi}$ 的通道 Leibniz 表示 | **新推导** |
| **(E-3)** | 部分雅可比非零列 = 全链列；一次前缀传播供给全部 $J_{c_i}$ | **新推导**（几何事实的 DQ 代数证明） |
| **(E-4)** | $\partial M/\partial q_k$ 的 Hessian+$\mathrm{ad}$ 分解 | **新整理**（构件：(D-7) + [LP17] $\mathrm{ad}$ 公式） |
| **(E-5)** | $\mathcal A_2$ 链 ≡ RNE 前向传播（DQ 形式） | **新推导**（RNE 本身 [LP17]；DQ-RNE [SQA22]） |
| **(E-6)** | 精确避奇异梯度（(2.3)+(D-7)） | **新组合**（(2.3) 为矩阵微积分标准结果） |
| (2.1)(2.2) | $M$ 装配、Christoffel $C$ | [LP17] 式(8.57)(8.51) 标准结果 |
| (5.2)(5.3)（扩展篇编号） | 计算力矩、操作空间方程 | [LP17] / [Kha87] |
| §3.3 传感层级表 | 差分噪声、力矩换算 | 扩展篇 §1（标准工程结果） |
| HDN 动力学先例 | 超对偶雅可比/速度、单刚体 EoM | [C&S17]（本文未复用其具体公式，仅定位） |
| DQ 动力学先例 | DQ twist/wrench 递推 NE、Gauss 原理 | [SQA22]（后向力传播接口） |

> **诚实性声明**：(E-1) 是一元截断 AD 的标准原理在 DQ 系数上的表述，数学内容接近平凡但项目内需要显式定理化；(E-3)(E-5) 的**结论**是机器人学常识（空间雅可比列性质、RNE 前向递推），本文贡献是其在左乘 DQ 约定与前缀积数据结构下的代数证明与复用方式；(E-4)(E-6) 是已知构件的新组合。真正无先例可循的核心推导集中在扩展篇 (D-4)(D-6)(D-7)。用于学术发表前，建议对 [C&S17] 与 [SQA22] 的具体公式做逐条查重比对（本文仅核对了其摘要级内容）。

---

## 7. 局限性与研究空白（诚实声明）

1. **本文与扩展篇均为纯理论文档**：(D-k)(E-k) 未编码实现，§4.3 二阶部分耗时是估算值；
2. **惯性参数是独立瓶颈**：HDQ 只承包几何层，$m_i,I_i$ 的辨识误差直接进入 $M,C$（§3.2）；项目 CoppeliaSim 场景未提取惯性参数；
3. **验证平台缺口**：现有实验走 `setJointTargetVelocity` 接口，力矩律 (5.2) 需 CoppeliaSim 力矩模式或实机；
4. **数值细节未处理**：单位 DQ 漂移在二阶通道的投影策略（主文档的归一化投影只处理 0 阶）、$\mathcal A_2$ 通道的数值尺度差异（$\ddot x$ 通道量级 $\sim\|\dot q\|^2$）等，实现时需补充；
5. **理论空白**：[P2] 的 H∞ 保证是运动学级的；级联"H∞ 外环 + 计算力矩内环"的整体 $L_2$ 增益界（考虑内环模型误差 $\Delta M,\Delta C$ 作为新扰动通道）尚无证明，是自然的后续理论问题。

---

## 8. 参考文献

1. **[P1]** A. Cohen, M. Shoham, *Hyper Dual Quaternions representation of rigid bodies kinematics*, Mechanism and Machine Theory 150 (2020) 103861.
2. **[P2]** L.F.C. Figueredo, B.V. Adorno, J.Y. Ishihara, *Robust H∞ kinematic control of manipulator robots using dual quaternion algebra*, Automatica 132 (2021) 109817.
3. **[C&S16]** A. Cohen, M. Shoham, *Application of Hyper-Dual Numbers to Multibody Kinematics*, ASME J. Mechanisms and Robotics 8(1) (2016) 011015.
4. **[C&S17]** A. Cohen, M. Shoham, *Application of hyper-dual numbers to rigid bodies equations of motion*, Mechanism and Machine Theory 111 (2017) 76–84.
5. **[F&A]** J.A. Fike, J.J. Alonso, *The Development of Hyper-Dual Numbers for Exact Second-Derivative Calculations*, AIAA Paper 2011-886, 2011.
6. **[SQA22]** F.F.A. Silva, J.J. Quiroz-Omaña, B.V. Adorno, *Dynamics of Mobile Manipulators Using Dual Quaternion Algebra*, ASME J. Mechanisms and Robotics 14(6) (2022) 061005.
7. **[Kha87]** O. Khatib, *A unified approach for motion and force control of robot manipulators: The operational space formulation*, IEEE J. Robotics and Automation 3(1), 1987.
8. **[LP17]** K.M. Lynch, F.C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017.
9. 主文档：`docs/数学理论与代码实现详解.md`；扩展篇：`docs/HDQ动力学建模扩展_Jdot与Hessian.md`（本项目）。
