# HDQ 在机械臂动力学建模中的扩展——$\dot J$、Hessian 的超对偶解析计算与电机反馈信息层级

> **文档定位**
>
> 本文是 `docs/数学理论与代码实现详解.md`（下称**主文档**）的动力学扩展篇。主文档的结论（§10）指出：HDQ 的真正潜力在**二阶量**——$\varepsilon\varepsilon^*$ 型高阶通道可携带加速度/曲率信息，指向动力学建模，超出原项目（一阶运动学控制）范围。本文将这一潜力落到严格的数学推导上：
>
> 1. 把项目现有的一阶 HDQ 链（[P1] 式(25)–(33)）**升阶**，得到 $\ddot{\boldsymbol x}$、$\dot{\boldsymbol\xi}$ 的一次传播算法；
> 2. 把 `pose_jacobian_hdq_fast` 的前缀/后缀结构**升阶**，得到配置空间 Hessian $\partial^2\boldsymbol x/\partial q_i\partial q_j$ 与 $\dot J$ 的 $O(n^2)$ 闭式解析公式；
> 3. 说明这些二阶量如何接入操作空间动力学，以及关节电机各层级反馈信息（角度/速度/加速度/力矩）在整条流水线中的位置与精度影响。
>
> **公式来源标注约定**（贯穿全文，详表见 §7）：
>
> - `[P1] 式(k)`：Cohen & Shoham, *Hyper Dual Quaternions representation of rigid bodies kinematics*, MMT 2020；
> - `[P2] 式(k)`：Figueredo, Adorno, Ishihara, *Robust H∞ kinematic control of manipulator robots using dual quaternion algebra*, Automatica 2021；
> - `主文档 (k)`：本项目主文档已有的推导（如 (5.1)(6.1)(6.2)）；
> - `[F&A]`：Fike & Alonso, *The Development of Hyper-Dual Numbers for Exact Second-Derivative Calculations*, AIAA 2011-886；
> - `[LP17]` / `[Kha87]` / `[SQA22]`：教科书/经典文献级标准结果（见 §8）；
> - **(D-k)**：**本文新推导**——在项目与文献中未现成给出、由本文完成证明的结果。
>
> **阅读前提**：熟悉主文档 §1–§6（四元数、DQ、HDQ 代数、链式传播）。全文沿用主文档记号：位姿 DQ $\boldsymbol x$，twist $\boldsymbol\xi$，左乘约定 $\dot{\boldsymbol x}=\tfrac12\boldsymbol\xi\boldsymbol x$（[P2] 式(1)），$\mathrm{vec}_6$ 取纯 DQ 的两个向量部。

---

## 目录

1. [工程应用流程：信息的输入、转换与输出](#1-工程应用流程信息的输入转换与输出)
2. [数学预备：幂零代数与自动微分的层级](#2-数学预备幂零代数与自动微分的层级)
3. [二阶时间链：一次传播同时求出 x、ẋ、ẍ 与 ξ̇](#3-二阶时间链一次传播同时求出-xẋẍ-与-ξ̇)
4. [Hessian 与 J̇ 的显式解析公式](#4-hessian-与-j̇-的显式解析公式)
5. [动力学接口：二阶量进入方程的位置](#5-动力学接口二阶量进入方程的位置)
6. [与项目代码的衔接点](#6-与项目代码的衔接点)
7. [公式来源总表](#7-公式来源总表)
8. [参考文献](#8-参考文献)

---

## 1. 工程应用流程：信息的输入、转换与输出

### 1.1 输入：关节电机反馈的信息层级

实际机械臂关节（电机 + 减速器 + 驱动器）能直接或间接提供四个层级的信息。下表为标准工程事实（非新推导）：

| 层级 | 物理量 | 典型来源 | 直接可测？ | 到关节量的转换 |
|---|---|---|---|---|
| **0 阶** | 关节角 $q_i$ | 增量/绝对编码器计数 $n_i$ | ✅ | $q_i=\dfrac{2\pi n_i}{\mathrm{CPR}\cdot N}-q_{i,\mathrm{off}}$（$N$ 为减速比，CPR 为每转计数） |
| **1 阶** | 关节速度 $\dot q_i$ | 驱动器内部估计，或对 $q_i$ 差分/观测器 | ⚠️ 半直接 | 后向差分 $\dot q_k\approx\dfrac{q_k-q_{k-1}}{\Delta t}$，或跟踪微分器/Kalman 观测器 |
| **2 阶** | 关节加速度 $\ddot q_i$ | 几乎不直接测量 | ❌ | 三条路线，见 §1.2 |
| **力矩** | 关节力矩 $\tau_i$ | 电机相电流 $i_q$（或关节力矩传感器） | ✅（含摩擦偏差） | $\tau_i = N K_t i_q - \tau_{f}(\dot q_i)$，摩擦模型 $\tau_f=F_c\,\mathrm{sgn}(\dot q)+F_v\dot q$ |

### 1.2 层级间的转换关系（尤其是 2 阶信息的三条获取路线）

设编码器角度噪声为零均值白噪声，方差 $\sigma_q^2$。差分对噪声的放大可由方差运算直接验证（标准结果）：

$$
\mathrm{Var}\bigl(\dot q_{\mathrm{diff}}\bigr)\approx\frac{2\sigma_q^2}{\Delta t^2},
\qquad
\mathrm{Var}\bigl(\ddot q_{\mathrm{diff}}\bigr)=\mathrm{Var}\Bigl(\tfrac{q_{k+1}-2q_k+q_{k-1}}{\Delta t^2}\Bigr)\approx\frac{6\sigma_q^2}{\Delta t^4}.
$$

$\Delta t\sim$ 毫秒级时 $1/\Delta t^4$ 是灾难性放大，因此**加速度层信息的获取方式本质上决定了二阶建模的精度上限**：

| 路线 | 公式 | 误差性质 |
|---|---|---|
| (a) 双重差分 | 上式 | 方差型（噪声放大 $6\sigma_q^2/\Delta t^4$），基本不可用 |
| (b) 状态观测器 | Kalman/龙伯格观测器融合 $q,\dot q$ 与模型 | 方差与偏差折中，有相位滞后 |
| (c) **由力矩经正动力学** | $\ddot{\boldsymbol q}=M(\boldsymbol q)^{-1}\bigl(\boldsymbol\tau-C(\boldsymbol q,\dot{\boldsymbol q})\dot{\boldsymbol q}-\boldsymbol g(\boldsymbol q)\bigr)$（[LP17] 第 8 章标准形式） | **偏差型**（误差来自模型参数 $\Delta M,\Delta C$，不随 $\Delta t$ 恶化） |

路线 (c) 说明：**力矩信息是加速度信息的低噪声等价物**——只要有动力学模型 $M,C,\boldsymbol g$，电流反馈即可替代不可测的 $\ddot q$。这就是动力学建模与运动学建模的信息闭环（§5.3 详述）。

### 1.3 输出与全流程框图

各层级输入分别激活正运动学链的不同"微分通道"（通道的数学含义见 §2）：

```
 电流 i_q ──τ=NKt·i−τf──▶ τ ──────────────┐
                                          │ 正动力学 M⁻¹(τ−Cq̇−g)
 编码器 n ──q=2πn/(CPR·N)──▶ q ──────┬────┼──▶ [0阶通道] DQ 链      →  x        (位姿)
                │                    │    │
                └─差分/观测器──▶ q̇ ──┼────┼──▶ [1阶通道] ε* 链      →  ẋ, ξ     (twist)
                                     │    ▼
                                     └── q̈ ──▶ [2阶通道] σ² 链      →  ẍ, ξ̇, J̇q̇ (加速度)
                                                        Hessian 链  →  H, J̇     (解析二阶)
 ─────────────────────────────────────────────────────────────────────────────
 输出去向：  x → H∞ 位姿反馈（[P2] 式(12)，项目现状）
            ξ → 速度前馈 / DQ-HDQ 交叉验证（项目现状）
            ξ̇, J̇ → 加速度分解控制、操作空间动力学、计算力矩（本文扩展）
            H  → 二阶灵敏度、误差预算、标定（本文扩展）
```

| 环节 | 输入 | 输出 | 作用 |
|---|---|---|---|
| 反馈解码 | 编码器计数、相电流 | $q,\dot q,\tau$ | 原始信号 → 关节空间物理量 |
| 0 阶 FK 链 | $q$ | $\boldsymbol x$ | 位姿；H∞ 误差 $\tilde{\boldsymbol z}$ 的输入（[P2] 式(8)(10)） |
| 1 阶 $\varepsilon^*$ 链 | $q,\dot q$ | $\dot{\boldsymbol x},\ \boldsymbol\xi$ | twist；速度前馈与验证（主文档 §6） |
| 2 阶 $\sigma^2$ 链（**本文 §3**） | $q,\dot q,\ddot q$ | $\ddot{\boldsymbol x},\ \dot{\boldsymbol\xi}$ | 任务空间加速度；$\dot J\dot q$ 免构造获取 |
| Hessian 链（**本文 §4**） | $q$（可加 $\dot q$） | $H_{ij},\ \dot J$ | 动力学控制律的解析系数；二阶灵敏度 |
| 动力学模型 | $q,\dot q,\tau$（或反向） | $\ddot q$（或 $\tau$） | 力矩⇄加速度双向换算（§5.3） |

### 1.4 各层级信息对建模准确性的影响

设各层反馈误差为 $\delta q,\delta\dot q,\delta\ddot q$。对输出量做一阶摄动分析：

**0 阶 → 位姿**。由主文档 (5.1)，$\partial\boldsymbol x/\partial q_i=\tfrac12\boldsymbol\jmath_i\boldsymbol x$（其中 $\boldsymbol\jmath_i=\overline{\mathrm{vec}}_6(J_i)$），故

$$
\delta\boldsymbol x\approx\frac12\,\overline{\mathrm{vec}}_6\bigl(J\,\delta\boldsymbol q\bigr)\,\boldsymbol x
\quad\Longrightarrow\quad
\|\delta\text{pose}\|\lesssim\|J\|_2\,\|\delta\boldsymbol q\|.
\tag{D-0}
$$

位姿误差只被 $\|J\|_2$（米/弧度量级）放大，故 0 阶信息最鲁棒——这解释了纯运动学控制（项目现状）为何工作良好。

**1 阶 → twist**。$\mathrm{vec}_6\,\boldsymbol\xi=J(\boldsymbol q)\dot{\boldsymbol q}$，摄动含**两项**：

$$
\delta\boldsymbol\xi = J\,\delta\dot{\boldsymbol q}
\;+\;\Bigl(\textstyle\sum_j \partial_j J\,\dot q_j\Bigr)\delta\boldsymbol q .
$$

注意第二项的系数正是 Hessian（$\partial_j J$ 的列即 §4 的 $h_{ij}$）——**要定量给出一阶输出的误差预算，已经需要二阶几何量**。这是 Hessian 除动力学之外的第二个用途。

**2 阶 → 加速度**。$\mathrm{vec}_6\,\dot{\boldsymbol\xi}=\dot J\dot{\boldsymbol q}+J\ddot{\boldsymbol q}$（(D-5)，见 §3.5），摄动被 $\delta\ddot q$ 主导。按 §1.2：差分路线方差爆炸，力矩路线把误差性质从"方差型"换成"偏差型"——**二阶建模准确性的关键不在数学层，而在选择哪条信息路线喂给 $\ddot q$ 通道**。

**一致性要求**：同一层级的信息必须来自同一时间基（同一滤波器/同一延迟），否则 $\varepsilon^*$ 通道与 $\sigma^2$ 通道之间出现相位错位，链式传播会把错位放大为系统性的 $\dot{\boldsymbol\xi}$ 偏差。

---

## 2. 数学预备：幂零代数与自动微分的层级

### 2.1 已有结构回顾

项目已有的代数塔（均见主文档 §2、§4）：

| 代数 | 幂零关系 | 通道含义 | 来源 |
|---|---|---|---|
| 对偶数 $\mathbb R[\varepsilon]/(\varepsilon^2)$ | $\varepsilon^2=0$ | $\varepsilon$：**平移**（几何用途，非微分） | [P1] §2.1 |
| DQ $\hat q=q_r+\varepsilon q_d$ | 同上 | 位姿 = 旋转 + $\varepsilon\cdot$平移 | [P1] 式(8) |
| HDQ $\breve q=\hat q+\varepsilon^*\dot{\hat q}$ | $\varepsilon^{*2}=0$ | $\varepsilon^*$：**一阶时间导数** | [P1] 式(25) |

HDQ 乘法（[P1] 式(14)）$\breve q_1\otimes\breve q_2=\hat q_{d1}\hat q_{d2}+\varepsilon^*(\hat q_{d1}\hat q_{hd2}+\hat q_{hd1}\hat q_{d2})$ 即 Leibniz 法则本身——这是自动微分能力的代数根源（主文档 §4.2）。

### 2.2 关键观察：两个幂零单位都已"占用"

要点：项目/[P1] 的 HDQ 中，$\varepsilon$ 承载平移、$\varepsilon^*$ 承载一阶导数，$\varepsilon\varepsilon^*$ 通道承载的是"平移分量的一阶导数"，**并不含二阶时间导数**。因此二阶量（$\ddot{\boldsymbol x}$、Hessian）必须引入**新的**微分方向。这不是实现缺陷，而是 [P1] 式(25) 运动学 HDQ 的结构性事实。

### 2.3 三种二阶扩展方案

| 方案 | 代数 | 得到什么 | 复杂度 | 本文位置 |
|---|---|---|---|---|
| **A. 二阶节（jet）代数** | $\mathcal A_2=\widehat{\mathbb H}\otimes\mathbb R[\sigma]/(\sigma^3)$ | 沿真实时间轨迹的 $\ddot{\boldsymbol x},\dot{\boldsymbol\xi}$，一次链传播 | $O(n)$ | §3 |
| **B. 双超对偶（bi-dual）** | $\widehat{\mathbb H}\otimes\mathbb R[\sigma_1,\sigma_2]/(\sigma_1^2,\sigma_2^2)$ | 任意混合偏导 $\partial^2\boldsymbol x/\partial q_i\partial q_j$（数值 AD 式） | $O(n^3)$ 全 Hessian | §4.4（验证用） |
| **C. 前缀/后缀显式展开** | 纯 DQ 运算（`hdq_fast` 升阶） | 全 Hessian 与 $\dot J$ 的**闭式解析** | $O(n^2)$ | §4.1–4.3（主推） |

方案 B 是 Fike & Alonso [F&A] 超对偶数二阶微分思想在 DQ 上的直接移植；方案 A、C 的具体公式为本文新推导。三者互为验证（§4.4）。

记号约定：$\widehat{\mathbb H}$ 表示对偶四元数代数；为避免与 [P1] 的 $\varepsilon^{*2}=0$ 冲突，新微分单位记作 $\sigma$（$\sigma^3=0$）与 $\sigma_1,\sigma_2$。截断 $\sigma^2$ 项即退化回 [P1] 的 $\varepsilon^*$。

---

## 3. 二阶时间链：一次传播同时求出 x、ẋ、ẍ 与 ξ̇

### 3.1 二阶节代数 $\mathcal A_2$

**定义**：$\mathcal A_2\triangleq\widehat{\mathbb H}[\sigma]/(\sigma^3)$，元素

$$
\breve a = a_0+\sigma a_1+\tfrac12\sigma^2 a_2,\qquad a_0,a_1,a_2\in\widehat{\mathbb H}\ (\text{各 8 维}),
$$

乘法由 DQ 乘法的双线性延拓加 $\sigma^3=0$ 唯一确定：

$$
\breve a\,\breve b
= a_0b_0
+\sigma\,( a_0b_1+a_1b_0)
+\tfrac12\sigma^2\,( a_0b_2+2a_1b_1+a_2b_0).
\tag{3.1}
$$

（(3.1) 是截断多项式环的标准乘法，属教科书事实；系数 $\tfrac12$ 的引入使 $\sigma^2$ 通道直接存放 $\ddot a$ 而非 $\tfrac12\ddot a$。）

### 3.2 提升算子与二阶 Leibniz 定理

**定义（二阶提升）**：对光滑 DQ 曲线 $t\mapsto a(t)$，

$$
T^2 a \triangleq a+\sigma\dot a+\tfrac12\sigma^2\ddot a\;\in\mathcal A_2 .
$$

> **定理 (D-1)（提升是乘法同态）**：对任意光滑 DQ 曲线 $a(t),b(t)$，
>
> $$
> T^2(ab)=T^2a\cdot T^2b .
> $$
>
> **证明**：DQ 乘法是 $\mathbb R^8\times\mathbb R^8\to\mathbb R^8$ 的双线性映射，故 Leibniz 法则对它成立（无需交换性）：
> $\ (ab)^{\cdot}=\dot ab+a\dot b$，$\ (ab)^{\cdot\cdot}=\ddot ab+2\dot a\dot b+a\ddot b$。
> 代入 (3.1) 逐通道比对：$\sigma^0$ 通道 $a_0b_0=ab$；$\sigma^1$ 通道 $a\dot b+\dot ab=(ab)^\cdot$；$\sigma^2$ 通道 $\tfrac12(a\ddot b+2\dot a\dot b+\ddot ab)=\tfrac12(ab)^{\cdot\cdot}$。∎

(D-1) 是主文档 §6.1 一阶链式法则（[P1] 式(28)）的严格升阶：**$n$ 个因子的 $\mathcal A_2$ 乘积，其 $\sigma$ 通道自动累加一阶 Leibniz 和（即 [P1] 式(28) 的 $\Sigma$），$\sigma^2$ 通道自动累加二阶 Leibniz 和**

$$
(x_1\cdots x_n)^{\cdot\cdot}
=\sum_{i}\Bigl(\prod_{j<i}x_j\Bigr)\ddot x_i\Bigl(\prod_{k>i}x_k\Bigr)
+2\sum_{i<j}\Bigl(\prod_{a<i}x_a\Bigr)\dot x_i\Bigl(\prod_{i<b<j}x_b\Bigr)\dot x_j\Bigl(\prod_{c>j}x_c\Bigr),
\tag{D-1'}
$$

而实现时**完全不需要写出这个双重求和**——它被乘法规则 (3.1) 隐式完成，与主文档 §6.4 "求和结构由乘法法则隐式完成"的观察一脉相承。

### 3.3 单关节因子的二阶展开

POE 关节因子（主文档 (6.2)，[P1] 式(29) 左乘版）：$x_i(t)=$ 螺旋指数的 DQ 形式，$\bar S_i\triangleq\overline{\mathrm{vec}}_6(S_i)$ 为**常值**纯 DQ 螺旋轴，

$$
\dot x_i=\tfrac12\,\dot\theta_i\,\bar S_i\,x_i. \tag{主文档 6.2}
$$

对时间再求导（$\bar S_i$ 常值）：

> **命题 (D-2)（关节因子的二阶导）**：
>
> $$
> \ddot x_i=\tfrac12\,\ddot\theta_i\,\bar S_i\,x_i+\tfrac14\,\dot\theta_i^2\,\bar S_i^2\,x_i,
> $$
>
> 且 $\bar S_i^2$ 是**对偶标量**（实部、对偶部的向量分量全为零）：写 $\bar S_i=\bar\omega_i+\varepsilon\bar v_i$（两个纯四元数），由纯四元数恒等式 $ab+ba=-2\langle a,b\rangle$ 得
>
> $$
> \bar S_i^2=\bar\omega_i^2+\varepsilon(\bar\omega_i\bar v_i+\bar v_i\bar\omega_i)
> =-\|\omega_i\|^2-2\varepsilon\langle\omega_i,v_i\rangle .
> $$
>
> 特别地：**转动关节**（单位轴 $\|\omega_i\|=1$，Plücker 条件 $v_i=o_i\times\omega_i\Rightarrow\langle\omega_i,v_i\rangle=0$）有 $\bar S_i^2=-1$，即 $\ddot x_i=\tfrac12\ddot\theta_i\bar S_ix_i-\tfrac14\dot\theta_i^2x_i$；**平动关节**（$\omega_i=0$）有 $\bar S_i^2=0$，即 $\ddot x_i=\tfrac12\ddot\theta_i\bar S_ix_i$。
>
> **证明**：对 (主文档 6.2) 直接求导并回代自身；$\bar S_i^2$ 的化简用 $\varepsilon^2=0$ 与纯四元数反对易恒等式。∎

DH 参数化的对应结果（与 `hdq_math.py` 的 $R_z,T_z$ 因子对齐）：$R_z(\theta)=\exp(\hat k\theta/2)$ 给出 $R_z''=-\tfrac14R_z$；$T_z(d)=1+\varepsilon\tfrac d2\hat k$ 给出 $T_z''=0$。故转动 DH 链节 $X_i''=-\tfrac14X_i$，平动链节 $X_i''=0$——与 (D-2) 的 POE 结论一致。**(D-2)**

### 3.4 整链传播与 $\dot{\boldsymbol\xi}$ 提取

> **算法（二阶 HDQ 链，方案 A）**：给定 $(\boldsymbol q,\dot{\boldsymbol q},\ddot{\boldsymbol q})$，构造每个关节的二阶因子
>
> $$
> \breve X_i=x_i+\sigma\bigl(\tfrac12\dot\theta_i\bar S_ix_i\bigr)
> +\tfrac12\sigma^2\bigl(\tfrac12\ddot\theta_i\bar S_ix_i+\tfrac14\dot\theta_i^2\bar S_i^2x_i\bigr)\in\mathcal A_2,
> $$
>
> 按 (3.1) 累乘 $\breve X=\breve X_1\cdots\breve X_n\cdot(M+0\sigma+0\sigma^2)$。由 (D-1)，三个通道**同时**给出
>
> $$
> \breve X = \boldsymbol x+\sigma\dot{\boldsymbol x}+\tfrac12\sigma^2\ddot{\boldsymbol x}.
> $$
>
> 代价：$n$ 次 $\mathcal A_2$ 乘法，每次含 6 次 DQ 乘（式 (3.1) 的 $a_0b_0,a_0b_1,a_1b_0,a_0b_2,a_1b_1,a_2b_0$），总 $O(n)$——与一阶 HDQ 链同阶，常数因子约 2 倍。

twist 导数的提取需要一个引理：

> **引理 (D-3)（单位 DQ 的共轭导数）**：$\boldsymbol x\boldsymbol x^*=1\Rightarrow\dot{\boldsymbol x}^*=-\boldsymbol x^*\dot{\boldsymbol x}\boldsymbol x^*$。
> **证明**：对 $\boldsymbol x\boldsymbol x^*=1$ 求导得 $\dot{\boldsymbol x}\boldsymbol x^*+\boldsymbol x\dot{\boldsymbol x}^*=0$，左乘 $\boldsymbol x^*$。∎

> **定理 (D-4)（twist 导数提取公式）**：在左乘约定 $\boldsymbol\xi=2\dot{\boldsymbol x}\boldsymbol x^*$（主文档 (6.1)）下，
>
> $$
> \boxed{\ \dot{\boldsymbol\xi}=2\ddot{\boldsymbol x}\boldsymbol x^{*}-\tfrac12\boldsymbol\xi^{2}\ }
> $$
>
> 且修正项 $\boldsymbol\xi^2$ 是对偶标量：$\boldsymbol\xi=\omega+\varepsilon v\Rightarrow\boldsymbol\xi^2=-\|\omega\|^2-2\varepsilon\langle\omega,v\rangle$。因此在 $\mathrm{vec}_6$（只取向量部）层面
>
> $$
> \mathrm{vec}_6\,\dot{\boldsymbol\xi}=\mathrm{vec}_6\bigl(2\ddot{\boldsymbol x}\boldsymbol x^{*}\bigr).
> $$
>
> **证明**：$\dot{\boldsymbol\xi}=2\ddot{\boldsymbol x}\boldsymbol x^*+2\dot{\boldsymbol x}\dot{\boldsymbol x}^*
> \overset{(\mathrm{D}\text{-}3)}{=}2\ddot{\boldsymbol x}\boldsymbol x^*-2\dot{\boldsymbol x}\boldsymbol x^*\dot{\boldsymbol x}\boldsymbol x^*
> =2\ddot{\boldsymbol x}\boldsymbol x^*-\tfrac12(2\dot{\boldsymbol x}\boldsymbol x^*)^2$。
> $\boldsymbol\xi^2$ 的对偶标量性同 (D-2) 的 $\bar S^2$ 计算。∎

**解读**：$-\tfrac12\boldsymbol\xi^2$ 是群流形上的"向心修正"——它只影响标量通道，保证 $\dot{\boldsymbol\xi}$ 保持纯 DQ（twist 的导数仍是 twist 型元素）。工程实现只需 $\mathrm{vec}_6(2\ddot{\boldsymbol x}\boldsymbol x^*)$，形式与一阶提取 `spatial_twist_from_hdq`（主文档 (6.1)）完全平行：一阶取 $\sigma$ 通道乘 $\boldsymbol x^*$，二阶取 $\sigma^2$ 通道乘 $\boldsymbol x^*$。

### 3.5 $\dot J\dot{\boldsymbol q}$ 的免构造获取

对 $\mathrm{vec}_6\,\boldsymbol\xi=J(\boldsymbol q)\dot{\boldsymbol q}$（[P2] 式(2)(3) 的矩阵形式）求时间导数：

$$
\mathrm{vec}_6\,\dot{\boldsymbol\xi}=\dot J\dot{\boldsymbol q}+J\ddot{\boldsymbol q}.
\tag{D-5}
$$

> **推论 (D-5)（播种 $\ddot{\boldsymbol q}=0$）**：以 $(\boldsymbol q,\dot{\boldsymbol q},\ddot{\boldsymbol q}\!=\!0)$ 运行 §3.4 的二阶链，则
>
> $$
> \mathrm{vec}_6\bigl(2\ddot{\boldsymbol x}\boldsymbol x^*\bigr)=\dot J\dot{\boldsymbol q},
> $$
>
> 即**不构造 $\dot J$ 矩阵本身**，一次 $O(n)$ 传播直接得到动力学中最常用的组合量 $\dot J\dot{\boldsymbol q}$（见 §5）。

这与主文档 §6 "不构造雅可比的速度求解"哲学严格对偶：一阶链免构造 $J$ 得 $J\dot q$；二阶链免构造 $\dot J$ 得 $\dot J\dot q$。多数动力学控制律（§5.1–5.2）只需要 $\dot J\dot{\boldsymbol q}$ 这个 6 维向量而非 $\dot J$ 全矩阵，故 (D-5) 通常已经够用；需要全矩阵时用 §4。

---

## 4. Hessian 与 J̇ 的显式解析公式

本节把 `pose_jacobian_hdq_fast` 的前缀/后缀技术（主文档 §5.3(iv)）升阶到二阶，得到闭式 Hessian。这是方案 C。

### 4.1 配置空间 Hessian $\partial^2\boldsymbol x/\partial q_i\partial q_j$

记链 $\boldsymbol x=X_1X_2\cdots X_n$，$X_k$ 只依赖 $q_k$。定义（与代码一致的）前缀/后缀积

$$
P_0\triangleq1,\quad P_k\triangleq X_1\cdots X_k;\qquad
S_{n+1}\triangleq1,\quad S_k\triangleq X_k\cdots X_n .
$$

一阶结果（主文档 (5.1) 的链式形式）：$\partial_i\boldsymbol x=P_{i-1}X_i'S_{i+1}$，其中 $X_i'\triangleq dX_i/dq_i$ 由 `dq_standard_dh_and_derivative` 解析给出。

> **定理 (D-6)（配置空间 Hessian 闭式）**：对 $i<j$，
>
> $$
> H_{ij}\triangleq\frac{\partial^2\boldsymbol x}{\partial q_i\partial q_j}
> =P_{i-1}\,X_i'\;\underbrace{\bigl(P_i^{*}P_{j-1}\bigr)}_{=X_{i+1}\cdots X_{j-1}}\;X_j'\,S_{j+1},
> \qquad
> H_{ii}=P_{i-1}\,X_i''\,S_{i+1},
> $$
>
> 其中中段积利用**单位 DQ 共轭即逆**（$P_i^*P_i=1$）由已缓存的前缀积拼出：$P_i^{*}P_{j-1}=X_{i+1}\cdots X_{j-1}$（$j=i+1$ 时为 $1$，自动退化正确）。二阶单链节导数为解析常式（§3.3）：
>
> $$
> X_i''=\begin{cases}-\tfrac14X_i,&\text{转动（DH 或单位轴 POE）},\\[2pt]0,&\text{平动}.\end{cases}
> $$
>
> **证明**：$X_k$ 只依赖 $q_k$，对 $\partial_i\boldsymbol x=X_1\cdots X_i'\cdots X_n$ 再对 $q_j$（$j>i$）求偏导，仅 $X_j$ 因子被替换为 $X_j'$；$i=j$ 时 $X_i'$ 被替换为 $X_i''$。中段恒等式由单位 DQ 群性质 $P_i^{-1}=P_i^*$ 得出。对称性 $H_{ij}=H_{ji}$ 由混合偏导可交换（各因子光滑）保证。∎

**复杂度**：$P,S$ 各 $n$ 次 DQ 乘预计算；每个 $H_{ij}$ 仅需 $O(1)$ 次 DQ 乘（一次共轭乘拼中段 + 4 次链乘）。全 Hessian $\tfrac{n(n+1)}2$ 项共 $O(n^2)$ 次 DQ 乘——**与输出规模同阶，渐近最优**。

### 4.2 任务空间二阶量与 Lie 括号注记

任务空间的一阶对象是雅可比列 $\boldsymbol\jmath_i=2(\partial_i\boldsymbol x)\boldsymbol x^*$（主文档 (5.1)）。其对 $q_j$ 的偏导：

> **定理 (D-7)（任务空间 Hessian）**：
>
> $$
> h_{ij}\triangleq\frac{\partial\boldsymbol\jmath_i}{\partial q_j}
> =2H_{ij}\,\boldsymbol x^{*}-\tfrac12\,\boldsymbol\jmath_i\boldsymbol\jmath_j .
> $$
>
> **证明**：$\partial_j\boldsymbol\jmath_i=2H_{ij}\boldsymbol x^*+2(\partial_i\boldsymbol x)(\partial_j\boldsymbol x)^*$。对 (D-3) 作参数化推广（把 $t$ 换成 $q_j$）得 $(\partial_j\boldsymbol x)^*=-\boldsymbol x^*(\partial_j\boldsymbol x)\boldsymbol x^*$，代入第二项：
> $2(\partial_i\boldsymbol x)(\partial_j\boldsymbol x)^*=-2(\partial_i\boldsymbol x)\boldsymbol x^*(\partial_j\boldsymbol x)\boldsymbol x^*=-\tfrac12\boldsymbol\jmath_i\boldsymbol\jmath_j$。∎

> **注记 (D-8)（不对称部分是 Lie 括号）**：虽然 $H_{ij}=H_{ji}$，但 $h_{ij}\neq h_{ji}$，其差
>
> $$
> h_{ij}-h_{ji}=\tfrac12\bigl(\boldsymbol\jmath_j\boldsymbol\jmath_i-\boldsymbol\jmath_i\boldsymbol\jmath_j\bigr)
> =-\tfrac12[\boldsymbol\jmath_i,\boldsymbol\jmath_j]
> $$
>
> 恰为两根关节螺旋轴的（负半）Lie 括号。数学上这正是群 $\mathrm{Spin}(3)\ltimes\mathbb R^3$ 非交换性的体现：任务空间"二阶导"不是平坦空间的对称 Hessian，而是带挠率的协变对象。对数学系读者：这与李群上左不变向量场的括号运算完全同源，也与螺旋理论中 $\mathrm{ad}_{\xi_1}\xi_2$ 的出现方式一致（[LP17] 第 8 章的 $\mathrm{ad}$ 项）。**(D-8)**

### 4.3 $\dot J$ 的列公式

> **推论 (D-9)（$\dot J$ 闭式）**：第 $i$ 列
>
> $$
> \dot{\boldsymbol\jmath}_i=\sum_{j=1}^n h_{ij}\dot q_j
> =2\Bigl(\sum_j H_{ij}\dot q_j\Bigr)\boldsymbol x^{*}-\tfrac12\,\boldsymbol\jmath_i\,\boldsymbol\xi,
> \qquad
> \dot J=\bigl[\mathrm{vec}_6\,\dot{\boldsymbol\jmath}_1\ \cdots\ \mathrm{vec}_6\,\dot{\boldsymbol\jmath}_n\bigr],
> $$
>
> 其中第二个等号用了 $\sum_j\boldsymbol\jmath_j\dot q_j=\boldsymbol\xi$（[P2] 式(2)）。
>
> **一致性校验**：将 $\dot{\boldsymbol\jmath}_i$ 乘 $\dot q_i$ 对 $i$ 求和，
> $\sum_i\dot{\boldsymbol\jmath}_i\dot q_i=2\bigl(\sum_{i,j}H_{ij}\dot q_i\dot q_j\bigr)\boldsymbol x^*-\tfrac12\boldsymbol\xi^2$。
> 另一方面按 (D-1') 取 $\ddot q=0$ 有 $\ddot{\boldsymbol x}\big|_{\ddot q=0}=\sum_{i,j}H_{ij}\dot q_i\dot q_j$，代入 (D-4) 得 $\dot{\boldsymbol\xi}\big|_{\ddot q=0}=2\ddot{\boldsymbol x}\boldsymbol x^*-\tfrac12\boldsymbol\xi^2=\sum_i\dot{\boldsymbol\jmath}_i\dot q_i$，与 (D-5) 的 $\dot J\dot q$ 完全吻合 ✓。**(D-9)**

**实现路线对比**（对齐主文档 §9.2 的表格风格，标注理论复杂度，未实测）：

| 目标量 | 方法 | 复杂度 | 精度 |
|---|---|---|---|
| $\dot J\dot{\boldsymbol q}$（6 维向量） | §3 二阶链，播种 $\ddot q=0$ | $O(n)$ | 机器精度 |
| $\dot J$ 全矩阵 | §4.1–4.3 前缀/后缀 Hessian | $O(n^2)$ | 机器精度 |
| $\dot J$ 全矩阵 | 中心差分 $\frac{J(q+\epsilon\dot q\,dt)-J(q-\epsilon\dot q\,dt)}{2\epsilon}$ | $O(n^2)$ | $O(\epsilon^2)$ 截断误差 |
| 全 Hessian | 方案 B 逐对播种（§4.4） | $O(n^3)$ | 机器精度（验证用） |

### 4.4 方案 B：双超对偶播种（数值验证器）

将 [F&A] 超对偶数的二阶微分思想移植到 DQ 系数：取代数 $\widehat{\mathbb H}[\sigma_1,\sigma_2]/(\sigma_1^2,\sigma_2^2)$，元素含 4 个 DQ 通道 $(1,\sigma_1,\sigma_2,\sigma_1\sigma_2)$，乘法按双线性展开并用 $\sigma_1^2=\sigma_2^2=0$ 截断。对指标对 $(i,j)$ 播种

$$
\breve X_k = X_k+\sigma_1\,[k\!=\!i]\,X_k'+\sigma_2\,[k\!=\!j]\,X_k'+\sigma_1\sigma_2\,[k\!=\!i\!=\!j]\,X_k'',
$$

（$[\cdot]$ 为 Iverson 括号）则整链乘积的 $\sigma_1\sigma_2$ 通道**恰为** $H_{ij}$——这是 [F&A] "超对偶数取二阶导无截断误差"结论在 DQ 系数上的逐字翻译，证明与 (D-1) 同型（双线性 + Leibniz）。每对 $(i,j)$ 一次 $O(n)$ 链传播，全 Hessian $O(n^3)$：**不作为生产路线，只作为 (D-6) 的独立数值验证器**——地位与主文档中逐列播种 `pose_jacobian_hdq` 之于 `hdq_fast` 完全相同（慢 7.6× 但互证正确性，主文档 §9.2）。

---

## 5. 动力学接口：二阶量进入方程的位置

本节说明 §3–§4 的产出如何被动力学消费。动力学方程本身是标准结果（[Kha87][LP17]），本文的贡献是给出其中几何系数的 HDQ 解析获取方式。

### 5.1 关节空间动力学与逆动力学

刚体机械臂关节空间模型（Euler–Lagrange 标准形式，[LP17] 第 8 章）：

$$
M(\boldsymbol q)\ddot{\boldsymbol q}+C(\boldsymbol q,\dot{\boldsymbol q})\dot{\boldsymbol q}+\boldsymbol g(\boldsymbol q)=\boldsymbol\tau .
\tag{5.1}
$$

计算力矩控制律的教科书朴素形式（[LP17]）：

$$
\boldsymbol\tau=M(\boldsymbol q)\ddot{\boldsymbol q}_{\mathrm{ref}}+C\dot{\boldsymbol q}+\boldsymbol g,
\qquad
\ddot{\boldsymbol q}_{\mathrm{ref}}=J^{+}\bigl(\dot{\boldsymbol\xi}_{d}+K_d\,\delta\boldsymbol\xi+K_p\,\delta\boldsymbol z-\dot J\dot{\boldsymbol q}\bigr).
\tag{5.2}
$$

> **注记（表示体系一致性修正，重要）**：朴素形式 (5.2) 隐含假设任务空间坐标取自**平坦向量空间**（如末端点位置 + 最小姿态参数），此时 $\delta\boldsymbol\xi=\boldsymbol\xi-\boldsymbol\xi_d$、$\delta\boldsymbol z$ 可以直接逐分量相减。但本文全程使用的表示体系是**空间 twist 表示**：$\boldsymbol\xi=2\dot{\boldsymbol x}\boldsymbol x^*$（左乘约定，主文档 (6.1)），$J$ 是 [P2] 式(3) 的空间雅可比，$\dot J\dot{\boldsymbol q}$ 与 (D-4) 的 $\dot{\boldsymbol\xi}$ 也全部定义在**当前位姿 $\boldsymbol x$ 处、绕基座原点**的切空间；而 $\boldsymbol\xi_d,\dot{\boldsymbol\xi}_d$ 是沿期望轨迹在**期望位姿 $\boldsymbol x_d$ 处**定义的量。两组量属于李群 $\mathrm{Spin}(3)\ltimes\mathbb R^3$ 上不同点的切空间，朴素差 $\boldsymbol\xi-\boldsymbol\xi_d$ 在几何上不一致：与几何一致定义之差为 $(\mathrm{Ad}_{\tilde{\boldsymbol x}}-\mathrm{id})\boldsymbol\xi_d$（$\mathrm{Ad}_{\boldsymbol h}(\boldsymbol a)\triangleq\boldsymbol h\boldsymbol a\boldsymbol h^*$，$\tilde{\boldsymbol x}=\boldsymbol x\boldsymbol x_d^*$ 为 [P2] 式(8) 的空间误差），该伪项随 $\|\boldsymbol\xi_d\|$ 线性放大，直接乘 $K_d$ 进入力矩后在高速工况不可忽略。**与前文 §3–§4 同一表示体系的一致写法**是先把期望量经 $\mathrm{Ad}_{\tilde{\boldsymbol x}}$ 搬运到当前切空间再作差：
>
> $$
> \ddot{\boldsymbol q}_{\mathrm{ref}}=J^{+}\Bigl(\underbrace{\mathrm{vec}_6\bigl(\mathrm{Ad}_{\tilde{\boldsymbol x}}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\,\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol\xi_d\bigr)}_{\text{几何一致前馈（搬运 + 输运修正）}}-K_d\,e_\xi-k_p\,A^{\top}(\tilde{\boldsymbol x})\,e_z-\dot J\dot{\boldsymbol q}\Bigr),
> \tag{5.2$'$}
> $$
>
> 其中 $\tilde{\boldsymbol\xi}\triangleq2\dot{\tilde{\boldsymbol x}}\tilde{\boldsymbol x}^*$ 为几何一致误差 twist、$e_\xi\triangleq\mathrm{vec}_6\,\tilde{\boldsymbol\xi}=\mathrm{vec}_6(\boldsymbol\xi-\mathrm{Ad}_{\tilde{\boldsymbol x}}\boldsymbol\xi_d)$（无扰时）、$e_z=[\mathrm{vec}_3\,\mathcal O;\mathrm{vec}_3\,\mathcal T]$ 沿用 [P2] 式(11)，$A(\tilde{\boldsymbol x})$ 为输出误差运动学矩阵（$\dot e_z=A e_\xi$ 的闭式系数），$\mathrm{ad}$ 输运项补偿 $\mathrm{Ad}$ 搬运随误差运动的时间变化。此时 (5.2$'$) 括号内所有 twist 级量——$\mathrm{Ad}_{\tilde{\boldsymbol x}}\dot{\boldsymbol\xi}_d$、$e_\xi$、$\dot J\dot{\boldsymbol q}$、$J\ddot{\boldsymbol q}$——**全部位于当前位姿处的同一空间表示**，与 §3 二阶链输出 (D-4)(D-5) 及 §4 的 $\dot J$ 列公式 (D-9) 直接兼容，不再混用切空间。完整推导（$A(\tilde{\boldsymbol x})$ 闭式、前馈项相消机制、闭环 Lyapunov/ISS 证明）见第四层文档《HDQ 动力学误差体系重构》的 (F-1)–(F-7)。低速小误差（$\tilde{\boldsymbol x}\to1$）时 $\mathrm{Ad}_{\tilde{\boldsymbol x}}\to\mathrm{id}$、$\mathrm{ad}$ 项为二阶小量，(5.2$'$) 退化回 (5.2)——这解释了朴素形式在低速验证中"看起来可用"的原因。

(5.2$'$) 的任务空间→关节空间加速度分解仍是标准操作（[Kha87]）；其中的**几何系数** $\dot J\dot{\boldsymbol q}$ 正是 (D-5) 的一次 $O(n)$ 链输出，无需差分、无需构造 $\dot J$。误差量 $e_z$ 的 0 阶定义沿用 [P2] 式(10)(11)，$e_\xi$ 为其几何一致的一阶升级（第四层文档 (F-2)），使 (5.2$'$) 与项目现有 H∞ 运动学外环共享同一套误差基座——运动学环（[P2] 式(12)）输出 $\dot q_{\mathrm{cmd}}$ 作外环，(5.2$'$) 作内环，即级联结构。

### 5.2 操作空间动力学

Khatib 操作空间公式（[Kha87]）：

$$
\Lambda(\boldsymbol q)\,\dot{\boldsymbol\xi}+\mu(\boldsymbol q,\dot{\boldsymbol q})+\boldsymbol p(\boldsymbol q)=\boldsymbol F,
\qquad
\Lambda=(JM^{-1}J^{T})^{-1},\quad
\mu=\Lambda\bigl(JM^{-1}C\dot{\boldsymbol q}-\dot J\dot{\boldsymbol q}\bigr).
\tag{5.3}
$$

HDQ 提供的量：$J$（`hdq_fast`，主文档 §5.3(iv)）、$\dot J\dot{\boldsymbol q}$（(D-5)）、$\dot{\boldsymbol\xi}$ 的测量估计（§3.4 二阶链喂入 $\ddot q$ 估计值）。惯性量 $M,C,\boldsymbol g$ 来自连杆质量参数，属 HDQ 几何层之外的输入。

### 5.3 电机反馈信息的双向闭环（呼应 §1.2）

式 (5.1) 是**力矩层与加速度层信息互换的桥**：

- **正向（力矩→加速度）**：$\ddot{\boldsymbol q}=M^{-1}(\boldsymbol\tau-C\dot{\boldsymbol q}-\boldsymbol g)$。由电流反馈算出 $\ddot q$，喂给 §3.4 二阶链得 $\dot{\boldsymbol\xi}$——绕开双重差分的 $6\sigma_q^2/\Delta t^4$ 噪声灾难（§1.2），代价是引入模型偏差 $\Delta M,\Delta C$；
- **反向（加速度→力矩）**：给定期望 $\ddot q_{\mathrm{ref}}$（由 (5.2) 的任务空间分解得到，消费 (D-5) 的 $\dot J\dot q$），(5.1) 直接给出前馈力矩。

**对建模准确性的分层结论**（综合 §1.4）：

| 反馈层 | 决定的建模能力 | 误差敏感度 |
|---|---|---|
| 仅 $q$ | 位姿级：FK、H∞ 位姿反馈（项目现状即可闭环） | 低（(D-0)，$\|J\|$ 线性放大） |
| $q+\dot q$ | twist 级：速度前馈、一阶 HDQ 链验证 | 中（差分噪声 $2\sigma_q^2/\Delta t^2$；且误差预算需 Hessian，§1.4） |
| $q+\dot q+\ddot q$（或 $\tau$+模型） | 加速度级：$\dot{\boldsymbol\xi}$、操作空间动力学 (5.3)、计算力矩 (5.2) | 高——路线选择决定误差性质：差分=方差型爆炸，力矩换算=模型偏差型 |
| $+\tau$ 独立测量 | 交互力级：外力估计 $\hat F_{\mathrm{ext}}=\boldsymbol\tau-M\ddot{\boldsymbol q}-C\dot{\boldsymbol q}-\boldsymbol g$、导纳/阻抗控制 | 依赖全链模型精度 |

结论呼应主文档 §10：一阶运动学控制只消费前两层，HDQ 无优势；**从第三层起，(D-4)(D-5)(D-6) 这些二阶解析量成为刚需，HDQ 的幂零自动微分结构才真正兑现价值**——二阶几何量既无闭式几何捷径（geometric 雅可比的 $[z;o\times z]$ 技巧不升阶，对应量含 Lie 括号 (D-8)），差分又不可靠（§1.2），解析链式传播是唯一同时满足"精确 + $O(n)/O(n^2)$"的路线。

### 5.4 关于Jacobi注记

操作空间方程在**任意速度表象下形式不变**，但三件套必须共轭自洽（功率配对 \(F^T\xi\) 不变）：

$$
\xi'=B\xi\ \Longrightarrow\ J'=BJ,\quad \Lambda'=B^{-T}\Lambda B^{-1},\quad F'=B^{-T}F .
$$

由此得到三条判定：

1. **扩展篇 (5.3) 用项目的 $J$ 是自洽合法的**：\(\Lambda,\mu\) 按同一表象算，(D-4) 提取的 \(\dot{\boldsymbol\xi}\) 恰是空间 twist 的导数，全套对得上。但此时 \(F\) 的物理解读是 **[绕基座原点的力矩; 力]**，**不是**末端点上的 (力矩; 力)——接触力/阻抗应用时必须经 \(B^{-T}\) 换算，否则力的作用点错了；
2. **不能混搭**：尤其加速度层——\(\xi'=B\xi\Rightarrow\dot\xi'=B\dot\xi+\dot B\xi\)，\(\dot B\) 含 \(\dot p\)，两表象的"加速度"相差一个速度二次项。若用项目 \(J\) 算 \(\Lambda\)、却塞入末端点表象的 \(\dot\xi\)，会产生随速度平方增长的系统性偏差（低速看不出、高速正好在动力学最需要的工况爆发）；
3. **不受影响的量**：关节空间 \(M,C,\boldsymbol g\) 与表象无关（动能 \(\tfrac12\dot q^TM\dot q\) 是标量不变量，\(B\) 对 \(\Lambda\) 是合同变换），故第三份文档 (2.1)(2.2)(E-3)(E-4) 的装配结论不受此问题波及；奇异位形集合也相同（\(B\) 恒可逆，\(\mathrm{rank}\,J'=\mathrm{rank}\,J\)）。


---

## 6. 与项目代码的衔接点

本文为纯理论文档，未修改代码。若实现，各推导与现有代码的自然衔接如下（供参考）：

| 推导 | 升阶自（现有代码） | 改动性质 |
|---|---|---|
| (3.1) $\mathcal A_2$ 乘法 | `HDQ.__mul__`（`hdq_math.py` L93–97，2 通道） | 增至 3 通道、6 次 `dq_mul` |
| (D-2) 二阶关节因子 | `hdq_from_spatial_screw`（L234–252） | 增加 $\sigma^2$ 通道 $\tfrac12\ddot\theta\bar Sx+\tfrac14\dot\theta^2\bar S^2x$，其中 $\bar S^2$ 为对偶标量常数 |
| §3.4 二阶整链 | `hdq_poe_chain_from_model`（L268–297） | 循环体不变，换 $\mathcal A_2$ 因子 |
| (D-4) $\dot\xi$ 提取 | `spatial_twist_from_hdq`（L126–138） | 平行新增 `spatial_twist_rate_from_...`：`2*dq_mul(X.h2, dq_conj(X.dq))` 取 vec6 |
| (D-6)(D-9) Hessian/$\dot J$ | `pose_jacobian_hdq_fast`（`robot_dh.py` L218–287） | 复用 prefix/suffix 数组，双重循环拼 $H_{ij}$；$X_i''$ 由 $X_i''=-\tfrac14X_i$（转动）直接给出，无需新解析导数函数 |
| §4.4 验证器 | `pose_jacobian_hdq` 逐列播种（L197–216） | 双指标播种版，仅用于单元测试互证 |

验证策略沿用主文档 §6.5 的互证方法学：三方案（A 链 / B 播种 / C 闭式）+ 差分参考，两两求残差应达机器精度（$\sim10^{-15}$），差分参考残差应为 $O(\epsilon^2)$。

---

## 7. 公式来源总表

| 编号 | 内容 | 来源 |
|---|---|---|
| [P1] 式(9)(10)(14)(25)(28)(29)(33) | HDN/HDQ 代数、一阶链式法则、twist 提取 | Cohen & Shoham 2020（主文档 §4、§6 已复现） |
| [P2] 式(1)(2)(3)(8)(10)(12) | 左乘运动学、雅可比列、误差体系、H∞ 控制律 | Figueredo et al. 2021（主文档 §3、§5、§7 已复现） |
| 主文档 (5.1)(6.1)(6.2) | $\boldsymbol\jmath_i=2\partial_i\boldsymbol x\,\boldsymbol x^*$、左乘 twist 提取、左乘关节因子 | 本项目主文档 |
| §1.1–1.2 各表 | 编码器/电流换算、差分噪声方差、摩擦模型 | 标准工程/信号处理结果（教科书级） |
| (3.1) | 截断多项式环乘法 | 教科书级（jet 代数标准构造） |
| §4.4 双超对偶播种原理 | 幂零单位取二阶导无截断误差 | [F&A] Fike & Alonso 2011（DQ 系数版为本文移植） |
| (5.1)(5.2) | 关节空间动力学、计算力矩、加速度分解 | [LP17]（Lynch & Park）标准结果 |
| (5.2$'$) | (5.2) 的空间表示几何一致修正版（$\mathrm{Ad}$ 搬运前馈 + $e_\xi,e_z$ 反馈） | **修正推导**（与第四层文档 (F-2)(F-7) 共同构成，见 §5.1 注记） |
| (5.3) | 操作空间动力学 $\Lambda,\mu$ | [Kha87] Khatib 1987 |
| **(D-0)** | 位姿误差一阶预算 $\|\delta\text{pose}\|\lesssim\|J\|\|\delta q\|$ | **新推导**（由主文档 (5.1) 直接摄动） |
| **(D-1)(D-1')** | 二阶提升 $T^2$ 是乘法同态；二阶 Leibniz 双重和 | **新推导**（[P1] 式(28) 的严格升阶） |
| **(D-2)** | 关节因子二阶导 $\ddot x_i$；$\bar S^2$ 为对偶标量、转动关节 $=-1$ | **新推导** |
| **(D-3)** | 单位 DQ 共轭导数 $\dot{\boldsymbol x}^*=-\boldsymbol x^*\dot{\boldsymbol x}\boldsymbol x^*$ | **新推导**（群论中平凡，此处为 DQ 情形显式化） |
| **(D-4)** | $\dot{\boldsymbol\xi}=2\ddot{\boldsymbol x}\boldsymbol x^*-\tfrac12\boldsymbol\xi^2$，vec6 层修正项消失 | **新推导** |
| **(D-5)** | 播种 $\ddot q=0$ 免构造获取 $\dot J\dot q$ | **新推导**（(D-4) 推论） |
| **(D-6)** | 配置空间 Hessian 前缀/后缀闭式，$O(n^2)$ | **新推导**（`hdq_fast` 的升阶） |
| **(D-7)** | 任务空间 Hessian $h_{ij}=2H_{ij}\boldsymbol x^*-\tfrac12\boldsymbol\jmath_i\boldsymbol\jmath_j$ | **新推导** |
| **(D-8)** | $h_{ij}-h_{ji}=-\tfrac12[\boldsymbol\jmath_i,\boldsymbol\jmath_j]$（Lie 括号） | **新推导**（结论与螺旋理论 $\mathrm{ad}$ 项一致，DQ 形式为新） |
| **(D-9)** | $\dot J$ 列闭式及与 (D-5) 的一致性校验 | **新推导** |

> **诚实性声明**：标注 (D-k) 的结果由本文完成推导并给出证明，其中 (D-3)(D-4)(D-8) 的**结论**在李群/螺旋理论文献中存在等价形式（如 body-frame 加速度提取、$\mathrm{ad}$ 算子），但其 DQ 左乘约定下的显式形式与本项目代码结构（prefix/suffix、播种）的结合为本文整理；使用于论文时建议对 (D-4)(D-6)(D-7) 补充与现有 DQ 文献（如 Adorno 的 DQ 动力学工作）的查重。

---

## 8. 参考文献

1. **[P1]** A. Cohen, M. Shoham, *Hyper Dual Quaternions representation of rigid bodies kinematics*, Mechanism and Machine Theory 150 (2020) 103861.（项目文件 `1-s2.0-S0094114X20300823-main.pdf`）
2. **[P2]** L.F.C. Figueredo, B.V. Adorno, J.Y. Ishihara, *Robust H∞ kinematic control of manipulator robots using dual quaternion algebra*, Automatica 132 (2021) 109817.（项目文件 `1-s2.0-S000510982100337X-main.pdf`）
3. **[F&A]** J.A. Fike, J.J. Alonso, *The Development of Hyper-Dual Numbers for Exact Second-Derivative Calculations*, AIAA Paper 2011-886, 2011.
4. **[Kha87]** O. Khatib, *A unified approach for motion and force control of robot manipulators: The operational space formulation*, IEEE J. Robotics and Automation 3(1), 1987.
5. **[LP17]** K.M. Lynch, F.C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017.（第 8 章：动力学；$\mathrm{ad}$ 算子与 POE 动力学）
6. 主文档：`docs/数学理论与代码实现详解.md`（本项目，公式 (5.1)(6.1)(6.2) 及 §9 实验数据）
