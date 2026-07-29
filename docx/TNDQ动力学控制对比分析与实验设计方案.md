# TNDQ/HDQ 几何一致动力学控制：仿真结果分析、动力学控制文献对比与 KUKA LBR4+ 动力学模式对比实验设计方案

> **文档定位**：本文回应"以运动学控制作为对比基准不再合适"这一判断，完成三件事：
> ① 分析 `TNDQ_sim/results/` 中四组仿真结果；② 系统梳理现代机械臂**动力学反馈控制**
> 的理论谱系与核心公式（含出处）；③ 给出在 **CoppeliaSim KUKA LBR4+（LWR4+）动力学
> （力矩）模式**下的完整对比实验设计方案。目前阶段仅为设计方案与理论分析，不含代码实现。
>
> **记号约定**（与 `TNDQ_sim/README.md`、`docs/` 各文档一致）：位姿单位对偶四元数
> $\boldsymbol x\in\mathrm{Spin}(3)\ltimes\mathbb R^3$，twist $\boldsymbol\xi=2\dot{\boldsymbol x}\boldsymbol x^*$，
> $\mathrm{vec}_6$ 顺序 $[\omega;v]$；关节变量 $q\in\mathbb R^7$；TNDQ 误差量
> $\tilde{\boldsymbol x}=\boldsymbol x\boldsymbol x_d^*$、$e_\xi$、$e_z$ 及 $A(\tilde{\boldsymbol x})$
> 见论文初稿定理 1/2（`TNDQ_sim/control/error_system.py`）。文献引用编号 [n] 见文末参考文献表。

---

## 目录

1. [results 仿真结果数据分析](#1-results-仿真结果数据分析)
2. [理论背景对比：运动学控制 vs 动力学控制](#2-理论背景对比运动学控制-vs-动力学控制)
3. [文献调研：现代机械臂动力学反馈控制理论与关键公式](#3-文献调研现代机械臂动力学反馈控制理论与关键公式)
4. [关键控制律公式汇总与 TNDQ/HDQ 方法的理论对比](#4-关键控制律公式汇总与-tndqhdq-方法的理论对比)
5. [KUKA LBR4+ 动力学模式对比实验设计方案](#5-kuka-lbr4-动力学模式对比实验设计方案)
6. [TNDQ/HDQ 动力学控制理论的优势分析](#6-tndqhdq-动力学控制理论的优势分析)
7. [局限性与诚实声明](#7-局限性与诚实声明)
8. [参考文献](#8-参考文献)

---

## 1. results 仿真结果数据分析

### 1.1 数据来源与仿真设定

四组数据均由 `TNDQ_sim/run_simulation.py` 产生：7R KUKA LBR4+ 构型，
**加速度级被控对象**（论文式 (5.1)）$\ddot q=\ddot q_{\mathrm{ref}}+w_{\mathrm{dyn}}$，
控制律为几何一致计算力矩律（式 (5.2)，`control/control_law.py`），$dt=1\,\mathrm{ms}$，
记录间隔 100 步。列含义：$|O|,|T|$ 为 $e_z$ 的旋转/平移通道范数，$|e_\xi|$ 为
twist 误差范数，$V$ 为存储函数，$c_0,c_1,c_2$ 为 TNDQ 约束族 (3.8) 残差，
runtime 为控制器单步耗时（FK + 误差 + 控制律）。

### 1.2 四组场景的统计摘要（由 npz 原始数据计算）

| 场景 | 时长 | $\|e_\xi\|$ 末值 | $\|e_\xi\|$ RMS | $\|e_\xi\|$ 峰值 | $V(0)\to V(\mathrm{end})$ | 单步耗时均值/峰值 |
|---|---|---|---|---|---|---|
| line, 无扰动 | 10 s | **5.42e-4** | 3.26e-2 | 9.52e-2 | 8.52e-2 → **2.19e-6** | 1.63 / 2.39 ms |
| line, L2 扰动 | 10 s | 1.98e-3 | 7.62e-2 | 1.88e-1 | 8.52e-2 → 1.90e-5 | 1.63 / 2.56 ms |
| line, bias 扰动 | 10 s | 4.86e-2 | 7.33e-2 | 1.26e-1 | 8.52e-2 → 7.04e-2（有界） | 1.62 / 1.78 ms |
| circle, 无扰动 | 3 s | 2.31e-2（仍在收敛） | 5.82e-2 | 9.52e-2 | 8.52e-2 → 3.99e-3 | 1.63 / 2.09 ms |

### 1.3 逐场景解读（对照定理 3 各分支）

**(a) 标称场景（line_none）——定理 3(b) 指数收敛的数值证据。**
$|O|$ 从 9.89e-2 单调降至 5.31e-4（10 s，约 2.3 个数量级），$|T|$ 从 2.94e-2 降至
3.9e-11；$V$ 下降 4.6 个数量级且**全程单调**（逐行核对表格无一次回升），与
$\dot V\le -e_\xi^TK_de_\xi$ 的严格耗散不等式一致。半衰期约 1.3 s，对应
$\lambda_{\min}$ 主导的指数率。控制量 $|\ddot q_{\mathrm{ref}}|$ 初始峰值仅 1.40 rad/s²，
之后平滑衰减——无饱和风险。

**(b) L2 有限能量扰动（line_l2）——H∞ 增益核验。**
扰动在 $t\approx1.1$ s 注入后 $|e_\xi|$ 峰值 1.88e-1，扰动能量耗尽后误差回落至
1.98e-3 并继续收敛（$V$ 终值 1.9e-5，仅比标称高一个数量级）。README 记录的
能量核算结果：**实测 L2 增益 0.121 ≤ 认证增益 0.125** $=1/\lambda_{\min}(K_d)$
（定理 3(c)、`control/performance.py` 的 `PerformanceAccumulator`）——理论证书
被数据严格满足且不保守（余量仅 3%）。

**(c) L∞ 有界偏差扰动（line_bias）——ISS 极限球核验。**
持续偏差使 $|e_\xi|$ 稳定振荡于 4.2e-2 ~ 1.26e-1 区间，稳态约 5.7e-2，
**≤ ISS 界 (5.7) 的 0.187**（README §参考数值结果），且 $V$ 保持有界
（末值 7.0e-2 < 峰值 8.6e-2）。误差不发散、不漂移，符合 ISS"极限球半径正比于
$\|d\|_\infty/\lambda_{\min}(K_d)$"的定性与定量预言。

**(d) 圆轨迹（circle_none）——曲率激励下的收敛性。**
3 s 内 $|e_\xi|$ 从 9.5e-2 降到 2.4e-2，趋势与直线场景重合（前 3 s 两表逐行
几乎相同），说明控制律的前馈项（$\mathrm{Ad}_{\tilde x}\dot\xi_d$ + 传输修正）正确
消化了圆轨迹的向心加速度；$|\ddot q_{\mathrm{ref}}|$ 在 $t\approx1.4$ s 出现 17.4 rad/s²
尖峰（曲率与位形接近奇异方向叠加），阻尼伪逆将其限制在一个采样步内，未破坏收敛。

**(e) 数值稳定性与计算效率（四场景共性）。**
- 约束残差 $c_0,c_1,c_2$ 全程 ≤ 2e-15 量级（机器精度）：TNDQ 链上"位姿-速度-加速度"
  三通道的代数一致性在闭环中不漂移，无需周期性重正交化（对比矩阵法的
  $R^TR\ne I$ 漂移问题，见 §6.3）。
- 单步耗时稳定在 **1.6 ms**（标准差极小，峰值 < 2.6 ms），纯 Python + numpy 下即可
  支撑 ≈500 Hz 控制环；其中含完整 $\mathcal A_2$ 链 FK、$J$、免构造 $\dot J\dot q$、
  误差系统与控制律全部计算。

### 1.4 对"对比基准"问题的直接含义

上述数据是**加速度级（动力学接口）**闭环的结果：控制器输出 $\ddot q_{\mathrm{ref}}$，
经名义计算力矩接口 $\tau=\hat M\ddot q_{\mathrm{ref}}+\hat C\dot q+\hat g$（§2.4）作用于
二阶被控对象。而项目此前 `hdq_hinf_coppeliasim/` 的全部实验（[P2] 式(12) 运动学 H∞ 律）
是**速度级**闭环：控制器输出 $\dot q_{\mathrm{cmd}}$ 直接发给理想速度环。两者的被控对象、
扰动通道、性能定义均不同（详见 §2），故公平的对比对象必须是**同为力矩/加速度级**的
经典动力学控制律——这正是 §3–§5 的任务。

---

## 2. 理论背景对比：运动学控制 vs 动力学控制

### 2.1 两类控制的被控对象与数学结构

| 维度 | 运动学控制（一阶） | 动力学控制（二阶） |
|---|---|---|
| 被控对象 | $\dot q=u$（速度积分器，假设速度环理想） | $M(q)\ddot q+C(q,\dot q)\dot q+g(q)=\tau$ [1,2,3] |
| 控制输入 | 关节速度 $\dot q_{\mathrm{cmd}}$ | 关节力矩 $\tau$（或加速度 $\ddot q_{\mathrm{ref}}$） |
| 误差动力学 | 一阶：$\dot{\tilde x}=f(\tilde x)u+d$ | 二阶级联：$(e_z,e_\xi)$ 或 $(e,\dot e)$ |
| 需要的模型 | 仅几何（DH/POE、$J$） | 几何 + 惯性参数 $m_i,I_i$、摩擦 |
| 隐含假设 | 速度环带宽无穷大；$\dot J\dot q$、$C\dot q$ 可忽略 | 无（显式补偿全部二阶效应） |
| 典型代表 | 微分逆运动学、CLIK、[P2] DQ-H∞ 律 [4] | 计算力矩 [1,5]、操作空间 [6]、自适应 [7]、PD+ [8] |
| Lyapunov 结构 | $V=\tfrac12\|e\|^2$ 一阶耗散 | 含交叉项的二阶存储函数（机械能型） |

### 2.2 为什么运动学控制不能作为动力学控制的对比基准

**(i) 被控对象不同 ⇒ 闭环方程不可比。**
运动学律 $\dot q_{\mathrm{cmd}}=J^+(\kappa\,\mathrm{err}+\text{前馈})$（[P2] 式(12)）的闭环是
一阶线性/近线性系统，其收敛率由软件增益 $\kappa$ 任意设定，与惯量无关；动力学模式下
同一条律必须先经过厂商速度环（一个未建模的高阶动态）才作用到 $M\ddot q+C\dot q+g=\tau$
上。在 CoppeliaSim 动力学模式中比较"运动学律 + 内置速度环"与"力矩律"，实际比较的是
**CoppeliaSim 内置 PID 的调参质量**，而非两种理论的优劣——这是方法学上的混淆变量。

**(ii) 误差来源不同 ⇒ 性能指标含义不同。**
运动学闭环的稳态误差只来自数值积分与伪逆残差；动力学闭环的误差来自
$\Delta M,\Delta C,\Delta g$、摩擦、$\dot J\dot q$ 未补偿项——量级随 $\|\dot q\|^2$ 增长
（`docs/HDQ高阶结构动力学创新应用分析.md` §3.1/§3.4）。用低速运动学实验的 1e-4 级
稳态误差去"对比"动力学实验的误差没有意义：前者根本没有暴露动力学误差通道。

**(iii) H∞ 保证的层级不同。**
[P2] 的 $L_2$ 增益界是**运动学级**的：扰动 $d$ 直接加在一阶误差方程上。TNDQ 定理 3
的增益界作用在**加速度级**误差系统 $(e_z,e_\xi)$ 上，扰动 $w_{\mathrm{dyn}}$ 代表
惯性参数失配经 $\hat M^{-1}$ 折算后的等效加速度扰动。两个 $\gamma$ 的物理量纲和
扰动定义都不同，数字上不可直接比较。

**(iv) 计算负载不可比。**
运动学律每周期只需 $J$（0.14–0.9 ms 量级，主文档 §9）；动力学律还需
$\dot J\dot q$、$M,C,g$ 装配或其递推等价物。拿运动学律的耗时去对比动力学律的耗时，
等于比较两种不同任务的工作量。

**结论**：公平基准 = **同一动力学被控对象 + 同一力矩接口 + 不同误差参数化/控制理论**。
即：矩阵/欧拉角或旋转矩阵参数化的经典计算力矩与鲁棒/自适应律、对偶四元数参数化的
动力学控制律（[9,10]），对比 TNDQ/HDQ 几何一致计算力矩律（式 (5.2)）。

---

## 3. 文献调研：现代机械臂动力学反馈控制理论与关键公式

### 3.1 基于拉格朗日方程的动力学模型（对比实验的公共被控对象）

$n$ 自由度串联刚性机械臂的 Euler–Lagrange 方程（[1] Ch.6、[2] Ch.8、[3] Ch.7）：

$$
M(q)\ddot q+C(q,\dot q)\dot q+g(q)+F_v\dot q+F_c\,\mathrm{sgn}(\dot q)=\tau+\tau_{\mathrm{ext}},
\tag{3.1}
$$

其中 $M(q)\succ0$ 为惯性矩阵（位形流形上的黎曼度量），$C$ 为 Christoffel 装配的
Coriolis/离心矩阵（[2] 式(8.51)），$g=\partial\mathcal P/\partial q$ 为重力项，
$F_v,F_c$ 为黏性/库仑摩擦。三条结构性质是所有动力学控制律证明的公共基石 [1,3]：

- **P1（有界性）**：$\mu_1I\preceq M(q)\preceq\mu_2I$；
- **P2（斜对称）**：$\dot M-2C$ 反对称，即 $x^T(\dot M-2C)x=0,\ \forall x$；
- **P3（参数线性）**：$M(q)\ddot q+C(q,\dot q)\dot q+g(q)=Y(q,\dot q,\ddot q)\,\theta$，
  $\theta\in\mathbb R^p$ 为惯性参数向量（自适应控制的基础）。

KUKA LBR4+（= LWR4+）的数值动力学模型可由 Gaz–Flacco–De Luca 的逆向工程辨识
结果获得（[11]，含 $M,C,g$ 的符号表达与参数表；另见 Jubien–Gautier–Janot 的
电机力矩辨识 [12]）——这使"用 LBR4+ 动力学模式做对比"具备了公开可查的
名义模型 $\hat M,\hat C,\hat g$。

### 3.2 计算力矩控制（Computed Torque Control, CTC）

关节空间 CTC / 逆动力学控制（[1] §8.5、[5]、[3] §6.5）：

$$
\tau=M(q)\bigl(\ddot q_d+K_d\dot e+K_pe\bigr)+C(q,\dot q)\dot q+g(q),
\qquad e\triangleq q_d-q .
\tag{3.2}
$$

模型精确时闭环**全局精确线性化**为解耦二阶线性系统：

$$
\ddot e+K_d\dot e+K_pe=0,
\tag{3.3}
$$

极点由 $K_p=\mathrm{diag}(\omega_i^2),\ K_d=\mathrm{diag}(2\zeta_i\omega_i)$ 任意配置。
模型失配 $\Delta M,\Delta C,\Delta g$ 时闭环变为

$$
\ddot e+K_d\dot e+K_pe=\hat M^{-1}\bigl(\Delta M\ddot q+\Delta C\dot q+\Delta g\bigr)\triangleq w_{\mathrm{dyn}},
\tag{3.4}
$$

即失配折算为加速度级扰动——这正是 TNDQ 论文式 (5.1) 中 $w_{\mathrm{dyn}}$ 的出处，
也说明本项目 `run_simulation.py` 的加速度级仿真与"CTC + 模型失配"在结构上等价。

**任务空间版（操作空间公式，Khatib [6]）**：

$$
\tau=J^T\Bigl[\Lambda(q)\bigl(\ddot x_d+K_d\dot e_x+K_pe_x\bigr)+\mu(q,\dot q)+p(q)\Bigr],
\qquad
\Lambda=(JM^{-1}J^T)^{-1},\ \
\mu=\Lambda\bigl(JM^{-1}C\dot q-\dot J\dot q\bigr),\ \
p=\Lambda JM^{-1}g .
\tag{3.5}
$$

注意 (3.5) 显式需要 $\dot J\dot q$——矩阵法通常靠数值差分或手工递推获得，
而 TNDQ 链以 $O(n)$ 免构造读出（(D-5)，`core/kinematics.py`），这是后文对比的
关键计算差异点。

### 3.3 反馈线性化控制（Feedback Linearization）

CTC 是反馈线性化在全驱动机械臂上的特例：取状态 $x=(q,\dot q)$、输入变换

$$
\tau=M(q)\,v+C(q,\dot q)\dot q+g(q)
\quad\Longrightarrow\quad
\ddot q=v,
\tag{3.6}
$$

系统被精确变换为 $n$ 个解耦双积分器（相对阶 $\{2,\dots,2\}$，无零动态）[13,1]。
外环 $v$ 可自由设计：PD（即 CTC）、LQR、时变跟踪等。姿态参数化的选择在此进入：
若任务空间误差用欧拉角，$v$ 中的表示雅可比在 gimbal lock 处奇异；若用单位四元数/DQ，
则需处理双覆盖（unwinding）——Bhat & Bernstein 证明**任何连续状态反馈都无法在
$SO(3)$（及其覆盖空间）上实现全局渐近稳定**，四元数连续反馈至多"几乎全局"[14]。
TNDQ 误差体系的符号翻转规则（定理 1(i)，README §实现要点）正是针对此问题的
标准处置。

### 3.4 无逆动力学的 PD 型律：PD+ 与重力补偿 PD

**重力补偿 PD（Takegaki–Arimoto [15]）**，调节问题全局渐近稳定：

$$
\tau=K_pe-K_d\dot q+g(q).
\tag{3.7}
$$

**PD+（Paden–Panja [8]）**，跟踪问题全局渐近稳定、无需求逆 $M$：

$$
\tau=M(q)\ddot q_d+C(q,\dot q)\dot q_d+g(q)+K_pe+K_d\dot e .
\tag{3.8}
$$

证明依赖性质 P2（斜对称）而非精确抵消——这类"被动性路线"[16] 与 CTC 的
"精确线性化路线"构成动力学控制的两大范式。TNDQ 定理 3 的存储函数
$V=\tfrac12\|e_\xi\|^2+\tfrac{k_p}2\|e_z\|^2$ 及交叉项精确对消（$A^T$ 整形反馈）
属于被动性范式在 DQ 几何误差上的移植。

### 3.5 自适应控制（Slotine–Li）

Slotine–Li 自适应律 [7]（亦见 [1] §9.4、[16]）：定义复合误差与参考速度

$$
s=\dot e+\Lambda e,\qquad \dot q_r=\dot q_d+\Lambda e,
$$

控制与参数更新律为

$$
\tau=\hat M(q)\ddot q_r+\hat C(q,\dot q)\dot q_r+\hat g(q)+K_Ds
=Y(q,\dot q,\dot q_r,\ddot q_r)\hat\theta+K_Ds,
\qquad
\dot{\hat\theta}=\Gamma\,Y^T s,
\tag{3.9}
$$

利用 P2、P3 可证 $s\to0$、$e\to0$（全局），且**无需测量 $\ddot q$、无需逆 $M$**。
这是处理惯性参数不确定的首选基准之一；其 DQ 版本已有先例（如 [17] 的移动机械臂
DQ 自适应控制路线）。

### 3.6 鲁棒控制方法

**(a) 滑模/统一鲁棒律（Spong [18]，[1] §9.3）**：在名义 CTC 外环上叠加鲁棒项

$$
v=\ddot q_d+K_d\dot e+K_pe+\Delta v,\qquad
\Delta v=-\rho(e,t)\,\frac{B^TPz}{\|B^TPz\|}\ \ (\|B^TPz\|>\epsilon),
\tag{3.10}
$$

$\rho$ 为不确定度上界函数，$z=(e,\dot e)$，$P$ 解 Lyapunov 方程；保证一致最终有界
（UUB），$\epsilon$-边界层消抖。

**(b) 非线性 H∞ / L2 增益控制（van der Schaft [19]；Chen–Lee–Chang [20]）**：
寻找存储函数 $V\ge0$ 使耗散不等式

$$
\dot V\le\frac12\gamma^2\|w\|^2-\frac12\|z\|^2
\tag{3.11}
$$

沿闭环成立（HJI 不等式的解），则扰动 $w$ 到性能输出 $z$ 的 $L_2$ 增益 $\le\gamma$。
[20] 给出机械臂状态反馈 H∞ 跟踪的可解条件与代数 Riccati 型构造。TNDQ 定理 3(c)
正是 (3.11) 在几何误差坐标 $(e_z,e_\xi)$ 上的实例：$K_d\succeq\tfrac12(\kappa^{-1}+\gamma_a^{-2})I_6$
（式 (5.6a)）保证认证增益 $1/\lambda_{\min}(K_d)$——§1.3(b) 的实测 0.121 ≤ 0.125
是该不等式的数值核验。

**(c) ISS 框架（Sontag [21]）**：有界扰动 $\Rightarrow$ 有界误差球

$$
\limsup_{t\to\infty}\|e(t)\|\le\rho\bigl(\|d\|_\infty\bigr),
\tag{3.12}
$$

TNDQ 式 (5.7) 给出显式半径 $\|d_b\|_\infty/\lambda_{\min}(K_d)$，§1.3(c) 的
bias 实验（0.057 ≤ 0.187）为其数值核验。

### 3.7 对偶四元数框架下的动力学建模与控制（最直接的同类工作）

- **DQ 动力学建模**：Silva–Quiroz-Omaña–Adorno [10] 给出 DQ twist/wrench 的
  递推牛顿-欧拉与 Gauss 最小约束原理两条路线；Cohen–Shoham [22] 用超对偶数写出
  单刚体动力学方程。二者证明四元数系代数可承载完整动力学，但**导数量仍靠手工
  递推**（对比 TNDQ 的"乘法规则自动生成递推"，(E-5)）。
- **DQ 动力学控制律**：DQ 位姿反馈的刚体动力学跟踪已有反馈线性化结果
  （单刚体：Wang–Yu [23]）；机械臂侧，基于单位 DQ 的鲁棒计算力矩控制近期已出现
  （Robust torque-computed control with unit dual quaternion [9]），其结构为
  "DQ 对数映射误差 + 名义 CTC + 鲁棒项"，是 §5 实验中最贴近的 DQ 基准 C3。
  任务空间 DQ 导纳/阻抗控制见 Fonseca–Adorno–Fraisse [24]。
- **与 TNDQ 的区别**：上述 DQ 工作在误差二阶导层面仍需显式构造 $\dot J$ 或做
  数值微分；TNDQ 把 $\ddot x,\dot\xi,\dot J\dot q$ 全部变成一条 $\mathcal A_2$ 链的
  $\sigma^2$ 通道读数（机器精度、$O(n)$），并给出**误差体系与控制律证明所需的
  全部微分恒等式的代数封闭形式**（定理 1/2、引理 1）。

---

## 4. 关键控制律公式汇总与 TNDQ/HDQ 方法的理论对比

### 4.1 核心公式速查表

| # | 控制律 | 公式 | 全局性 | 需要的模型量 | 出处 |
|---|---|---|---|---|---|
| L1 | 关节空间 CTC | $\tau=M(\ddot q_d+K_d\dot e+K_pe)+C\dot q+g$ | 全局（模型精确） | $M,C,g$ | [1,5] |
| L2 | 操作空间 CTC | 式 (3.5) | 局部（$J$ 满秩，姿态参数化域内） | $M,C,g,J,\dot J\dot q,\Lambda$ | [6] |
| L3 | PD+ | $\tau=M\ddot q_d+C\dot q_d+g+K_pe+K_d\dot e$ | 全局 | $M,C,g$（不求逆） | [8] |
| L4 | Slotine–Li 自适应 | $\tau=Y\hat\theta+K_Ds,\ \dot{\hat\theta}=\Gamma Y^Ts$ | 全局 | 回归矩阵 $Y$ | [7] |
| L5 | Spong 鲁棒 | 式 (3.10) | UUB | 名义 $\hat M,\hat C,\hat g$+上界 $\rho$ | [18] |
| L6 | 非线性 H∞ | HJI/(3.11) 构造 | $L_2$ 增益 $\le\gamma$ | $M,C,g$ | [19,20] |
| L7 | DQ 鲁棒 CTC | DQ log 误差 + CTC + 鲁棒项 | 几乎全局（unwinding 处理后） | $M,C,g$ + DQ FK | [9] |
| L8 | **TNDQ 几何一致 CTC（式 5.2）** | 见 (4.1) | 几乎全局 + H∞/ISS 证书 | $\hat M,\hat C,\hat g$ + TNDQ 链 | 本项目 |

TNDQ 律（`control/control_law.py` 实现）：

$$
\ddot q_{\mathrm{ref}}
=J^{+}\Bigl(\underbrace{\mathrm{vec}_6\bigl(\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde\xi}\,\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d\bigr)}_{\text{前馈（引理 1）}}
\underbrace{-K_de_\xi-k_pA^T(\tilde x)e_z}_{\text{反馈（定理 3(b) 交叉项精确对消）}}
\underbrace{-\dot J\dot q}_{\text{TNDQ 链免构造}}\Bigr),
\qquad
\tau=\hat M\ddot q_{\mathrm{ref}}+\hat C\dot q+\hat g .
\tag{4.1}
$$

### 4.2 理论层面对比分析

**(i) 误差参数化：几何一致性。**
L1/L3/L4 在关节空间工作，回避了姿态参数化问题但无法直接表达任务空间指标；
L2 需选任务空间姿态表示——欧拉角有表示奇异，旋转矩阵误差 $\tfrac12(R_d^TR-R^TR_d)^\vee$
在大姿态误差下增益退化；L7 与 L8 使用 DQ 双覆盖，仅剩 unwinding 一个拓扑障碍
（[14] 表明这是不可消除的下限），由符号翻转处置。L8 的独特点：误差对
$(e_z,e_\xi)$ 的选取使 $\dot e_z=A(\tilde x)e_\xi$ **精确成立**（定理 2，无线性化），
故 Lyapunov 证明无小角度假设，收敛域覆盖除对径点外的全位形空间。

**(ii) 二阶几何量的获取方式。**
L2/L7 需要 $\dot J\dot q$（及 L2 的 $\Lambda,\mu$）：矩阵法用数值差分
（$O(\epsilon)$ 误差 + 噪声放大，见 `docs/HDQ高阶结构动力学创新应用分析.md` §5.1）
或手工递推（[10] 路线，逐式推导易错）；L8 由 $\mathcal A_2$ 链 $\sigma^2$ 通道以
$O(n)$ 机器精度读出（(D-5)），且同一条链同时供给 $x,\xi,J$——**一次传播，全量输出**。

**(iii) 性能证书。**
L1–L3 给渐近/指数收敛；L4 给全局收敛但无显式扰动增益；L5 给 UUB；L6 给 $L_2$
增益但构造需解 HJI；L8 把 H∞ 增益条件退化为**一个特征值不等式**
$\lambda_{\min}(K_d)\ge\tfrac12(\kappa^{-1}+\gamma_a^{-2})$（式 5.6a，可在线核验），
并同时给出 ISS 极限球 (5.7) 与分通道增益 (5.6')——证书可在每次实验中用能量核算
直接检验（§1.3 已验证其非保守性）。

**(iv) 数值一致性自监控。**
矩阵法闭环中 $R$ 的正交漂移、四元数范数漂移需外加投影且**无内在指示器**；
L8 的约束族 (3.8) 残差 $c_0,c_1,c_2$ 是运动学链自身的代数恒等式，实时充当
"数值健康度传感器"（§1.3(e)：全程机器精度）。

---

## 5. KUKA LBR4+ 动力学模式对比实验设计方案

### 5.1 总体设计

**目的**：在同一动力学被控对象上，比较 TNDQ 几何一致 CTC（L8）与经典矩阵法
（L1/L2/L4/L5）及 DQ 法（L7）的误差收敛性、计算效率、实时性与鲁棒性，验证
§4.2 的四点理论优势在动力学闭环中兑现。

**平台**：CoppeliaSim ≥ 4.5，KUKA LBR4+ 模型，**动力学模式**（引擎建议
Mujoco 或 Newton；所有关节设为 force/torque 模式，控制接口
`sim.setJointTargetForce`，禁用内置位置/速度 PID）。ZMQ Remote API 同步步进
（`sim.setStepping(true)`），复用 `hdq_hinf_coppeliasim/sim/coppelia_client.py`
与 `TNDQ_sim/interfaces/coppeliasim_interface.py` 的接口契约。

**场景实例化**：仓库现有场景 `TNDQ_sim/KUKALBR4+_sim.ttt`（机械臂 + 椅子 +
杯子）的定点/圆周/扰动实验落地设计见配套文档
《KUKALBR4p场景_定点与圆周扰动对比实验设计.md》（本目录），其中杯子充当
定点目标与突加负载源、椅子充当障碍与接触扰动源；场景对象路径、关节模式、
惯性参数须按该文档 §1.2 清单在里程碑 0 运行时枚举核实。

**名义模型**：$\hat M,\hat C,\hat g$ 取 Gaz–Flacco–De Luca LWR4+ 辨识模型 [11]
（公开参数表）；同时从 CoppeliaSim 场景读取引擎侧惯性参数作为"真值"，两者之差
即受控的模型失配源（可人为放大做鲁棒性扫描）。

**控制频率**：力矩环 500 Hz（$dt=2$ ms，与 §1.3(e) 实测 1.6 ms/步的预算相容）；
物理引擎步长 1 ms（每控制步 2 个物理子步）。

### 5.2 对比控制器组

| 编号 | 控制器 | 误差参数化 | $\dot J\dot q$ 来源 | 代表的理论谱系 |
|---|---|---|---|---|
| C1 | 关节空间 CTC（L1，式 3.2） | 关节角 $e=q_d-q$（IK 预解） | 不需要 | 经典矩阵法上界参照 |
| C2 | 操作空间 CTC（L2，式 3.5） | $R$ 误差 $\tfrac12(R_d^TR-R^TR_d)^\vee$ + 平移 | 数值差分 $\dot J\approx\Delta J/\Delta t$ | 传统矩阵任务空间法 |
| C3 | DQ 鲁棒 CTC（L7，[9] 结构） | DQ 对数映射 $\ln(\boldsymbol x_d^*\boldsymbol x)$ | 手工递推或差分 | 现有 DQ 动力学控制 |
| C4 | Slotine–Li 自适应（L4，式 3.9） | 关节空间 $s=\dot e+\Lambda e$ | 不需要 | 自适应谱系（参数失配场景专用） |
| C5 | **TNDQ 几何一致 CTC（L8，式 4.1）** | $(e_z,e_\xi)$，定理 1/2 | $\mathcal A_2$ 链免构造 | 本项目新理论 |

设计原则：C1 提供"关节空间理想上界"（无任务空间参数化损失）；C2/C3 与 C5 同为
任务空间律，是核心对比组；C4 仅在参数失配场景（E3）中加入。所有控制器共用
同一 $\hat M,\hat C,\hat g$、同一阻尼伪逆参数与同一增益整定规则
（统一按闭环带宽 $\omega_n=4$ rad/s、$\zeta=1$ 折算 $K_p,K_d$，C5 按 (5.6a)
核验后取 $K_d=8I_6,\ k_p=16$ 量级并保持谱等价），排除调参偏袒。

### 5.3 实验场景矩阵

| 场景 | 轨迹 | 扰动/失配 | 对应理论分支 | 复用现有设施 |
|---|---|---|---|---|
| E1 标称收敛 | 直线（10 s）+ 圆（10 s，两种速率 0.05/0.25 m/s） | 无 | 定理 3(b) vs (3.3) | `simdata/trajectory_generator.py` |
| E2 L2 扰动 | 直线 | 关节力矩注入有限能量扰动（`input_simulation.py` L2 波形经 $\hat M$ 折算为 $\tau$ 级） | 定理 3(c) vs L6 | `PerformanceAccumulator` |
| E3 参数失配/负载 | 直线 | 末端突加 1.5 kg 负载（$t=3$ s）；$\hat M,\hat g$ 全局 ±20% 缩放 | ISS (5.7) vs UUB/L4 | `disturbances.py` 模式 |
| E4 大姿态误差 | 定点调节：初始姿态误差 120°、150°、170°（接近对径） | 无 | 几乎全局性 vs 参数化退化 | 新增初始化脚本 |
| E5 高速域 | 圆轨迹提速至 $\|\dot q\|_{\max}\approx1.5$ rad/s | 无 | $\dot J\dot q$ 补偿质量 | — |
| E6 测量噪声 | 直线 | $q$ 加量化噪声（16 bit 编码器模型），$\dot q$ 差分+低通 | 差分 vs 代数通道的噪声敏感度 | `input_simulation.py` 噪声 |
| E7 接触扰动 | 圆（含椅背擦碰弧段） | 末端 ≤2 cm 过盈擦过椅背，引擎解算非建模接触力脉冲 | 证书对任意 $L_2/L_\infty$ 扰动的承诺 | 场景篇 §6.3 |

每场景 × 每控制器重复 10 次（噪声/扰动随机种子不同），报告均值 ± 标准差。

### 5.4 性能指标与评价标准

**A. 误差收敛性**
- 任务空间统一外部量测（与控制器内部参数化解耦，保证公平）：位置误差
  $\|p-p_d\|$（m）、姿态测地距离 $\theta_e=2\arccos|\langle r,r_d\rangle|$（rad），
  由引擎真值位姿计算；
- 指标：2% 安定时间 $t_s$、稳态误差（末 2 s 均值）、RMS、ITAE、超调；
- E4 附加：是否出现 unwinding（累计旋转 > 2π−初始误差）、是否收敛失败。

**B. 计算效率与实时性**
- 每控制步壁钟耗时（分解为 FK/微分量、动力学装配、控制律三段计时）、
  均值/99 分位/最大抖动；
- 关键单项：C2 的 $\dot J$ 差分（额外一次全 $J$）vs C5 的 $\sigma^2$ 通道读出
  （预期 $O(n)$ 且与 $J$ 共享链，§1.3(e) 已有 1.6 ms 全流程实测参照）；
- 超时率（> 2 ms 的步占比，衡量 500 Hz 硬实时可行性）。

**C. 鲁棒性**
- E2：实测 L2 增益 $\sqrt{\int\|e_\xi\|^2/\int\|d\|^2}$ 对比各自理论界
  （C5 有认证值 $1/\lambda_{\min}(K_d)$，C2/C3 无先验界，仅报实测）；
- E3：负载突变后的误差峰值、恢复时间、稳态球半径 vs ISS 界 (5.7)；
  C4 报告参数收敛轨迹；
- E6：误差方差放大系数（输出误差方差 / 输入噪声方差）。

**D. 数值稳定性**
- C2：$\|R^TR-I\|_F$ 漂移；C3：DQ 范数漂移 $|\|\boldsymbol x\|-1|$；
  C5：约束残差 $c_0,c_1,c_2$（判据：全程 < 1e-12，参照 §1.3(e) 的 2e-15 实测）；
- 奇异邻域行为：E5 中最小奇异值 $\sigma_{\min}(J)$ 与控制量峰值联动记录。

**判优规则**（预注册，避免事后择优）：每指标对 C2/C3/C5 做配对 Wilcoxon 检验
（10 次重复），显著性 0.05；C5 声称成立的条件为——A 类指标不劣于 C2/C3 且
E4/E5 显著更优，B 类中微分量获取耗时显著更低，C 类实测增益满足认证界，
D 类残差恒为机器精度。

### 5.5 预期结果（基于 §1 数据与理论的可证伪预测）

1. **E1 低速**：C1/C2/C3/C5 稳态误差同量级（低速下 $\dot J\dot q$ 小，§3.4 扩展篇
   结论），C5 不应更差——此为"无退化"检查；
2. **E1 高速与 E5**：C2 因 $\dot J$ 差分误差与参数化增益退化，跟踪误差随速度
   平方增长快于 C5；C5 的前馈 + 免构造 $\dot J\dot q$ 预期将高速圆轨迹稳态误差
   压低 3–10 倍（对应 §1.3(d) 中前馈正确消化向心项的证据外推）；
3. **E2**：C5 实测增益 ≤ 认证值（复现 §1.3(b) 的 0.121 ≤ 0.125 模式）；C2/C3
   实测增益预期更大且无证书；
4. **E3**：C5 稳态误差落入 ISS 球 (5.7)；加入 C4 后 C4 渐近误差最小但收敛慢、
   计算量含回归矩阵装配；
5. **E4**：C2 在 170° 初始误差下增益退化甚至收敛失败；C3/C5 借双覆盖正常收敛，
   C5 因 $\dot e_z=Ae_\xi$ 精确性（无小角度近似）收敛轨迹更接近理论指数包络；
6. **B 类**：C5 全流程单步 ≲ 2 ms（Python，§1.3(e) 实测外推 + $M,C,g$ 装配
   ≈1 ms 估算，扩展篇 §4.3），与 C2 相当或更低（C2 需两次 $J$ + 差分）；
   若需 1 kHz，指明 C/JIT 化路径；
7. **D 类**：C5 约束残差机器精度、无漂移；C2 的 $R$ 漂移需周期性再正交化。

若 2/5/7 未达成，则说明 TNDQ 优势不能从加速度级理想仿真外推到含引擎接触/
摩擦的动力学闭环，需回到 §7 的局限性讨论——该证伪出口是方案完整性的一部分。

### 5.6 实施步骤（不含代码，仅里程碑）

1. 场景搭建：LBR4+ 力矩模式场景、惯性参数导出与 [11] 模型比对（1 周）；
2. 公共设施：统一量测、计时、日志（扩展 `output/data_logger.py` 表格式）（1 周）；
3. C1/C2 基准实现与增益整定 → C3 → C5 接入（复用 `control/` 全部模块）（2 周）；
4. E1–E6 批量运行与统计分析、撰写实验报告（2 周）。

---

## 6. TNDQ/HDQ 动力学控制理论的优势分析

### 6.1 误差收敛性

- **几乎全局的收敛域**：$(e_z,e_\xi)$ 误差体系不含任何线性化（定理 2 精确成立），
  收敛保证覆盖除对径点外全部初始位姿误差；矩阵法任务空间律的收敛证明普遍依赖
  小误差线性化或参数化非奇异域。已有数据支撑：§1.3(a) 中 $V$ 全程严格单调下降
  4.6 个数量级，初始误差 $|O|\approx0.1$（约 11° 姿态误差）下无任何瞬态回升；
- **可认证的扰动衰减**：H∞ 增益条件是单个特征值不等式，实验可在线核验且已被
  验证非保守（实测 0.121 vs 认证 0.125，余量 3%）；ISS 界同样被 bias 实验满足
  （0.057 ≤ 0.187）。经典 CTC/操作空间律不提供同类显式证书；
- **前馈的几何精确性**：引理 1 的传输修正 $\mathrm{ad}_{\tilde\xi}\mathrm{Ad}_{\tilde x}\xi_d$
  使误差动力学的非反馈项**精确对消**（非近似抵消），§1.3(d) 圆轨迹数据表明
  向心加速度被前馈完全消化（前 3 s 收敛曲线与直线场景重合）。

### 6.2 计算效率

- **一条链，全量输出**：$\mathcal A_2$ 链一次 $O(n)$ 传播同时给出
  $x,\dot x,\ddot x\Rightarrow\xi,\dot\xi,J,\dot J\dot q$；矩阵法获取同样信息需
  FK + $J$ 装配 + $\dot J$ 差分（两次 $J$）或 $O(n^2)$ 手工递推；
- **实测参照**：完整"FK + 误差 + 控制律"单步 1.63 ms（Python + numpy，7-DOF，
  无任何编译优化），峰值 < 2.6 ms、抖动极小（RMS≈均值），已满足 500 Hz 原型
  力矩环预算；DQ 乘法为固定 24-乘-16-加核，C 化后预计 < 0.1 ms（定性估计，
  扩展篇 §4.3，未实测）；
- **导数精度无成本**：差分路线的误差随微分阶按 $\epsilon^{-k}$ 恶化且放大编码器
  噪声（方差 $6\sigma_q^2/\Delta t^4$，扩展篇 §1.2）；节代数通道是恒等式，精度
  = 机器精度，E6 噪声实验预期直接体现此差异。

### 6.3 数值稳定性

- **内建一致性监控**：约束族 (3.8) 残差 $c_0,c_1,c_2$ 在全部四组 10 s 闭环中
  保持 ≤ 2e-15，等于免费获得一个实时数值健康度指示器；矩阵法需外加
  $R^TR-I$ 检查与再正交化（Gram–Schmidt/SVD，额外成本且引入投影跳变）；
- **紧凑参数化**：DQ 8 参数 vs 齐次矩阵 12 参数，约束流形维数差（2 vs 6）决定
  漂移自由度更少；重投影（§3.4，README）只需一次归一化级操作；
- **奇异处理透明**：阻尼伪逆残差被显式归入定理 3 的扰动通道 d(t)（诚实注记 (i)），
  性能证书对其依然成立——奇异邻域行为有理论覆盖而非工程补丁。

### 6.4 理论体系的可扩展性

- 同一代数塔向上：$\mathcal A_3$ 通道给 jerk 级量（(E-2)），服务轨迹光滑约束与
  柔性抑制；向侧：多参数幂零单位给 Hessian（(D-6)(D-7)），解锁 $C$ 矩阵解析
  装配 (E-4) 与精确避奇异梯度 (E-6)（7R 冗余臂零空间优化立即可用）；
- RNE 前向传播 ≡ $\mathcal A_2$ 链（(E-5)）：动力学递推不再手写，由乘法规则
  自动生成——这是对 [10,22] 手工递推路线的结构性简化。

---

## 7. 局限性与诚实声明

1. §1 的全部数据来自**加速度级理想被控对象**（式 5.1），尚未经过含接触、摩擦、
   电机动态的物理引擎动力学闭环；§5 的预期结果 2/6/7 是外推，属待证伪假设；
2. $\hat M,\hat C,\hat g$ 装配代码尚未实现（`nominal_computed_torque` 为接口桩），
   扩展篇 (E-3)(E-4) 的装配路线未经数值验证；其耗时 ≈1 ms 为估算值；
3. 级联"TNDQ 外环 + 计算力矩内环"的整体 $L_2$ 增益界（$\Delta M,\Delta C$ 为新
   扰动通道）尚无定理，E2/E3 实验只能核验现有加速度级证书的等效折算形式；
4. [9] 的具体公式本文仅按其摘要级结构引用，实现 C3 前需精读原文并逐条比对；
   [11] 的 LWR4+ 参数表与 CoppeliaSim 模型惯性参数的一致性需在里程碑 1 中核实；
5. Unwinding 的处置（符号翻转）使控制律不连续，[14] 意义下"几乎全局"是拓扑
   上限而非本方法缺陷，但 E4 的 170° 用例仍可能触发翻转瞬态，需在数据中如实报告。

---

## 8. 参考文献

1. M. W. Spong, S. Hutchinson, M. Vidyasagar, *Robot Modeling and Control*, 2nd ed., Wiley, 2020.（式 3.1–3.4、L1/L5 出处；性质 P1–P3）
2. K. M. Lynch, F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017.（式(8.51)(8.57)：$C$ 的 Christoffel 装配与 $M$ 的装配；RNE 算法 8.1）
3. B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009.（第 7/8 章：拉格朗日模型与逆动力学控制）
4. L. F. C. Figueredo, B. V. Adorno, J. Y. Ishihara, "Robust H∞ kinematic control of manipulator robots using dual quaternion algebra," *Automatica* 132 (2021) 109817.（[P2]，运动学 H∞ 律，本项目一阶层基准）
5. J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 4th ed., Pearson, 2018.（计算力矩/分段线性化控制，Ch.10）
6. O. Khatib, "A unified approach for motion and force control of robot manipulators: The operational space formulation," *IEEE J. Robotics and Automation* 3(1), 1987, 43–53.（式 3.5：$\Lambda,\mu,p$）
7. J.-J. E. Slotine, W. Li, "On the adaptive control of robot manipulators," *Int. J. Robotics Research* 6(3), 1987, 49–59.（式 3.9）
8. B. Paden, R. Panja, "Globally asymptotically stable 'PD+' controller for robot manipulators," *Int. J. Control* 47(6), 1988, 1697–1712.（式 3.8）
9. "Robust torque-computed control for a robot manipulator with unit dual quaternion," 2025（ResearchGate 预印/期刊版，实施 C3 前需精读核对）.（DQ 动力学鲁棒 CTC，最贴近的 DQ 基准）
10. F. F. A. Silva, J. J. Quiroz-Omaña, B. V. Adorno, "Dynamics of Mobile Manipulators Using Dual Quaternion Algebra," *ASME J. Mechanisms and Robotics* 14(6), 2022, 061005.（DQ twist/wrench 递推 NE 与 Gauss 原理）
11. C. Gaz, F. Flacco, A. De Luca, "Identifying the dynamic model used by the KUKA LWR: A reverse engineering approach," *IEEE ICRA*, 2014, 1386–1392.（LWR4+/LBR4+ 名义 $M,C,g$ 参数来源）
12. A. Jubien, M. Gautier, A. Janot, "Dynamic identification of the Kuka LWR robot using motor torques and joint torque sensors data," *IFAC World Congress*, 2014.（LWR4+ 辨识交叉验证）
13. A. Isidori, *Nonlinear Control Systems*, 3rd ed., Springer, 1995.（反馈线性化一般理论）
14. S. P. Bhat, D. S. Bernstein, "A topological obstruction to continuously global stabilization of rotational motion and the unwinding phenomenon," *Systems & Control Letters* 39(1), 2000, 63–70.（连续反馈全局镇定的拓扑障碍）
15. M. Takegaki, S. Arimoto, "A new feedback method for dynamic control of manipulators," *ASME J. Dynamic Systems, Measurement, and Control* 103(2), 1981, 119–125.（式 3.7）
16. R. Ortega, M. W. Spong, "Adaptive motion control of rigid robots: A tutorial," *Automatica* 25(6), 1989, 877–888.（被动性范式综述）
17. B. V. Adorno et al., 对偶四元数机器人建模与控制三部曲教程（Part I: Fundamentals），hal-01478225, 2017.（DQ 建模控制系统性参考）
18. M. W. Spong, "On the robust control of robot manipulators," *IEEE Trans. Automatic Control* 37(11), 1992, 1782–1786.（式 3.10）
19. A. van der Schaft, *L2-Gain and Passivity Techniques in Nonlinear Control*, 3rd ed., Springer, 2017.（式 3.11：耗散不等式与非线性 H∞）
20. B.-S. Chen, T.-S. Lee, J.-H. Feng, "A nonlinear H∞ control design in robotic systems under parameter perturbation and external disturbance," *Int. J. Control* 59(2), 1994, 439–461.（机械臂非线性 H∞ 跟踪）
21. E. D. Sontag, "Input to state stability: Basic concepts and results," in *Nonlinear and Optimal Control Theory*, Springer, 2008, 163–220.（式 3.12：ISS 框架）
22. A. Cohen, M. Shoham, "Application of hyper-dual numbers to rigid bodies equations of motion," *Mechanism and Machine Theory* 111 (2017) 76–84.（[C&S17]，HDN 刚体动力学）
23. X. Wang, C. Yu, "Unit dual quaternion-based feedback linearization tracking problem for attitude and position dynamics," *Systems & Control Letters* 62(3), 2013, 225–233.（单刚体 DQ 反馈线性化）
24. M. de A. Fonseca, B. V. Adorno, P. Fraisse, "Coupled task-space admittance controller using dual quaternion logarithmic mapping," *IEEE Robotics and Automation Letters* 5(4), 2020, 6057–6064.（DQ 任务空间交互控制）
25. A. Cohen, M. Shoham, "Hyper Dual Quaternions representation of rigid bodies kinematics," *Mechanism and Machine Theory* 150 (2020) 103861.（[P1]，HDQ 运动学）

**项目内部文档**：`docs/数学理论与代码实现详解.md`（主文档）；
`docs/HDQ动力学建模扩展_Jdot与Hessian.md`（(D-1)–(D-9)）；
`docs/HDQ高阶结构动力学创新应用分析.md`（(E-1)–(E-7)、§3.4 分速度域评估）；
`docs/TNDQ论文初稿_运动学重构_误差体系与控制律.md`（定理 1/2/3、式 (5.1)(5.2)(5.6)(5.7)）。

---

*文档生成说明：§1 全部数值直接取自 `TNDQ_sim/results/` 的 npz 原始数组与定宽表格
（统计量由 npz 重新计算核对）；§3–§4 公式均标注文献出处；§5 为设计方案，
未包含任何代码实现。*
