# Third-Order Dual Quaternion (TODQ) Kinematics with Geometrically Consistent Error Systems: A Computed Torque Approach to H∞/ISS Contro

## 摘要

对偶四元数（DQ）为机械臂位姿提供了全局无奇异参数化，但现有框架仅携带零阶信息，向动力学接口延伸时存在位姿/速度误差分离、加速度扰动无入口、偏差型不确定性不满足 $L_2$ 假设三个结构性缺口。本文引入**三阶对偶四元数**（TODQ）代数 $\mathcal A_2$ [Con25]，使正运动学一次连乘同时输出位姿、twist 与任务空间加速度。在此基础上，利用 HDQ 截断与乘法的相容性，将误差体系定义在两项 HDQ 上：一次乘法同时生成右不变位姿误差与几何一致 twist 误差（定理 1），并导出严格级联误差运动学（定理 2）。针对动力学接口设计几何一致计算力矩律，证明闭环满足精确耗散等式 $\dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d$（无任何放缩），据此给出无扰时的水平集不变性与渐近收敛、$L_2$ 扰动下 H∞ 增益的 Schur 补充分必要条件，以及 $L_\infty$ 扰动下 twist 误差的均方极限界（定理 3）。在 7 自由度机械臂力矩模式仿真中验证静态刚度标度律与定理 3 的性能保证。

**关键词**：对偶四元数；超对偶四元数；三阶对偶四元数；多项式代数；几何一致误差；H∞ 控制；输入-状态稳定

---

## 1. 引言

### 1.1 背景与动机

刚体位姿（姿态 + 位置）的参数化是机器人控制的起点。单位对偶四元数将两者装入一个 8 维代数对象，运算全局无奇异，且乘法直接实现位姿复合；[P2] 在此参数化上给出了带 L₂ 扰动衰减保证的运动学跟踪控制器，是 DQ 控制的代表性结果。Cohen & Shoham [P1] 进一步引入超对偶四元数（HDQ）：给 DQ 增加一个幂零单位 $\varepsilon^*$（$\varepsilon^{*2}=0$），使一个 HDQ 元素同时携带位姿与其一阶时间导数，链式乘法自动执行微分（Leibniz 法则内化于乘法），从而正运动学一次传播同时输出位姿与 twist。

但 HDQ 到动力学层面仍差一阶：计算力矩控制需要任务空间加速度 $\dot{\boldsymbol\xi}$ 与雅可比导数项 $\dot J\dot{\boldsymbol q}$，而 [P1] 的 HDQ 结构中两个幂零单位（$\varepsilon$ 承载平移、$\varepsilon^*$ 承载一阶导）均已占用，$\varepsilon\varepsilon^*$ 项只携带"平移分量的一阶导数"，不含二阶时间导数。更重要的是**误差体系**：现有 DQ 框架的误差对象只有位姿一层，扰动模型假设速度级加性扰动且属于 $L_2$；进入动力学接口后出现三个结构性缺口——位姿与速度误差分离（[Ch20] 类律的位姿反馈取螺旋对数，其导数映射在大误差处奇异）、加速度层扰动无入口、偏差型不确定性不满足 $L_2$ 假设，既有证书不再适用。

### 1.2 本文贡献

1. **TODQ 运动学应用**（§3）：定义三项代数 $\mathcal A_2$，使串联链的正运动学一次 $O(n)$ 连乘同时输出 $(\hat{\underline x},\boldsymbol\xi,\dot J\dot{\boldsymbol q})$。
2. **HDQ 误差体系**（§4）：定义 HDQ 误差元素 $\breve{\tilde x}=\breve x(\breve x_d)^*$，一次乘法同时生成位姿误差与几何一致 twist 误差（定理 1），并导出闭式级联运动学 $\dot e_z=A(\tilde x)e_\xi$（定理 2）。
3. **几何一致控制律与混合性能保证**（§5）：设计 $A^\top$-整形计算力矩律，证明闭环误差动态为级联标准形且存储函数满足精确耗散等式；对 $L_2$ 扰动给出 H∞ 增益的 Schur 补**充要**判据（含旋转/平移分量拆分），对 $L_\infty$ 扰动（含乘性不确定性）给出 twist 误差的均方极限界（定理 3）。
4. **仿真验证**（§6）：在 7 自由度机械臂力矩模式仿真中验证静态刚度标度律与定理 3 的性能保证。

TODQ 是 [P1] HDQ 思想（幂零单位承载导数）向二阶的最小扩展 [Con25]；0 阶误差退化为 [P2]（§4.4）。

---

## 2. 预备知识

**记号约定**（全文统一）：同一条位姿曲线用同一核心字母（如 $x$），字母上方的装饰指明它所处的代数层：

| 记号 | 对象 
|---|---|
| $\hat a$ | 四元数 |
| $\hat{\underline a}$ | 对偶四元数 |
| $\breve a$ | HDQ（超对偶四元数） | 
| $\bar a$ | TODQ（三阶对偶四元数） |
| $\tilde{(\cdot)}$ | 误差量 | 
| 粗体 | 纯 DQ（twist 等）、向量与矩阵 |

（表 0：记号约定。标称模型矩阵 $\hat M,\hat C,\hat g$ 上的 hat 沿用控制文献"标称估计"的习惯用法，与四元数装饰无关；四元数虚单位 $\hat\imath,\hat\jmath,\hat k$ 为固定符号；对偶四元数代数统一记作 DQ（§2.1），其元素是一般（未必单位）对偶四元数；带下标的项记号 $\hat{\underline a}_k,\hat{\underline b}_k$（§2.3、定义 1）表示一般对偶四元数项，未必单位——如曲线表示的导数项 $\dot{\hat{\underline x}}$。）

### 2.1 四元数与对偶四元数

单位四元数 $\hat r=\cos\frac\phi2+n\sin\frac\phi2$ 表示绕单位轴 $n$ 转角 $\phi$ 的旋转。

对偶四元数（DQ）代数为 $\mathbb H\oplus\varepsilon\mathbb H$，$\varepsilon^2=0$（$\varepsilon$ 与四元数单位交换）。**单位 DQ**

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

其中 $\omega$ 为角速度。$\boldsymbol\xi$ 是纯 DQ，等价写法即左乘运动学 $\dot{\hat{\underline x}}=\tfrac12\boldsymbol\xi\hat{\underline x}$（[P2] 式(1) 约定）。对 $n$ 关节串联机械臂，正运动学 $\hat{\underline x}(\boldsymbol q)=\prod_{i=1}^n\hat{\underline x}_i(q_i)$（各关节单位 DQ 之积），微分给出 $\mathrm{vec}_6\,\boldsymbol\xi=J(\boldsymbol q)\dot{\boldsymbol q}$，$J\in\mathbb R^{6\times n}$ 称几何雅可比。

**伴随作用与李括号**：$\mathrm{Ad}_{\hat{\underline x}}\boldsymbol a\triangleq\hat{\underline x}\boldsymbol a\hat{\underline x}^*$（twist 的参考系搬运）、$\mathrm{ad}_{\boldsymbol a}\boldsymbol b\triangleq\tfrac12(\boldsymbol{ab}-\boldsymbol{ba})$（李括号的 DQ 形式），二者均保持纯 DQ 结构，详见 [4]。

### 2.3 超对偶四元数（HDQ）

HDQ 元素形如 $\breve a=\hat{\underline a}_0+\varepsilon^*\hat{\underline a}_1$，两个项 $\hat{\underline a}_0,\hat{\underline a}_1$ 均为 DQ（§2.1）；$\varepsilon^*$ 与全部四元数单位交换且 $\varepsilon^{*2}=0$，乘法由分配律给出：

$$
(\hat{\underline a}_0+\varepsilon^*\hat{\underline a}_1)(\hat{\underline b}_0+\varepsilon^*\hat{\underline b}_1)=\hat{\underline a}_0\hat{\underline b}_0+\varepsilon^*(\hat{\underline a}_0\hat{\underline b}_1+\hat{\underline a}_1\hat{\underline b}_0).
\tag{2.3}
$$


**HDQ 共轭**（定理 1 中 $(\breve x_d)^*$ 的含义）定义为**逐项的 DQ 共轭**：

$$
\breve a^{\,*}\triangleq\hat{\underline a}_0^{\,*}+\varepsilon^*\hat{\underline a}_1^{\,*}.
\tag{2.4}
$$

(2.4) 使共轭与求导可交换（$ (\breve x)^* $ 即曲线 $\hat{\underline x}^*(t)$ 的 HDQ 表示），且仍为反自同构：$ (\breve a\breve b)^*=\breve b^{\,*}\breve a^{\,*} $。

---

## 3. TODQ：三阶对偶四元数与运动学重构

### 3.1 代数定义

> **定义 1（TODQ）**[Con25]：三阶对偶四元数（TODQ）的元素形如
>
> $$
> \bar a=\hat{\underline a}_0+\sigma\hat{\underline a}_1+\tfrac12\sigma^2\hat{\underline a}_2,\qquad \hat{\underline a}_0,\hat{\underline a}_1,\hat{\underline a}_2\ \text{均为 DQ}.
> \tag{3.1}
> $$
>
> 全部 TODQ 元素构成的代数记 $\mathcal A_2$。$\sigma$ 与全部四元数单位交换，$\sigma^3=0$。乘法由分配律与 $\sigma^3=0$ 唯一确定：
>
> $$
> \bar a\,\bar b=\hat{\underline a}_0\hat{\underline b}_0+\sigma(\hat{\underline a}_0\hat{\underline b}_1+\hat{\underline a}_1\hat{\underline b}_0)+\tfrac12\sigma^2\bigl(\hat{\underline a}_0\hat{\underline b}_2+2\hat{\underline a}_1\hat{\underline b}_1+\hat{\underline a}_2\hat{\underline b}_0\bigr).
> \tag{3.2}
> $$
>
> **TODQ 共轭**定义为逐项 DQ 共轭（与 (2.4) 同一约定）：$\bar a^{\,*}\triangleq\hat{\underline a}_0^{\,*}+\sigma\hat{\underline a}_1^{\,*}+\tfrac12\sigma^2\hat{\underline a}_2^{\,*}$，它是 $\mathcal A_2$ 上的反自同构$(\bar a\bar b)^*=\bar b^{\,*}\bar a^{\,*}$。


### 3.2 TODQ 串联运动学表示

给定光滑单位 DQ 曲线 $\hat{\underline x}(t)$，其**TODQ 表示**定义为把 $\hat{\underline x}$ 与它的两阶导数按 $\sigma$ 的幂次装入三个项：

$$
\bar x\triangleq\hat{\underline x}+\sigma\dot{\hat{\underline x}}+\tfrac12\sigma^2\ddot{\hat{\underline x}}\ \in\mathcal A_2 .
\tag{3.3a}
$$


对串联臂 $\hat{\underline x}(\boldsymbol q)=\prod_i\hat{\underline x}_i(q_i(t))$，先写出每个关节因子的 TODQ 表示 $\bar x_i$（由 $\partial\hat{\underline x}_i/\partial q_i=\tfrac12\boldsymbol s_i\hat{\underline x}_i$ 与链式法则给出闭式，只依赖 $q_i,\dot q_i,\ddot q_i$），再按 (3.2) 逐个连乘：

$$
\bar x=\bar x_1\,\bar x_2\cdots\bar x_n=\prod_{i=1}^{n}\bar x_i
\tag{3.4}
$$

一次 $O(n)$ 链连乘同时输出 $\hat{\underline x},\dot{\hat{\underline x}},\ddot{\hat{\underline x}}$（$\bar x$ 的三个项）。导出量：

$$
\boldsymbol\xi=2\dot{\hat{\underline x}}\hat{\underline x}^*,\qquad
\dot{\boldsymbol\xi}=2\ddot{\hat{\underline x}}\hat{\underline x}^*-\tfrac12\boldsymbol\xi^2\ \text{（取纯部）},\qquad
\mathrm{vec}_6\dot{\boldsymbol\xi}=\dot J\dot{\boldsymbol q}+J\ddot{\boldsymbol q}.
\tag{3.5}
$$



## 4. 几何一致误差体系

### 4.1 误差对象的阶数

计算力矩律的反馈项（§5）为 $-K_de_\xi-A^\top K_pe_z$，只使用位姿与速度两阶：参考加速度走前馈（来自期望轨迹，确定量），加速度层不确定性归入扰动（进入 $\dot e_\xi$ 方程的 $d(t)$，§5.2）；引入加速度误差只会把高噪声的加速度估计（差分方差 $\propto\Delta t^{-4}$）带入反馈并增加一维动态。因此**正运动学与期望轨迹用 TODQ 建模（前馈需要 $\sigma^2$ 项），误差体系定义在 HDQ 上**。

### 4.2 定理 1：误差的 HDQ 表示

实测链取 HDQ 表示 $\breve x=\hat{\underline x}+\varepsilon^*\dot{\hat{\underline x}}$；期望链取 $\breve x_d=\hat{\underline x}_d+\varepsilon^*\dot{\hat{\underline x}}_d$。

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
> 即一次 HDQ 乘法（3 次 DQ 乘）同时给出位姿误差 $\tilde x$ 与其导数；0 阶项正是 [P2] 的 $\tilde x$。进一步定义**几何一致 twist 误差**
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
> 即"实际 twist 减去**搬运到当前位姿处**的期望 twist"。
>
> **证明**：(4.2)：将 (2.3) 用于 $\hat{\underline a}_0=\hat{\underline x},\hat{\underline a}_1=\dot{\hat{\underline x}},\hat{\underline b}_0=\hat{\underline x}_d^{\,*},\hat{\underline b}_1=\dot{\hat{\underline x}}_d^{\,*}$（共轭与求导交换保证 $(\breve x_d)^*$ 的 $\varepsilon^*$ 项为 $\dot{\hat{\underline x}}_d^{\,*}$），$\varepsilon^*$ 项即 Leibniz 展开的 $\frac{d}{dt}(\hat{\underline x}\hat{\underline x}_d^{\,*})$。(i)(ii)(iii) 的推导见附录 A.1——(iii) 的关键一步：$\dot{\tilde x}=\dot{\hat{\underline x}}\hat{\underline x}_d^{\,*}+\hat{\underline x}\dot{\hat{\underline x}}_d^{\,*}=\tfrac12\boldsymbol\xi\tilde x-\tfrac12\tilde x\boldsymbol\xi_d$（用 $\dot{\hat{\underline x}}_d^{\,*}=-\tfrac12\hat{\underline x}_d^{\,*}\boldsymbol\xi_d$），右乘 $2\tilde x^*$ 得 (4.4)。∎

### 4.3 定理 2：输出误差的级联运动学

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
> **证明**：旋转分量：由 $\dot{\tilde r}=\tfrac12\tilde\omega\tilde r$（定理 1(ii) 的四元数部），$\dot{\mathcal O}=-\mathrm{Im}(\tfrac12\tilde\omega\tilde r)=-\tfrac12(\tilde\eta I_3+[\mathcal O]_\times)\tilde\omega$（用 $\tilde\omega\times\tilde\mu=[\mathcal O]_\times\tilde\omega$）。平移分量：由 twist 约定 $\tilde v=\dot{\tilde p}+\tilde p\times\tilde\omega$ 得 $\dot{\mathcal T}=\tilde v-[\mathcal T]_\times\tilde\omega$。∎

定理 1 + 定理 2 给出严格的两层级联：$e_z\xrightarrow{A}e_\xi$，$\dot e_z=Ae_\xi$；$\dot e_\xi$ 的动态由控制律与扰动决定（§5）。误差状态共 12 维——不含加速度层，这是 §4.1 论证的结构性取舍。

---

## 5. 几何一致控制律与混合 H∞/ISS 性能

### 5.1 扰动项：显式解算、乘法项分离与假设集

真实关节空间动力学（含摩擦 $\boldsymbol f$、执行器实现误差 $\delta\boldsymbol\tau$ 与环境接触 $\boldsymbol\tau_{\mathrm{ext}}$）记为 $M(\boldsymbol q)\ddot{\boldsymbol q}+C(\boldsymbol q,\dot{\boldsymbol q})\dot{\boldsymbol q}+\boldsymbol g(\boldsymbol q)+\boldsymbol f=\boldsymbol\tau+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}}$；内环以标称模型执行计算力矩 $\boldsymbol\tau=\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+\hat C\dot{\boldsymbol q}+\hat g+\hat{\boldsymbol f}$。失配量取"标称减真实"：$\Delta M\triangleq\hat M-M$，$\Delta C,\Delta\boldsymbol g,\Delta\boldsymbol f$ 同理。代入并左乘 $M^{-1}$ 得 $\ddot{\boldsymbol q}=\ddot{\boldsymbol q}_{\mathrm{ref}}+\boldsymbol w_{\mathrm{dyn}}$，其中

$$
\boldsymbol w_{\mathrm{dyn}}=M^{-1}\bigl(\Delta M\,\ddot{\boldsymbol q}_{\mathrm{ref}}+\Delta C\dot{\boldsymbol q}+\Delta\boldsymbol g+\Delta\boldsymbol f+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}}\bigr).
\tag{5.1}
$$

**乘法分量与外生分量的分离**。将 $\ddot{\boldsymbol q}_{\mathrm{ref}}=J^{+}(u_{\mathrm{ff}}+u_{\mathrm{fb}}-\dot J\dot{\boldsymbol q})$ 代入 (5.1)，利用 $JJ^{+}=I_6$ 得扰动分解

$$
d=\Theta(\boldsymbol q)\,u_{\mathrm{fb}}+d_{\mathrm{ex}},\qquad
\Theta\triangleq J M^{-1}\Delta M\,J^{+}\in\mathbb R^{6\times6},\;
\tag{5.1d}
$$

其中 $\Theta$ 为乘性分量、$d_{\mathrm{ex}}$ 为外生扰动（详见附录 C.1）。$d_{\mathrm{ex}}$ 再按时间特性分解为 $d_{\mathrm{ex}}=d_{L_2}+d_b$：$d_{L_2}\in L_2$（噪声型）由定理 3(c) 处理，$d_b\in L_\infty$（偏差型）由定理 3(d) 处理。

**假设集**（定理 3 全文沉默使用，在此一次性列出）：

- **(A1) 雅可比正则**：存在工作集 $\mathcal Q$ 使 $J(\boldsymbol q)$ 在 $\mathcal Q$ 上行满秩且 $\sigma_{\min}(J)\ge\underline\sigma>0$，从而 $JJ^{+}=I_6$（奇异邻域取阻尼伪逆 [Nak86]，残差归入 $d_{\mathrm{ex}}$）。
- **(A2) 惯量与失配有界**：$M(\boldsymbol q)\succeq \underline m I_n$（$\underline m>0$），且 $\Delta M,\Delta C,\Delta\boldsymbol g,\Delta\boldsymbol f$ 在 $\mathcal Q\times\{\|\dot{\boldsymbol q}\|\le\bar v\}$ 上有界。
- **(A3) 乘法小增益条件**：
  $$
  \alpha\triangleq\sup_{\boldsymbol q\in\mathcal Q}\bigl\|\Theta(\boldsymbol q)\bigr\|_2
  \;<\;\frac{\lambda_{\min}(K_d)}{\lambda_{\max}(K_d)}=\frac1{\mathrm{cond}_2(K_d)}\ \le 1 .
  \tag{5.1f}
  $$
  各向同性阻尼时退化为经典计算力矩鲁棒性条件 $\alpha<1$ [Spo92]。
- **(A4) 期望轨迹与外生信号**：$\hat{\underline x}_d(t)$ 为 $C^2$ 且 $\|\boldsymbol\xi_d\|,\|\dot{\boldsymbol\xi}_d\|$ 有界；$d_{L_2}\in L_2$、$d_b\in L_\infty$，且 $D_{\mathrm{ex}}\triangleq\|d_{\mathrm{ex}}\|_{L_\infty}<\infty$（对 $\dot{\boldsymbol v}_w$ 的有界性要求实现上取滤波导数 [Ber93]）。

### 5.2 控制律

取 $K_p$ 对称正定（典型取块对角 $K_p=\mathrm{diag}(p_O I_3,p_T I_3)$；取 $K_p=k_pI_6$ 即恢复单标量增益情形）、$K_d$ 对称正定（若需定理 3(c-2) 的旋转/平移逐分量 H∞ 界，则取块对角 $K_d=\mathrm{diag}(K_\omega,K_v)$，$K_\omega,K_v\in\mathbb R^{3\times3}$ 对称正定），定义参考加速度（几何一致计算力矩律）：

$$
\ddot{\boldsymbol q}_{\mathrm{ref}}
=J^{+}\Bigl(\underbrace{\mathrm{vec}_6\bigl(\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d+\mathrm{ad}_{\tilde{\boldsymbol\xi}}\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d\bigr)}_{\text{前馈：搬运的期望加速度 + 输运修正}}
-K_d\,e_\xi-A^{\top}(\tilde x)\,K_p\,e_z-\dot J\dot{\boldsymbol q}\Bigr).
\tag{5.2}
$$

其中 $A^\top$-整形位姿反馈使 Lyapunov 导数的交叉项精确相消（定理 3 证明），前馈与 $\dot J\dot{\boldsymbol q}$ 分别取自期望链的 $\sigma^2$ 项与 (3.5)；全部反馈量取自定理 1 的 HDQ 误差元素，不需要任何加速度误差的测量或估计。

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
> **证明**：附录 A.2（由 $\dot{\tilde x}=\tfrac12\tilde{\boldsymbol\xi}\tilde x$ 与其共轭直接展开）。

### 5.3 主定理

本节四个结果（定理 3(a)–3(d)）共用假设 (A1)–(A4)（§5.1）、控制律 (5.2) 与扰动模型 (5.1)/(5.1d)；统一存储函数

$$
V(e_z,e_\xi)=\tfrac12\|e_\xi\|^2+\tfrac12\,e_z^{\top}K_pe_z\;\ge0 ,
\tag{5.4a}
$$

$K_p\succ0$ 对称（取 $K_p=k_pI_6$ 即回到标量形式 $\tfrac{k_p}2\|e_z\|^2$）。以下两点决定结论的强弱：(i) $\tilde\eta=0$ 处 $A(\tilde x)$ 奇异且 $SO(3)$ 的双覆盖带来拓扑障碍，全部收敛声明都是工作域（水平集）内的局部结论；(ii) $\dot V$ 只含 $-\|e_\xi\|^2$ 型负项而无 $-\|e_z\|^2$ 项（扰动到 $e_z$ 的相对阶为 2），故 $V$ 不能单独给出 $e_\xi$ 的逐点极限球（定理 3(d) 注记）。

> **定理 3(a)（闭环误差动态）**：误差坐标 $(e_z,e_\xi)$ 满足级联标准形
>
> $$
> \dot e_z=A(\tilde x)\,e_\xi,\qquad
> \dot e_\xi=-K_d e_\xi-A^{\top}(\tilde x)\,K_p\,e_z+d(t),
> \tag{5.5}
> $$
>
> 其中 $d=J\boldsymbol w_{\mathrm{dyn}}+\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$ 汇集全部加速度层扰动，并按 (5.1d) 精确分解为 $d=\Theta u_{\mathrm{fb}}+d_{\mathrm{ex}}$。

> **证明**：将 (5.2) 代入 (3.5) 并利用引理 1，前馈项与期望/输运项精确相消即得 (5.5)。

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

> **证明**：关键步骤是 $A^\top$-整形反馈使交叉项 $e_z^\top K_pAe_\xi$ 精确相消，得 $\dot V=-e_\xi^\top K_de_\xi$；LaSalle 定理与 $A$ 在 $\Omega_c$ 上的一致可逆性（附录 C.2）给出渐近收敛。详见附录 C.6。

> **注记**：全局指数稳定不可得，原因在于 $SO(3)$ 双覆盖的拓扑障碍（unwinding）与 $A$ 在 $\tilde\eta=0$ 处的奇异性。水平集条件 (5.5b) 可数值核验：§6 tuned 档 $c^{*}=160$，实测 $V^{\mathrm{tuned}}_{\mathrm{peak}}\le0.494$，余度约 2.5 个数量级（§6.4）。当 $\tilde\eta<0$ 时将 HDQ 误差整体翻转符号（定理 1(i) 不改变 $\tilde{\boldsymbol\xi}$）即强制 $\tilde\eta\ge0$。∎

> **定理 3(c)（$L_2$ 扰动：H∞ 二次型/Schur 补判据与旋转/平移分量拆分）**：设 $d=d_{L_2}\in L_2$。
>
> **证书参数**：$\kappa>0$ 是**误差罚权的倒数**（供给率中 $\|e_\xi\|^2$ 项的系数取 $\tfrac1{2\kappa}$，量纲 s），$\gamma_a>0$ 是**待认证的加速度层扰动衰减水平**（量纲 s$^{1/2}$），被认证量是 $d\to e_\xi$ 的 $L_2$ 能量增益 $\gamma_a\sqrt\kappa$（无量纲）。二者均为**分析参数**：不出现在控制律 (5.2) 中，只出现在证书 (5.6a) 中；分量拆分版本中 $(\kappa_\omega,\gamma_\omega)$ 与 $(\kappa_v,\gamma_v)$ 可独立指定。
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
> 即加速度层扰动到 twist 误差能量的 $L_2$ 增益 $\le\gamma_a\sqrt\kappa$（零初值时退化为纯增益界；此界只约束 $e_\xi$）。
>
> **(c-2) 分量拆分判据**（$K_d=\mathrm{diag}(K_\omega,K_v)$ 块对角，且 $K_p=\mathrm{diag}(K_{p,O},\,k_{p,T}I_3)$——平移刚度块须**各向同性**，必要性见附录 C.3）：记 $e_\xi=[\tilde\omega;\tilde v]$、$d=[d_\omega;d_v]$，$V_\omega\triangleq\tfrac12\|\tilde\omega\|^2+\tfrac12\mathcal O^\top K_{p,O}\mathcal O$、$V_v\triangleq\tfrac12\|\tilde v\|^2+\tfrac{k_{p,T}}2\|\mathcal T\|^2$（$V=V_\omega+V_v$）。若
>
> $$
> K_\omega\succeq\tfrac12\bigl(\kappa_\omega^{-1}+\gamma_\omega^{-2}\bigr)I_3,
> \qquad
> K_v\succeq\tfrac12\bigl(\kappa_v^{-1}+\gamma_v^{-2}\bigr)I_3,
> \tag{5.6b}
> $$
>
> 则旋转/平移两分量**各自独立**满足
>
> $$
> \int_0^\infty\kappa_\omega^{-1}\|\tilde\omega\|^2dt\le\gamma_\omega^2\int_0^\infty\|d_\omega\|^2dt+2V_\omega(0),
> \qquad
> \int_0^\infty\kappa_v^{-1}\|\tilde v\|^2dt\le\gamma_v^2\int_0^\infty\|d_v\|^2dt+2V_v(0),
> \tag{5.6$'$}
> $$
>
> 且逐分量量纲齐次（不再混合 $(\mathrm{rad/s})^2$ 与 $(\mathrm{m/s})^2$），$\kappa_\omega,\gamma_\omega,\kappa_v,\gamma_v$ 可独立指定。
>
> **证明**：含扰时交叉项仍精确相消（与 $d$ 无关），得 $\dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d$；将性能目标写为 $[e_\xi;d]^\top M[e_\xi;d]\ge0$，取 Schur 补即得充要判据 (5.6a)；分量解耦依赖两处代数恒零（$\mathcal T\times\mathcal T=0$、$\mathcal T\cdot(\mathcal T\times\tilde\omega)=0$）。详见附录 C.7。


> **定理 3(d)（$L_\infty$ 扰动：乘法分量分离与 twist 误差的均方极限界）**：设 (A1)–(A4) 成立，扰动项按 (5.1d) 分解为 $d=\Theta u_{\mathrm{fb}}+d_{\mathrm{ex}}$，$\alpha\triangleq\sup_t\|\Theta\|_2$ 满足小增益条件 (5.1f)，$D_{\mathrm{ex}}\triangleq\|d_{\mathrm{ex}}\|_{L_\infty}<\infty$。记**有效阻尼**
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
> 即：偏差型（$L_\infty$）不确定性不破坏有界性，只按 $D/\lambda_{\mathrm{eff}}$ 抬高 twist 误差的均方稳态水平。乘法分量 $\Theta u_{\mathrm{fb}}$ 的作用有二——以 $\alpha\lambda_{\max}(K_d)$ 折减有效阻尼（分母）、以 $\alpha\lambda_{\max}(K_p)\|e_z\|$ 抬高等效扰动幅值（分子）——二者均在 $\alpha\to0$ 时消失。位姿误差 $e_z$ 不在本定理结论之内，其稳态量级由近恒等线性化模型 (5.9) 给出准静态估计。

> **证明**：将乘性扰动 $\Theta u_{\mathrm{fb}}$ 拆为阻尼折减项与等效扰动项，分别用 Young 不等式回收，积分即得 RMS 界 (5.7)。详见附录 C.8。

> **注记**：因 $\dot V$ 的负项只含 $-\|e_\xi\|^2$（扰动到 $e_z$ 的相对阶为 2），此处给出的是 RMS 界而非逐点 ISS 界；严格化需 strictification（附录 C.4）。$\Omega_c$ 前提需事后数值核验（§6.4：实测余度约 2.5 个数量级）。∎

**边界说明**：(i) 奇异邻域内取阻尼伪逆时 $JJ^+\ne I$，残差归入 $d_{\mathrm{ex}}$；(ii) 本文不声称全状态 ISS，也不声称与运动学外环级联后的整体 H∞ 界；(iii) (5.6)/(5.6$'$)/(5.7) 均只约束 $e_\xi$；$e_z$ 的稳态量级由 (5.9) 给出工程估计，严格上界需 strictification（附录 C.4）；(iv) 分量拆分 (c-2) 依赖 $K_d$ 块对角与 $K_{p,T}$ 各向同性，否则退回 (c-1)。

### 5.4 近恒等线性化模型与静态刚度标度律

定理 3 给出的是定性与能量层面的结论，不直接给出增益数值。本节在原点邻域把 (5.5) 线性化，得到一个可直接用于增益整定且可实验证伪的两分量二阶模型。

取 $K_d=\mathrm{diag}(K_\omega,K_v)$、$K_p=\mathrm{diag}(K_{p,O},k_{p,T}I_3)$，在 $\tilde x\to1$ 处 $A\to A_0=\mathrm{diag}(-\tfrac12I_3,I_3)$，消去 $e_\xi$ 得两条解耦的二阶方程：

$$
\boxed{\;
\ddot{\mathcal O}+K_\omega\dot{\mathcal O}+\tfrac14K_{p,O}\,\mathcal O=-\tfrac12\,d_\omega ,
\qquad
\ddot{\mathcal T}+K_v\dot{\mathcal T}+k_{p,T}\,\mathcal T=+\,d_v .\;}
\tag{5.8}
$$

旋转块 $-\tfrac12I_3$ 使有效旋转刚度为 $\tfrac14K_{p,O}$；为配平带宽，取 $K_{p,O}=4k_{p,T}I_3$。令 $\ddot{(\cdot)}=\dot{(\cdot)}=0$ 得**静态刚度标度律**

$$
\boxed{\;
\|\mathcal T\|_{\mathrm{ss}}=\frac{\|d_v\|}{k_{p,T}} ,
\qquad
\|\mathcal O\|_{\mathrm{ss}}=\frac{2\,\|d_\omega\|}{\lambda(K_{p,O})} ,\;}
\tag{5.9}
$$

即稳态残差**只**由静态刚度决定、与阻尼无关。该折减因子将在 §6 中由独立增益档的扰动反演一致性进行实验确认。极点分配规则：若各分量目标极点为 $\{-a,-b\}$（$a,b>0$），则

$$
K_\omega=K_v=(a+b)I_3,\qquad k_{p,T}=ab,\qquad K_{p,O}=4ab\,I_3 .
$$

由 (5.9) 可得两个实验预言：**(P1)** 刚度提高 $\rho$ 倍，稳态残差降至 $1/\rho$；**(P2)** 同一物理工况下，不同增益档反演的 $\|d_v\|,\|d_\omega\|$ 应一致。(5.8)–(5.9) 为 $\tilde x\to1$ 的局部近似；严格结论以定理 3 为准。预言 (P2) 将在 §6.3.1 中由独立增益档的实验数据进行检验。

---

## 6. 仿真验证

### 6.1 平台与任务

在 CoppeliaSim 中以 7-DoF KUKA LBR4+ 力矩模式仿真（控制周期 5 ms），名义动力学取自 [Gaz14]。$t=2.5\,\text{s}$ 时 0.25 kg 未建模负载刚性附着，构成持续偏差型扰动 $d_{\mathrm{ex}}$。任务为抓取–搬运–圆周跟踪（半径 0.06 m，标准角速度 1.0 rad/s，高速工况 2.5 rad/s）。对比三种控制器：**C1**（本文，式 (5.2)）、**C2**（[Ch20]，twist 误差与本文相同但位姿反馈取螺旋对数）、**C3**（[P2] 经速度伺服桥接至加速度级）。三律共享同一轨迹、初始条件与名义模型，仅 $\ddot{\boldsymbol q}_{\mathrm{ref}}$ 计算分支不同；DC 刚度均配平至 80，闭环极点 $\{-4,-20\}$，以隔离结构差异。

### 6.2 验证目标：从定理到可观测预言

仿真不追求"性能竞赛"式的百分比优胜，而是检验 §3–5 理论框架的三类可证伪预言：

**(V1) 静态刚度标度律 (5.9)**：若定理 3(d) 的扰动模型与 §5.4 的线性化正确，则同一物理扰动 $d_{\mathrm{ex}}$ 应由不同增益档的稳态残差反演出一致幅值；且 1/4 旋转刚度折减因子应被独立确认。

**(V2) 证书分化的极限暴露**：C1 与 C2 共享解析前馈与几何一致 twist 误差，预期在近恒等域数值等价（确认"同信息集"）；但 C1 具备定理 3 的严格耗散等式与 H∞ 证书，C2/C3 没有。这一分化应在离开理想条件的极限工况（高速、噪声）下表现为 C1/C2 与 C3 的结构性分离——而非随机波动。

**(V3) 水平集工程余度**：定理 3(b) 给出水平集 $\Omega_c$ 的正向不变性，预期在实际运行中应留有充分余度，以确认局部结论的工程可用性。

### 6.3 结果

#### 6.3.1 静态刚度标度律与扰动反演（验证 V1）

带载稳态（$t\ge 12.5\,\text{s}$）下，C1-tuned 与 C1-fast 的位姿残差如下：

| 档位 | $K_p=\mathrm{diag}(p_O I_3, p_T I_3)$ | $\|\mathcal T\|_{\mathrm{rms}}$ (m) | $\|\mathcal O\|_{\mathrm{rms}}$ |
|---|---|---|---|
| tuned | $(320, 80)$ | $4.86\times10^{-3}$ | $4.27\times10^{-3}$ |
| fast | $(720, 180)$ | $2.20\times10^{-3}$ | $1.93\times10^{-3}$ |

由 (5.9) 反演等效扰动：
$$\|d_v\| = k_{p,T}\|\mathcal T\|_{\mathrm{ss}}, \qquad \|d_\omega\| = \tfrac12\lambda(K_{p,O})\|\mathcal O\|_{\mathrm{ss}}.$$
得 tuned 档 $(\|d_v\|, \|d_\omega\|) = (0.389, 0.683)$，fast 档 $(0.396, 0.696)$，两档一致至 **1.93%**。该强一致性（两个独立增益档反演出同一物理量）直接验证了定理 3(d) 扰动模型的结构正确性——若 1/4 旋转折减因子缺失，旋转反演值将与平移分量相差 3.5 倍，一致性立即破坏。

作为对照，base 档（$K_p=16I_6$，未补偿 1/4 折减，有效旋转刚度仅 4）带载姿态误差放大至 tuned 档的 12.3 倍，反演扰动偏离 35%–38%。这从反面验证了 §5.4 的 1/4 折减补偿规则的必要性：不补偿则旋转/平移带宽严重失配，(5.9) 失效。

#### 6.3.2 极限工况下的证书分化（验证 V2）

标准工况下，三律位置级残差差异 $<0.1\%$（静态刚度锁定），速度级 $\|e_\xi\|_{\mathrm{rms}}$（×10⁻³）如下：

| 工况 | C1 | C2 | C3 |
|---|---|---|---|
| 标准 | 0.940 | 0.940 | 0.955 |
| 高速 ($\omega=2.5$) | 2.314 | 2.314 | 2.361 |
| 噪声 | 3.164 | 3.165 | 3.234 |

**近恒等域的结构等价**：C1 与 C2 在全部工况下数值等价（相对差 $\le 0.05\%$），确认二者共享同一信息集（解析前馈 + $\mathrm{Ad}$-搬运 twist 误差）。差异仅在于位姿反馈整形（$A^\top$ vs 螺旋对数），在近恒等域为高阶项——这定量支持了本文的核心主张：C1 相对 C2 的优势不在精度，而在证书（定理 3 的耗散等式、水平集不变性、均方界对 $A^\top$ 整形成立，对螺旋对数不成立）。

**极限工况的结构性分离**：C3 在噪声下劣化最明显（3.234 vs 3.165，相对 +2.2%），与其桥接差分 $\Delta\dot q_{\mathrm{cmd}}/\Delta t$ 的噪声放大机制一致；在高速下劣化 +2.0%，源于其一阶前馈无法解析补偿向心加速度的 $\omega^2$ 项。C1/C2 的 TNDQ 解析前馈避免了数值差分，故在极限工况下保持紧致——这验证了"有严格动力学证书"与"无证书桥接"在工程上的分化。

#### 6.3.3 水平集余度（验证 V3）

杯附着后负载突变瞬间，$V^{\mathrm{base}}$ 峰值 $2.47\times10^{-2}$，在 $1.50\,\text{s}$ 内回落至 $3.36\times10^{-4}$；换算至 tuned 档权重后 $V^{\mathrm{tuned}}_{\mathrm{peak}}\le 0.494$，距定理 3(b) 水平集阈值 $c^*=160$ 余度约 **2.5 个数量级**。这表明定理 3(b) 的局部结论在实际运行中具有充足的工程裕度，并非仅存在于任意小邻域的理论陈述。

### 6.4 讨论：理论优势的仿真映射

上述结果可映射回 §3–5 的理论结构如下：

**(i) 定理 3(d) 的实用性**：$L_\infty$ 均方界 (5.7) 预测了偏差型扰动下的稳态误差水平，而 (5.9) 进一步给出了增益整定的解析规则——仿真中 1.93% 的反演一致性确认了该规则的有效性。

**(ii) 定理 3(b) 的工程余度**：水平集 $\Omega_c$ 在实际运行中留有 2.5 个数量级余度，说明局部指数稳定结论在典型工况下是保守但可用的。

**(iii) 证书分化的可观测性**：C1 与 C2 的数值等价确认了"同信息集"前提，使得"C1 有证书、C2 无证书"成为可辩护的理论分化而非精度吹嘘；C3 在极限工况下的系统性劣化（噪声/高速）则从反面验证了动力学证书的价值——无严格前馈结构时，离散化与差分误差在应力下累积。

**(iv) 局限**：每组仅单次运行，1.5%–2% 的相对差异不具备统计显著性；真机实验与更大负载范围为后续工作。

---

## 7. 结论

本文以三项代数 $\mathcal A_2$（TODQ）重构机械臂运动学，核心是两条法则：连乘法则 $\overline{xy}=\bar x\,\bar y$（使位姿/速度/加速度一次链连乘同时得到）与截断相容性 $\breve x=$"$\bar x$ 的前两项"（使误差体系可以无损地定义在两项 HDQ 上）。误差体系由一次 HDQ 乘法生成（定理 1），经输出映射闭合为级联运动学（定理 2）；几何一致计算力矩律使闭环达到级联标准形，并在一个共同的存储函数上得到三类证书：无扰时的**水平集不变性 + 渐近收敛 + 局部指数稳定**（定理 3(b)）、$L_2$ 扰动下的 **H∞ 二次型/Schur 补当且仅当判据**与旋转/平移分量的精确拆分（定理 3(c)）、$L_\infty$ 扰动下 twist 误差的**均方（RMS）极限界**（定理 3(d)）。加速度层被证明不需要误差项：期望加速度走前馈、不确定性走扰动——这一结构性取舍同时简化了状态空间（12 维）与实现（误差层只用 DQ/HDQ 乘法）。近恒等线性化模型 (5.8) 进一步揭示了一个容易被忽略的实现陷阱：$A_0$ 的旋转块 $-\tfrac12I_3$ 使旋转刚度受 **1/4 折减**，不补偿则两分量带宽严重失配（§6.3.1 以 base 档 12.3 倍的姿态误差放大从反面验证）。CoppeliaSim/KUKA LBR4+ 力矩模式仿真（§6）定量核验了上述主张：静态刚度标度律 (5.9) 的**等效扰动反演一致性**在两个分量、两个增益档上同步符合至 1.93%；所提控制律在全部极限工况（高速、噪声）的速度级指标上以方向一致性优于一阶桥接基线 C3（+2.0%–2.2%）；与忠实 [Ch20] 二阶基线 C2 数值等价（≤0.05%）——该等价定量确认了两律同信息集的结构分析，本文相对 [Ch20] 的主张为同性能下的证书增益（定理 3 的耗散等式/水平集不变性/均方界）与大误差几何鲁棒性，而非精度提升；水平集条件与两类证书均未被违反。

**局限与后续工作**：(i) strictification（附录 C.4）未完成，$\Omega_c$ 前提尚未自洽闭合；(ii) 稳定性结论均为工作域局部——unwinding 与 $\tilde\eta=0$ 处 $A$ 的奇异是拓扑障碍；(iii) 真机实验待验证。

---

## 附录 A：DQ/HDQ 层的验证性推导

### A.1 定理 1 的 (i)(ii)(iii)

(ii)：由 (4.3) 直接右乘 $\tilde x$。(i)：翻转不变性：$2(-\dot{\tilde x})(-\tilde x^*)=2\dot{\tilde x}\tilde x^*$。(iii)：$\dot{\tilde x}=\dot{\hat{\underline x}}\hat{\underline x}_d^{\,*}+\hat{\underline x}\dot{\hat{\underline x}}_d^{\,*}$；用 $\dot{\hat{\underline x}}=\tfrac12\boldsymbol\xi\hat{\underline x}$ 与 $\dot{\hat{\underline x}}_d^{\,*}=-\tfrac12\hat{\underline x}_d^{\,*}\boldsymbol\xi_d$（后者由 $\dot{\hat{\underline x}}_d=\tfrac12\boldsymbol\xi_d\hat{\underline x}_d$ 取共轭）：
$\dot{\tilde x}=\tfrac12\boldsymbol\xi\tilde x-\tfrac12\hat{\underline x}\hat{\underline x}_d^{\,*}\boldsymbol\xi_d=\tfrac12\boldsymbol\xi\tilde x-\tfrac12\tilde x\boldsymbol\xi_d$。右乘 $2\tilde x^*$：$\tilde{\boldsymbol\xi}=\boldsymbol\xi-\tilde x\boldsymbol\xi_d\tilde x^*=\boldsymbol\xi-\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d$。含速度级扰动 $\boldsymbol v_w,\boldsymbol v_c$（[P2] 模型）时右端加 $\boldsymbol v_w+\boldsymbol v_c$。

### A.2 引理 1 的证明

$\tfrac{d}{dt}(\tilde x\boldsymbol a\tilde x^*)
=\dot{\tilde x}\boldsymbol a\tilde x^*+\tilde x\dot{\boldsymbol a}\tilde x^*+\tilde x\boldsymbol a\dot{\tilde x}^*$。代入 $\dot{\tilde x}=\tfrac12\tilde{\boldsymbol\xi}\tilde x$、$\dot{\tilde x}^*=-\tfrac12\tilde x^*\tilde{\boldsymbol\xi}$：
$=\tfrac12\tilde{\boldsymbol\xi}(\mathrm{Ad}_{\tilde x}\boldsymbol a)+\mathrm{Ad}_{\tilde x}\dot{\boldsymbol a}-\tfrac12(\mathrm{Ad}_{\tilde x}\boldsymbol a)\tilde{\boldsymbol\xi}
=\mathrm{Ad}_{\tilde x}\dot{\boldsymbol a}+\mathrm{ad}_{\tilde{\boldsymbol\xi}}(\mathrm{Ad}_{\tilde x}\boldsymbol a)$。(5.4)：对 (4.4) 逐项求导并取 $\boldsymbol a=\boldsymbol\xi_d$。∎

## 附录 C：控制层的补充推导

### C.1 扰动项的适定性（(5.1) 为何是显式等式，以及 $\alpha$ 条件的真正作用）

$\boldsymbol\tau=\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+\hat C\dot{\boldsymbol q}+\hat g$ 代入真实动力学 $M\ddot{\boldsymbol q}+C\dot{\boldsymbol q}+\boldsymbol g=\boldsymbol\tau+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}}$ 并解出 $\ddot{\boldsymbol q}$：
$$
\ddot{\boldsymbol q}=M^{-1}\hat M\ddot{\boldsymbol q}_{\mathrm{ref}}+M^{-1}\bigl(\Delta C\dot{\boldsymbol q}+\Delta\boldsymbol g+\delta\boldsymbol\tau+\boldsymbol\tau_{\mathrm{ext}}\bigr),
\qquad M^{-1}\hat M=I+M^{-1}\Delta M ,
$$
即 (5.1)。注意 $\ddot{\boldsymbol q}_{\mathrm{ref}}$ 由 (5.2) 完全由**当前可测量** $(\boldsymbol q,\dot{\boldsymbol q},\hat{\underline x}_d,\boldsymbol\xi_d,\dot{\boldsymbol\xi}_d)$ 给出，**不依赖 $\ddot{\boldsymbol q}$**，故 (5.1) 对 $\ddot{\boldsymbol q}$ 是**显式赋值**：不存在隐式代数环，既不需要移项、也不需要 Neumann 级数，更不会产生 $\alpha/(1-\alpha)$ 型的等效扰动增益因子。

由此可见 $\alpha<1/\mathrm{cond}_2(K_d)$（(5.1f)）与适定性无关，而是为了使定理 3(d) 中的**有效阻尼保持正定**：乘性分量 $\Theta u_{\mathrm{fb}}$ 中与 $-K_de_\xi$ 同向的部分会削弱阻尼，由 $|e_\xi^\top\Theta(-K_de_\xi)|\le\alpha\lambda_{\max}(K_d)\|e_\xi\|^2$ 得 $\lambda_{\mathrm{eff}}=\lambda_{\min}(K_d)-\alpha\lambda_{\max}(K_d)$，正性恰好等价于 (5.1f)。$\alpha$ 条件是一个小增益型的证书条件，而非微分方程适定性条件；若 $\alpha$ 过大，闭环仍然定义良好，但本文的 Lyapunov 证书失效。

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
在 $\Omega_c$ 内 $\tilde\eta\ge\eta_0$（(5.5c)）且 $\|\mathcal T\|\le\sqrt{2c/\lambda_{\min}(K_{p,T})}$，故 $\sigma_{\min}(A)\ge c_A(\eta_0,c)>0$ 一致成立。于是 LaSalle 第 4 步中由 $A^\top K_pe_z\equiv0$ 推出 $\|e_z\|\le\|K_p^{-1}\|_2\|A^{-\top}\|_2\cdot0=0$ 严格成立。局部指数率由线化矩阵 $F$ 的谱给出，逐分量具体极点见 (5.8)。域限制 $\tilde\eta>0$ 与 [P2] Remark 1 的 unwinding 条件一致。

> 注：无扰情形下 LaSalle 路线自身是完整的，但不能改用级联 ISS 论证闭合。

### C.3 定理 3(c) 的二次型/Schur 补细节

**(c-1) Schur 补判据。** 记供给率 $s(e_\xi,d)\triangleq-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$。性能目标 $\dot V\le s$ 等价于 $-e_\xi^\top K_de_\xi+e_\xi^\top d-s(e_\xi,d)\le0$，整理为 $-[e_\xi;d]^\top M[e_\xi;d]\le0$，其中 $M$ 为 (5.6a) 的分块矩阵。$M\succeq0$ 且右下块 $\tfrac{\gamma_a^2}2I\succ0$，取 Schur 补得
$$
M\succeq0\iff K_d-\tfrac1{2\kappa}I-\tfrac1{2\gamma_a^2}I\succeq0\iff K_d\succeq\tfrac12(\kappa^{-1}+\gamma_a^{-2})I .
$$
这是**当且仅当**判据：不定号交叉项 $e_\xi^\top d$ 保留在二次型内整体判定，无任何符号放缩。

**(c-2) 两处恒零的几何含义与失效条件。** 拆分精确成立依赖两条叉积混合积恒等式：(i) $(A^\top e_z)_\omega$ 中 $[\mathcal T]_\times\mathcal T=\mathcal T\times\mathcal T=0$；(ii) $\dot{\mathcal T}=-[\mathcal T]_\times\tilde\omega+\tilde v$ 中耦合项对 $\tfrac12\|\mathcal T\|^2$ 不做功，$\mathcal T^\top(\mathcal T\times\tilde\omega)=0$。二者是代数恒等式，不依赖工作域，故 (c-2) 与 (c-1) 一样是全局结论。若 $K_d$ 非块对角，$-e_\xi^\top K_de_\xi$ 含 $\tilde\omega^\top K_{\omega v}\tilde v$ 型交叉项，两分量能量不再分离，退回合并判据 (c-1)。

**$K_{p,T}$ 各向同性的必要性。** 将平移刚度写作一般对称正定 $K_{p,T}$ 时，上述两处恒零均失效：(i) $(A^\top K_pe_z)_\omega=A_{11}^\top K_{p,O}\mathcal O+[\mathcal T]_\times K_{p,T}\mathcal T$，而 $[\mathcal T]_\times K_{p,T}\mathcal T=\mathcal T\times(K_{p,T}\mathcal T)\ne0$ 除非 $\mathcal T$ 是 $K_{p,T}$ 的特征向量；(ii) 平移储能变为 $\tfrac12\mathcal T^\top K_{p,T}\mathcal T$，其导数中的耦合项 $-\mathcal T^\top K_{p,T}[\mathcal T]_\times\tilde\omega$ 不再为零（仅当 $K_{p,T}=k_{p,T}I_3$ 时由 $\mathcal T^\top[\mathcal T]_\times=0$ 而消失）。两项残留均是 $\tilde\omega$–$\mathcal T$ 型耦合，使 $V_\omega,V_v$ 不再各自闭合，退回 (c-1)。

### C.4 strictification 与局部 ISS（未来工作）

定理 3(d) 的三条缺口的共同根源是 $\dot V$ 中没有 $-\|e_z\|^2$ 型负项。标准补救是 strictification：取 $W=V+\epsilon e_z^\top K_pA(\tilde x)e_\xi$（$\epsilon>0$ 待定），在 $\Omega_c$ 内 $\epsilon$ 足够小时 $W$ 与 $V$ 等价，而求导后新增 $-\epsilon e_z^\top K_pAA^\top K_pe_z$ 项提供 $e_z$ 方向负定。若能验证剩余交叉项（含 $\dot A$ 项）可被两个负定项吸收，则 $\dot W\le-c_1\|(e_z,e_\xi)\|^2+c_2\|d\|\cdot\|(e_z,e_\xi)\|$，得到真正的局部 ISS-Lyapunov 函数。本文未完成此验证（关键难点是 $\dot A$ 项的一致界与 $\epsilon$ 的可行区间非空性），列为 §7 局限 (i)。



### C.6 定理 3(b) 的完整证明

1. **交叉项精确相消**：沿 (5.5)（$d\equiv0$），
$$
\dot V=e_\xi^\top\dot e_\xi+e_z^\top K_p\dot e_z
=e_\xi^\top\bigl(-K_de_\xi-A^\top K_pe_z\bigr)+e_z^\top K_pAe_\xi
=-e_\xi^\top K_de_\xi ,
$$
因 $e_z^\top K_pAe_\xi=(A^\top K_p^\top e_z)^\top e_\xi=(A^\top K_pe_z)^\top e_\xi$——这里**只**用到 $K_p$ 对称与 $K_p$ 写在 $A^\top$ 内侧，不需要 $K_p$ 为标量。故 $\dot V\le0$，$\Omega_c$ 正向不变；又 $V\le c$ 给出 $\|e_\xi\|\le\sqrt{2c}$、$\|e_z\|\le\sqrt{2c/\lambda_{\min}(K_p)}$，故 $\Omega_c$ 紧。

2. **工作域保号 (5.5c)**：$V\le c$ 蕴含 $\tfrac12\mathcal O^\top K_{p,O}\mathcal O\le c$，即 $\|\mathcal O\|^2\le2c/\lambda_{\min}(K_{p,O})<1$（由 $c<c^*$）。由 $\mathcal O=-\mathrm{Im}\,\tilde r$ 与 $\|\tilde r\|=1$ 得 $\tilde\eta^2+\|\mathcal O\|^2=1$，故 $|\tilde\eta|\ge\eta_0>0$；$\tilde\eta(t)$ 连续且恒不为零，符号不可突变，由 $\tilde\eta(0)>0$ 得 (5.5c)。

3. **$A$ 的行列式**：$A$ 块下三角（(4.5)），故
$$
\det A=\det\bigl(-\tfrac12(\tilde\eta I_3+[\mathcal O]_\times)\bigr)\cdot\det I_3
=\bigl(-\tfrac12\bigr)^3\tilde\eta\bigl(\tilde\eta^2+\|\mathcal O\|^2\bigr)=-\tfrac18\tilde\eta ,
$$
用到 $\det(aI_3+[b]_\times)=a(a^2+\|b\|^2)$ 与 $\tilde\eta^2+\|\mathcal O\|^2=1$。定量奇异值下界 $\sigma_{\min}(A)\ge\bigl[2(1+\|\mathcal T\|)/\tilde\eta+1\bigr]^{-1}$ 见附录 C.2。

4. **LaSalle**：$\Omega_c$ 紧且不变，$E\triangleq\{\dot V=0\}\cap\Omega_c=\{e_\xi=0\}\cap\Omega_c$。若轨迹全程留在 $E$ 内：$e_\xi\equiv0\Rightarrow\dot e_\xi\equiv0\Rightarrow A^\top K_pe_z\equiv0$；由第 3 步 $A$ 可逆、$K_p\succ0$ 得 $e_z\equiv0$。故 $E$ 内最大不变集为 $\{(0,0)\}$，由 LaSalle 不变集定理（[Kha02] Thm 4.4）得 (iii)。

5. **局部指数稳定**：在 $(0,0)$ 处 $\tilde x\to1$，$A\to A_0=\mathrm{diag}(-\tfrac12I_3,I_3)$，(5.5) 的雅可比为
$$
F=\begin{bmatrix}0_6 & A_0\\ -A_0^\top K_p & -K_d\end{bmatrix}.
$$
对该 LTI 系统同一个 $V$ 仍给出 $\dot V=-e_\xi^\top K_de_\xi\le0$，且 $A_0$ 可逆，重复第 4 步得 LTI 系统渐近稳定，故 $F$ 为 Hurwitz；再由 Lyapunov 线化定理（[Kha02] Thm 4.7）得非线性系统在原点邻域指数稳定。块对角 $K_d,K_p$ 下 $F$ 逐分量解耦，其两个二阶多项式与极点由 (5.8) 显式给出。

### C.7 定理 3(c) 的完整证明

**第一步（$\dot V$ 精确式）**：含扰时定理 3(b) 证明第 1 步的交叉项相消与 $d$ 无关，故
$$
\dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d
$$
精确成立（此式不含任何放缩；$e_\xi^\top d$ 可正可负）。

**第二步（(c-1) 判据的等价性）**：性能目标即 $-e_\xi^\top K_de_\xi+e_\xi^\top d+\tfrac1{2\kappa}\|e_\xi\|^2-\tfrac{\gamma_a^2}2\|d\|^2\le0$，等价于二次型不等式
$$
\begin{bmatrix}e_\xi\\ d\end{bmatrix}^{\!\top}\!M\!
\begin{bmatrix}e_\xi\\ d\end{bmatrix}\ge0\quad\forall(e_\xi,d)\in\mathbb R^{12},
$$
即 $M\succeq0$。右下块 $\tfrac{\gamma_a^2}2I\succ0$，取 Schur 补得 $K_d-\tfrac1{2\kappa}I-\tfrac1{2\gamma_a^2}I\succeq0$，即 (5.6a)。关键在于不定号交叉项 $e_\xi^\top d$ 保留在二次型内整体判定，不经任何符号放缩。

**第三步（全局存在性）**：由 $M\succeq0$ 得 $\dot V\le\tfrac{\gamma_a^2}2\|d\|^2$，故 $V(t)\le V(0)+\tfrac{\gamma_a^2}2\|d_{L_2}\|_{L_2}^2<\infty$，$(e_z,e_\xi)$ 一致有界，解在 $[0,\infty)$ 上存在（无有限时间逃逸）。

**第四步（积分收尾）**：在 $[0,T]$ 上积分 $\dot V\le-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2$，弃去 $V(T)\ge0$，令 $T\to\infty$（单调收敛）即得 (5.6)。

**第五步（(c-2) 分量解耦）**：块对角 $K_d$ 与各向同性平移刚度 $K_{p,T}=k_{p,T}I_3$ 下两分量储能精确解耦，关键是两处混合积恒零：由 (4.5)，$(A^\top K_pe_z)_\omega=A_{11}^\top K_{p,O}\mathcal O+k_{p,T}[\mathcal T]_\times\mathcal T=A_{11}^\top K_{p,O}\mathcal O$（$\mathcal T\times\mathcal T=0$，故旋转反馈不含 $\mathcal T$）、$(A^\top K_pe_z)_v=k_{p,T}\mathcal T$；又 $\dot{\mathcal T}=-[\mathcal T]_\times\tilde\omega+\tilde v$ 中的耦合项做功为零（$\mathcal T\cdot(\mathcal T\times\tilde\omega)=0$）。于是位姿交叉项在两分量内分别由 $K_{p,O}$ 对称与 $k_{p,T}$ 为标量而精确相消，
$$
\dot V_\omega=-\tilde\omega^\top K_\omega\tilde\omega+\tilde\omega^\top d_\omega,
\qquad
\dot V_v=-\tilde v^\top K_v\tilde v+\tilde v^\top d_v ,
$$
对每个分量重复第二至第四步的论证（$I_6\to I_3$）即得 (5.6b)⇒(5.6$'$)。两处恒零为代数恒等式，故 (c-1)/(c-2) 的全部结论均不依赖工作域 $\tilde\eta>0$。

### C.8 定理 3(d) 的完整证明

1. **精确耗散等式与乘法项展开**：由定理 3(c) 证明第一步，$\dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d$ 精确成立（无任何放缩）。代入 (5.1d) 与 (5.2) 的 $u_{\mathrm{fb}}=-K_de_\xi-A^\top K_pe_z$：
$$
\dot V=-e_\xi^\top K_de_\xi\;\underbrace{-,e_\xi^\top\Theta K_de_\xi}_{\text{与阻尼同类}}\;\underbrace{-,e_\xi^\top\Theta A^\top K_pe_z}_{\text{与扰动同类}}\;+\;e_\xi^\top d_{\mathrm{ex}} .
$$

2. **两类乘法项的分别回收**：$|e_\xi^\top\Theta K_de_\xi|\le\alpha\lambda_{\max}(K_d)\|e_\xi\|^2$（回收进阻尼，得 (5.7a) 的 $\lambda_{\mathrm{eff}}$；由 (A3)/(5.1f) 即 $\alpha<\lambda_{\min}(K_d)/\lambda_{\max}(K_d)$ 得 $\lambda_{\mathrm{eff}}>0$）；$|e_\xi^\top\Theta A^\top K_pe_z|\le\alpha\|A\|_2\lambda_{\max}(K_p)\|e_z\|\,\|e_\xi\|$（回收进等效扰动幅值 $D$）。

3. **$A$ 的谱范数界**：由 $A_{11}^\top A_{11}=\tfrac14(I_3-\mathcal O\mathcal O^\top)$ 得 $\|A_{11}\|_2=\tfrac12$（**精确值**，与 $\tilde x$ 无关；奇异值计算见附录 C.2），再由 $A$ 的块下三角结构得 $\|A\|_2\le\max\{\|A_{11}\|_2,1\}+\|[\mathcal T]_\times\|_2=1+\|\mathcal T\|$。在 $\Omega_c$ 上 $\|\mathcal T\|\le\sqrt{2c/\lambda_{\min}(K_{p,T})}$、$\|e_z\|\le\sqrt{2c/\lambda_{\min}(K_p)}$，代入第 2 步即得 (5.7b) 与
$$
\dot V\ \le\ -\lambda_{\mathrm{eff}}\|e_\xi\|^2+D\,\|e_\xi\| .
\tag{5.7d}
$$

4. **Young 与积分收尾**：$D\|e_\xi\|\le\tfrac{\lambda_{\mathrm{eff}}}2\|e_\xi\|^2+\tfrac{D^2}{2\lambda_{\mathrm{eff}}}$，故 $\dot V\le-\tfrac{\lambda_{\mathrm{eff}}}2\|e_\xi\|^2+\tfrac{D^2}{2\lambda_{\mathrm{eff}}}$。在 $[0,T]$ 上积分并弃去 $V(T)\ge0$：
$$
\tfrac{\lambda_{\mathrm{eff}}}2\int_0^T\|e_\xi\|^2dt\ \le\ V(0)+\tfrac{D^2}{2\lambda_{\mathrm{eff}}}\,T ,
$$
两端除以 $\tfrac{\lambda_{\mathrm{eff}}}2T$ 即 (5.7c)；令 $T\to\infty$ 得 (5.7)。$\alpha\to0$ 时 $D\to D_{\mathrm{ex}}$、$\lambda_{\mathrm{eff}}\to\lambda_{\min}(K_d)$，退化为经典形式。

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
18. **[Con25]** D. Condurache, *An overview of higher-order kinematics of rigid body and multibody systems with nilpotent algebra*, Mechanism and Machine Theory 209 (2025) 105959.
