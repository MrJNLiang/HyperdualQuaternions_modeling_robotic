# 论文理论问题数学推导说明：扰动模型与 H∞ 证书

> 整理日期：2026-09-15。
> 范围：`texdocs/sections/`（第 3–5 章、附录 C）、`reBotArm_control_py/`（实机控制律与记录）。
> 结论先行：删除残留两项 $\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$ 不破坏后面的 $H_\infty$ 耗散证明；但当前中文版仍有五处表述未闭合（下文问题 1–5）。所有推导均已对照源文件逐行核实；实机原始数据不受任何影响。
> 本文档仅为分析说明，未修改论文任何 `.tex` 文件。

---

## 目录

- [问题 0：定理 3(a) 证明中残留的两项](#问题-0定理-3a-证明中残留的两项)
- [问题 1：总扰动 d 与外生扰动 d_ex 的证书不能混用（最重要）](#问题-1总扰动-d-与外生扰动-d_ex-的证书不能混用最重要)
- [问题 2：定理 3(d) 的循环前提与"证明了有界性"的过度声明](#问题-2定理-3d-的循环前提与证明了有界性的过度声明)
- [问题 3：C.5 第三步"V 有界 ⇒ 解全局存在"对实机不成立](#问题-3c5-第三步v-有界--解全局存在对实机不成立)
- [问题 4：归一化 ℓ 与"可同时对角化"的范围](#问题-4归一化-ℓ-与可同时对角化的范围)
- [问题 5：实机实现与理想证书的 gap](#问题-5实机实现与理想证书的-gap)
- [已核实成立的部分](#已核实成立的部分)
- [修改建议清单](#修改建议清单)

---

## 问题 0：定理 3(a) 证明中残留的两项

**现状**：正文已将扰动定义为 $d=J\boldsymbol w_{\mathrm{dyn}}$（[sec5-control.tex:71](../texdocs/sections/sec5-control.tex#L71)），但定理 3(a) 证明内嵌句仍保留"（含扰时右端加 $\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$）"（[sec5-control.tex:80](../texdocs/sections/sec5-control.tex#L80)）。这两个符号**全文仅此一处、无任何定义**，是上一轮修改（把扰动模型从两个速度级扰动项改为 $d=J\boldsymbol w_{\mathrm{dyn}}$）只改正文、没清理证明内嵌句留下的残留。

### 为什么这两项既多余、又不可能出现在 (5.5a) 里

式 (5.5a) 是对式 (4.4) 求导得到的，全程只用误差定义与伴随求导法则，**不经过任何被控对象模型**：

$$
\dot{\tilde{\boldsymbol\xi}}=\dot{\boldsymbol\xi}-\frac{d}{dt}\bigl(\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d\bigr)
=\dot{\boldsymbol\xi}-\mathrm{Ad}_{\tilde x}\dot{\boldsymbol\xi}_d-\mathrm{ad}_{\tilde{\boldsymbol\xi}}\bigl(\mathrm{Ad}_{\tilde x}\boldsymbol\xi_d\bigr)
\tag{0.1}
$$

它是**运动学恒等式**——无论有没有扰动，右端都不可能多出任何项。扰动进入系统的唯一通道是下一步"代入实际 $\dot{\boldsymbol\xi}$"。由式 (3.5)、(5.1)、(5.2) 三步代入：

$$
\mathrm{vec}_6\dot{\boldsymbol\xi}
=\dot J\dot{\boldsymbol q}+J\ddot{\boldsymbol q}
=\dot J\dot{\boldsymbol q}+J\bigl(\ddot{\boldsymbol q}_{\mathrm{ref}}+\boldsymbol w_{\mathrm{dyn}}\bigr)
\overset{JJ^+=I_6}{=}\underbrace{u_{\mathrm{ff}}+u_{\mathrm{fb}}-\dot J\dot{\boldsymbol q}}_{J\ddot{\boldsymbol q}_{\mathrm{ref}}}
+\dot J\dot{\boldsymbol q}+J\boldsymbol w_{\mathrm{dyn}}
=u_{\mathrm{ff}}+u_{\mathrm{fb}}+J\boldsymbol w_{\mathrm{dyn}}.
\tag{0.2}
$$

再减去 (0.1) 右端的两项输运项（恰为 $u_{\mathrm{ff}}$ 的定义，见式 5.2 前的定义），得

$$
\dot{\boldsymbol e}_\xi=-K_d e_\xi-A^\top(\tilde x)K_p e_z+\underbrace{J\boldsymbol w_{\mathrm{dyn}}}_{d}.
\tag{0.3}
$$

所以摩擦、力矩失配、外力矩这类**通过关节力矩进入**的扰动已被式 (5.1) 的 $\boldsymbol w_{\mathrm{dyn}}$ 完整吸收；在 (5.5a) 右端再加任何项都是重复计数。

### caveat：若两项本意是别的物理效应，不能用"删除"代替"建模"

若当初两项想表达的是**不经关节力矩进入**的效应——连杆柔性（编码器刚体正运动学与真实端位姿之差）、基座运动（$\mathrm{Ad}_{\tilde x}$ 的参考系本身在动）、位姿测量误差——则它们的注入点在**误差定义层**（式 4.3–4.4），数学上表现为

$$
\dot e_z=A(\tilde x)e_\xi+\text{额外项}\quad\text{或}\quad e_z\ \text{本身带偏},
$$

而不是 $\dot e_\xi$ 里多一项。把不同注入点的扰动都塞进 $d=J\boldsymbol w_{\mathrm{dyn}}$ 是范畴错误。正确做法二选一：在假设集中明确排除这些因素；或按其实际注入位置重新定义残差。

---

## 问题 1：总扰动 $d$ 与外生扰动 $d_{\mathrm{ex}}$ 的证书不能混用（最重要）

### 3(c) 的证明到底证了什么

C.5 第二步把耗散目标写成二次型：

$$
\dot V\le-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}2\|d\|^2
\;\Longleftrightarrow\;
\begin{bmatrix}e_\xi\\ d\end{bmatrix}^{\!\top}\!M\!\begin{bmatrix}e_\xi\\ d\end{bmatrix}\ge0
\quad\forall(e_\xi,d)\in\mathbb R^{12},
\tag{1.1}
$$

其中 $d$ 被当作**独立的外部输入**——不等式对一切 $(e_\xi,d)$ 组合点态成立，这是标准 $H_\infty$ 证书的结构。但式 (5.1d) 说 $d=\Theta u_{\mathrm{fb}}+d_{\mathrm{ex}}$，而 $u_{\mathrm{fb}}=-K_de_\xi-A^\top K_pe_z$ 是**误差状态自己的函数**。$\dot V$ 里的交叉项是

$$
e_\xi^\top d=e_\xi^\top\Theta u_{\mathrm{fb}}+e_\xi^\top d_{\mathrm{ex}},
$$

第一项状态相关，没有被 (1.1) 的点态不等式覆盖。因此**仅由 $d_{\mathrm{ex}}\in L_2$ 推不出 (5.6) 对 $d_{\mathrm{ex}}$ 成立**——把 (5.6) 里的 $d$ 换成 $d_{\mathrm{ex}}$ 是非法代入。

### 标量反例（完整推导）

忽略位姿反馈，取 $K_d=b$（标量），代入 $d=\Theta u_{\mathrm{fb}}+d_{\mathrm{ex}}$：

$$
\dot e=-be+\Theta\underbrace{(-be)}_{u_{\mathrm{fb}}}+d_{\mathrm{ex}}=-b(1+\Theta)\,e+d_{\mathrm{ex}}.
\tag{1.2}
$$

取 $\Theta=-0.2$。假设 (A3) 只要求 $\alpha=|\Theta|<\lambda_{\min}(K_d)/\lambda_{\max}(K_d)=1$，满足。但闭环成为 $\dot e=-0.8b\,e+d_{\mathrm{ex}}$，其外生通道 $L_2$ 增益（线性稳定系统的 $H_\infty$ 范数）为

$$
\sup_\omega\Bigl|\frac{1}{j\omega+0.8b}\Bigr|=\frac{1}{0.8b}=\frac{1.25}{b}\;>\;\frac1b .
\tag{1.3}
$$

而定理 3(c′) 认证的 $\gamma^*_{\mathrm{opt}}=1/\lambda_{\min}(K_d)=1/b$。用实机数代入：$b=15\,\mathrm{s^{-1}}$，真实外生通道增益 $\approx0.083\,\mathrm s$，证书值 $\approx0.067\,\mathrm s$——**被超出 25%**。即"同一个界适用于 $d_{\mathrm{ex}}\to e_\xi$"是数学上假的命题。

### 为什么 A3 恰好是补救条件、但代价是什么

把 $\Theta u_{\mathrm{fb}}$ 当反馈回路做小增益/回路变换，闭环条件是 $\gamma^*\,\alpha\,\lambda_{\max}(K_d)<1$；代 $\gamma^*=1/\lambda_{\min}(K_d)$ 得到的**正是式 (5.1f)**。所以 A3 不是随意加的——它恰好是把 3(c) 的证书经回路变换延拓到外生通道所需的条件。但延拓后的认证值是

$$
\frac{1}{\lambda_{\mathrm{eff}}}=\frac{1}{\lambda_{\min}(K_d)-\alpha\,\lambda_{\max}(K_d)}
\quad\Bigl(\text{标量即 } \frac{1}{b(1-\alpha)}=\frac{1}{0.8b}\Bigr),
\tag{1.4}
$$

**不是** $1/\lambda_{\min}(K_d)$。定理 3(d) 用的正是这个 $\lambda_{\mathrm{eff}}$——3(d) 的路线是对的；错的是 3(c)/A4/3(c′) 链条上的叙述把两种含义混为一谈。

### 文本不一致的具体位置

- 式 (5.1d) 之后（sec5-control.tex:25）：$d$"按时间特性分为 $d_{L_2}$ 和 $d_b$"——但 $\Theta u_{\mathrm{fb}}$ 既不是外生信号、也不按时间特性分类，它是闭环反馈信号。
- A4（sec5-control.tex:38）：写"定理 3(c) 仅要求**加性**扰动 $d=d_{L_2}\in L_2$"。
- 定理 3(a)（sec5-control.tex:71）：写 $d=J\boldsymbol w_{\mathrm{dyn}}=\Theta u_{\mathrm{fb}}+d_{\mathrm{ex}}$（**含乘性**）。

同一个小节里 $d$ 被赋予两种含义。修法：把 3(c) 明确写成"**总扰动** $d\in L_2$（含乘性分量）的证书"；3(d) 则是另一条含 Young 回收的条件性路线；两者不可互相推出。

---

## 问题 2：定理 3(d) 的循环前提与"证明了有界性"的过度声明

### (a) $d\neq0$ 时 $\Omega_c$ 根本不是不变集

在边界 $\partial\Omega_c$ 上取一点：$e_z=0$、$e_\xi=\varepsilon\hat d$（$\hat d$ 为单位向量，$\varepsilon<\sqrt{2c}$，故该点确在 $\Omega_c$ 内）。此时

$$
\dot V=-\varepsilon^2\lambda_i+\varepsilon\|d\|=\varepsilon\bigl(\|d\|-\lambda_i\varepsilon\bigr)>0
\qquad\text{当 }\varepsilon<\|d\|/\lambda_i .
\tag{2.1}
$$

即只要扰动持续存在，$V$ 可以**穿过**水平面 $c$ 向外走——水平集不变性只在 $d\equiv0$ 时成立（那正是定理 3(b) 的情形）。所以 3(d) 只能把"轨迹留在 $\Omega_c$"当作**前提**（正文 sec5-control.tex:183 已如此写，"需事后数值核验"），不能从定理本身得到。

### (b) $D$ 依赖 $c$ 造成循环

等效扰动幅值 $D$（式 5.7b）由下列界拼出：

$$
\|e_z\|\le\sqrt{\frac{2c}{\lambda_{\min}(K_p)}},\qquad
\|\mathcal T\|\le\sqrt{\frac{2c}{\lambda_{\min}(K_{p,T})}},
\tag{2.2}
$$

而这些界来自 $V\le c$ 这个前提。想用界 (5.7c) 反过来证 $V\le c$，就是用结论证前提。所以 (5.7c)/(5.7) 是**条件式**：*给定*轨迹留在 $\Omega_c$，RMS 不超过 $D/\lambda_{\mathrm{eff}}$。由此：

- 第 202 行"偏差型（$L_\infty$）不确定性**不破坏有界性**"超出了已证内容——定理从头到尾没证过有界性，它只是*在有界性前提下*给出均方水平。此句应删。
- 只在 $[0,T_{\mathrm{exp}}]$（10 s 单次运行）核验过水平集前提，推不出 $T\to\infty$ 的 $\limsup$ 形式（sec5-control.tex:210 的注记已承认此点，但第 202 行未同步收窄）。

### (c) $K_{p,T}$ 子块界需要块对角 $K_p$（反例）

$V\le c$ 即 $e_z^\top K_pe_z\le2c$。若 $K_p$ 有旋转–平移交叉块，取 $2\times2$ 缩比模型

$$
K_p=\begin{bmatrix}1&\rho\\ \rho&1\end{bmatrix},\qquad \rho\to1,
$$

令 $e_z=(\mathcal O,\mathcal T)=(-t,t)$，则

$$
e_z^\top K_pe_z=2(1-\rho)\,t^2\ \xrightarrow[\rho\to1]{}\ 0 ,
\tag{2.3}
$$

即 $\|\mathcal T\|$ 可以任意大而 $V\le c$ 依然成立。所以式 (2.2) 第二个界**只在 $K_p=\mathrm{diag}(K_{p,O},K_{p,T})$ 时成立**（块对角时 $e_z^\top K_pe_z$ 按块相加，各块分别 $\le2c$）。而 3(d) 的前提只写了"设 (A1)–(A3) 成立"，记号 $K_{p,T}$ 在 3(d) 陈述内没有定义（它在 3(b) 和 (c-2) 里才有）——假设范围未交代。

**根源**：C.6 的四步代数（乘性项 Young 回收）本身无误；问题出在把一个**条件性估计**叙述成带鲁棒性口吻的结论。

---

## 问题 3：C.5 第三步"$V$ 有界 ⇒ 解全局存在"对实机不成立

### 抽象系统层：这一步其实可以补全

由 $M\succeq0$ 得点态 $\dot V\le\frac{\gamma_a^2}2\|d\|^2$。沿最大存在区间 $[0,t_{\max})$ 积分：

$$
V(t)\le V(0)+\frac{\gamma_a^2}2\|d_{L_2}\|^2_{L_2[0,t]}<\infty,
$$

故 $V$ 在有限时间内不可能爆破 ⇒ $(e_z,e_\xi)$ 留在紧集；误差系统的向量场在 $\tilde x\in S^3$（紧）上光滑、在 $e_\xi$ 上仿射，紧集上有界 ⇒ 按 ODE 延拓定理 $t_{\max}=\infty$。论证成立，但需要写出（并说明 $\tilde\eta$ 分支固定的细节），而不是 C.5 第三步（appendix.tex:97）一句话带过。

### 实机层：这一步是断的

$V$ 有界只约束**任务空间误差** $(e_z,e_\xi)$，而证书的全部假设 (A1)–(A3) 定义在**关节空间工作集** $\mathcal Q$ 上。任务误差有界不能反推关节轨迹留在 $\mathcal Q$，因为位姿→关节的逆映射在奇异处失效：

$$
\sigma_{\min}(J)\to0
\quad\Longrightarrow\quad
\|J^+\|_2=\frac{1}{\sigma_{\min}(J)}\to\infty
\quad\Longrightarrow\quad
\|\Theta\|=\bigl\|JM^{-1}\Delta M\,J^+\bigr\|\le\|JM^{-1}\Delta M\|\cdot\frac1{\sigma_{\min}(J)}\to\infty,
\tag{3.1}
$$

即 (A2) 的界与 (A3) 的小增益条件同时失效，控制律 (5.2) 本身病态。而机械臂完全可能在任务误差仍不大的情况下逼近奇异（多解、冗余、关节限位附近）。

**根源**：证书定义在 12 维误差流形上，正则性条件定义在 $n$ 维关节空间上，两套几何之间的映射不是全局同胚——$\Omega_c$ 是误差空间的集合，不是 $\mathcal Q$ 的集合。这是所有任务空间控制证明的公共陷阱；修法是把"闭环轨迹全程留在 $\mathcal Q$、逆映射有效"作为显式假设携带，而不是由 $V$ 的水平集"推出"。

---

## 问题 4：归一化 $\ell$ 与"可同时对角化"的范围

### (a) 合并范数的量纲缝合参数 $\ell$ 没有全局声明

第 4 章（sec4-error.tex:47）说 $\bar{\mathcal T}=\mathcal T/\ell$、$\bar v=v/\ell$、"为简洁起见，后文仍记作 $\mathcal T,v$"。问题在于**所有数值结论都隐式依赖 $\ell$**：

- $V$（式 5.4a）与水平集半径 (5.5c)：$\tilde\eta_0=\sqrt{1-2c/\lambda_{\min}(K_{p,O})}$ 中的 $c$ 含平移块贡献，换 $\ell$ 即变；
- 等效扰动幅值 $D$（式 5.7b）；
- 证书余量，以及 3(b) 后注记中的"实测 $V$ 峰值约 $0.5$、$c^*=72$"（换 $\ell$ 全变——平移块的 $K_{p,T}$ 在归一化坐标下数值相差 $\ell^2$ 倍关系）。

另外 $J$、$d$、$\Theta$、控制量的坐标约定（空间系 twist？平移分量除没除 $\ell$？）没有一张统一对照表，合并范数与数值证书因此有歧义。**修法不费事：固定一个 $\ell$、一次性声明各量的量纲/坐标即可，不需要任何新推导。**

### (b) C.7 的标量推导本身是对的，缺口在向 MIMO 外推

复核线性化：旋转通道

$$
\ddot{\mathcal O}+K_\omega\dot{\mathcal O}+\tfrac14K_{p,O}\,\mathcal O=-\tfrac12\,d_\omega
\qquad(\text{由 } \tilde\omega=-2\dot{\mathcal O} \text{ 换算})，
\tag{4.1}
$$

平移通道 $\ddot{\mathcal T}+K_v\dot{\mathcal T}+k_{p,T}\,\mathcal T=d_v$；且

$$
|G(j\omega)|^2=\frac{u}{(c-u)^2+b^2u},\quad u=\omega^2,
\qquad
\frac{d}{du}\Bigl[\frac{u}{(c-u)^2+b^2u}\Bigr]\propto(c-u)(c+u)
\;\Rightarrow\; |G|_{\max}=\frac1b\ \text{于}\ u=c .
\tag{4.2}
$$

即每条**标量**通道的峰值增益确为 $1/b=1/\lambda(K_d)$，与 3(c′) 一致。但矩阵增益下 $d\to e_\xi$ 是 MIMO 传递函数

$$
G(s)=s\,(s^2I+K_ds+K_{p\text{-块}})^{-1},
\tag{4.3}
$$

其 $H_\infty$ 范数等于 $1/\lambda_{\min}(K_d)$ **当且仅当**该通道内的 $K_d$ 与刚度块可同时对角化（对称矩阵可交换时各模态解耦为标量二阶环节）；不可交换时模态耦合会抬高峰值。文中"或与相应刚度块可同时对角化"一句没有写出分块条件（$K_\omega$ 与 $K_{p,O}$ 交换、$K_v$ 与 $k_{p,T}I_3$ 交换——后者平凡）。

实机用的是 $K_d=15I_6$、$K_p=\mathrm{diag}(4kI_3,kI_3)$，完全各向同性，所以**把 C.7 的声明收窄到各向同性情形即可无缝闭合**，且与实机一致。

---

## 问题 5：实机实现与理想证书的 gap

### (a) 阻尼一致逆破坏 $JJ^+=I_6$

CSV 参数头（`reBotArm_control_py/results/tndq_mit_20260905_173920.csv` 第 10 行）记录 `control: damping=0.0001`，且力矩级后端传入 $M$，走 `control_law.py:123` 的 Khatib 一致逆：

$$
\bar J=M^{-1}J^\top\bigl(JM^{-1}J^\top+\lambda^2I\bigr)^{-1},\qquad \lambda=10^{-3}.
\tag{5.1}
$$

记 $A_q\triangleq JM^{-1}J^\top\succ0$（$J$ 行满秩时），则

$$
J\bar J=A_q(A_q+\lambda^2I)^{-1}=I-\lambda^2(A_q+\lambda^2I)^{-1}\neq I,
\qquad
\|J\bar J-I\|\le\frac{\lambda^2}{\sigma_{\min}(A_q)} .
\tag{5.2}
$$

数值上 $\lambda^2=10^{-8}$ 极小，但**原理上非零**，且 $u_{\mathrm{ff}}+u_{\mathrm{fb}}-\dot J\dot{\boldsymbol q}$ 被 $J\bar J$ 作用后前馈抵消不再精确，闭环恒等式 (5.5) 多出 $-\lambda^2(A_q+\lambda^2I)^{-1}u_{\mathrm{task}}$ 一项。

**精确化说明**：**无阻尼**的一致逆 $\bar J_0=M^{-1}J^\top(JM^{-1}J^\top)^{-1}$ 其实满足 $J\bar J_0=I$，所以问题不出在 M 加权本身，而出在 $\lambda\neq0$ 以及乘性矩阵变成

$$
\Theta'=JM^{-1}\Delta M\,\bar J\ \neq\ \Theta=JM^{-1}\Delta M\,J^+,
\qquad
\|\bar J\|\ \text{可大于}\ \|J^+\|=\frac{1}{\sigma_{\min}(J)},
\tag{5.3}
$$

$\alpha$ 的界需按 $\bar J$ 重推。

### (b) 文本自相矛盾

A1（sec5-control.tex:29）明文写"阻尼伪逆仅作为奇异邻域的实现策略……**本文证书不覆盖该情形**"——而实机 C1 恰恰全程使用带阻尼的一致逆。按 A1 自己的口径，实机数据落在证书覆盖范围之外。

### (c) 其余实现 gap

连续时间证书假设理想反馈，而实机是：

1. $10\,\mathrm{ms}$ 零阶保持：$K_d\Delta t=15\times0.01=0.15$，相位滞后非可忽略；
2. 加速度治理器介入帧占比最高 $11.2\%$（$r_{\mathrm{gov}}$ 按附加扰动处理、明确不认证——这个处理本身是对的）；
3. 摩擦前馈低速淡出窗口。

任何一项单独都不算大，但合起来意味着"严格连续时间 $H_\infty$ 证书在实机上成立"这句话说不出口。第 7 章"不声称连续时间证书仍严格成立"的措辞方向正确，需与 A1/§5 的叙述对齐。

**根源**：理论假设（精确 MP 伪逆、连续时间、无饱和、刚体）与工程实现（阻尼一致逆、离散、限幅、摩擦补全）之间的固有差距；这不是错误，而是**证书的适用范围必须显式收窄**。

---

## 已核实成立的部分

关键恒等式 $\dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d$ 只依赖一件事：$u_{\mathrm{fb}}$ 中 $-A^\top K_pe_z$ 与 $\dot V$ 展开里的 $e_z^\top K_pAe_\xi=(A^\top K_pe_z)^\top e_\xi$ 精确相消（只用 $K_p$ 对称）。以下推导全部重算过，**在各自前提内成立**：

| 推导 | 判断 | 备注 |
|---|---|---|
| HDQ 误差求导、伴随输运、式 (4.4)/(4.5) | 成立 | 纯运动学，与扰动无关 |
| $A^\top K_p$ 反馈使交叉项相消 | 成立 | 只用 $K_p$ 对称 |
| 3(b) 的 $c^*=\frac12\lambda_{\min}(K_{p,O})$、$\tilde\eta\ge\eta_0$ | 成立 | $2c/\lambda_{\min}<1\Leftrightarrow c<c^*$ |
| 3(b) 的 $\det A=-\frac18\tilde\eta$ 与 LaSalle | 成立 | $\det(aI+[b]_\times)=a(a^2+\|b\|^2)$ |
| 3(c) 的 Schur 补充要判据 | 成立 | "充要"限于选定存储函数与供给率的点态不等式 |
| 3(c′) 的 AM–GM 下界与 $(\kappa,\gamma_a)=(\lambda^{-1},\lambda^{-1/2})$ 可达点 | 成立 | $\gamma^*_{\mathrm{opt}}=1/\lambda_{\min}(K_d)$ 是该证书族最优值 |
| C.7 的 $\lvert G\rvert_{\max}=1/b$ | 成立 | 标量通道；MIMO 需同时对角化（见问题 4b） |
| C.6 的 Young 回收四步代数 | 成立 | 问题只在条件性措辞（见问题 2） |

**删两个无定义符号、收窄 3(c)/3(d)/C.7 表述，都不碰这条主线。**

---

## 修改建议清单（不改任何实机数据）

1. **删除**定理 3(a) 证明中"（含扰时右端加 $\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$）"一句（sec5-control.tex:80）；若需保留柔性/基座/测量误差的讨论，按注入点单独建模或明确排除。
2. **统一 $d$ 的语义**：3(c) 明确为"总扰动 $d=\Theta u_{\mathrm{fb}}+d_{\mathrm{ex}}\in L_2$ 的证书"；A4 去掉"加性"措辞；3(d) 保持 $\lambda_{\mathrm{eff}}$ 路线并声明它与 3(c) 不可互相推出（问题 1）。
3. **收窄 3(d)**：删"不破坏有界性"句；显式写出块对角 $K_p$ 假设；$\limsup$ 形式仅限全程水平集前提（问题 2）。
4. **补全 C.5 第三步**：区分抽象误差系统的延拓论证与实机"全程留在 $\mathcal Q$"假设（问题 3）。
5. **统一归一化**：固定 $\ell$ 并给出各量坐标/量纲对照表；C.7 收窄到各向同性（与实机一致）（问题 4）。
6. **对齐实机叙述**：A1 已排除阻尼伪逆，而实机用了 `damping=1e-4` 的动力学一致逆——要么补一段逆映射残差 $\lambda^2/\sigma_{\min}(A_q)$ 与 $\Theta'$ 的量化分析，要么把实机章节明确定位为"实现可行性与跟踪对比验证"，不宣称连续时间 $H_\infty$ 证书（问题 5）。
7. **实机解释定位**：现有数据与统计（关节记录、p95、`u_fb=-K_d@e_xi-A.T@pose`）完全不受上述修改影响；p95 随 $k$ 改善只能作为与 $\gamma^*$ 收紧"方向一致"的证据，不能宣称最坏情形 $L_2$ 增益已被实机验证（无外生扰动注入、单轨迹非最坏情形、p95 量的是位姿而证书约束的是 $e_\xi$）。

---

### 附：涉及文件与行号速查

| 位置 | 内容 |
|---|---|
| `texdocs/sections/sec5-control.tex:80` | 残留 $\dot{\boldsymbol v}_w+\dot{\boldsymbol v}_c$ |
| `texdocs/sections/sec5-control.tex:25,38,71` | $d$ 的两种含义（总 vs 加性） |
| `texdocs/sections/sec5-control.tex:183,202,210` | 3(d) 的水平集前提与"不破坏有界性" |
| `texdocs/sections/appendix.tex:97` | C.5 第三步"解在 $[0,\infty)$ 上存在" |
| `texdocs/sections/sec4-error.tex:47` | 特征长度 $\ell$ 归一化 |
| `texdocs/sections/appendix.tex:133–145` | C.7 线性化与同时对角化条件 |
| `reBotArm_control_py/results/tndq_mit_20260905_173920.csv:10` | `control: damping=0.0001` |
| `reBotArm_control_py/reBotArm_control_py/tndq/control_law.py:37–56,110–123` | 动力学一致逆与反馈实现 |
