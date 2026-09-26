# 多 Gamma 力矩级鲁棒控制：直接构造储能函数的简化推导

## 本文主线与完成内容

**主线：在现有对偶四元数误差与力矩控制律上，直接证明旋转/平移、位姿/twist 的四 Gamma 性能，再由 Gamma 反算 \(K_p,K_d\)。**

本文直接定义新的储能函数 \(W_\omega,W_v\)，从完整误差方程求导，不依赖之前的储能函数或证明。所得到的核心结论是

\[
\begin{aligned}
\dot W_\omega+
\frac{\|\mathcal O\|^2}{\gamma_{\omega,z}^2}+
\frac{\|\tilde\omega\|^2}{\gamma_{\omega,\xi}^2}
&\le \|d_\omega\|^2,\\
\dot W_v+
\frac{\|\mathcal T\|^2}{\gamma_{v,z}^2}+
\frac{\|\tilde v\|^2}{\gamma_{v,\xi}^2}
&\le \|d_v\|^2.
\end{aligned}
\tag{1}
\]

**完成内容：** 对当前各向同性旋转/平移增益块，给出非线性工作域内的显式 \(K_p(\gamma),K_d(\gamma)\) 充分设计公式，见式 (14)；第 5 节进一步给出持续有界扰动的 ISS/最终有界证明，并把关节力矩、外力矩和乘性惯量失配接入同一耗散证书。证明使用四元数标量部、叉乘恒等式与 Schur 补，不需要估计 \(\dot A\)。

适用范围是 \(\tilde\eta\ge\eta_0>0\)、\(\|\tilde\omega\|\le\bar\omega\) 的连续时间误差模型；附录 A 给出有限能量扰动下的留域检查，第 5 节给出有界扰动下的留域半径条件。这里证明的是**可认证的充分设计**，没有声称是所有控制器中的最小增益或全局最优 \(H_\infty\) 设计。

---

## 1. 先说清楚系统、扰动和目标

### 1.1 使用当前符号，把六维方程展开即可

保留现有误差
\(e_z=[\mathcal O;\mathcal T]\)、\(e_\xi=[\tilde\omega;\tilde v]\)，以及控制律

\[
\dot e_z=A(\tilde x)e_\xi,\qquad
\dot e_\xi=-K_de_\xi-A^\top K_pe_z+d.
\]

取当前控制器允许的增益块

\[
K_p=\operatorname{diag}(p_OI_3,p_TI_3),\qquad
K_d=\operatorname{diag}(b_\omega I_3,b_vI_3).
\tag{2}
\]

由于
\(A=[-\tfrac12(\tilde\eta I_3+[\mathcal O]_\times),0;
-[\mathcal T]_\times,I_3]\)，且
\([\mathcal O]_\times\mathcal O=[\mathcal T]_\times\mathcal T=0\)，
完整非线性方程直接化为

\[
\boxed{
\begin{aligned}
\dot{\mathcal O}&=-\tfrac12\tilde\eta\tilde\omega
                  -\tfrac12\mathcal O\times\tilde\omega,\\
\dot{\tilde\eta}&=\tfrac12\mathcal O^\top\tilde\omega,
\qquad \tilde\eta^2+\|\mathcal O\|^2=1,\\
\dot{\tilde\omega}&=-b_\omega\tilde\omega
                    +\tfrac12p_O\tilde\eta\mathcal O+d_\omega,\\
\dot{\mathcal T}&=\tilde v-\mathcal T\times\tilde\omega,\\
\dot{\tilde v}&=-b_v\tilde v-p_T\mathcal T+d_v.
\end{aligned}}
\tag{3}
\]

这是从当前四元数/对偶四元数运动学展开的精确方程，**不是小角度线性化**。
后面的证明只使用 (3)。

\(b_\omega,b_v,p_O,p_T\) 只是已有增益矩阵的标量块值。
除新储能函数名称、目标 Gamma 和固定工作域界外，不增加误差坐标或控制器状态。

### 1.2 扰动是什么

\(d=[d_\omega;d_v]\) 是进入 twist 加速度方程的总等效扰动。原始动力学接口是

\[
d=JM^{-1}\bigl(
\Delta M\ddot{\boldsymbol q}_{\rm ref}
+\Delta C\dot{\boldsymbol q}
+\Delta\boldsymbol g+\Delta\boldsymbol f
+\delta\boldsymbol\tau+\boldsymbol\tau_{\rm ext}
\bigr).
\tag{4}
\]

这里 \(\boldsymbol\tau_{\rm ext}\) 是关节广义外力矩；若测量的是末端 wrench，
应先用对应雅可比转成关节力矩。

第 4 节的无穷时域 \(L_2\) 结论要求 \(d_\omega,d_v\in L_2\)：扰动能量有限，例如有限时长的力矩脉冲。
常值负载通常属于 \(L_\infty\) 而不属于 \(L_2\)，不能据此声称无穷时域 \(L_2\) 增益成立；
但在任意有限时间区间，仍可使用 (1) 的积分形式，第 5 节另给出 \(L_\infty\) 最终有界证明。

计算力矩接口还要求 \(M\) 正定、\(J\) 满行秩且 \(JJ^+=I_6\)，机械臂轨迹留在正则工作集。
饱和、阻尼伪逆和模型失配产生的额外项必须计入总扰动，并检查其假设。
**不能仅凭参数误差有界，就推出闭环总扰动 \(d\in L_2\)。**
乘性模型失配另见附录 C。

### 1.3 四个 Gamma 的意思

- \(\gamma_{\omega,z}\)：\(d_\omega\) 到旋转位姿误差 \(\mathcal O\) 的目标增益。
- \(\gamma_{\omega,\xi}\)：\(d_\omega\) 到旋转 twist 误差 \(\tilde\omega\) 的目标增益。
- \(\gamma_{v,z}\)：\(d_v\) 到平移位姿误差 \(\mathcal T\) 的目标增益。
- \(\gamma_{v,\xi}\)：\(d_v\) 到平移 twist 误差 \(\tilde v\) 的目标增益。

位姿 Gamma 的单位为 \(\mathrm{s}^2\)，twist Gamma 的单位为 \(\mathrm{s}\)。
这里旋转输出是当前 \(\mathcal O\)，不是旋转角度；不要把二者的 Gamma 直接等同。

式 (1) 每行都是一个**双输出共享输入能量预算**：两项误差能量之和受控。
零初值下，它也分别蕴含各个输出的单独增益界。

证明按旋转、平移分别写，量纲各自齐次。若将两行合并成一个六维性能指标，
应先固定旋转与平移的尺度；本文不重定义任何误差坐标。

---

## 2. 直接定义新的储能函数

### 2.1 第一阶段：先由 twist Gamma 选阻尼

取

\[
\boxed{
b_\omega=\frac{2}{\gamma_{\omega,\xi}},\qquad
b_v=\frac{2}{\gamma_{v,\xi}}.
}
\tag{5}
\]

系数 2 是这套简洁设计预留位姿性能余量的选择，**不是最小阻尼结论**。
后面由同一个储能函数同时认证两个输出，不需要另建一套 twist 证明。

### 2.2 一开始就使用下面这组储能函数

\[
\boxed{
\begin{aligned}
W_\omega=b_\omega\Bigl[
&\tfrac12\|\tilde\omega\|^2
+\tfrac12p_O\|\mathcal O\|^2
+b_\omega^2(1-\tilde\eta)
-\tfrac{b_\omega}{2}\mathcal O^\top\tilde\omega
\Bigr],\\
W_v=b_v\Bigl[
&\tfrac12\|\tilde v\|^2
+\tfrac12\left(p_T+\tfrac{b_v^2}{4}\right)\|\mathcal T\|^2
+\tfrac{b_v}{4}\mathcal T^\top\tilde v
\Bigr].
\end{aligned}}
\tag{6}
\]

它们是性能证明的工具，不要求等于机械臂的物理动能。它们**同时含有刚度和阻尼**。

设计理由只有两点：

1. 位姿与 twist 的交叉项求导，会产生所需要的负位姿项；
2. \(b_\omega^2(1-\tilde\eta)\) 和平移势能中的 \(b_v^2/4\)，专门消去交叉项求导产生的阻尼交叉项。

其中 \(1-\tilde\eta\) 直接使用四元数标量部。这是本证明利用四元数结构的关键，
不能将它随意替换成另一个欧氏误差的平方。

### 2.3 正定性一行配方即可证明

由 \(1-\tilde\eta=\|\mathcal O\|^2/(1+\tilde\eta)\)，在正四元数半球内，

\[
\begin{aligned}
W_\omega
&\ge\frac{b_\omega}{2}
\left[
\left\|\tilde\omega-\frac{b_\omega}{2}\mathcal O\right\|^2+
\left(p_O+\frac{3b_\omega^2}{4}\right)\|\mathcal O\|^2
\right],\\
W_v
&=\frac{b_v}{2}
\left[
\left\|\tilde v+\frac{b_v}{4}\mathcal T\right\|^2+
\left(p_T+\frac{3b_v^2}{16}\right)\|\mathcal T\|^2
\right].
\end{aligned}
\tag{7}
\]

因此 \(p_O,p_T>0\) 时两者正定；在 \(\tilde\eta\ge\eta_0>0\) 内，
它们分别与对应误差的平方范数上下等价。无需额外搜索 \(\varepsilon\)、\(r\) 或 Lyapunov 矩阵。

---

## 3. 求导，再用 Schur 补一次性处理扰动

### 3.1 对 (6) 直接求导

把 (3) 代入 (6)，用
\((a\times b)^\top b=0\)，得到精确等式

\[
\boxed{
\begin{aligned}
\dot W_\omega={}&
-b_\omega^2\left(1-\frac{\tilde\eta}{4}\right)\|\tilde\omega\|^2
-\frac{b_\omega^2p_O\tilde\eta}{4}\|\mathcal O\|^2
+b_\omega\left(\tilde\omega-\frac{b_\omega}{2}\mathcal O\right)^\top d_\omega,\\
\dot W_v={}&
-\frac{3b_v^2}{4}\|\tilde v\|^2
-\frac{b_v^2p_T}{4}\|\mathcal T\|^2
-\frac{b_v^2}{4}\mathcal T^\top[\tilde\omega]_\times\tilde v
+b_v\left(\tilde v+\frac{b_v}{4}\mathcal T\right)^\top d_v.
\end{aligned}}
\tag{8}
\]

为检查最容易出错的抵消：

\[
\frac{d}{dt}\bigl[b_\omega^2(1-\tilde\eta)\bigr]
=-\frac{b_\omega^2}{2}\mathcal O^\top\tilde\omega,
\]

而 \(-\tfrac{b_\omega}{2}\mathcal O^\top\tilde\omega\) 的导数中，
恰好出现 \(+\tfrac{b_\omega^2}{2}\mathcal O^\top\tilde\omega\)。
平移侧也由 \(\tfrac{b_v^2}{8}\|\mathcal T\|^2\) 消去对应的阻尼交叉项。

**目前只有一项几何耦合没有消去：**
\(\mathcal T^\top[\tilde\omega]_\times\tilde v\)。
保留这个叉乘矩阵，直接交给 Schur 补，不把它拆成多个 Young 上界。

### 3.2 旋转侧：保留 \(\tilde\eta\)，精确反解 \(p_O\)

将旋转目标 (1) 移到左侧，代入 (5)、(8)，按
\([\mathcal O;\tilde\omega;d_\omega]\) 排列变量。左侧是以下矩阵对应二次型的负值，
因此要求矩阵半正定，就能保证旋转耗散目标：

\[
\begin{bmatrix}
\left(p_O\tilde\eta/\gamma_{\omega,\xi}^2-\gamma_{\omega,z}^{-2}\right)I_3
&0&\gamma_{\omega,\xi}^{-2}I_3\\
0&(3-\tilde\eta)\gamma_{\omega,\xi}^{-2}I_3
&-\gamma_{\omega,\xi}^{-1}I_3\\
\gamma_{\omega,\xi}^{-2}I_3&-\gamma_{\omega,\xi}^{-1}I_3&I_3
\end{bmatrix}.
\]

右下输入块是 \(I_3\)。先取其 Schur 补，得到以下 \(2\times2\) 标量矩阵
（每个元素乘 \(I_3\)）：

\[
\begin{bmatrix}
\dfrac{p_O\tilde\eta}{\gamma_{\omega,\xi}^2}
-\dfrac1{\gamma_{\omega,z}^2}
-\dfrac1{\gamma_{\omega,\xi}^4}
&
\dfrac1{\gamma_{\omega,\xi}^3}\\[4pt]
\dfrac1{\gamma_{\omega,\xi}^3}
&
\dfrac{2-\tilde\eta}{\gamma_{\omega,\xi}^2}
\end{bmatrix}\succeq0.
\tag{9}
\]

其右下块正定。再取 Schur 补，**等价地**得到

\[
p_O\ge
\frac{\gamma_{\omega,\xi}^2}{\tilde\eta\gamma_{\omega,z}^2}
+\frac1{\tilde\eta\gamma_{\omega,\xi}^2}
\left(1+\frac1{2-\tilde\eta}\right).
\tag{10}
\]

在 \(\tilde\eta\in[\eta_0,1]\) 内，\(\tilde\eta\) 和
\(\tilde\eta(2-\tilde\eta)\) 随 \(\tilde\eta\) 增加而增加，
所以右端最大值在 \(\eta_0\)。只需将 (10) 中的 \(\tilde\eta\) 换成 \(\eta_0\)。

这里没有把同一个 \(\tilde\eta\) 在不同项中分别取互不相容的最坏值。

### 3.3 平移侧：利用叉乘矩阵的谱结构反解 \(p_T\)

同样按 \([\mathcal T;\tilde v;d_v]\) 排列变量，移项并消去正定输入块后，
以下二次型矩阵半正定就能保证平移耗散目标：

\[
\begin{bmatrix}
\left(
\dfrac{p_T}{\gamma_{v,\xi}^2}
-\dfrac1{\gamma_{v,z}^2}
-\dfrac1{4\gamma_{v,\xi}^4}
\right)I_3
&
\dfrac{[\tilde\omega]_\times}{2\gamma_{v,\xi}^2}
-\dfrac{I_3}{2\gamma_{v,\xi}^3}\\[5pt]
-\dfrac{[\tilde\omega]_\times}{2\gamma_{v,\xi}^2}
-\dfrac{I_3}{2\gamma_{v,\xi}^3}
&
\dfrac{I_3}{\gamma_{v,\xi}^2}
\end{bmatrix}\succeq0.
\tag{11}
\]

再次取 Schur 补，得到

\[
\left(
\frac{p_T}{\gamma_{v,\xi}^2}
-\frac1{\gamma_{v,z}^2}
-\frac1{2\gamma_{v,\xi}^4}
\right)I_3
+\frac{[\tilde\omega]_\times^2}{4\gamma_{v,\xi}^2}
\succeq0.
\tag{12}
\]

由
\([\tilde\omega]_\times^2
=\tilde\omega\tilde\omega^\top-\|\tilde\omega\|^2I_3\)，
其最小特征值是 \(-\|\tilde\omega\|^2\)。故 (12) **等价于**

\[
p_T\ge
\frac{\gamma_{v,\xi}^2}{\gamma_{v,z}^2}
+\frac1{2\gamma_{v,\xi}^2}
+\frac{\|\tilde\omega\|^2}{4}.
\tag{13}
\]

用工作域界 \(\|\tilde\omega\|\le\bar\omega\) 替换最后一项即可。

这一步保留了“标量矩阵 + 反对称叉乘矩阵”的结构。
平方时混合项精确相消，因此没有出现逐项范数放缩产生的额外乘积项。

---

## 4. 两阶段调参的最终公式与定理

**第一阶段选阻尼，第二阶段选刚度；两者使用同一组储能函数。**

对任意四个正 Gamma，固定 \(\eta_0\in(0,1)\)、\(\bar\omega>0\)，取

\[
\boxed{
\begin{aligned}
K_d&=\operatorname{diag}
\left(\frac{2}{\gamma_{\omega,\xi}}I_3,
      \frac{2}{\gamma_{v,\xi}}I_3\right),\\
p_O&=
\frac{\gamma_{\omega,\xi}^2}{\eta_0\gamma_{\omega,z}^2}
+\frac1{\eta_0\gamma_{\omega,\xi}^2}
\left(1+\frac1{2-\eta_0}\right),\\
p_T&=
\frac{\gamma_{v,\xi}^2}{\gamma_{v,z}^2}
+\frac1{2\gamma_{v,\xi}^2}
+\frac{\bar\omega^2}{4},\\
K_p&=\operatorname{diag}(p_OI_3,p_TI_3).
\end{aligned}}
\tag{14}
\]

**定理。** 对 (3) 的任意解，只要
\(\tilde\eta(t)\ge\eta_0\)、\(\|\tilde\omega(t)\|\le\bar\omega\)，
式 (14) 的增益与式 (6) 的储能函数满足 (1)。在任意这样的时间区间 \([0,T]\)，

\[
\boxed{
\begin{aligned}
\frac{\|\mathcal O\|_{L_2(0,T)}^2}{\gamma_{\omega,z}^2}
+\frac{\|\tilde\omega\|_{L_2(0,T)}^2}{\gamma_{\omega,\xi}^2}
&\le\|d_\omega\|_{L_2(0,T)}^2+W_\omega(0),\\
\frac{\|\mathcal T\|_{L_2(0,T)}^2}{\gamma_{v,z}^2}
+\frac{\|\tilde v\|_{L_2(0,T)}^2}{\gamma_{v,\xi}^2}
&\le\|d_v\|_{L_2(0,T)}^2+W_v(0).
\end{aligned}}
\tag{15}
\]

**证明。** 式 (7) 保证储能非负；式 (9)—(13) 的 Schur 条件由 (14) 满足，
所以 (8) 蕴含 (1)。对 (1) 积分并删去非负的终端储能，即得 (15)。
全时留域且扰动有限能量时，令 \(T\to\infty\) 得无穷时域结论。证毕。

这也回答了“能否灵活调参”：

- **可以显式反算。** 固定工作域后，\(K_p,K_d\) 就是四个 Gamma 的函数。
- 减小位姿 Gamma 会增加对应刚度；减小 twist Gamma 会增加对应阻尼，并改变刚度预算。
- 正 Gamma 在理想误差模型中都能由 (14) 给出增益，但留域、力矩上限、采样带宽和模型失配可能限制可实现范围。
- 这些是离线固定参数。直接令 Gamma 随时间变化会使储能导数增加参数变化项，需要另行证明。

例如取
\(\gamma_{\omega,\xi}=0.2\)、\(\gamma_{\omega,z}=0.05\)、
\(\gamma_{v,\xi}=0.3\)、\(\gamma_{v,z}=0.1\)，
\(\eta_0=0.8\)、\(\bar\omega=1\)，则

\[
b_\omega=10,\quad b_v=6.6667,\quad p_O=77.2917,\quad p_T=14.8056.
\]

最后应区分两层“紧性”：**Schur 反演对选定二次型是精确的；选定储能函数和 \(b_s=2/\gamma_{s,\xi}\) 本身是充分设计选择。**
因此本证明有清楚的保守性来源，但不宣称式 (14) 是所有储能函数中的最优解。

---

## 5. 持续有界扰动与直接力矩级闭合

这一节补上前面 \(L_2\) 证书没有覆盖的部分。这里的关键不是把旧储能函数再 strictification 一次，而是注意到新储能函数 (6) 已经满足

\[
\dot W_\omega\le
-\left(
\frac{\|\mathcal O\|^2}{\gamma_{\omega,z}^2}+
\frac{\|\tilde\omega\|^2}{\gamma_{\omega,\xi}^2}
\right)+\|d_\omega\|^2,
\qquad
\dot W_v\le
-\left(
\frac{\|\mathcal T\|^2}{\gamma_{v,z}^2}+
\frac{\|\tilde v\|^2}{\gamma_{v,\xi}^2}
\right)+\|d_v\|^2 .
\tag{22}
\]

因此，式 (1) 本身已经是严格的状态耗散不等式；不需要再引入 \(e_z^\top K_pA e_\xi\)，也不需要估计 \(\dot A\)。下面只需把右端的输入能量换成输入幅值。

### 5.1 从耗散率得到 \(L_\infty\) 到状态的最终有界性

先给出不使用 Young 不等式的精确谱界。按 \([\mathcal O;\tilde\omega]\) 和 \([\mathcal T;\tilde v]\) 排列变量，定义

\[
Q_\omega=
\begin{bmatrix}
\gamma_{\omega,z}^{-2}&0\\0&\gamma_{\omega,\xi}^{-2}
\end{bmatrix},\qquad
H_\omega^{\rm up}=
\begin{bmatrix}
p_O+\dfrac{2b_\omega^2}{1+\eta_0}&-\dfrac{b_\omega}{2}\\[4pt]
-\dfrac{b_\omega}{2}&1
\end{bmatrix},
\tag{23}
\]

\[
Q_v=
\begin{bmatrix}
\gamma_{v,z}^{-2}&0\\0&\gamma_{v,\xi}^{-2}
\end{bmatrix},\qquad
H_v^{\rm up}=
\begin{bmatrix}
p_T+\dfrac{b_v^2}{4}&\dfrac{b_v}{4}\\[4pt]
\dfrac{b_v}{4}&1
\end{bmatrix}.
\tag{24}
\]

由 \(1-\tilde\eta=\|\mathcal O\|^2/(1+\tilde\eta)\) 和 \(\tilde\eta\ge\eta_0\)，式 (6) 给出

\[
\begin{aligned}
W_\omega
&\le \frac{b_\omega}{2}
\begin{bmatrix}\mathcal O\\\tilde\omega\end{bmatrix}^{\!\top}
H_\omega^{\rm up}
\begin{bmatrix}\mathcal O\\\tilde\omega\end{bmatrix},\\
W_v
&= \frac{b_v}{2}
\begin{bmatrix}\mathcal T\\\tilde v\end{bmatrix}^{\!\top}
H_v^{\rm up}
\begin{bmatrix}\mathcal T\\\tilde v\end{bmatrix}.
\end{aligned}
\tag{25}
\]

两个 \(H^{\rm up}\) 都正定。令

\[
c_\omega=
\frac{2}
{b_\omega\,
\lambda_{\max}\!\left(
Q_\omega^{-1/2}H_\omega^{\rm up}Q_\omega^{-1/2}
\right)},\qquad
c_v=
\frac{2}
{b_v\,
\lambda_{\max}\!\left(
Q_v^{-1/2}H_v^{\rm up}Q_v^{-1/2}
\right)}.
\tag{26}
\]

其依据是对 \(s\in\{\omega,v\}\) 都有
\(H_s^{\rm up}\preceq
\lambda_{\max}(Q_s^{-1/2}H_s^{\rm up}Q_s^{-1/2})Q_s\)。
把此矩阵序关系代入 (25)，即
\(W_s\le q_s/c_s\)，其中 \(q_s\) 是 (1) 左侧的两个输出项之和。故得到对上述二次型上界最紧的常数所对应的充分界

\[
\frac{\|\mathcal O\|^2}{\gamma_{\omega,z}^2}+
\frac{\|\tilde\omega\|^2}{\gamma_{\omega,\xi}^2}
\ge c_\omega W_\omega,\qquad
\frac{\|\mathcal T\|^2}{\gamma_{v,z}^2}+
\frac{\|\tilde v\|^2}{\gamma_{v,\xi}^2}
\ge c_v W_v .
\tag{27}
\]

假设

\[
\|d_\omega(t)\|\le D_\omega,\qquad
\|d_v(t)\|\le D_v
\quad\text{几乎处处成立}.
\tag{28}
\]

将 (27) 代入 (22)，得到

\[
\dot W_\omega\le-c_\omega W_\omega+D_\omega^2,\qquad
\dot W_v\le-c_v W_v+D_v^2 .
\tag{29}
\]

比较引理给出完整的瞬态和最终界：

\[
\boxed{
\begin{aligned}
W_\omega(t)
&\le e^{-c_\omega t}W_\omega(0)
+\frac{D_\omega^2}{c_\omega}
(1-e^{-c_\omega t}),\\
W_v(t)
&\le e^{-c_v t}W_v(0)
+\frac{D_v^2}{c_v}
(1-e^{-c_v t}).
\end{aligned}}
\tag{30}
\]

为了把储能界换成误差界，定义

\[
\begin{aligned}
\underline w_{\omega,z}
&=\frac{b_\omega}{2}
\left(p_O+\frac{3b_\omega^2}{4}\right),&
\underline w_{\omega,\xi}
&=\frac{b_\omega}{2}
\left(1-\frac{b_\omega^2}{4(p_O+b_\omega^2)}\right),\\
\underline w_{v,z}
&=\frac{b_v}{2}
\left(p_T+\frac{3b_v^2}{16}\right),&
\underline w_{v,\xi}
&=\frac{b_v}{2}
\left(1-\frac{b_v^2}{16(p_T+b_v^2/4)}\right).
\end{aligned}
\tag{31}
\]

式 (7) 及其 Schur 补下界保证

\[
W_\omega\ge
\underline w_{\omega,z}\|\mathcal O\|^2,\qquad
W_\omega\ge
\underline w_{\omega,\xi}\|\tilde\omega\|^2,
\tag{32}
\]

\[
W_v\ge
\underline w_{v,z}\|\mathcal T\|^2,\qquad
W_v\ge
\underline w_{v,\xi}\|\tilde v\|^2.
\tag{33}
\]

所以，例如旋转位姿误差满足

\[
\boxed{
\|\mathcal O(t)\|^2
\le
\frac{e^{-c_\omega t}W_\omega(0)
+(D_\omega^2/c_\omega)(1-e^{-c_\omega t})}
{\underline w_{\omega,z}}.
}
\tag{34}
\]

其他三个误差的公式由 (31)--(33) 同理得到。特别地，

\[
\boxed{
\begin{aligned}
\limsup_{t\to\infty}\|\mathcal O(t)\|
&\le\frac{D_\omega}
{\sqrt{c_\omega\underline w_{\omega,z}}},&
\limsup_{t\to\infty}\|\tilde\omega(t)\|
&\le\frac{D_\omega}
{\sqrt{c_\omega\underline w_{\omega,\xi}}},\\
\limsup_{t\to\infty}\|\mathcal T(t)\|
&\le\frac{D_v}
{\sqrt{c_v\underline w_{v,z}}},&
\limsup_{t\to\infty}\|\tilde v(t)\|
&\le\frac{D_v}
{\sqrt{c_v\underline w_{v,\xi}}}.
\end{aligned}}
\tag{35}
\]

这证明了当前新储能函数对持续有界等效扰动的局部 ISS/最终有界结论。证明只用了 Schur 证书、广义特征值和比较引理，没有使用 Young 不等式。

### 5.2 把关节力矩和外力矩接入同一证书

令

\[
B(\boldsymbol q)\triangleq J(\boldsymbol q)M(\boldsymbol q)^{-1},
\qquad
B_\omega=[B]_{1:3,:},\qquad
B_v=[B]_{4:6,:}.
\tag{36}
\]

将计算力矩中的反馈项记为

\[
u_{\rm fb}=-K_de_\xi-A^\top K_pe_z,
\tag{37}
\]

在当前任务空间计算力矩接口中，前馈项与反馈项通过

\[
\ddot{\boldsymbol q}_{\rm ref}
=J^+\bigl(u_{\rm ff}+u_{\rm fb}-\dot J\dot{\boldsymbol q}\bigr)
\tag{37a}
\]

接入关节逆动力学。式 (37a) 只是运动学关系
\(\ddot x=J\ddot{\boldsymbol q}+\dot J\dot{\boldsymbol q}\)
在满行秩条件下的理想接口写法。实现若使用阻尼伪逆或加速度治理器，
则 \(JJ^+u_{\rm fb}=u_{\rm fb}\) 一般不再成立；其反馈相关残差必须和
\(\Theta u_{\rm fb}\) 一起界定，前馈相关残差才可并入 \(d_{\rm ff}\)。

把惯量失配中由该反馈项产生的乘性部分单独取出，则完整扰动可写成

\[
\boxed{
d=\Theta u_{\rm fb}
+B(\delta\boldsymbol\tau+\boldsymbol\tau_{\rm ext})
+d_{\rm ff},
}
\tag{38}
\]

其中

\[
\Theta=JM^{-1}\Delta M J^+,
\tag{39}
\]

\[
d_{\rm ff}
=JM^{-1}\!\left[
\Delta M J^+(u_{\rm ff}-\dot J\dot{\boldsymbol q})
+\Delta C\dot{\boldsymbol q}
+\Delta\boldsymbol g+\Delta\boldsymbol f
\right].
\tag{40}
\]

式 (38) 在 (37a) 的理想满行秩接口下，只是把式 (4) 按“反馈相关”和“外生/前馈相关”重新分组，没有改变控制律或误差坐标。
具体地，\(\Delta M\ddot{\boldsymbol q}_{\rm ref}
=\Delta M J^+u_{\rm fb}
+\Delta M J^+(u_{\rm ff}-\dot J\dot{\boldsymbol q})\)；
先左乘 \(JM^{-1}\)，再分别归入 (39) 和 (40)，其余原始力矩经 \(B=JM^{-1}\) 进入 (38)。

先考虑不含乘性项 \(\Theta u_{\rm fb}\) 的情形。若在机械臂正则工作集上

\[
\bar B_\omega=\sup\|B_\omega\|_2,\qquad
\bar B_v=\sup\|B_v\|_2,
\tag{41}
\]

且

\[
\|d_{{\rm ff},\omega}(t)\|\le D_{{\rm ff},\omega},\qquad
\|d_{{\rm ff},v}(t)\|\le D_{{\rm ff},v},
\tag{42}
\]

同时有

\[
\|\delta\boldsymbol\tau(t)\|\le\bar\tau,\qquad
\|\boldsymbol\tau_{\rm ext}(t)\|\le\bar\tau_{\rm ext},
\tag{43}
\]

则

\[
\boxed{
\begin{aligned}
D_\omega
&=\bar B_\omega(\bar\tau+\bar\tau_{\rm ext})+D_{{\rm ff},\omega},\\
D_v
&=\bar B_v(\bar\tau+\bar\tau_{\rm ext})+D_{{\rm ff},v}
\end{aligned}}
\tag{44}
\]

满足 (28)，因此 (30)--(35) 直接给出物理力矩到任务误差的最终界。

如果输入是有限能量力矩，且 \(\Theta=0\)，则

\[
\boxed{
\begin{aligned}
\|d_\omega\|_{L_2}
&\le \bar B_\omega
\bigl(\|\delta\boldsymbol\tau\|_{L_2}
+\|\boldsymbol\tau_{\rm ext}\|_{L_2}\bigr)
+\|d_{{\rm ff},\omega}\|_{L_2},\\
\|d_v\|_{L_2}
&\le \bar B_v
\bigl(\|\delta\boldsymbol\tau\|_{L_2}
+\|\boldsymbol\tau_{\rm ext}\|_{L_2}\bigr)
+\|d_{{\rm ff},v}\|_{L_2}.
\end{aligned}}
\tag{45}
\]

把 (45) 代入式 (15)，得到直接以关节力矩为输入的四个 \(L_2\) 认证上界。例如零初值时，

\[
\|\mathcal O\|_{L_2}
\le
\gamma_{\omega,z}\!
\left[
\bar B_\omega
\bigl(\|\delta\boldsymbol\tau\|_{L_2}
+\|\boldsymbol\tau_{\rm ext}\|_{L_2}\bigr)
+\|d_{{\rm ff},\omega}\|_{L_2}
\right].
\tag{46}
\]

当 \(\Theta\ne0\) 时，(45) 不再是总扰动的上界；应先按附录 C 的有限区间小增益条件闭合，再把外生力矩范数代入放大后的 Gamma。

### 5.3 乘性惯量失配下的 \(L_\infty\) 小增益闭合

下面不再把 \(\Theta u_{\rm fb}\) 偷换成外生扰动。此处把旋转量和平移量放进同一个六维范数，须预先固定单位归一化（例如角度以 \(1\,\mathrm{rad}\)、长度以 \(1\,\mathrm m\) 为单位取数值，速度及加速度沿用对应单位）；\(W_\omega+W_v\)、\(\alpha\)、\(L\) 和 \(D_{\rm ex}\) 都按同一数值尺度计算。改变尺度会改变这些常数，不能直接把不同物理单位的原始数值相加。设所选紧工作集上

\[
\|\Theta\|_2\le\alpha,\qquad
\|A(\tilde x)\|_2\le\bar A,
\tag{47}
\]

并令

\[
\underline w=
\min\left\{
\frac{b_\omega}{2}\lambda_{\min}\!\begin{pmatrix}
p_O+b_\omega^2&-b_\omega/2\\-b_\omega/2&1
\end{pmatrix},\ 
\frac{b_v}{2}\lambda_{\min}\!\begin{pmatrix}
p_T+b_v^2/4&b_v/4\\b_v/4&1
\end{pmatrix}
\right\},
\quad
c=\min\{c_\omega,c_v\},
\tag{48}
\]

\[
\bar k_d=\|K_d\|_2,\qquad
\bar k_p=\|K_p\|_2,\qquad
L=\sqrt{\frac{\bar k_d^2+\bar A^2\bar k_p^2}{\underline w}}.
\tag{49}
\]

由 \(1-\tilde\eta\ge\|\mathcal O\|^2/2\)，式 (6) 的旋转部分下界正是 (48) 第一个矩阵乘以 \(b_\omega/2\) 的联合二次型；平移部分等于第二个矩阵乘以 \(b_v/2\) 的联合二次型。因此由最小特征值关系有

\[
\|e_z\|^2+\|e_\xi\|^2\le\frac{W_\omega+W_v}{\underline w},
\tag{50}
\]

所以由 (37)、(49) 和二维系数向量的 Cauchy--Schwarz 不等式

\[
\|u_{\rm fb}\|
\le L\sqrt{W_\omega+W_v}.
\tag{51}
\]

记

\[
d_{\rm ex}=B(\delta\boldsymbol\tau+\boldsymbol\tau_{\rm ext})+d_{\rm ff},
\qquad
\|d_{\rm ex}(t)\|\le D_{\rm ex}.
\tag{52}
\]

把式 (38) 代入两条耗散不等式并相加，利用 (27)、(47)、(51)，得到

\[
\dot W
\le-cW+\bigl(\alpha L\sqrt W+D_{\rm ex}\bigr)^2,
\qquad W=W_\omega+W_v.
\tag{53}
\]

若

\[
\boxed{\alpha L<\sqrt c,}
\tag{54}
\]

则当

\[
\sqrt W>\frac{D_{\rm ex}}{\sqrt c-\alpha L}
\tag{55}
\]

时，(53) 的右端严格小于零。因为

\[
-cW+(\alpha L\sqrt W+D_{\rm ex})^2
=-(\sqrt c\,\sqrt W-\alpha L\sqrt W-D_{\rm ex})
(\sqrt c\,\sqrt W+\alpha L\sqrt W+D_{\rm ex}).
\tag{56}
\]

更具体地，令 (57) 右端为比较方程的非负平衡点。对任何大于该值的阈值，(53) 的右端在阈值处严格为负；标量比较和首次越界反证表明，解只能向该阈值以下运动且不能从下向上穿越。让阈值趋向平衡点，得到

\[
\boxed{
\limsup_{t\to\infty}W(t)
\le
\frac{D_{\rm ex}^2}{(\sqrt c-\alpha L)^2}.
}
\tag{57}
\]

再结合 (31)--(33)，得到四个任务误差的最终界。例如

\[
\limsup_{t\to\infty}\|\mathcal O(t)\|
\le
\frac{D_{\rm ex}}
{(\sqrt c-\alpha L)\sqrt{\underline w_{\omega,z}}}.
\tag{58}
\]

式 (54) 是持续有界物理扰动下的闭环小增益条件。它清楚地说明：增大 \(K_p\) 虽然能减小理想模型的位姿 Gamma，却也会增大 \(L\)，从而降低对惯量乘性失配的鲁棒余量。这正是力矩级调参时必须同时报告的折衷。

还可以把这个半径直接用于工作域留域检查。令

\[
W_{\partial,\omega}
=\min\left\{
\underline w_{\omega,z}(1-\eta_0^2),\,
\underline w_{\omega,\xi}\bar\omega^2
\right\},
\qquad
R_W=\frac{D_{\rm ex}^2}{(\sqrt c-\alpha L)^2}.
\tag{58a}
\]

若初值满足 \(W(0)<W_{\partial,\omega}\)，并且

\[
\boxed{\max\{W(0),R_W\}<W_{\partial,\omega},}
\tag{58b}
\]

则 (53) 在阈值外给出严格下降，故 \(W(t)\) 不会越过
\(W_{\partial,\omega}\)。结合 (32)，旋转四元数半球条件
\(\tilde\eta\ge\eta_0\) 和速度条件
\(\|\tilde\omega\|\le\bar\omega\) 在整个闭环过程中保持成立。
工程上先根据任务轨迹、传感器范围和雅可比正则性选取候选
\((\eta_0,\bar\omega)\)，再用 (58b) 检查；若不满足，应扩大工作域、降低扰动包络或重新选择 Gamma，不能把 (14) 当作无条件全局结论。

### 5.4 这一节结论的边界

式 (30)--(35) 处理的是等效扰动幅值已知的连续时间结论；式 (44)--(46) 把关节力矩和外力矩换算成该幅值或能量；式 (54)--(58) 才进一步闭合了反馈相关的惯量乘性失配。所有结论仍要求轨迹留在一个使 \(J,M,A\) 有界且 \(J\) 满行秩的紧工作集内。

因此，常值负载现在可以处理，但结论是**局部最终有界**，不是零误差，也不是全局保证。采样、延迟、力矩饱和、阻尼伪逆及加速度治理器的残差若未按反馈相关性纳入 \(\alpha\) 或 \(D_{\rm ex}\)，不能直接援引 (57)。

---

## 附录 A：怎样保证解确实留在工作域

这一段只用于核对定理前提，不参与增益反演。

由 (7) 及旋转储能的二次型下界，

\[
\begin{aligned}
W_\omega&\ge
\frac{b_\omega}{2}
\left(p_O+\frac{3b_\omega^2}{4}\right)\|\mathcal O\|^2,\\
W_\omega&\ge
\frac{b_\omega}{2}
\left(1-\frac{b_\omega^2}{4(p_O+b_\omega^2)}\right)\|\tilde\omega\|^2.
\end{aligned}
\tag{16}
\]

第二行由对 \(\mathcal O\) 的二次型取 Schur 补得到。若初始在工作域内，且

\[
\boxed{
W_\omega(0)+\|d_\omega\|_{L_2}^2<
\min\left\{
\frac{b_\omega}{2}
\left(p_O+\frac{3b_\omega^2}{4}\right)(1-\eta_0^2),\quad
\frac{b_\omega}{2}
\left(1-\frac{b_\omega^2}{4(p_O+b_\omega^2)}\right)\bar\omega^2
\right\},
}
\tag{17}
\]

则解全时满足 \(\tilde\eta>\eta_0\)、\(\|\tilde\omega\|<\bar\omega\)。

证明采用首次出域反证：在首次出域前，(1) 给出
\(W_\omega(t)\le W_\omega(0)+\|d_\omega\|_{L_2(0,t)}^2\)；
而到达任何一条边界，(16) 都要求储能至少达到 (17) 右端，产生矛盾。
四元数使用连续分支，初始选 \(\tilde\eta(0)>0\)。

这只保证**任务误差工作域**。关节限位、雅可比正则性与完整机械臂解的延拓仍须单独保证。

## 附录 B：如何换成原始力矩输入的 Gamma

仅考虑执行器力矩扰动时，\(d=JM^{-1}\delta\boldsymbol\tau\)。因此零初值下，

\[
\|\mathcal O\|_{L_2}
\le\gamma_{\omega,z}\|d_\omega\|_{L_2}
\le\gamma_{\omega,z}
\sup_t\|JM^{-1}\|_2\,\|\delta\boldsymbol\tau\|_{L_2}.
\tag{18}
\]

其他三个输出同理。**原始力矩级增益的认证上界，是 (14) 中的 Gamma 乘以该输入映射上界。**

若事先给定原始力矩级目标，应先除以这个固定上界，得到 (14) 使用的等效加速度 Gamma。
此换算可能保守，因为使用了最坏奇异值；不能将它称为原始力矩传递函数的精确范数。

多个独立物理输入必须明确其堆叠能量权重。
持续外力不满足无穷时域 \(L_2\) 假设；其幅值已知时应使用第 5 节的最终有界结论，而不是把它当作 \(L_2\) 输入。

## 附录 C：惯量乘性失配怎样闭合

总扰动的性能证书不自动等于外生残差的性能证书。
若当前接口分解为

\[
d=\Theta u_{\rm fb}+d_{\rm ex},\qquad
\|\Theta\|_2\le\alpha,
\]

则需同时控制完整位姿与 twist 反馈：

\[
u_{\rm fb}=
\begin{bmatrix}
-b_\omega\tilde\omega+\tfrac12p_O\tilde\eta\mathcal O\\
-b_v\tilde v-p_T\mathcal T
\end{bmatrix}.
\]

在固定旋转/平移尺度的六维度量下，Cauchy–Schwarz 给出

\[
\begin{aligned}
\|u_{\rm fb}\|^2
\le&
\max\left\{
4+\frac{p_O^2\gamma_{\omega,z}^2}{4},\quad
4+p_T^2\gamma_{v,z}^2
\right\}\\
&\cdot\left(
\frac{\|\mathcal O\|^2}{\gamma_{\omega,z}^2}
+\frac{\|\tilde\omega\|^2}{\gamma_{\omega,\xi}^2}
+\frac{\|\mathcal T\|^2}{\gamma_{v,z}^2}
+\frac{\|\tilde v\|^2}{\gamma_{v,\xi}^2}
\right).
\end{aligned}
\tag{19}
\]

结合零初值的 (15)，一个包含刚度项的充分小增益条件是

\[
\boxed{
\alpha
\sqrt{\max\left\{
4+\frac{p_O^2\gamma_{\omega,z}^2}{4},\quad
4+p_T^2\gamma_{v,z}^2
\right\}}<1.
}
\tag{20}
\]

满足 (20)、\(d_{\rm ex}\in L_2\) 并留域时，
\(d_{\rm ex}\) 到四个输出的认证 Gamma 分别放大为

\[
\frac{\gamma_{s,z}\ \text{或}\ \gamma_{s,\xi}}
{1-\alpha\sqrt{\max\left\{
4+p_O^2\gamma_{\omega,z}^2/4,\ 4+p_T^2\gamma_{v,z}^2
\right\}}},\qquad s\in\{\omega,v\}.
\tag{21}
\]

推理应先在有限区间进行：由 (15) 控制加权输出能量，由 (19) 控制反馈，再用
\(d=\Theta u_{\rm fb}+d_{\rm ex}\) 收拢不等式；
(20) 使分母为正，随后才能推出总扰动有限能量。这样不会循环假设 \(d\in L_2\)。

式 (20) 是附加的保守小增益条件，不是主证明的精确 Schur 条件。
\(d_{\rm ex}\) 仍含状态相关动力学余项，其有限能量性质必须单独验证；
若只知道其持续有界，应使用第 5 节的幅值型小增益结论 (53)--(58)，而不能使用 (21)。

---

## 核验记录

配套核验覆盖 5000 组非线性状态与参数，检查：

- 直接梯度求导与 (8) 完全一致；
- 最不利瞬时输入下，两条耗散不等式成立；
- Schur 矩阵半正定；
- 新增的广义特征值谱界成立；
- 六维反馈所需的联合储能下界成立；
- 旋转和平移的解析边界取等，核对常数与叉乘符号。

结果通过，导数最大相对误差约 \(3.8\times10^{-13}\)，新增谱界的最小数值残差约 \(1.7\times10^{-7}\)。
数值核验用于发现代数错误；理论依据仍是上面的正定性与 Schur 证明。

这份文档现在完成了论文主线的连续时间部分：结构化储能函数、显式多 Gamma 增益综合、\(L_2\) 性能、持续有界扰动的最终有界性、关节/外力矩映射，以及乘性惯量失配下的 \(L_\infty\) 小增益闭合。离散采样与延迟、力矩饱和的严格离散证明、实机已知力矩扰动实验和对已有文献的更广泛定量比较仍须另行完成。

## 配套 L2 数值实验（2026-09-27）

[四 Gamma 力矩级 L2 实验记录](四Gamma力矩级L2实验_耗散与同预算比较_20260927.md)
给出 27 条有限能量关节力矩脉冲轨迹的耗散检验，以及 C1、Chandra C2、
解耦四元数 C4 在共享四参数候选池和非重力力矩预算下的 75 条比较轨迹。
连续时间残差非正，离散积分残差最大为 \(7.88\times10^{-13}\)；
预算合格样本中，三律在当前近恒等工况的误差表现基本重合，不能据此声称 C1 跟踪更优。
这一数值实验并不补足上段列出的离散采样、模型失配及实机扰动边界证明。
