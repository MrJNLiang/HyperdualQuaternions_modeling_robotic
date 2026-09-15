# TODQ 理论改进方案：四元数结构优势与高质量期刊路线

> 本文是独立分析文档。它不修改 `texdocs/` 中的任何论文内容，也不替换现有定理。
>
> 核心结论：若想把工作提升到更高质量期刊，不应继续堆叠 TODQ 阶数或增加一个普通负反馈项，而应把理论主线改成：
>
> **利用单位四元数的二次范数恒等式和双覆盖结构，构造不依赖李群对数的、符号不变的几何误差；在离散采样、模型失配和任务空间扰动下给出可计算的区域证书。**

---

## 1. 先判断现有理论的真实强度

### 1.1 现有闭环结构

当前控制器的误差模型可抽象为

\[
\dot e_z=A(\tilde x)e_\xi,
\qquad
\dot e_\xi=-K_de_\xi-A(\tilde x)^\top K_pe_z+d.
\tag{1}
\]

取

\[
V=\frac12e_\xi^\top e_\xi+\frac12e_z^\top K_pe_z,
\tag{2}
\]

则

\[
\begin{aligned}
\dot V
&=e_\xi^\top\dot e_\xi+e_z^\top K_p\dot e_z\\
&=e_\xi^\top(-K_de_\xi-A^\top K_pe_z+d)
  +e_z^\top K_pAe_\xi\\
&=-e_\xi^\top K_de_\xi+e_\xi^\top d.
\end{aligned}
\tag{3}
\]

这是一个漂亮的代数恒等式，但它的逻辑含义必须准确：

- 它是对**特定误差坐标、特定反馈和特定存储函数**成立的耗散恒等式；
- 它不是一个新的通用 $H_\infty$ 理论；
- Schur 补给出的是该二次证书族的可行条件，不是所有控制器的性能最优性；
- 加入 $\Delta M$ 后，实际扰动含有 $\Theta u_{\rm fb}$，因此 $1/\lambda_{\min}(K_d)$ 不能直接作为真实不确定机械臂的增益；
- 当前 $L_\infty$ 结果是工作集内的 twist RMS 界，不是完整 ISS 或全局位姿有界性；
- $A(\tilde x)$ 在四元数标量部为零时奇异，所以不能宣称全局几何鲁棒性。

因此，现有理论本身更适合被称为：

> 一个基于双对偶/三阶对偶四元数的局部几何耗散框架。

### 1.2 为什么“再加一项负反馈”不是高水平理论

设把控制律改为

\[
u_{
m fb}=-K_de_\xi-A^\top K_pe_z-K_x\phi(e_z,e_\xi).
\tag{4}
\]

若 $\phi=e_\xi$ 或 $\phi=e_z$，本质上只是改变等效阻尼或刚度。任何基线都可以加入同样的项；在相同力矩、带宽和饱和预算下，优势不能归因于 TODQ。除非新增项同时带来以下至少一项，否则不是高质量理论贡献：

1. 新的可测信息或新的扰动观测量；
2. 对离散采样、延迟、饱和或模型失配的严格保证；
3. 一个基线不能直接复制的几何不变量；
4. 更大的可证明吸引域或更弱的正则性假设；
5. 与安全约束、接触任务或在线认证直接相关的设计规则。

---

## 2. 推荐的理论主线：四元数内积误差的区域认证

### 2.1 四元数真正独有的结构

单位旋转四元数写为

\[
r=\eta+\boldsymbol\mu,
\qquad
\eta^2+\|\boldsymbol\mu\|^2=1.
\tag{5}
\]

它相对欧拉角的优势是避免坐标图奇异；相对李群对数的优势是：

- 不需要计算 $\log(R)$；
- 不需要除以 $\sin(\theta/2)$ 或 $\theta$；
- 乘法、共轭和内积都是多项式/双线性运算；
- $r$ 和 $-r$ 表示同一个旋转；
- $\eta=\cos(\theta/2)$ 和 $\boldsymbol\mu=\boldsymbol a\sin(\theta/2)$ 直接给出有限角误差的几何量。

但四元数也有一个不能回避的事实：双覆盖意味着不存在连续、全局、无奇异的唯一误差向量。任何声称“全局无奇异”的控制律都必须说明它采用的是：

- 固定半球的 almost-global 结果；或
- hybrid/discontinuous 的全局结果；或
- 只对速度误差给出符号不变性，而非对位姿向量给出全局连续性。

### 2.2 建议的误差变量

对实际和期望旋转四元数 $r,r_d$，定义右误差

\[
\tilde r=r r_d^*
      =\tilde\eta+\tilde{\boldsymbol\mu}.
\tag{6}
\]

为消除双覆盖的表示歧义，定义符号不变标量

\[
s(\tilde r)=|\tilde\eta|,
\tag{7}
\]

以及在固定半球内使用的向量误差

\[
e_R=\operatorname{sgn}(\tilde\eta)\tilde{\boldsymbol\mu},
\qquad \tilde\eta\ne0.
\tag{8}
\]

则

\[
s^2+\|e_R\|^2=1.
\tag{9}
\]

这个恒等式是后续区域证书的核心。它完全不需要 $\operatorname{Log}:SO(3)\to so(3)$。

对位移，保留右不变误差 $e_p$，并引入一个特征长度 $\ell>0$ 做尺度统一：

\[
e_z^s=
\begin{bmatrix}
 e_R\\ e_p/\ell
\end{bmatrix}.
\tag{10}
\]

对应的 twist 误差采用同样的尺度：

\[
e_\xi^s=
\begin{bmatrix}
\tilde\omega\\ \tilde v/\ell
\end{bmatrix}.
\tag{11}
\]

这样旋转和平移才可以放入同一个欧氏存储函数，而不会把 rad 与 m、rad/s 与 m/s 直接混加。

---

## 3. 四元数区域几何的详细推导

### 3.1 旋转误差运动学

采用左乘约定

\[
\dot{\tilde r}=\frac12\tilde\omega\tilde r,
\qquad
\tilde\omega=0+\boldsymbol\omega_e.
\tag{12}
\]

四元数乘法给出

\[
\dot{\tilde\eta}
=-\frac12\boldsymbol\omega_e^\top\tilde{\boldsymbol\mu},
\tag{13}
\]

\[
\dot{\tilde{\boldsymbol\mu}}
=\frac12\bigl(\tilde\eta I_3+[\tilde{\boldsymbol\mu}]_\times\bigr)\boldsymbol\omega_e.
\tag{14}
\]

在固定半球 $\tilde\eta>0$ 内，$e_R=\tilde{\boldsymbol\mu}$，因此

\[
\dot e_R
=\frac12B(\tilde r)\boldsymbol\omega_e,
\qquad
B(\tilde r)=\tilde\eta I_3+[e_R]_\times.
\tag{15}
\]

由 $[e_R]_\times^\top=-[e_R]_\times$ 和

\[
[e_R]_\times^2=e_Re_R^\top-\|e_R\|^2I_3,
\tag{16}
\]

可得

\[
B^\top B
=(\tilde\eta I_3-[e_R]_\times)(\tilde\eta I_3+[e_R]_\times)
=I_3-e_Re_R^\top.
\tag{17}
\]

因为 $\tilde\eta^2+\|e_R\|^2=1$，矩阵 $B^\top B$ 的特征值为

\[
\{\tilde\eta^2,1,1\}.
\tag{18}
\]

所以

\[
\sigma_{\min}(B)=\tilde\eta,
\qquad
\sigma_{\max}(B)=1.
\tag{19}
\]

这说明四元数向量误差到角速度误差的映射在半球内部可逆，其条件数为

\[
\operatorname{cond}_2(B)=\frac1{\tilde\eta}.
\tag{20}
\]

当姿态误差接近 $180^\circ$ 时，$\tilde\eta\to0$，条件数必然发散。这不是推导缺陷，而是双覆盖拓扑和连续向量误差的必然后果。

### 3.2 一个比原有 $A$ 更简单的旋转势函数

定义符号不变的势函数

\[
\Psi_R(\tilde r)=1-|\tilde\eta|.
\tag{21}
\]

在固定半球 $\tilde\eta>0$ 内，$\Psi_R=1-\tilde\eta$。由式 (13)：

\[
\dot\Psi_R
=-\dot{\tilde\eta}
=\frac12e_R^\top\boldsymbol\omega_e.
\tag{22}
\]

这一步非常重要：旋转势函数的导数是一个**精确线性内积**，不需要构造

\[
A_{11}=-\frac12(\tilde\eta I+[e_R]_\times)
\]

也不需要通过完整的 $A^\top$ 反馈处理叉乘耦合。

若定义旋转存储函数

\[
V_R=\frac12\|\boldsymbol\omega_e\|^2+k_R\Psi_R,
\tag{23}
\]

并选择旋转误差反馈

\[
\dot{\boldsymbol\omega}_e
=-K_\omega\boldsymbol\omega_e-k_Re_R+d_R,
\tag{24}
\]

则

\[
\begin{aligned}
\dot V_R
&=\boldsymbol\omega_e^\top
  (-K_\omega\boldsymbol\omega_e-k_Re_R+d_R)
  +k_R\frac12e_R^\top\boldsymbol\omega_e.
\end{aligned}
\tag{25}
\]

这里出现一个系数匹配问题：由四元数定义，势函数 $1-\eta$ 的导数含 $1/2$。因此应取

\[
\dot{\boldsymbol\omega}_e
=-K_\omega\boldsymbol\omega_e-rac{k_R}{2}e_R+d_R,
\tag{26}
\]

从而

\[
\boxed{
\dot V_R
=-\boldsymbol\omega_e^\top K_\omega\boldsymbol\omega_e
+\boldsymbol\omega_e^\top d_R.
}
\tag{27}
\]

这是一种比原有 $A^\top K_pe_z$ 更直接的四元数耗散结构。它使用了单位四元数的标量部和向量部关系，而不是把姿态误差先当作普通三维欧氏坐标。

### 3.3 平移部分

若平移误差采用

\[
\dot e_p=\tilde v-[e_p]_\times\tilde\omega,
\tag{28}
\]

则平移势函数

\[
\Psi_p=\frac12\|e_p/\ell\|^2
\tag{29}
\]

满足

\[
\dot\Psi_p
=\frac1{\ell^2}e_p^\top\tilde v,
\tag{30}
\]

因为

\[
e_p^\top[e_p]_\times\tilde\omega=0.
\tag{31}
\]

选择平移误差动力学

\[
\dot{\tilde v}
=-K_v\tilde v-\frac{k_p}{\ell^2}e_p+d_v,
\tag{32}
\]

并取

\[
V_p=\frac12\|\tilde v\|^2+rac{k_p}{2\ell^2}\|e_p\|^2,
\tag{33}
\]

可得

\[
\boxed{
\dot V_p=-\tilde v^\top K_v\tilde v+\tilde v^\top d_v.
}
\tag{34}
\]

### 3.4 统一的四元数/平移证书

定义

\[
V=V_R+V_p
=\frac12\|e_\xi^s\|^2
+k_R(1-|\tilde\eta|)
+\frac{k_p}{2\ell^2}\|e_p\|^2.
\tag{35}
\]

在固定符号半球内，闭环满足

\[
\boxed{
\dot V
=-(e_\xi^s)^\top K_d^s e_\xi^s
+(e_\xi^s)^\top d^s.
}
\tag{36}
\]

与原有二次 $e_z^\top K_pe_z$ 存储函数相比，这个存储函数的旋转势是四元数内积势，而不是把四元数向量误差当作全局欧氏坐标。它的优势是：

- 旋转势 $1-|\eta|$ 对双覆盖表示不变；
- 在小角度处 $1-\eta\approx\theta^2/8$，与真实角度平方一致到二阶；
- 不需要 $\log(R)$；
- 证明只用 $\dot\eta=-\omega_e^\top\mu/2$；
- 反馈项的系数可直接从四元数运动学得到，不需要完整的 6×6 $A$ 矩阵。

---

## 4. 可证明的区域稳定定理

### 定理 A：固定半球内的局部指数稳定

设 $K_\omega\succ0$、$K_v\succ0$、$k_R>0$、$k_p>0$，且初值满足

\[
\tilde\eta(0)>0,
\qquad
V(0)<k_R.
\tag{37}
\]

则：

1. $\tilde\eta(t)>0$，符号半球不发生切换；
2. $V(t)\le V(0)$；
3. $e_R\to0$、$e_p\to0$、$e_\xi^s\to0$；
4. 在远离 $\tilde\eta=0$ 的紧子水平集内，平衡点局部指数稳定。

#### 证明要点

由式 (36) 在无扰情况下有

\[
\dot V\le0.
\tag{38}
\]

由 $V(0)<k_R$ 和 $V(t)\le V(0)$：

\[
k_R(1-|\tilde\eta(t)|)\le V(0)<k_R,
\tag{39}
\]

故

\[
|\tilde\eta(t)|>0.
\tag{40}
\]

连续性和初始正号给出 $\tilde\eta(t)>0$。在该集合中，式 (22)、(27) 和 (34) 成立。令 $\dot V=0$，则 $e_\xi^s=0$；由误差运动学式 (22)、(30)，进一步得到 $e_R=0,e_p=0$。LaSalle 定理给出渐近收敛。在线性化系统中，$B(1)=I_3$，闭环是标准二阶系统，故局部指数稳定。

### 这个定理相对原定理的改进

它不声称全局稳定，而是给出一个透明的区域：

\[
V(0)<k_R.
\tag{41}
\]

这个条件直接说明初始姿态不能穿过 $180^\circ$ 分支。相比原有以 $\lambda_{\min}(K_{p,O})$ 写出的抽象水平集条件，它更接近四元数几何本身，解释也更清楚。

---

## 5. 扰动证书：把“证书”从形式变成可计算规则

### 5.1 加性扰动下的 $L_2$ 证书

由式 (36)，对任意 $\kappa>0$ 和 $\gamma_a>0$，若

\[
K_d^s\succeq
\frac12\left(\kappa^{-1}+\gamma_a^{-2}\right)I,
\tag{42}
\]

则

\[
\dot V
\le-rac1{2\kappa}\|e_\xi^s\|^2
+\frac{\gamma_a^2}{2}\|d^s\|^2.
\tag{43}
\]

零初值下积分得到

\[
\|e_\xi^s\|_{L_2}
\le \gamma_a\sqrt\kappa\,\|d^s\|_{L_2}.
\tag{44}
\]

令 $\gamma^*=\gamma_a\sqrt\kappa$，在该二次证书族中最小值仍为

\[
\gamma^*_{\rm cert}=
\frac1{\lambda_{\min}(K_d^s)}.
\tag{45}
\]

注意这只是**给定存储函数和加性扰动模型的最小证书值**。若 $d^s$ 含反馈相关的 $\Theta u_{\rm fb}$，应使用下面的鲁棒界。

### 5.2 乘性模型失配下的鲁棒界

设

\[
d^s=\Theta u_{\rm fb}+d_{\rm ex}^s,
\qquad
\|\Theta\|_2\le\alpha,
\tag{46}
\]

并且

\[
u_{\rm fb}=-K_de_\xi^s-G(e_R,e_p),
\tag{47}
\]

其中 $G$ 是姿态和平移反馈项。则

\[
\begin{aligned}
(e_\xi^s)^\top d^s
&=-(e_\xi^s)^\top\Theta K_de_\xi^s
 -(e_\xi^s)^\top\Theta G
 +(e_\xi^s)^\top d_{\rm ex}^s.
\end{aligned}
\tag{48}
\]

由谱范数不等式：

\[
-(e_\xi^s)^\top\Theta K_de_\xi^s
\le \alpha\lambda_{\max}(K_d)\|e_\xi^s\|^2.
\tag{49}
\]

若工作集内 $\|G\|\le \bar G$，且 $\|d_{\rm ex}^s\|\le D_{\rm ex}$，则

\[
\dot V
\le -\lambda_{\rm eff}\|e_\xi^s\|^2+D\|e_\xi^s\|,
\tag{50}
\]

其中

\[
\lambda_{\rm eff}
=\lambda_{\min}(K_d)-\alpha\lambda_{\max}(K_d),
\tag{51}
\]

\[
D=\alpha\bar G+D_{\rm ex}.
\tag{52}
\]

因此，在

\[
\boxed{
\alpha<
\frac{\lambda_{\min}(K_d)}{\lambda_{\max}(K_d)}
}
\tag{53}
\]

下，得到

\[
\limsup_{T\to\infty}
\left(\frac1T\int_0^T\|e_\xi^s\|^2dt\right)^{1/2}
\le
\frac{D}{\lambda_{\rm eff}}.
\tag{54}
\]

这个结果的正确表述是：**乘性失配下的条件性 twist RMS 界**。不能把它写成全状态 ISS 或全局位姿最终界。

---

## 6. 真正体现四元数优势的两个可投稿理论结果

### 6.1 结果一：无对数、无 Euler 奇异的有限角区域证书

可以提出以下命题：

> 对所有满足 $|\tilde\eta|\ge\eta_0>0$ 的旋转误差，四元数内积势和向量误差反馈不需要 $SO(3)$ 对数；其误差映射条件数满足 $\operatorname{cond}_2(B)\le1/\eta_0$，并可直接计算区域内的最小奇异值和证书余量。

与李群对数比较：

\[
\operatorname{Log}(R)=
\frac{\theta}{2\sin(\theta/2)}(R-R^\top)^\vee,
\tag{55}
\]

当 $\theta\to\pi$ 时分母趋于零，轴方向不唯一；而四元数只需检查 $|\eta|\ge\eta_0$。四元数并没有消除 $180^\circ$ 的拓扑障碍，但把它变成了一个**直接可检查的标量条件**。

与欧拉角比较：欧拉角的雅可比在特定姿态发生坐标奇异，而四元数的覆盖空间保持光滑；因此四元数的优势是**坐标表达无 chart singularity**，不是全局连续误差向量的存在。

### 6.2 结果二：符号不变的离散采样证书

这是比连续时间恒等式更有工程价值、也更可能提高期刊档次的方向。

设采样周期为 $h$，采用零阶保持，离散更新满足

\[
z_{k+1}=z_k+h f(z_k,d_k)+r_k,
\tag{56}
\]

其中 $z=[e_z^s;e_\xi^s]$，$r_k$ 是局部截断项。若在工作集内

\[
\|Df(z)\|\le L_f,
\qquad
\|D^2V(z)\|\le L_V,
\tag{57}
\]

则 Taylor 展开给出

\[
V_{k+1}-V_k
\le h\dot V_k+rac12L_Vh^2\|f(z_k,d_k)\|^2+O(h^3).
\tag{58}
\]

无扰时，由

\[
\dot V_k\le-\lambda_d\|e_{\xi,k}^s\|^2,
\qquad
\lambda_d=\lambda_{\min}(K_d),
\tag{59}
\]

若工作集内存在 $c_f>0$ 使

\[
\|f(z,0)\|^2\le c_f V(z),
\tag{60}
\]

则

\[
V_{k+1}
\le
V_k-h\lambda_d\|e_{\xi,k}^s\|^2
+\frac12L_Vc_fh^2V_k+O(h^3).
\tag{61}
\]

取

\[
h<h_{\max}
\triangleq
\frac{\lambda_d}{L_Vc_f}
\tag{62}
\]

并进一步把 $O(h^3)$ 吸收到二阶项中，即可得到

\[
V_{k+1}-V_k
\le
-h\mu\|e_{\xi,k}^s\|^2,
\qquad \mu>0.
\tag{63}
\]

有扰动时，若 $d_k$ 为分段常值并满足 $\|d_k\|\le D$，可以得到

\[
V_{k+1}-V_k
\le
-h\mu\|e_{\xi,k}^s\|^2
+h\frac{\gamma_d^2}{2}\|d_k\|^2
+h^2c_hV_k.
\tag{64}
\]

这会给出一个真正贴近实机的结论：

> 给定工作域的曲率界、阻尼增益和采样周期，离散实现仍保持耗散性的充分条件。

四元数在这里的特殊作用是：单位约束

\[
\eta_k^2+\|\mu_k\|^2=1
\tag{65}
\]

可以显式提供工作域内的有界导数和曲率界；而欧拉角需要额外避开坐标奇异，李群对数需要处理对数映射的分支和导数界。

> 注意：式 (62) 只是推导模板，不应直接作为最终定理中的数值公式。正式论文必须根据实际离散算法、投影归一化、零阶保持和限幅器，明确计算 $L_f,L_V,c_f,c_h$。

---

## 7. TODQ 应该怎样保留，怎样降级

### 7.1 TODQ 不应继续作为“性能优势”

TODQ 的合理定位是：

\[
\mathcal A_2
=\widehat{\mathbb H}[\sigma]/(\sigma^3),
\tag{66}
\]

\[
T^2x=x+\sigma\dot x+\frac12\sigma^2\ddot x.
\tag{67}
\]

对串联链 $x=\prod_i x_i$：

\[
T^2x=\prod_iT^2x_i,
\tag{68}
\]

因为二阶广义 Leibniz 公式：

\[
(ab)^{(2)}=a^{(2)}b+2a^{(1)}b^{(1)}+ab^{(2)}.
\tag{69}
\]

这证明了一个表示和实现事实：一次链乘同时得到 $x,\dot x,\ddot x$。但它不自动证明：

- 计算比递归牛顿–欧拉更快；
- 数值误差更小；
- 控制精度更高；
- 对任意机器人模型都更优。

因此 TODQ 应作为**实现层/接口层贡献**，而把高水平理论贡献放在四元数区域证书和离散耗散定理上。

### 7.2 最简洁的理论架构

推荐的论文理论结构为：

1. **命题 1：TODQ 提升同态**
   \[
   T^2(ab)=T^2a\,T^2b.
   \]

2. **定理 1：四元数内积误差的区域几何**
   \[
   \dot\Psi_R=\frac12e_R^\top\omega_e,
   \qquad
   \sigma_{\min}(B)=|\eta|.
   \]

3. **定理 2：连续时间局部耗散和扰动证书**
   \[
   \dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d.
   \]

4. **定理 3：离散采样保持证书**
   \[
   V_{k+1}-V_k
   \le-h\mu\|e_{\xi,k}\|^2+h\Gamma\|d_k\|^2.
   \]

5. **推论：四元数区域、采样周期和阻尼的联合设计条件**。

这个结构比现在把多个 $L_2$、$L_\infty$、乘性、分量拆分结果全部放在同一个定理 3 中更清楚，也更容易让审稿人识别主贡献。

---

## 8. 与李群、欧拉角的严格对比表

| 项目 | 欧拉角 | $SO(3)$ 李群对数 | 单位四元数方案 |
|---|---|---|---|
| 位姿表达 | 3 参数，存在 chart singularity | 3 参数局部坐标 | 4 参数带单位约束，无坐标 chart 奇异 |
| 大角度计算 | 依赖选定顺序，可能失效 | $\log(R)$ 在 $\pi$ 附近分支不唯一 | 乘法/共轭直接可算 |
| 误差向量 | 坐标依赖 | 几何自然，但需 log/Jacobian | 向量部简单，需处理双覆盖 |
| 乘法 | 非线性且不统一 | 群乘法 | 四元数双线性乘法 |
| 导数传播 | 需显式 Jacobian | 需 adjoint/Jacobian 公式 | 可用 DQ/HDQ/TODQ 乘法内化 |
| 拓扑限制 | 坐标奇异明显 | $\pi$ 分支/无全局连续 log | $\pm q$ 双覆盖，半球证书直接可检验 |
| 数值实现 | 三角函数和逆矩阵较多 | 矩阵指数/对数或 Jacobian 逆 | 乘加、共轭、归一化 |
| 可证明优势 | 很弱 | 几何解释强 | 区域证书、符号不变势、低阶多项式运算 |

正确的论文表述不是“ quaternion globally solves all singularities”，而是：

> 四元数把姿态表示从三参数坐标图提升到光滑的单位球面；本文进一步利用其标量部、向量部和单位范数恒等式，将双覆盖问题转化为一个可计算的半球工作域条件。

---

## 9. 需要修正的现有数学问题

在任何投稿前，以下问题必须解决，否则理论升级没有意义。

### 9.1 阻尼伪逆与精确分解冲突

若采用阻尼伪逆 $J^\#$，一般有

\[
JJ^\#\ne I_6.
\tag{70}
\]

因此不能同时使用 $JJ^+=I_6$ 的精确扰动分解。应当：

- 主定理只使用满行秩 Moore–Penrose 伪逆；或
- 把 $I-JJ^\#$ 产生的项建模为新的乘性不确定性。

### 9.2 摩擦项必须前后一致

若关节动力学含 $\Delta f$，附录中的显式代入必须保留 $\Delta f$。否则“精确扰动表达式”不成立。

### 9.3 线性化 $H_\infty$ 等式需限制增益结构

\[
\|G\|_\infty=1/b
\tag{71}
\]

是标量或可同时对角化通道的结论。一般 SPD $K_p,K_d$ 不可直接使用该等式。必须限制到各向同性/可交换增益，或者只给 MIMO 上界。

### 9.4 单一范数需要尺度

必须使用特征长度 $\ell$，例如式 (10)–(11)，否则旋转和平移量纲不一致，$\gamma$ 和谱范数没有明确物理意义。

### 9.5 实机治理器改变闭环方程

加速度治理器或力矩限幅器介入时，实际系统不再严格满足连续时间控制律。应把限幅差定义为

\[
r_{\rm sat}=u_{\rm applied}-u_{\rm law},
\tag{72}
\]

并将其作为扰动输入；或只在未介入区间声明证书。

---

## 10. 期刊层级与理论门槛

### 10.1 只做表达收紧

论文可以尝试：

- 《机器人》；
- 《信息与控制》；
- 《电机与控制学报》。

定位为“几何控制框架和实机实现”，不要包装成性能领先控制器。

### 10.2 加入四元数区域证书和正确的乘性失配分析

可以尝试：

- 《控制理论与应用》；
- 《控制与决策》。

但需要完整证明、清楚限定工作域，并把实验结论改成轨迹级检查。

### 10.3 加入离散采样证书或一个不可被普通负反馈复制的结果

才有机会考虑更高层次的控制/机器人期刊。真正有辨识度的结果应类似：

\[
\text{四元数工作域下的}\quad
(h,\alpha,K_d,\eta_0)\quad
\text{联合可行条件}.
\tag{73}
\]

也就是给定：

- 初始姿态半球余量 $\eta_0$；
- 模型失配上界 $\alpha$；
- 阻尼 $K_d$；
- 采样周期 $h$；

直接计算一个离散闭环仍保持耗散性的充分条件。这种结果比“把负反馈增大”更难由基线直接复制，也更有工程含义。

---

## 11. 推荐的最终研究路线

### 路线 A：最现实

不再新增复杂控制状态，重新组织理论：

1. TODQ 作为二阶运动学接口；
2. 四元数内积势替代部分 $A$ 矩阵反馈解释；
3. 连续时间区域耗散定理；
4. 严格限定加性扰动和局部线性化；
5. 实机只作为可实现性和轨迹条件检查。

目标：中等偏上的机器人/控制期刊。

### 路线 B：最有价值

在路线 A 上增加离散采样证书：

1. 先定义归一化和符号分支；
2. 给出工作域内 $Df$、$D^2V$ 的显式界；
3. 对零阶保持推导 $V_{k+1}-V_k$；
4. 把力矩/加速度限幅作为残差输入；
5. 得到 $h_{\max}$、$\alpha_{\max}$ 和 $K_d$ 的联合条件。

目标：更高质量控制/机器人期刊。

### 路线 C：不建议

保持原有理论不动，加入一个普通负反馈项，再用仿真宣称性能提升。其问题是：

- 基线可以复制；
- 理论结构被破坏；
- 没有同平台实验证明；
- 很难解释为什么 TODQ 是必要的；
- 审稿人会把它归类为调参或增益修改。

---

## 12. 最终判断

这项工作的真正潜力不在于证明“四元数控制误差更小”，因为现有实验证明在相同预算下误差几乎相同。潜力在于证明：

> **四元数的双线性乘法、单位范数恒等式和双覆盖结构，可以把姿态工作域、模型失配、采样周期和耗散性能统一成一个直接可计算的证书。**

若完成这一点，TODQ 的角色也会更清晰：它负责把加速度级几何量以统一的 jet 代数传递给控制器；四元数结构负责给出区别于一般李群坐标的区域几何和实现证书；控制律负责保持耗散性。

这条路线的优点是：

- 数学结构比当前“多层证书叠加”更简洁；
- 没有靠人为增加反馈项制造性能差异；
- 充分发挥四元数的真实优势，但不夸大全局性；
- 即使实验平台不能再复用，也能形成清楚的理论论文；
- 结果具有可迁移性，未来换机械臂仍然适用。

但必须诚实：如果不完成离散证书、乘性失配闭合和尺度归一化，这个方向仍然只是比现有稿件更清楚的理论包装，不能自动跃升到高质量期刊。
