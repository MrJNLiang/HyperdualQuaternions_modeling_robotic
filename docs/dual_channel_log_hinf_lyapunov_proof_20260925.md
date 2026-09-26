# 螺旋对数误差双通道多输入 \(H_\infty\) 证明（Schur 补形式）

## 1. 误差结构

当前和期望单位对偶四元数为

$$
\breve x=r+\varepsilon\frac12pr,\qquad
\breve x_d=r_d+\varepsilon\frac12p_dr_d,
$$

$$
r=\cos\frac\theta2+\mathbf n\sin\frac\theta2,\qquad
r_d=\cos\frac{\theta_d}2+\mathbf n_d\sin\frac{\theta_d}2.
$$

右不变旋转误差为

$$
\widetilde{r}=rr_d^*=\cos\frac{\widetilde{\theta}}2+
\widetilde{\mathbf n}\sin\frac{\widetilde{\theta}}2,
$$

位姿误差为

$$
\widetilde{\breve{x}}=\breve x\breve x_d^*
=\widetilde{r}+\varepsilon\frac12\widetilde{p}\widetilde{r}.
$$

在螺旋对数的局部光滑分支内，写成

$$
2\ln \widetilde{x}=\widetilde{\theta}\widetilde{\mathbf n}+\varepsilon\widetilde{\mathbf h}.
$$

\(\widetilde{\mathbf h}\) 是对偶对数部分，包含旋转轴、转角和位移耦合，不是传统的纯平移误差。定义两个性能输出

$$
z_R=\widetilde{\theta}\widetilde{\mathbf n},\qquad
z_H=\widetilde{\mathbf h}.
$$

## 2. 两类、每类两路扰动

期望轨迹 \(\breve x_d(t)\) 已知且可微，控制器使用确定的

$$
\breve\xi_d=2\dot{\breve x}_d\breve x_d^*.
$$

因此不把期望速度再定义为扰动。只保留与原论文一致的两类输入：

1. 外部 twist 扰动 \(\breve v_w=v_w+\varepsilon v_w'\)；
2. 控制实现和模型误差 \(\breve v_c=v_c+\varepsilon v_c'\)，包括雅可比误差、伪逆残差、关节速度跟踪误差和 dexp 截断误差。

旋转通道的两个输入为 \(v_{wR},v_{cR}\)，螺旋通道的两个输入为 \(v_{wH},v_{cH}\)。这就是单通道多输入结构。

## 3. 控制律

令 \(\alpha_R>0\)、\(\alpha_H>0\) 为李雅普诺夫权重，令

$$
\kappa_R=\frac{1}{2\alpha_R}+\frac{\alpha_R}{2}
\left(\gamma_{Rw}^{-2}+\gamma_{Rc}^{-2}\right),
$$

$$
\kappa_H=\frac{1}{2\alpha_H}+\frac{\alpha_H}{2}
\left(\gamma_{Hw}^{-2}+\gamma_{Hc}^{-2}\right).
$$

速度控制律为

$$
\boxed{
\dot q_c=J^+\left[
\operatorname{vec}_6\left(\operatorname{Ad}_{\widetilde{\breve{x}}}\breve\xi_d\right)
-\operatorname{dexp}_{\ell}
\begin{bmatrix}
\kappa_R\widetilde{\theta}\widetilde{\mathbf n}\\[1mm]
\kappa_H\widetilde{\mathbf h}
\end{bmatrix}
\right],
\qquad
\ell=\operatorname{vec}_6(2\ln \widetilde{x}).
}
$$

这里 \(\operatorname{dexp}_{\ell}\) 只把螺旋对数反馈转换为当前位姿下的 twist 修正。

## 4. 李雅普诺夫函数

取

$$
V_R=\alpha_R\|\widetilde{\theta}\widetilde{\mathbf n}\|^2,\qquad
V_H=\alpha_H\|\widetilde{\mathbf h}\|^2.
$$

由螺旋对数微分关系和控制律，在局部误差坐标中，旋转和螺旋通道的导数分别满足

$$
\dot V_R\leq-2\alpha_R\kappa_R\|z_R\|^2
+2\alpha_Rz_R^Tv_{wR}+2\alpha_Rz_R^Tv_{cR},
$$

$$
\dot V_H\leq-2\alpha_H\kappa_H\|z_H\|^2
+2\alpha_Hz_H^Tv_{wH}+2\alpha_Hz_H^Tv_{cH}.
$$

这里没有把确定的期望速度当成扰动；只有外部输入和实现/模型误差出现在交叉项中。

## 5. Schur 补证明：旋转通道

希望满足

$$
\dot V_R+\|z_R\|^2
-\gamma_{Rw}^2\|v_{wR}\|^2
-\gamma_{Rc}^2\|v_{cR}\|^2\leq0.
$$

把 \(z_R,v_{wR},v_{cR}\) 排成一个向量，上式等价于二次型矩阵

$$
\begin{bmatrix}z_R\\v_{wR}\\v_{cR}\end{bmatrix}^{T}
\begin{bmatrix}
1-2\alpha_R\kappa_R&\alpha_R&\alpha_R\\
\alpha_R&-\gamma_{Rw}^2&0\\
\alpha_R&0&-\gamma_{Rc}^2
\end{bmatrix}
\begin{bmatrix}z_R\\v_{wR}\\v_{cR}\end{bmatrix}\leq0.
$$

右下块为负定矩阵。对它取 Schur 补，矩阵半负定的充要条件为

$$
1-2\alpha_R\kappa_R
+\alpha_R^2\gamma_{Rw}^{-2}
+\alpha_R^2\gamma_{Rc}^{-2}\leq0.
$$

因此

$$
\boxed{
\kappa_R\geq
\frac{1}{2\alpha_R}+\frac{\alpha_R}{2}
\left(\gamma_{Rw}^{-2}+\gamma_{Rc}^{-2}\right).}
$$

这正是控制律中的旋转增益条件，不需要 Young 不等式，也不需要额外稳定裕度。

## 6. Schur 补证明：螺旋通道

完全相同地，性能不等式

$$
\dot V_H+\|z_H\|^2
-\gamma_{Hw}^2\|v_{wH}\|^2
-\gamma_{Hc}^2\|v_{cH}\|^2\leq0
$$

等价于矩阵

$$
\begin{bmatrix}
1-2\alpha_H\kappa_H&\alpha_H&\alpha_H\\
\alpha_H&-\gamma_{Hw}^2&0\\
\alpha_H&0&-\gamma_{Hc}^2
\end{bmatrix}\preceq0.
$$

Schur 补给出

$$
\boxed{
\kappa_H\geq
\frac{1}{2\alpha_H}+\frac{\alpha_H}{2}
\left(\gamma_{Hw}^{-2}+\gamma_{Hc}^{-2}\right).}
$$

因此旋转和螺旋位移两个性能通道可以独立调参。

## 7. 最小反馈增益

令

$$
S_R=\gamma_{Rw}^{-2}+\gamma_{Rc}^{-2},\qquad
S_H=\gamma_{Hw}^{-2}+\gamma_{Hc}^{-2}.
$$

虽然证明中只需使用这两个量，优化仍可直接对原式进行。对

$$
\frac{1}{2\alpha_R}+\frac{\alpha_R}{2}S_R
$$

求最小值，得到

$$
\alpha_R=S_R^{-1/2},\qquad
\kappa_R=\sqrt{S_R}
=\sqrt{\gamma_{Rw}^{-2}+\gamma_{Rc}^{-2}}.
$$

同理

$$
\alpha_H=S_H^{-1/2},\qquad
\kappa_H=\sqrt{S_H}
=\sqrt{\gamma_{Hw}^{-2}+\gamma_{Hc}^{-2}}.
$$

这就是原论文式样的最小瞬时控制增益结果。若所有 gamma 相同，分别有

$$
\kappa_R=\frac{\sqrt2}{\gamma_R},\qquad
\kappa_H=\frac{\sqrt2}{\gamma_H}.
$$

## 8. 主定理

若螺旋对数处于同一光滑局部分支，且控制器使用上述 Schur 补增益，则：

1. 当 \(v_w=v_c=0\) 时，
   
   $$
   \widetilde{\theta}\widetilde{\mathbf n}\to0,\qquad
   \widetilde{\mathbf h}\to0;
   $$

2. 零初始误差下，旋转输出满足

   $$
   \int_0^\infty\|z_R\|^2dt\leq
   \gamma_{Rw}^2\int_0^\infty\|v_{wR}\|^2dt+
   \gamma_{Rc}^2\int_0^\infty\|v_{cR}\|^2dt;
   $$

3. 零初始误差下，螺旋位移输出满足

   $$
   \int_0^\infty\|z_H\|^2dt\leq
   \gamma_{Hw}^2\int_0^\infty\|v_{wH}\|^2dt+
   \gamma_{Hc}^2\int_0^\infty\|v_{cH}\|^2dt.
   $$

证明就是将两个 Schur 补得到的微分不等式积分；不需要在性能结论中添加初始误差项。

## 9. 说明

螺旋对数的第二通道不是传统平移通道，直接采用对偶对数部分 \(\widetilde{\mathbf h}\) 作为性能输出。期望速度在给定轨迹下是确定信号，因此不作为扰动。外部扰动和模型/实现误差必须在仿真中分别注入、分别统计。以上结果是局部对数分支下的速度级 \(H_\infty\) 证明；速度饱和、雅可比不可逆和跨越 \(180^\circ\) 分支时需要另外处理。
