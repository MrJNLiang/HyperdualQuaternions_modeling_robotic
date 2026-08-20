# THEO（控制理论专家）审查报告：§5.3 主定理的数学审查与理论价值分析

> 审查对象：`texdocs/sections/sec5-control.tex`（全文 223 行）、`texdocs/sections/appendix.tex`（C.1–C.8）、
> `sec3-todq.tex`、`sec4-error.tex`、`sec2-preliminaries.tex`、`main_extracted.txt`（§5 行 331–745、附录行 915–1350）。
> 任务：①审查定理 3(a)–(d) 证明链；②【重点】严格核验/撰写"$e_\xi$ 逐点最终界"新增推论；③理论价值清单；
> ④弱项与边界及正面改写；⑤§5.3 文本级修改建议 (a)–(f)；⑥strictification 完成方案要点。
> 结论性质：**证明链全部成立**；发现 1 处需要补的界（$d_{ex}$ 的"ad 泄漏项"，一行可改）、1 处量纲表述错误、
> 1 处注记表述过强（"只能给 RMS 界"）；新增推论**推导成立**但需如实标注其与平凡界的强弱关系。

---

## 0. 结论速览（10 条）

1. **定理 3(a)–(d) 的证明链（C.1–C.8）逐环节成立**：交叉项精确相消是等式性质（只用 $K_p$ 对称）、Schur 补是精确充要（无符号放缩）、(c-2) 的两处混合积恒零正确、$K_{p,T}$ 各向同性的必要性论证正确、(5.7d) 的推导正确。未发现数学错误。
2. **协调员提出的逐点界推导成立**：由 (5.7d) + 位能壳条件可推出 $\dot V \le -\lambda_{eff}V + \lambda_{eff}c + D^2/(2\lambda_{eff})$，比较引理给出 $V(t) \le V(0)e^{-\lambda_{eff}t} + (c + D^2/2\lambda_{eff}^2)(1-e^{-\lambda_{eff}t})$，从而 $\|e_\xi(t)\| \le \sqrt{2V(0)e^{-\lambda_{eff}t} + 2c + D^2/\lambda_{eff}^2}$，极限形式 $\limsup_{t\to\infty}\|e_\xi(t)\|\le\sqrt{2c+D^2/\lambda_{eff}^2}$。**建议作为"推论 3(e)"写入论文**（严格陈述含三条前提 (P1) 工作域/(P2) $\hat d$ 有界/(P3) $\lambda_{eff}>0$，证明草稿见 §2.2，可直接采用）。
3. **但必须如实标注强弱关系**：在强前提（$V\le c$，即 $\Omega_c$）下该界**严格弱于**平凡界 $\|e_\xi\|\le\sqrt{2c}$；但在弱前提（位能壳 $e_z^\top K_pe_z\le2c$，即 (5.7d) 的最小假设）下平凡界不成立、该推论是**唯一可用的逐点界**（与协调员数值验证口径一致；§2.6 THEO 独立复现：三运行零违反、界保守 12.6–13.9×、$\sqrt{2c}$ 为主导项）。其价值：(a) 显式指数暂态（速率 $\lambda_{eff}$）；(b) 把"缺 $-\|e_z\|^2$ 负项"的代价**精确量化**为 $2c$ 伪影（注记 2 限制的严格化）；(c) 先验保证窗口 $T_*$。**RMS 界 (5.7) 仍是稳态常数最锐利的证书**，两者互补而非替代。
4. **前提可弱化**：(5.7d) 及其推论实际只需**位能壳条件** $e_z^\top K_p e_z \le 2c$（比 $V \le c$ 更弱），与 §6.4 事后核验及协调员数值验证的口径一致——推论陈述应使用该弱前提。
5. **发现一处需补的界（"ad 泄漏项"）**：$d_{ex}$ 中含经 $\Delta M$ 泄漏的状态项 $JM^{-1}\Delta MJ^+\mathrm{vec}_6(\mathrm{ad}_{\tilde\xi}\mathrm{Ad}_{\tilde x}\xi_d)$（$u_{ff}$ 的 $\mathrm{ad}$ 部分），它不是纯外生量；在 $\Omega_c$ 上有界 $\le \alpha\sqrt{2c}\,\|\xi_d\|_{L_\infty}$。建议在 (5.7b) 的 $D$ 或 (A4) 中显式并入（一行可改，§1.3-问题6）。
6. **发现一处量纲表述错误**：定理 3(c) 中"$d\to e_\xi$ 的 $L_2$ 能量增益 $\gamma_a\sqrt\kappa$（**无量纲**）"应为"量纲为时间 s（加速度层→速度层的积分增益天然以时间计）"（§1.3-问题7）。
7. **理论价值清单成立**（§3）：相对 Spong92/[P2]/LMI H∞/Ber93 的四项对比，每项都有一行数学证据。
8. **弱项边界转化建议**（§4）：8 条弱项全部可正面改写为卖点，核心叙事为"12 维状态只用 6 维 twist 通道携带全部能量证书 + 可测量的局部性 + 证书条件与适定性分离"。
9. **strictification 完成方案**（§6）：$W = V + \epsilon e_z^\top K_p A e_\xi$ 的 $\dot W$ 上界、$\|\dot A\|_2 \le C_1\|e_\xi\|$ 闭式、$\epsilon$ 可行区间的显式条件均已给出要点；完成后可移除 $2c$ 伪影、给 $e_z$ 严格上界、得全状态局部 ISS。
10. **术语建议**（§5.5/5.6，含与 LIT 对齐）：全文"H∞"改为"$L_2$ 增益证书（BRL/耗散不等式风格）"；标题/摘要/关键词不出现裸 "ISS"（关键词删"输入-状态稳定"），正文统一为"$e_\xi$ 的**实用 ISS 型界** + $e_z$ 的位能壳处理"（§9 对齐清单）。

---

## 1. 定理 3(a)–(d) 证明链逐项审查

### 1.1 总评表

| 环节 | 位置 | 判定 | 关键依据 |
|---|---|---|---|
| 扰动分解 $d=\Theta u_{fb}+d_{ex}$，$\Theta=JM^{-1}\Delta MJ^+$ | §5.1 (5.1d) / C.1 | ✅ 成立 | $J\ddot q=JJ^+(u_{ff}+u_{fb}-\dot J\dot q)+Jw_{dyn}$，$JJ^+=I_6$；$\ddot q_{ref}$ 不依赖 $\ddot q$，显式赋值、无代数环、无 $\alpha/(1-\alpha)$ 因子 |
| 闭环级联标准形 (5.5) | 定理 3(a) | ✅ 成立 | 前馈 $u_{ff}$ 与期望/输运项逐项相消（引理 1 伴随输运公式 A.2） |
| 交叉项精确相消 $\dot V=-e_\xi^\top K_d e_\xi+e_\xi^\top d$ | C.6 第 1 步 / C.7 第 1 步 | ✅ 成立（**等式**，无放缩） | $e_z^\top K_pAe_\xi=(A^\top K_pe_z)^\top e_\xi$ 只用 $K_p$ 对称；与 $d$ 无关 |
| 水平集不变性 + $\tilde\eta$ 保号 + $\det A=-\tfrac18\tilde\eta$ | 定理 3(b)(i)(ii) / C.6 第 2–3 步 | ✅ 成立 | $\|\mathcal O\|^2\le 2c/\lambda_{\min}(K_{p,O})<1$（$c<c^*$）；$\det(aI+[b]_\times)=a(a^2+\|b\|^2)$ |
| $A$ 奇异值精确值 $\{\tfrac12\tilde\eta,\tfrac12,\tfrac12\}$、$\|A_{11}\|_2=\tfrac12$、$\|A_{11}^{-1}\|_2=2/\tilde\eta$ | C.2 | ✅ 成立 | $A_{11}^\top A_{11}=\tfrac14(I-\mathcal O\mathcal O^\top)$，特征值 $\{\tfrac14\tilde\eta^2,\tfrac14,\tfrac14\}$ |
| LaSalle 收敛 (iii) + 局部指数 (iv) | C.6 第 4–5 步 | ✅ 成立 | $E=\{e_\xi=0\}$ 上 $A^\top K_pe_z\equiv0\Rightarrow e_z\equiv0$（$A$ 一致可逆）；线化 $F$ Hurwitz |
| (c-1) Schur 补**充要**判据 (5.6a) | 定理 3(c) / C.7 | ✅ 成立 | 不定号交叉项保留在二次型内整体判定：$M\succeq0\iff K_d-\tfrac1{2\kappa}I-\tfrac1{2\gamma_a^2}I\succeq0$（右下块正定，Schur 等价无放缩） |
| (c-2) 分量拆分与两处恒零 | 定理 3(c) / C.7 第 5 步 / C.3 | ✅ 成立 | $\mathcal T\times\mathcal T=0$、$\mathcal T\cdot(\mathcal T\times\tilde\omega)=0$ 为代数恒等式，全局成立；$K_{p,T}$ 各向同性是**拆分**的必要条件（论证正确） |
| (5.6) 积分界与 $L_2$ 增益 | C.7 第 3–4 步 | ✅ 成立 | $\dot V\le\tfrac{\gamma_a^2}{2}\|d\|^2\Rightarrow$ 全局存在性；积分弃 $V(T)$ |
| (5.7d) $\dot V\le-\lambda_{eff}\|e_\xi\|^2+D\|e_\xi\|$ | C.8 第 1–3 步 | ✅ 成立 | $|e_\xi^\top\Theta K_de_\xi|\le\alpha\lambda_{\max}(K_d)\|e_\xi\|^2$；$\|A\|_2\le 1+\|\mathcal T\|$；Young 回收 |
| RMS 界 (5.7c)/(5.7) 与 $\alpha\to0$ 极限 | C.8 第 4 步 | ✅ 成立 | $\tfrac{\lambda_{eff}}2\int\|e_\xi\|^2\le V(0)+\tfrac{D^2}{2\lambda_{eff}}T$；$\alpha\to0$ 时 $D\to D_{ex}$、$\lambda_{eff}\to\lambda_{\min}(K_d)$ |
| 线性化 (5.8)–(5.9) 与静态刚度标度律 | §5.4 | ✅ 成立 | 消去 $e_\xi$ 得 $\ddot{\mathcal O}+K_\omega\dot{\mathcal O}+\tfrac14K_{p,O}\mathcal O=-\tfrac12 d_\omega$、$\ddot{\mathcal T}+K_v\dot{\mathcal T}+k_{p,T}\mathcal T=d_v$；稳态 $1/k_{p,T}$ 与 $2/\lambda(K_{p,O})$ |

### 1.2 关键机制复核（三处最容易被审稿人攻击的环节）

**(a) 交叉项精确相消的机制（C.6 第 1 步）**：
$\dot V=e_\xi^\top(-K_de_\xi-A^\top K_pe_z)+e_z^\top K_pAe_\xi=-e_\xi^\top K_de_\xi$，
其中 $e_z^\top K_pAe_\xi=(A^\top K_p e_z)^\top e_\xi$。**只需 $K_p$ 对称**（$K_p^\top=K_p$），不需要 $K_p$ 为标量、不需要 $A$ 可逆。含扰时 $e_\xi^\top d$ 线性加入，等式结构不变。这是全文的"零放缩"基石，验证无误。注意 $A^\top$-整形（$A^\top$ 放在 $K_p$ 左侧）是相消成立的唯一位置选择，正文可补一句"若把 $A^\top$ 移到 $K_p$ 右侧则相消破坏"（可选）。

**(b) Schur 补的充要性（C.7 第 2 步）**：性能目标 $\dot V\le-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}{2}\|d\|^2$ 逐时刻等价于 $[e_\xi;d]^\top M[e_\xi;d]\ge0$ 对一切 $(e_\xi,d)\in\mathbb R^{12}$，即 $M\succeq0$；右下块 $\tfrac{\gamma_a^2}{2}I\succ0$ 时 Schur 补 $K_d-\tfrac1{2\kappa}I-\tfrac1{2\gamma_a^2}I\succeq0$ 与 $M\succeq0$ **等价**（非充分型放缩）。这是 (c-1) 敢称"当且仅当"的依据，验证无误。**唯一需补的限定**：该充要性是"对给定的存储函数 $V$ 与供给率类"而言，不声称增益 $\gamma_a\sqrt\kappa$ 在全部 Lyapunov 函数类中最优（见 §1.3-问题4）。

**(c) (c-2) 的两处恒零与各向同性必要性（C.3/C.7 第 5 步）**：
$(A^\top K_pe_z)_\omega=A_{11}^\top K_{p,O}\mathcal O-[\mathcal T]_\times k_{p,T}\mathcal T=A_{11}^\top K_{p,O}\mathcal O$（$\mathcal T\times\mathcal T=0$）；$\dot{\mathcal T}=-[\mathcal T]_\times\tilde\omega+\tilde v$ 中 $k_{p,T}\mathcal T^\top[\mathcal T]_\times\tilde\omega=k_{p,T}\mathcal T\cdot(\mathcal T\times\tilde\omega)=0$。二者均为代数恒等式、不依赖工作域，故 (c-2) 与 (c-1) 同属全局结论。$K_{p,T}\ne k_{p,T}I_3$ 时：$(A^\top K_pe_z)_\omega\ni[\mathcal T]_\times K_{p,T}\mathcal T=\mathcal T\times(K_{p,T}\mathcal T)\ne0$（除非 $\mathcal T$ 为 $K_{p,T}$ 特征向量），且 $\mathcal T^\top K_{p,T}[\mathcal T]_\times\tilde\omega\ne0$——拆分失效，退回 (c-1)。"必要性"指**对拆分结论**的必要性，非稳定性必要性，正文已用"否则退回 (c-1)"交代清楚，建议再加半句"这是拆分的必要条件而非稳定性的必要条件"。

### 1.3 发现的"精确性问题"清单（均非错误，但建议修/澄清，按优先级）

**问题 1（最高优先，需补的界）——$d_{ex}$ 含"ad 泄漏项"，非纯外生**。
$w_{dyn}=M^{-1}(\Delta M\ddot q_{ref}+\cdots)$，$\ddot q_{ref}=J^+(u_{ff}+u_{fb}-\dot J\dot q)$，而 $u_{ff}=\mathrm{vec}_6(\mathrm{Ad}_{\tilde x}\dot\xi_d+\mathrm{ad}_{\tilde\xi}\mathrm{Ad}_{\tilde x}\xi_d)$ 中的 $\mathrm{ad}_{\tilde\xi}$ 项依赖状态 $e_\xi$。故 $d_{ex}$ 含
$$J M^{-1}\Delta M J^+\mathrm{vec}_6(\mathrm{ad}_{\tilde\xi}\mathrm{Ad}_{\tilde x}\xi_d),\qquad \|\mathrm{vec}_6\,\mathrm{ad}_{\tilde\xi}\mathrm{Ad}_{\tilde x}\xi_d\|\le\|\tilde\xi\|\|\xi_d\|\le\sqrt{2c}\,\|\xi_d\|_{L_\infty}\ \text{于 }\Omega_c.$$
因此 (A4) 的 $D_{ex}=\|d_{ex}\|_{L_\infty}<\infty$ 严格说**依赖轨迹在 $\Omega_c$ 内**（其状态相关部分以 $c$ 为上界）。**一行修复**：在 (5.7b) 的 $D$ 中并入该泄漏项，或把 (A4) 改写为"$D_{ex}$ 指 $d_{ex}$ 的外生部分在 $\Omega_c$ 上的 $L_\infty$ 上界与状态泄漏部分上界 $\alpha\sqrt{2c}\|\xi_d\|_{L_\infty}$ 之和"：
$$D \triangleq D_{ex}+\alpha\sqrt{2c}\,\|\xi_d\|_{L_\infty}+\alpha\lambda_{\max}(K_p)\Bigl(1+\sqrt{\tfrac{2c}{\lambda_{\min}(K_{p,T})}}\Bigr)\sqrt{\tfrac{2c}{\lambda_{\min}(K_p)}}.$$
同理，奇异邻域阻尼伪逆残差 $(I-JJ^+)(u_{ff}+u_{fb}-\dot J\dot q)$ 归入 $d_{ex}$ 时也是状态相关的，建议在 (A1) 后注明"数值算例远离奇异邻域（§6），且残差在 $\Omega_c$ 上有界"。审稿人若细读 C.1 必会抓住此项，务必主动补上。

**问题 2（量纲表述错误）——$\gamma_a\sqrt\kappa$ 不是无量纲**。
量纲核算：$V\sim(\mathrm{rad/s})^2$，$\dot V\sim\mathrm{rad^2/s^3}$；$-\tfrac1{2\kappa}\|e_\xi\|^2$ 需 $\mathrm{rad^2/s^3}$ $\Rightarrow \kappa\sim\mathrm{s}$ ✓；$\tfrac{\gamma_a^2}{2}\|d\|^2$，$\|d\|^2\sim\mathrm{rad^2/s^4}$ $\Rightarrow\gamma_a\sim\mathrm{s^{1/2}}$ ✓（正文对 $\kappa,\gamma_a$ 的量纲正确）。但 (5.6) 取根号得 $\|e_\xi\|_{L_2}\le\gamma_a\sqrt\kappa\,\|d\|_{L_2}$：$\|e_\xi\|_{L_2}\sim\mathrm{rad/s^{1/2}}$，$\|d\|_{L_2}\sim\mathrm{rad/s^{3/2}}$，故增益 $\gamma_a\sqrt\kappa\sim\mathrm{s^{1/2}\cdot s^{1/2}}=\mathrm{s}$（**时间量纲**）——加速度层到速度层的积分增益天然以时间计。正文"（无量纲）"应改为"（量纲为时间 s）"。分量拆分 (c-2) 的两条增益同为 s，量纲齐次性表述不受影响（这正是 (c-2) 的意义：不再混合 $(\mathrm{rad/s})^2$ 与 $(\mathrm{m/s})^2$ 的**能量**量纲）。

**问题 3（注记表述过强）——"只能给 RMS 界"不准确**。
注记 2 称"因 $\dot V$ 负项只含 $-\|e_\xi\|^2$…故只能给 RMS 界而非逐点 ISS 界"。严格说：**逐点界存在但含水平集伪影 $2c$**（§2 推论 3(e)），RMS 界是"无伪影的锐利稳态常数"。建议改为："负项只含 $-\|e_\xi\|^2$ 使逐点界仅能以水平集半径的保守常数给出（推论 3(e)），RMS 界给出无水平集伪影的锐利稳态常数；两者互补，去除 $2c$ 伪影需 strictification（附录 C.4）。"

**问题 4（充要性的边界）——(c-1) 的"当且仅当"是对"该 $V$ + 该供给率类"**。
建议在定理 3(c) 后加半句："充要性指：在存储函数 (5.4a) 与供给率 $-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}{2}\|d\|^2$ 的类内，判据 (5.6a) 无任何可去除的保守；它不排除其他存储函数给出更优增益。" 防止审稿人过度解读为"增益本身的最优性"。
**（与 LIT 对齐后的最终措辞，可直接进论文）**："判据 (5.6a) 的'当且仅当'指：对给定的存储函数 (5.4a) 与供给率 $-\tfrac1{2\kappa}\|e_\xi\|^2+\tfrac{\gamma_a^2}{2}\|d\|^2$，耗散不等式成立 $\iff K_d\succeq\tfrac12(\kappa^{-1}+\gamma_a^{-2})I_6$——它排除了对该 $V$ 的一切符号放缩（附录 C.3），但不涉及对存储函数的搜索，故不声称 $\gamma_a\sqrt\kappa$ 是系统 $L_2$ 增益对任意 $V$ 的下界；量纲一致性 $\tfrac12(\kappa^{-1}+\gamma_a^{-2})\sim\mathrm{s^{-1}}$ 与 $K_d$ 相同。"（后者对应 LIT 攻击点 8 的量纲表，建议并入 §5.3 前统一量纲注记。）

**问题 5——(5.1f) 与 Spong92 的 $\alpha$：继承关系 + 范数对象差异（与 LIT 对齐后合成一句）**。
正文"各向同性阻尼时退化为经典计算力矩鲁棒性条件 $\alpha<1$ [Spo92]"宜补注："(5.1f) 是 Spong 关节空间乘性条件（$\|M^{-1}\Delta M\|$ 类 $\alpha<1$，逐点 UUB 球）的**任务空间矩阵化推广**：$K_d=k_dI$ 时形式退化为 $\alpha<1$，且把标量 $\alpha$ 推广为任务空间投影算子 $\Theta=JM^{-1}\Delta MJ^+$ 的谱范数上界、把'逐点 UUB 球'推广为 $\lambda_{eff}$ 有效阻尼折减下的 RMS 稳态标度（定理 3(d)）；两者范数对象不同（任务空间投影 vs 关节空间），仅在 $J$ 正则且投影不放大时直观对应。" 同时可加一句："(5.1f) 等价于 $\lambda_{eff}>0$，即证书非退化条件"（C.1 已隐含，正文明说更好）。

**问题 6——(c)/(d) 两定理对同一轨迹的扰动口径**。
(c) 假设 $d=d_{L_2}\in L_2$（整体），(d) 用 $D_{ex}=\|d_{ex}\|_{L_\infty}$（含 $L_2$ 部分的 sup 范数）。两类型并存时建议补一句："同一轨迹上 $d_{L_2}$ 与 $d_b$ 并存时：能量界 (5.6) 以 $d_{L_2}$ 为输入成立（$d_b$ 经线性交叉项 $e_\xi^\top d_b$ 进入 $\dot V$），RMS 界 (5.7) 以完整 $d$ 为输入成立（$L_2$ 部分经其 $L_\infty$ 范数进入 $D$）；两证书使用同一存储函数、不互相排斥。"

**问题 7（可选）——$\|A_{11}\|_2=\tfrac12$ 的精确性与实现安全性可更早亮相**。
控制律 (5.2) 只用 $A^\top$（永不奇异），$A$ 在 $\tilde\eta=0$ 的奇异**只进证书不进实现**——这是"几何一致反馈"的重要卖点，建议在 3(b) 意义句中明说（见 §5.2）。

**问题 8——定理 3(c) 的全局性值得明说**。
(c-1)/(c-2) 的全部结论不依赖 $\tilde\eta>0$（C.7 已注明），且第三步的全局存在性正是靠 (c-1) 判据全局成立。建议在正文补："（c）无工作域限制：判据、增益与全局存在性均在 $\mathbb R^{12}$ 上成立；$\tilde\eta>0$ 仅用于 (b) 的 LaSalle 与水平集机器。"

---

## 2. 【重点】$e_\xi$ 逐点最终界推论的严格验证与新增推论

### 2.1 协调员推导的核验结论

协调员（`_integration_notes.md` A-2）主张：由 (5.7d) 与 $\Omega_c$ 内 $\|e_\xi\|^2=2V-e_z^\top K_pe_z\ge2V-2c$ 得 $\dot V\le-\lambda_{eff}V+\lambda_{eff}c+D^2/(2\lambda_{eff})$，从而 $V(t)\le V(0)e^{-\lambda_{eff}t}+(c+D^2/2\lambda_{eff}^2)(1-e^{-\lambda_{eff}t})$，即 $\|e_\xi(t)\|\le\sqrt{2V(0)e^{-\lambda_{eff}t}+2c+D^2/\lambda_{eff}^2}$。

**我的核验：推导成立，无漏洞，但有两处需在论文中限定**：
1. **有效性窗口**：中间每一步（(5.7d) 的 $\|e_z\|,\|\mathcal T\|$ 上界、$e_z^\top K_pe_z\le2c$）都要求轨迹停留在位能壳内，故比较引理只能在"轨迹在位能壳内的时段"上应用。正式陈述须带该前提（与定理 3(d) 的前提同构）。
2. **前提的最弱形式是位能壳而非全水平集**：链上实际用到的只是 $e_z^\top K_pe_z\le2c$（含 $\tfrac12\mathcal T^\top K_{p,T}\mathcal T\le c$、$\tfrac12\mathcal O^\top K_{p,O}\mathcal O\le c$），不必要求 $V\le c$ 全时成立。$V\le c\Rightarrow$ 位能壳，故论文的 $\Omega_c$ 前提蕴含之；反之不必然。协调员数值验证取 $c=\tfrac12\max_t(e_z^\top K_pe_z)$ 正是该弱前提的实例——与 §2.2 的陈述一致。

### 2.2 新增推论（可直接进论文，建议标号"推论 3(e)"或"推论 4"）

> **推论（$e_\xi$ 的逐点实用最终界；定理 3(d) 的升级）**。设 (A1)–(A4) 与控制律 (5.2) 成立。前提（三条，缺一不可）：
> **(P1) 工作域**：轨迹在考察时段 $[0,T_*)$ 内满足**位能壳条件** $e_z(t)^\top K_p e_z(t)\le 2c$（由 $\Omega_c$ 蕴含，故定理 3(d) 的前提充分；蕴含 $\tilde\eta\ge\eta_0>0$ 与 $A$ 一致可逆，见 C.2）；
> **(P2) 扰动有界**：重建扰动 $\hat d\triangleq\dot e_\xi+K_de_\xi+A^\top(\tilde x)K_pe_z$ 在时段上有界，$D\triangleq\sup_{[0,T_*)}\|\hat d(t)\|<\infty$（由 (A4) 的 $D_{ex}$ 与 Ω_c 上 ad 泄漏项界 $\alpha\sqrt{2c}\|\xi_d\|_{L_\infty}$ 保证，见 §1.3-问题1）；
> **(P3) 证书非退化**：$\lambda_{eff}>0$（等价于 (5.1f)）。
> 则对一切 $t\in[0,T_*)$：
> $$\boxed{\;\|e_\xi(t)\|^2\ \le\ 2V(0)\,e^{-\lambda_{eff}t}+\Bigl(2c+\tfrac{D^2}{\lambda_{eff}^2}\Bigr)\bigl(1-e^{-\lambda_{eff}t}\bigr)\ \le\ 2V(0)e^{-\lambda_{eff}t}+2c+\tfrac{D^2}{\lambda_{eff}^2}\;,}$$
> 且 $e_\xi$ 以指数速率 $\lambda_{eff}/2$（范数意义）进入最终球，极限形式为
> $$\boxed{\;\limsup_{t\to\infty}\|e_\xi(t)\|\ \le\ \sqrt{2c+D^2/\lambda_{eff}^2}\ \le\ \sqrt{2c}+D/\lambda_{eff}\;.}$$

**证明草稿**：(5.7d) 在位能壳上成立（附录 C.8 第三步；其 $\|e_z\|\le\sqrt{2c/\lambda_{\min}(K_p)}$、$\|\mathcal T\|\le\sqrt{2c/\lambda_{\min}(K_{p,T})}$ 恰由位能壳给出）。由 $e_z^\top K_pe_z\le2c$：
$$\|e_\xi\|^2=2V-e_z^\top K_pe_z\ge2V-2c\ \Longrightarrow\ -\tfrac{\lambda_{eff}}2\|e_\xi\|^2=-\lambda_{eff}V+\tfrac{\lambda_{eff}}2 e_z^\top K_pe_z\le-\lambda_{eff}V+\lambda_{eff}c .$$
Young 不等式 $D\|e_\xi\|\le\tfrac{\lambda_{eff}}2\|e_\xi\|^2+\tfrac{D^2}{2\lambda_{eff}}$ 代入 (5.7d)：
$$\dot V\le-\lambda_{eff}V+\lambda_{eff}c+\tfrac{D^2}{2\lambda_{eff}},$$
比较引理（Gronwall）于 $[0,t]$：$V(t)\le V(0)e^{-\lambda_{eff}t}+\bigl(c+\tfrac{D^2}{2\lambda_{eff}^2}\bigr)(1-e^{-\lambda_{eff}t})$；由 $\|e_\xi\|^2\le2V$ 得结论。$\blacksquare$

**证明的严谨性注记（须随推论写入正文）**：
(i) 该前提与定理 3(d) 同构、无新增假设；(ii) 若轨迹在某时刻 $T_*$ 离开位能壳，界对 $t<T_*$ 仍成立；(iii) 若 $D=0$（无扰），推论给出 $\|e_\xi\|^2\le2V(0)e^{-\lambda_{eff}t}+2c(1-e^{-\lambda_{eff}t})$，与定理 3(b) 的 $\dot V=-e_\xi^\top K_de_\xi\le0$ 及水平集不变性一致；(iv) **先验保证窗口**（推论的第 2 个新内容）：当 $D>0$、$V(0)<c$ 时，比较界 $\bar V(t)\le c$ 的时段即轨迹可证留在 $\Omega_c$（从而界先验成立）的时段：
$$T_*^{est}=\frac1{\lambda_{eff}}\ln\Bigl(1+\frac{2\lambda_{eff}^2\,(c-V(0))}{D^2}\Bigr),\qquad D=0\Rightarrow T_*^{est}=\infty,$$
即"保证窗口—界常数"折衷：$c\uparrow$ 窗口 $\uparrow$ 但常数 $\sqrt{2c+D^2/\lambda_{eff}^2}\uparrow$；$c\downarrow V(0)$ 窗口 $\to0$。这是可写进 §5.3 注记或 §6.4 的工程语句。

### 2.3 强弱关系分析（必须如实写，防止审稿人反打）

强弱关系**取决于采用哪种前提**，必须分别如实陈述：

- **前提 A（强前提 $V(t)\le c$，即论文的 $\Omega_c$）**：平凡界 $\|e_\xi(t)\|^2\le2V(t)\le2c$ 已直接成立，推论 3(e) 的稳态常数 $\sqrt{2c+D^2/\lambda_{eff}^2}$ **严格弱于平凡界**——在此前提下推论不提供比水平集本身更多的稳态信息，其价值仅在暂态结构。
- **前提 B（弱前提，位能壳 $e_z^\top K_pe_z\le2c$，即 (5.7d) 的**最小**假设）**：此时 $V$ 不被 $c$ 界定、**平凡界 $\sqrt{2c}$ 不成立**，推论 3(e) 是**唯一可用的逐点界**——非空洞、且携带指数暂态。协调员的数值验证（A-2）恰在此口径下零违反，且 $\|e_\xi\|$ 峰值（9.5e-2/1.9e-1/2.6e-1）低于界，说明在弱前提下推论具有真实信息量。

因此推论的正确"卖点"表述是：**在 (5.7d) 的最小前提（位能壳）下，它是唯一的逐点实用界；在更强的 $\Omega_c$ 前提下它退化为暂态细化。** 其余结论：

- **推论 3(e) 的三项真实价值**：(a) 显式指数暂态（速率 $\lambda_{eff}$）与先验保证窗口 $T_*^{est}$；(b) 把"$\dot V$ 缺 $-\|e_z\|^2$ 项的代价"精确量化为 $2c$ 伪影——注记 2 限制的**严格化**，也是 strictification 收益的量化（去掉的正是 $2c$）；(c) 与工程直觉一致的"实用 ISS 形状"。
- **RMS 界 (5.7) 保留价值**：稳态常数 $D/\lambda_{eff}$ 无水平集伪影、可直接对照仿真 RMS 指标（§6.3 的 $rms\_bound\_5\_7$ 列）、极限形式简洁、$\alpha\to0$ 极限封闭。**推论与 RMS 互补而非替代**：RMS 给锐利稳态、推论给暂态/窗口/弱前提下的逐点保证。
- 量级对比：$\sqrt{2c+(D/\lambda_{eff})^2}-D/\lambda_{eff}\approx\lambda_{eff}c/D$（$D\gg\lambda_{eff}\sqrt{2c}$ 时），即 $2c$ 项在 $D$ 小时主导、在 $D$ 大时退化为加法修正 $\sqrt{2c}$。正文应注明："逐点界的 $2c$ 项反映位能壳几何（弱前提下的必要保守），RMS 界提供更紧的稳态常数"。

### 2.4 马尔可夫密度升级（免费附加，一行注记）

由 (5.7c) $\tfrac1T\int_0^T\|e_\xi\|^2dt\le\tfrac{D^2}{\lambda_{eff}^2}+\tfrac{2V(0)}{\lambda_{eff}T}$ 与切比雪夫不等式：对任意 $\delta\in(0,1)$、
$$\frac1T\,\mathrm{meas}\Bigl\{t\in[0,T]:\ \|e_\xi(t)\|^2>\frac{D^2}{\lambda_{eff}^2}+\frac{2V(0)}{\lambda_{eff}T\delta}\Bigr\}\ \le\ \delta,$$
即 **RMS 界蕴含"除任意小时间密度集外，$\|e_\xi\|$ 逐点 $\le D/\lambda_{eff}$（+ 可忽略项）"**——把 RMS 界局部化为占用时间意义上的逐点结论，零额外假设。建议作为注记半行加入 §5.3（或推论后），是审稿人问"RMS 到底意味着什么"时的标准答案。

### 2.5 注记 2 改写建议（最终措辞）

> **注记（RMS 与逐点界的分工）**。因 $\dot V$ 的负项只含 $-\|e_\xi\|^2$（扰动到 $e_z$ 的相对阶为 2），$e_\xi$ 的逐点最终界仅能以水平集半径的保守常数给出（推论 3(e)）：稳态球 $\sqrt{2c+D^2/\lambda_{eff}^2}$ 含 $2c$ 伪影，该伪影正是"缺 $-\|e_z\|^2$ 项"的精确代价；RMS 界 (5.7) 提供无伪影的锐利稳态常数 $D/\lambda_{eff}$，可直接对照仿真 RMS 指标，并由切比雪夫不等式升级为"除任意小时间密度集外的逐点界"。去除 $2c$ 伪影需 strictification（附录 C.4）。$\Omega_c$/位能壳前提需事后数值核验（§6.4：实测余度约 2.5 个数量级）。$\blacksquare$

### 2.6 数值交叉印证（协调员复核，THEO 独立复现）

数据：`TNDQ_sim/results/tndq_internal_line_bias.npz`、`tndq_internal_line_l2.npz`、`tndq_internal_setpoint_mismatch.npz`（含 $t,e_z,e_\xi,V,K_d,k_p,dt$；用 `/home/ljn/miniconda3/bin/python` 读，numpy 可用）。方法（与协调员 A-2 相同口径，THEO 独立重写）：按定理 2 从 $e_z=[\mathcal O;\mathcal T]$ 重构 $A(\tilde x)$（$\tilde\eta$ 取正分支 $\tilde\eta=\sqrt{1-\|\mathcal O\|^2}$），中心差分求 $\dot e_\xi,\dot e_z,\dot V$，重建 $\hat d=\dot e_\xi+K_de_\xi+A^\top K_pe_z$，取 $\lambda_{eff}=\lambda_{\min}(K_d)=8$（内部对象 $\alpha=0$）、$D=\sup_t\|\hat d(t)\|_2$、$c=\tfrac12\max_t e_z^\top K_pe_z$（位能壳口径）。结果：

| 检查项 | line_bias | line_l2 | setpoint_mismatch |
|---|---|---|---|
| $\tilde\eta_{\min}$（>0，工作域有效） | 0.9951 | 0.9951 | 0.9671 |
| 耗散等式残差 $\max|\dot V-(-e_\xi^\top K_de_\xi+e_\xi^\top\hat d)|$ | 3.95e-2（集中于 $t=0$ 端点） | 1.08e-1（集中于 $t=1.0$ 注入时刻） | 3.78e-1（集中于 $t=0$ 端点） |
| 逐点界 $V(t)\le V(0)e^{-\lambda t}+(c+D^2/2\lambda^2)$（松形式）：最大违反 | **0**（−6.4e-1） | **0**（−3.1e0） | **0**（−5.9e0） |
| 逐点界 $\|e_\xi\|^2\le2\cdot$（紧形式，含 $(1-e^{-\lambda t})$）：最大违反 | **0**（−1.7e-1） | **0**（−1.7e-1） | **0**（−4.4e-30） |
| 极限常数 $\sqrt{2c+D^2/\lambda_{eff}^2}$ vs 实测 $\|e_\xi\|$ 峰值 | 1.20 vs 0.095（12.6×） | 2.52 vs 0.190（13.3×） | 3.60 vs 0.258（13.9×） |
| RMS 界：$\tfrac1T\int\|e_\xi\|^2$ vs $D^2/\lambda^2+2V(0)/(\lambda T)$ | 2.6e-3 ≤ 1.28 ✓ | 5.8e-3 ≤ 6.16 ✓ | 1.0e-2 ≤ 11.8 ✓ |

**结论（如实陈述）**：
1. **耗散等式残差为差分噪声、非证书违反**：残差集中在 $t=0$ 端点与 $t=1.0$ 扰动注入时刻（有限差分端点/瞬态误差）；且该等式对 $\hat d$ 的构造而言是**代数恒等**（$\hat d$ 即按 (5.5) 反解出的扰动，$e_\xi^\top\dot e_\xi+e_z^\top K_p\dot e_z\equiv-e_\xi^\top K_de_\xi+e_\xi^\top\hat d$ 由定义恒成立）；数据内部一致性残差（$\max|\dot V_{\mathrm{FD}}-(e_\xi^\top\dot e_\xi+e_z^\top K_p\dot e_z)|$）为 3.95e-2 / 1.08e-1 / 3.78e-1，量级与协调员 A-2 的 2.8e-3~3.8e-2 一致（FD 方案细节差异）。
2. **逐点界（推论 3(e)）在三运行上零违反、恒成立**（紧/松两形式均零违反）；RMS 界同时成立。这为"定理 3(d) 升级为 $e_\xi$ 逐点实用 ISS 界 + RMS 界双结论"提供现成数据背书，**无需新仿真**。
3. **界约保守 12.6–13.9×**：极限常数 $\sqrt{2c+D^2/\lambda_{eff}^2}$ 高于实测峰值约一个数量级，其中 $\sqrt{2c}$（位能壳半径 0.413/0.413/1.071）为主导项——与 §2.3 强弱分析一致（$2c$ 伪影占优），也与协调员 C-2 的保守性归因（$\|A\|_2\le1+\|\mathcal T\|$ 最坏方向、$D$ 取 $\hat d$ 峰值、Young 等号条件远未达到）吻合。故"非平凡/非空洞"应表述为"**界恒成立、有限、可先验计算**"，而非"紧"；RMS 界与实测余量 8–59×（`_integration_notes.md` B-2）同理。
4. 可复现性：以上全部对既有 npz 只读计算（numpy，miniconda3），无新仿真。

---

## 3. 其他理论无法提供、而本文提供的分析价值清单

（每条附一行数学证据；供 §1.2 贡献改写与 §5.3 文本采用）

1. **精确耗散等式（等式而非不等式）**：$\dot V=-e_\xi^\top K_de_\xi+e_\xi^\top d$ 逐时刻成立、无任何放缩（C.6/C.7 第一步）；由此 (c-1) 的 Schur 判据是**当且仅当**。Spong92 类的鲁棒项估计、Ber93 的无源性不等式均为充分型放缩，无法给出充要判据。
2. **扰动分解的两个独立通道（乘性/外生）**：$d=\Theta u_{fb}+d_{ex}$ 中乘性项被精确分成"阻尼折减 $\lambda_{eff}=\lambda_{\min}(K_d)-\alpha\lambda_{\max}(K_d)$"与"等效扰动 $D\ni\alpha\lambda_{\max}(K_p)(1+\|\mathcal T\|)\|e_z\|$"两半，且 $\alpha\to0$ 极限 $\|d_{ex}\|_{L_\infty}/\lambda_{\min}(K_d)$ 是**显式稳态标度律**。Spong92 只给 $\alpha<1$ 下的存在性 UUB 界（含 Lyapunov 函数相关常数），无闭式稳态标度。
3. **加速度层扰动的显式入口（无代数环）**：$d=Jw_{dyn}+\dot v_w+\dot v_c$，$w_{dyn}=M^{-1}(\Delta M\ddot q_{ref}+\cdots)$ 为显式赋值（$\ddot q_{ref}$ 不依赖 $\ddot q$），无 Neumann 级数、无 $\alpha/(1-\alpha)$ 型等效扰动因子（C.1）。[P2] 的速度层 $L_2$ 扰动经 $\dot v_w$ 到加速度层后**不保 $L_2$**（$L_2$ 不保证导数 $L_2$），本文把 $L_2$ 假设直接放在加速度层，并把 $L_\infty$ 部分单列处理。
4. **偏差型（$L_\infty$）不确定性有定量证书**：$d_b\in L_\infty$ 的 RMS 界 $D/\lambda_{eff}$（(5.7)）；[P2] 的 $L_2$ 假设对偏差型失效（$\int_0^\infty\|d_b\|^2dt=\infty$），其框架对该类扰动无结论。
5. **量纲齐次的分量证书**：(c-2) 依赖两条叉积恒零（$\mathcal T\times\mathcal T=0$、$\mathcal T\cdot(\mathcal T\times\tilde\omega)=0$），旋转/平移各自独立满足 $L_2$ 界，(5.6b) 中 $(\kappa_\omega,\gamma_\omega)$ 与 $(\kappa_v,\gamma_v)$ 可独立指定、不再混合 $(\mathrm{rad/s})^2$ 与 $(\mathrm{m/s})^2$ 能量量纲。标准 LMI H∞ 对 12 维混合量纲状态（rad、m、rad/s、m/s）求解 $P\succeq0$，无此逐分量结构且数值条件数敏感。
6. **纯分析参数与控制器解耦**：$\kappa,\gamma_a$ 只出现在证书 (5.6a) 中、不出现在控制律 (5.2) 中（§5.3 已声明，且有 gamma_sweep 组 A 数据背书：扫 $\gamma_a$ 不改变误差轨迹）；标准 LMI H∞ 的决策变量（Lyapunov 矩阵）与控制器耦合，参数意义不透明。
7. **实现安全性：只用 $A^\top$、从不求逆 $A$**：控制律 (5.2) 在 $\tilde\eta=0$ 处仍然良定义（$A$ 的奇异只破坏 LaSalle/水平集证书，不破坏实现）；Ch20 类律的螺旋对数位姿反馈在大误差处导数奇异。$\sigma_{\min}(A)\ge[2(1+\|\mathcal T\|)/\tilde\eta+1]^{-1}>0$ 只被证书使用。
8. **12 维状态、6 维证书通道**：全部耗散等式/增益/RMS 证书只约束 $e_\xi$（6 维），$e_z$ 只以势能项与位能壳几何进入证书常数——"用一半状态携带全部性能证书"，另一半（位姿）的稳态由 (5.9) 准静态预测、严格界留给 strictification（C.4）。这是结构性取舍的定量表述，Spong92/Ber93/LMI 均无此分层。
9. **可测量的局部性**：水平集条件 $c<c^*=\tfrac12\lambda_{\min}(K_{p,O})$ 显式且可数值核验（§6.4：$c^*=160$ vs $V_{peak}\le0.494$，余度约 2.5 个数量级），把"局部稳定"变成"可核验的工作域"，而非无穷小邻域。
10. **证书条件与适定性分离**：(5.1f) 等价于 $\lambda_{eff}>0$（证书非退化），与微分方程适定性无关（C.1：即使 $\alpha$ 违反 (5.1f)，闭环仍良定义，只是证书失效）——这使 (5.1f) 的违反**不**意味着不稳定，可正面表述为"证书的可检验边界"。

---

## 4. 定理 3 的弱项与边界 + 主动正面改写建议

### 4.1 弱项清单（如实列出）

| # | 弱项 | 事实陈述 |
|---|---|---|
| W1 | 只约束 $e_\xi$ | (5.6)/(5.7) 均不含 $e_z$ 项；$e_z$ 无严格上界，(5.9) 只是准静态近似 |
| W2 | 无全状态 ISS | 12 维状态，证书只给 twist 通道；"不声称全状态 ISS"（边界说明 ii） |
| W3 | RMS 非逐点 | 注记 2 的原始表述 |
| W4 | $L_2$ 扰动不保证收敛到零 | (5.6) 只给能量增益；$d\in L_2$ 仅得 $\kappa^{-1}\|e_\xi\|^2\in L_1$ |
| W5 | $\Omega_c$ 前提需事后核验 | 定理 3(d) 前提（§6.4 已核验，余度 2.5 个数量级） |
| W6 | strictification 未完成 | C.4 自认；$e_z$ 严格界缺失 |
| W7 | 小增益条件只是证书条件 | 非适定性条件（C.1 已说明） |
| W8 | (c-2) 依赖 $K_d$ 块对角 + $K_{p,T}$ 各向同性 | 否则退回 (c-1)（C.3） |
| W9 | 局部性（unwinding、$\tilde\eta=0$ 奇异） | 拓扑障碍，正文已声明 |

### 4.2 主动正面改写建议（把边界转化为卖点）

**改 W1+W2 → "证书分层"**（放 3(d) 后或边界说明）：
> 本文的性能证书（耗散等式、$L_2$ 增益、RMS 稳态标度）全部只依赖 6 维 twist 误差通道；位姿通道以势能项与位能壳几何进入证书常数，其稳态量级由 (5.9) 给出可实验证伪的准静态预测、严格逐点上界留给 strictification（附录 C.4）。12 维闭环状态仅用一半维度即获得全部能量证书——这是本文"证书分层"设计的有意取舍：**每一张证书都可独立核验、独立对照数据**，而非整机式不可证伪声明。

**改 W3 → "RMS 与逐点分工"**：见 §2.5 注记改写（RMS 锐利稳态 + 推论 3(e) 暂态/窗口 + 切比雪夫密度升级）。

**改 W4 → "双通道分离是特性"**：
> 把扰动按时间特性拆为 $d_{L_2}+d_b$ 并分通道发证，是本文的结构性选择：$L_2$ 通道给出能量增益（对照仿真能量指标），$L_\infty$ 通道给出稳态标度（对照静态刚度律）；两者在同一存储函数上同时成立、不互相排斥，且 $d_{L_2}\in L_2$ 不保证收敛到零这一点被如实声明——增益型保证与收敛型保证的边界清晰，避免过度声称。

**改 W5 → "可测量的局部性"**：
> 全部稳定性结论是工作域局部的，但工作域由显式水平集条件 (5.5b) 界定、可数值核验（§6.4 余度约 2.5 个数量级），并可经推论 3(e) 的先验窗口 $T_*^{est}$ 在运行前给出保证时段——这不是"无穷小邻域"意义上的局部，而是"可测量的局部"。

**改 W6 → "开放问题即未来工作"**：§7 局限 (i) 保留，但补一句收益清单（见 §6）：strictification 一旦完成将同时给出 $e_z$ 严格上界、无伪影逐点界、全状态局部 ISS 三件事，且方案要点已给出（附录 C.4 升级版）。

**改 W7 → "证书条件与适定性的分离是优点"**：
> (5.1f) 不是适定性条件：即使 $\alpha$ 违反 (5.1f)，闭环仍良定义（C.1），只是本 Lyapunov 证书失效；等价地，(5.1f) 是"证书非退化条件"（$\lambda_{eff}>0$ 的充要条件）。因此 (5.1f) 的边界即**证书的可检验边界**，而非系统不稳定的信号。

**改 W8 → "证书族的闭性"**：
> (c-2) 的块对角/各向同性要求是**拆分结论**的必要条件而非稳定性必要条件；不满足时自动退回 (c-1)——证书体系对增益结构是**闭**的（族内任意退化都有对应证书），工程上可按"先 (c-1) 验证、再 (c-2) 细化"的两步走使用。

**改 W9 → 拓扑的正面表述**：保持现有声明，可加一句"unwinding 与 $\tilde\eta=0$ 奇异是 $SO(3)$ 双覆盖的固有拓扑障碍，任何全局 $SE(3)$ 指数稳定控制器都无法回避（与 [P2] Remark 1 一致），本文以显式水平集条件给出其可核验的回避方式"。

---

## 5. §5.3 文本级修改建议

### 5.1 (a) 开篇段落改写（三缺口闭合叙事，建议替换现 §5.3 首段）

> 定理 3 的四个结论在引言所述三个结构性缺口的逐一闭合中扮演确定角色，且全部闭合发生在同一存储函数 (5.4a) 上、以无放缩的等式性质完成：
> **（缺口 1：位姿/速度误差分离）** 定理 3(a) 的级联标准形 $\dot e_z=A(\tilde x)e_\xi$、$\dot e_\xi=-K_de_\xi-A^\top(\tilde x)K_pe_z+d$ 使两阶误差各有独立方程与独立角色——$e_z$ 只作为势能进入 $V$，全部耗散、增益与稳态证书由 $e_\xi$ 通道携带；$A^\top$-整形位姿反馈使 Lyapunov 导数中的交叉项**逐项精确相消**（等式，非放缩），无需螺旋对数类在大误差处奇异的位姿反馈。
> **（缺口 2：加速度层扰动无入口）** 扰动 $d=Jw_{dyn}+\dot v_w+\dot v_c$ 显式进入 $\dot e_\xi$ 方程，且 (5.1d) 的分解 $d=\Theta u_{fb}+d_{ex}$ 是精确等式（附录 C.1，无隐式代数环）；定理 3(c) 对 $d_{L_2}$ 给出**充要**的 Schur 判据与旋转/平移分量拆分，(c-1)/(c-2) 全部结论不依赖工作域。
> **（缺口 3：偏差型不确定性不满足 $L_2$）** 定理 3(d) 对 $d_{ex}\in L_\infty$（含乘性 $\Theta u_{fb}$）给出 twist 误差的 RMS 稳态标度 $D/\lambda_{eff}$ 及 $\alpha\to0$ 极限，推论 3(e) 进一步给出带显式指数暂态的逐点实用界（含先验保证窗口）。三个缺口在同一 $V$ 上闭合，闭合机制彼此正交（相消、判据、标度），这正是混合证书体系的效率来源。

**（与 LIT 对齐：开篇/定位段须先声明相对 [P2] 的差异，防"Schur 补已有先例"之嫌）**：在以上三缺口叙事之前（或之后一段）插入相对 [P2] 的定位句，五点差异直接采用 LIT 报告 §7 第一段的表述：①层——[P2] 速度层（扰动进位姿微分方程）vs 本文加速度层（扰动进 $\dot e_\xi$ 方程，相对阶 2）；②输出——[P2] 位姿误差 $O(\tilde z),\mathcal T(\tilde z)$ vs 本文几何一致 twist 误差 $e_\xi$；③判据形态——[P2] 标量增益条件（$\kappa_O,\kappa_T$ 即控制律增益、$\gamma$ 与 $\kappa$ 绑定）vs 本文矩阵当且仅当 $K_d\succeq\tfrac12(\kappa^{-1}+\gamma_a^{-2})I_6$；④证书参数位置——[P2] 进控制律 vs 本文 $\kappa,\gamma_a$ 纯分析（事后认证不改控制律）；⑤量纲——[P2] 位姿输出混 $(\mathrm{rad})^2$ 与 $\mathrm{m}^2$ vs 本文 (c-2) 逐分量量纲齐次。其余四段（Ch20/Wang–Yu、Spong92、LMI/van der Schaft、Pham 2025）属 §1.1/§4 定位，按 LIT §7 原文采用。

### 5.2 (b) 每个定理的"意义句"改写

- **定理 3(a)**（在证明后加一句）：
  > 级联标准形 (5.5) 是精确等式而非近似；$A^\top$-整形反馈使交叉项相消只需 $K_p$ 对称，且控制律只用 $A^\top$（永不奇异）——$A$ 的奇异只进证书不进实现。
- **定理 3(b)**（把 (i)-(iv) 后的现有说明浓缩）：
  > 无扰结论是"工作域内全局渐近 + 原点局部指数"的双层结构：水平集 (5.5b) 显式、可核验（§6.4 余度 2.5 个数量级）、$A$ 的一致可逆性有闭式下界（C.2 给出奇异值 $\{\tilde\eta,1,1\}$ 的精确值）。
- **定理 3(c)**：
  > 充要性：判据 (5.6a) 在给定存储函数与供给率的类内无任何可去除保守（不定号交叉项保留在二次型内整体判定）；(c-2) 把 6 维判据拆为两条 3 维判据，逐分量量纲齐次、参数独立可调；$\kappa,\gamma_a$ 为纯分析参数，不进入控制律——证书与控制器解耦，扫参不改变闭环（§6 组 A 数据）。
- **定理 3(d)**：
  > 乘性不确定性被精确拆成两半（以 $\alpha\lambda_{\max}(K_d)$ 折减有效阻尼、以 $\alpha\lambda_{\max}(K_p)\|e_z\|$ 抬高等效扰动），给出可对照仿真 RMS 的显式稳态标度 $D/\lambda_{eff}$ 及其 $\alpha\to0$ 极限 $\|d_{ex}\|_{L_\infty}/\lambda_{\min}(K_d)$；推论 3(e) 补充指数暂态与保证窗口。

### 5.3 (c) 证书对比表（建议放 §5.3 末，Markdown 版 + LaTeX 草稿）

| 方法 | 扰动模型 | 证书类型 | 充要/充分 | 稳态/增益常数形式 | 几何/分量结构 | 相对本文缺口 |
|---|---|---|---|---|---|---|
| 本文 3(b) | $d\equiv0$ | 水平集不变 + 渐近收敛 + 局部指数 | — | $\Omega_c$ 紧、$\tilde\eta\ge\eta_0$、$\sigma_{\min}(A)$ 闭式下界 | 级联标准形、几何一致 $e_\xi$ | — |
| 本文 3(c) | $d_{L_2}\in L_2$（加速度层） | $L_2$ 增益（BRL 风格） | **充要**（Schur） | 增益 $\gamma_a\sqrt\kappa$（量纲 s）；判据 $K_d\succeq\tfrac12(\kappa^{-1}+\gamma_a^{-2})I$ | 旋转/平移分量拆分、量纲齐次、$\kappa,\gamma_a$ 纯分析 | — |
| 本文 3(d) | $\Theta u_{fb}+d_{ex}$，$d_{ex}\in L_\infty$ | RMS 稳态标度 + 逐点实用界（推论 3(e)） | 充分（证书条件 $\lambda_{eff}>0$ 等价于 (5.1f)） | $D/\lambda_{eff}$，$\alpha\to0$ 极限 $\|d_{ex}\|_\infty/\lambda_{\min}(K_d)$ | 乘性/外生两通道分离 | — |
| Spong92 | $\|M^{-1}\Delta M\|<\alpha<1$ 类 | 鲁棒计算力矩 UUB | 充分 | 存在性界（含 Lyapunov 常数，无闭式稳态） | 关节空间、无几何误差坐标 | 无显式稳态标度律；无加速度层入口分解；偏差型仅 UUB |
| [P2] | 速度层加性 $L_2$ 扰动 | 运动学层 $H_\infty$（$\gamma$ 为综合参数定增益） | 充分 | 增益含 $\gamma$ 与 $\kappa_O=\sqrt2/\gamma$ 类参数 | DQ 参数化、无分量拆分 | 扰动在速度层（加速度层无入口）；无 $L_\infty$ 偏差型；$\gamma$ 进控制器 |
| 标准 LMI H∞ | 线性化/LPV 模型 | BRL/LMI 可行性 | 充分（LMI 无充要闭式） | $P$ 矩阵决策变量，无解析稳态界 | 全状态（12 维混合量纲）、无分量结构 | 需线性化/LPV 调度；量纲混杂；决策变量与控制器耦合 |
| 无源性 Ber93 | $L_2$/有界扰动（速度层） | 无源性稳定性/ISS | 充分 | 无显式增益证书 | 无 | 无增益证书；无显式 $D/\lambda_{eff}$ 类标度 |

LaTeX 草稿骨架（tabularx，列宽可调）：
```
\begin{table}[t]\centering\small
\caption{证书体系对比：本文定理 3 vs 既有方法}\label{tab:cert-compare}
\begin{tabularx}{\linewidth}{@{}llllX@{}}
\toprule
方法 & 扰动模型 & 证书类型 & 充要性 & 稳态/增益常数形式 \\
\midrule
本文 3(b) & $d\equiv0$ & 水平集+渐近+局部指数 & --- & $\sigma_{\min}(A)\ge[2(1+\|\mathcal T\|)/\tilde\eta+1]^{-1}$ \\
本文 3(c) & $d_{L_2}\in L_2$ & $L_2$ 增益 & 充要(Schur) & $\gamma_a\sqrt\kappa$，$K_d\succeq\tfrac12(\kappa^{-1}+\gamma_a^{-2})I$ \\
本文 3(d) & $\Theta u_{fb}+d_{ex}$ & RMS+逐点实用界 & 证书条件 & $D/\lambda_{eff}\xrightarrow[\alpha\to0]{}\|d_{ex}\|_\infty/\lambda_{\min}(K_d)$ \\
Spong92 & 乘性 $<\!\alpha$ & UUB & 充分 & 存在性界（无闭式） \\
[P2] & 速度层 $L_2$ & 运动学 $H_\infty$ & 充分 & $\gamma$ 进控制器 \\
LMI H∞ & 线性化/LPV & LMI 可行性 & 充分 & $P$ 决策变量 \\
Ber93 & 速度层 $L_2$ & 无源性 ISS & 充分 & 无显式增益 \\
\bottomrule
\end{tabularx}
\end{table}
```

### 5.4 (d) 反演小节：面向工程读者的"增益整定公式"（建议放 §5.3 末或 §5.4 前）

> **反演 1（$L_2$ 能量预算 → $K_d$ 下界）**。给定扰动能量预算 $E\triangleq\|d_{L_2}\|_{L_2}^2$ 与加权误差能量预算 $\bar\epsilon>2V(0)$：任取 $\kappa>0$，令
> $$\gamma_a=\sqrt{\frac{\bar\epsilon-2V(0)}{E}}\ (E=0\ \text{时取}\ \gamma_a\to\infty),\qquad K_d\succeq\tfrac12\Bigl(\kappa^{-1}+\frac{E}{\bar\epsilon-2V(0)}\Bigr)I_6,$$
> 则 $\int_0^\infty\kappa^{-1}\|e_\xi\|^2dt\le\bar\epsilon$（由 (5.6) 直接代入）。工程解读：预算越紧（$\bar\epsilon\downarrow$）或扰动越强（$E\uparrow$）→ 所需 $K_d$ 越大；$\kappa$ 是自由标度参数，把"误差能量预算"换算为"加权误差能量"，不影响控制律。
>
> **反演 2（稳态 RMS 允许值 → 增益不等式，各向同性情形）**。取 $K_d=k_dI_6$、$K_p=k_pI_6$（$\alpha<1$ 即满足 (5.1f)）。给定 $D_{ex}$、$\alpha$、允许稳态 RMS $\bar\epsilon_{RMS}$ 与位能壳半径 $c$：
> $$D=D_{ex}+\alpha k_p\Bigl(1+\sqrt{2c/k_p}\Bigr)\sqrt{2c/k_p},\qquad k_d\ \ge\ \frac{D}{(1-\alpha)\,\bar\epsilon_{RMS}},$$
> 即 $\lambda_{eff}=k_d(1-\alpha)\ge D/\bar\epsilon_{RMS}$。工程解读：$k_p$ 只进 $D$（分子）、$k_d$ 只进 $\lambda_{eff}$（分母），两不等式解耦；$k_p\uparrow$ 降低 $e_z$ 稳态（(5.9)）但抬高 $D$——存在一维最优刚度（扫描即可）；$c$ 取事后核验的位能峰值（§6.4）。
>
> **反演 3（保证窗口）**。推论 3(e)：$T_*^{est}=\lambda_{eff}^{-1}\ln\bigl(1+2\lambda_{eff}^2(c-V(0))/D^2\bigr)$，即"界常数—保证时长"折衷的显式曲线。

### 5.5 (e) "H∞" 术语精确化建议

本文是**耗散不等式风格的 $L_2$ 增益证书（有界实引理 BRL 风格）**，不是传递函数意义下的 $H_\infty$ 范数、也不是 LMI $H_\infty$ 综合。建议：
1. §5 标题"几何一致控制律与混合 H∞/ISS 性能"→"几何一致控制律与混合 $L_2$ 增益/ISS 性能（耗散不等式证书）"；若保留"H∞"，加脚注："本文的 'H∞' 指非线性耗散意义下的有限 $L_2$ 增益证书（BRL 风格），非传递函数 $H_\infty$ 范数"。
2. 定理 3(c) 标题"…H∞ 二次型/Schur 补判据…"→"…$L_2$ 增益的二次型/Schur 补充要判据…"。
3. 边界说明 (ii) "不声称…整体 H∞ 界"→"不声称…整体 $L_2$ 增益界"。
4. §1.2 贡献第 3 条中的"H∞ 增益"→"$L_2$ 增益（H∞ 意义下）"，避免审稿人按线性 $H_\infty$ 理论（需传递函数、需 LMI 综合）提问。
5. **（与 LIT 对齐）**：van der Schaft 1992（非线性 $H_\infty$ 的 Hamilton–Jacobi 框架）与 ZDG96 并引于定位段，支撑"标准 H∞ 工具对非线性机器人需线性化/LPV、丢失充要性与量纲齐次性"的论断（LIT §3-7）。

### 5.6 (f) ISS 表述建议（与 LIT 对齐后的统一术语处理）

**背景**：LIT 指出最大表述风险是标题/摘要/关键词的 "H∞/ISS" 与正文"不声称全状态 ISS"、strictification 未完成直接冲突（LIT §5.2/§6-1）。对策——**正文用"实用 ISS 型界"、标题/摘要/关键词绝不用裸 "ISS"**：

- **标题/关键词**：首选"混合 $L_2$ 增益/$L_\infty$ 稳态性能（耗散证书）"，次选 LIT 的"H∞/L∞ 混合性能"（注意本文 $L_\infty$ 通道给的是 RMS/逐点稳态界而非 $L_\infty$ 增益，措辞以"稳态性能"为准）；关键词删除"输入-状态稳定"，如必须保留改"输入-状态稳定型界（目标）"。
- **正文统一术语**（替换现文"逐点实用界"的随意叫法）：定理 3(d) 的 RMS 界 + 推论 3(e) 的逐点界合并表述为"**$e_\xi$ 的实用 ISS 型界**（practical ISS-type bound for $e_\xi$）"：输入为等效扰动幅值 $D$（含 $d_{ex}$ 与乘性 $\alpha$ 项）、渐近增益 $1/\lambda_{eff}$、偏置 $\sqrt{2c}$（位能壳伪影）；全文只用"ISS-型/ISS 风格"描述性短语，不加引号、不大写口号化。
- **整合句（可直接进论文）**：
> 本文的输入-状态型主张精确化为：**$e_\xi$ 的实用 ISS 型界**（RMS 界 (5.7) 给出无伪影稳态常数 $D/\lambda_{eff}$；推论 3(e) 给出显式指数暂态与最终球 $\sqrt{2c+D^2/\lambda_{eff}^2}$，含先验保证窗口 $T_*^{est}$）+ **$e_z$ 的位能壳/准静态处理**（(5.9) 稳态预测 + C.4 严格化）。全状态（12 维）ISS 与运动学外环级联的整体 $L_2$ 增益不在本文范围内；每一条现有证书均可独立核验。
- 并二选一：**(方案 A)** 完成 strictification（§6 方案要点已给闭式条件，作为未来工作附在 C.4）；**(方案 B)** 若暂不完成，把"拆分式证书"写成特色：同一 $V$ 上同时给出 $L_2$ 能量界、$L_\infty$ RMS 标度、逐点实用界三件套，且三者对增益结构闭（(c-1)↔(c-2) 退化自动切换）。无论选哪个，摘要/标题都不出现裸 "ISS"。

---

## 6. strictification（附录 C.4）完成方案要点（未来工作/可选）

**候选**：$W=V+\epsilon\Phi$，$\Phi=e_z^\top K_pA(\tilde x)e_\xi$，$\epsilon>0$ 待定；$\bar A\triangleq1+\sqrt{2c/\lambda_{\min}(K_{p,T})}$，$c_A\triangleq[2\bar A/\eta_0+1]^{-1}$（C.2 的一致下界）。

**第 1 步（等价性）**：在 $\Omega_c$ 上 $|\Phi|\le\lambda_{\max}(K_p)\bar A\|e_z\|\|e_\xi\|\le\tfrac{\lambda_{\max}(K_p)\bar A}{\min(1,\lambda_{\min}(K_p))}V$，取 $\epsilon\le\tfrac{\min(1,\lambda_{\min}(K_p))}{2\lambda_{\max}(K_p)\bar A}$ 得 $\tfrac12V\le W\le\tfrac32V$。

**第 2 步（$\dot W$ 精确式）**：用 (5.5) 逐项求导，
$$\dot W=-e_\xi^\top K_de_\xi+e_\xi^\top d+\epsilon\Bigl[-\|A^\top K_pe_z\|^2+e_\xi^\top A^\top K_pAe_\xi-e_z^\top K_pAK_de_\xi+e_z^\top K_p\dot Ae_\xi+e_z^\top K_pAd\Bigr],$$
其中新项 $-\epsilon\|A^\top K_pe_z\|^2\le-\epsilon\lambda_{\min}(K_p)^2c_A^2\|e_z\|^2$ 提供 $e_z$ 方向负定——这是 strictification 的核心收益。

**第 3 步（交叉项闭式上界，全部为 $\Omega_c$ 上的显式常数）**：
- $\|A\|_2\le\bar A$、$\|AK_d\|_2\le\bar A\lambda_{\max}(K_d)$；
- **$\dot A$ 项的一致界（关键难点，可闭式给出）**：由 $\dot{\mathcal O}=A_{11}\tilde\omega$、$\tilde\eta^2+\|\mathcal O\|^2=1$ 得 $|\dot{\tilde\eta}|=|\mathcal O^\top\dot{\mathcal O}|/\tilde\eta\le\tfrac{\|\mathcal O\|}{2\eta_0}\|\tilde\omega\|$，$\|\dot{\mathcal T}\|\le\|\mathcal T\|\|\tilde\omega\|+\|\tilde v\|$，故
$$\|\dot A\|_2\le\Bigl[\tfrac{\|\mathcal O\|}{4\eta_0}+\tfrac12+\|\mathcal T\|\Bigr]\|\tilde\omega\|+\|\tilde v\|\le C_1\|e_\xi\|,\quad C_1\triangleq\max\Bigl\{\tfrac{\sqrt{2c/\lambda_{\min}(K_{p,O})}}{4\eta_0}+\tfrac12+\sqrt{2c/\lambda_{\min}(K_{p,T})},\ 1\Bigr\};$$
- 立方项合并：$\epsilon\lambda_{\max}(K_p)C_1\|e_z\|\|e_\xi\|^2\le\epsilon\lambda_{\max}(K_p)C_1\sqrt{2c/\lambda_{\min}(K_p)}\,\|e_\xi\|^2$（用 $\|e_z\|\le\sqrt{2c/\lambda_{\min}(K_p)}$）。

**第 4 步（参数条件，均可显式求解）**：
$$\epsilon<\epsilon_1\triangleq\frac{\lambda_{\min}(K_d)}{\lambda_{\max}(K_p)\bigl(\bar A^2+C_1\sqrt{2c/\lambda_{\min}(K_p)}\bigr)},$$
交叉项 $\epsilon\lambda_{\max}(K_p)\lambda_{\max}(K_d)\bar A\|e_z\|\|e_\xi\|$ 由 Young 吸收给出 $\epsilon$ 的二次不等式（解出 $\epsilon\in(0,\epsilon_2)$）；$d$ 项 $\|e_\xi\|\|d\|+\epsilon\lambda_{\max}(K_p)\bar A\|e_z\|\|d\|$ 吸收后得
$$\dot W\le-c_1\|e_z\|^2-c_2\|e_\xi\|^2+c_3\|d\|^2,$$
即标准 ISS-Lyapunov（对输入 $d$，$L_2$ 增益与增益裕度 $c_3/\min(c_1,c_2)$ 可显式写出）。等价性与 $\epsilon\in(0,\min(\epsilon_1,\epsilon_2,\epsilon_3))$ 非空性在"$c$ 与 $K_p,K_d$ 满足显式代数不等式"时成立（与局部性一致：小 $c$ + 小 $\epsilon$ 可行）。

**第 5 步（完成后的三件收益，供 §7 局限 (i) 改写）**：① 推论 3(e) 的 $2c$ 伪影被替换为无伪影逐点界；② $e_z$ 的严格上界（替换 (5.9) 的准静态估计）；③ 全状态（12 维）局部 ISS 陈述。

**（与 LIT 对齐：把"未完成"升级为"成熟方法的直接应用"）**：$W=V+\epsilon e_z^\top K_pA(\tilde x)e_\xi$ 正是 Sontag–Wang 1995（ISS $\Leftrightarrow$ ISS-Lyapunov 函数刻画）与 Malisoff–Mazenc 2009（严格 Lyapunov 函数标准构造手册）的**标准 strictification 构造**（加 $\epsilon$ 交叉项回收 $-\|e_z\|^2$ 负定），建议在附录 C.4 与 §7 局限 (i) 中引用二者，并把表述从"本文未完成此验证"改为"该路线是成熟方法的直接应用，本文未完成的部分仅是 $\epsilon$ 可行区间非空性的显式代数化简与数值条件数验证——属收尾工作量问题，而非方法缺失"。同时保留诚实声明：工作量 ≠ 已完成，正文不得以"路线成熟"代替"验证完成"。三件收益的叙述保持（供 §7 局限 (i) 改写）。

---

## 7. 其他校对级问题清单

1. 定理 3(c) 中 "$\gamma_a\sqrt\kappa$（无量纲）" → "（量纲为时间 s）"（见 §1.3-问题2）。
2. 定理 3(b) 注记中 "余度约 2.5 个数量级" 建议同时给出数值（$c^*=160$，$V_{peak}\le0.494$）——已有一处，保持一致。
3. 附录 C.8 第三步中 $\|A\|_2\le1+\|\mathcal T\|$ 的来源（块三角分解 + $\|A_{11}\|_2=\tfrac12$ 精确值）建议在正文 3(d) 证明中引用一次，防止读者误以为有放缩。
4. (5.1f) 后建议加"（等价地：$\lambda_{eff}>0$，即证书非退化条件）"。
5. §5.4 标题"近恒等线性化模型与静态刚度标度律"前可加一句"本节的 (5.8)–(5.9) 为 $\tilde x\to1$ 的局部近似，严格结论以定理 3 为准"（现已有，保持）。
6. 引言"三个结构性缺口"与 §5.3 开篇改写的措辞保持完全一致（位姿/速度误差分离、加速度层扰动无入口、偏差型不满足 $L_2$），建议 §1.1 与 §5.3 逐字对应。

---

## 8. 与协调员整合笔记（`_integration_notes.md`）的交叉引用

- **A-2 逐点界数值背书**：协调员以 $c=\tfrac12\max_t(e_z^\top K_pe_z)$（位能壳口径）对三组 npz 验证零违反。**THEO 已独立复现（§2.6）**：同一口径下三运行逐点界（紧/松两形式）最大违反均为 0，RMS 界亦成立，耗散等式残差为 FD 噪声（集中于端点/注入时刻）。建议把"前提=位能壳"写进推论陈述（§2.2 的 (P1)），并在 §6 用一句话引用该验证；"非平凡"建议表述为"**界恒成立、有限、可先验计算**"（实测峰值低于界约 12.6–13.9×，保守性可归因，见 C-2）。
- **B-2 RMS 界对照**（$rms\_bound\_5\_7$ 8–59 倍余量）：可作为定理 3(d) 与推论 3(e) 的联合实验证据；注意协调员注明的口径问题（$\alpha\to0$ 乐观侧、重建 $d$ 的范数口径），正文引用时注明。
- **B-3 gamma_sweep**：组 A（$\gamma_a$ 纯分析、扫参不改变轨迹）直接支撑 §5.5 术语建议与"纯分析参数"卖点；组 C（C3 的 $\gamma$ 进控制器、无认证列）是"分析参数 vs 综合参数"的对照组。引用前需声明读法 A/B（`docs/TNDQ论文删除的内容.md` 的区分）。
- **C-2 早期草稿保守性归因**：$\|A\|_2\le1+\|\mathcal T\|$、$\|A_{11}\|_2=\tfrac12$ 最坏方向、$d\perp e_\xi$ 近似、Young 等号条件——这些是 RMS 界保守性 8–59 倍的来源，建议在 §5.3 或 §6 作为"保守性的可归因清单"公开（主动透明化可防审稿人"界太松"的质疑）。

---

## 9. 与 LIT 文献结论的对齐（吸收/拒绝清单）

> 本节在读完 `docx/agent_LIT_report.md` 全文后追加，逐条说明对 LIT 五项指令的采纳/拒绝，并给出已并入本报告对应小节的具体位置。

### 9.1 采纳项（已并入本报告正文）

| # | LIT 结论 | 采纳后的处理 | 本报告位置 |
|---|---|---|---|
| 1 | **"H∞/ISS" 表述风险**（LIT §5.2/§6-1）：标题/摘要/关键词的裸 "ISS" 与"不声称全状态 ISS"+strictification 未完成冲突 | 统一术语方案：标题/关键词改为"混合 $L_2$ 增益/$L_\infty$ 稳态性能（耗散证书）"（次选 LIT 的"H∞/L∞ 混合性能"）；关键词删"输入-状态稳定"；正文把定理 3(d) RMS 界 + 推论 3(e) 逐点界合并为"**$e_\xi$ 的实用 ISS 型界**"（描述性短语、小写、不加引号）；任何地方不出现裸大写 "ISS" 声明 | §5.6 整节改写（含整合句）；§0 速览项 3 |
| 2 | **Schur"充要"必须限定**（LIT §6-1）：(5.6a) 当且仅当是"对给定存储函数 $V$ 的耗散不等式"，非系统 $L_2$ 增益本身（$V$ 未搜索） | 给出可直接进论文的最终措辞（含量纲一致性 $\tfrac12(\kappa^{-1}+\gamma_a^{-2})\sim\mathrm{s^{-1}}$ 与 $K_d$ 相同，回应 LIT 攻击点 8） | §1.3-问题4（已更新） |
| 3 | **[P2] 已用 Schur 补**（LIT §2.1/§5.2/§7）：必须显式声明五点差异 | §5.3 开篇/定位段插入相对 [P2] 的五点差异（层/输出/判据形态/参数位置/量纲），直接采用 LIT §7 第一段表述；其余四段（Ch20–Wang/Yu、Spong92、LMI–van der Schaft、Pham 2025）按 LIT §7 原文用于 §1.1/§4 | §5.1（已加定位注记） |
| 4 | **(5.1f) 是 Spong92 α<1 的任务空间矩阵化推广**（LIT §5.2/§7）：写一句继承关系声明 | 合成一句"继承关系 + 范数对象差异"："(5.1f) 是 Spong 关节空间乘性条件的任务空间矩阵化推广：$K_d=k_dI$ 时形式退化为 $\alpha<1$，且把标量 $\alpha$ 推广为 $\Theta=JM^{-1}\Delta MJ^+$ 谱范数、把逐点 UUB 球推广为 $\lambda_{eff}$ 折减下 RMS 稳态标度；范数对象不同（任务空间投影 vs 关节空间）" | §1.3-问题5（已更新） |
| 5 | **补引**（LIT §8）：Wang & Yu 2013（Ch20 论断真正来源）；Sontag & Wang 1995 + Malisoff & Mazenc 2009（strictification 理论依据）；van der Schaft 1992（非线性 H∞ 框架）；Pham 2025（直接竞争者） | Wang & Yu 2013 → §1.1 缺口处与 §4 定理 1/2 对比；Sontag–Wang 1995 + Malisoff–Mazenc 2009 → 附录 C.4 与 §7 局限 (i)，把 strictification 从"未完成"升级为"成熟方法的直接应用、仅剩收尾工作量"（同时保留诚实声明）；van der Schaft 1992 → §5.3 定位段（与 ZDG96 并引）；Pham 2025 → §1.1 综述句 + §5.3 对比注记 | §5.5（van der Schaft）；§6 第 5 步（Sontag–Wang/Malisoff–Mazenc）；§5.1（引用 LIT §7） |

### 9.2 折中/补充项（部分采纳）

1. **LIT 攻击点 4（定理 3(d) 自我未闭合，$\Omega_c$ 靠事后核验）**：采纳"标注为条件性结果"；补充——推论 3(e) 的先验保证窗口 $T_*^{est}$（§2.2）正是"给出 $V$ 沿含扰轨迹的先验衰减窗口"的部分闭合，可在 §5.3 注明"窗口内自洽闭合、全时域闭合需 strictification"。
2. **LIT 攻击点 5（局部性/翻转相容性）**：采纳，对应本报告 §4 W9 改写（"可测量的局部性"）；符号翻转与 $A^\top$-整形反馈的相容性（翻转不改变 $\tilde{\boldsymbol\xi}$，定理 1(i)）已在论文注记 1 覆盖，建议 §5.3 加半句显式说明。
3. **LIT 攻击点 6（仿真说服力）**：属 DATA 专家范围，THEO 不表态；仅提示：§5.3 的 RMS 界对照图（协调员 B-2）与推论 3(e) 零违反数据（§2.6）是"证书可证伪性"的最直接证据。

### 9.3 拒绝/不采纳项（附理由）

1. **LIT 标题建议"H∞/L∞ 混合性能"部分拒绝**：本文 $L_\infty$ 通道给出的不是 $L_\infty$ 增益（无 $\|d_b\|_{L_\infty}\to\|e_\xi\|_{L_\infty}$ 型增益），而是 RMS/逐点**稳态界**（$D/\lambda_{eff}$），措辞以"$L_2$ 增益/$L_\infty$ 稳态性能"为准更准确；"H∞/L∞ 混合性能"仅作次选并加脚注。
2. **LIT 对 strictification 的"成熟方法"升级完全采纳，但拒绝"以路线成熟替代验证完成"**：正文必须保留"$\epsilon$ 区间非空性未显式验证"的诚实声明（工作量 ≠ 已完成），与 §7 局限 (i) 一致；升级只改变定性（方法有效），不改变当前结论状态（未完成）。
3. **LIT 建议补引 Adorno–Marinho 2022 / Arrizabalaga–Ryll 2023**：属 §1.1 谱系与前沿句（LIT §8 第 4、5 条），非 §5.3 必需；THEO 不反对，但标注为 §1.1 责任、不在本报告展开。

---

*报告完毕（含 §9 LIT 对齐）。所有推导均经手算复核；与定理 3 相关的全部结论为"成立 + 若干需澄清的精确性点"，无致命错误。*
