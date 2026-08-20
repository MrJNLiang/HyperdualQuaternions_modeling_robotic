# 文献调研报告（LIT 专家）

**任务**：为《Third-Order Dual Quaternion (TODQ) Kinematics with Geometrically Consistent Error Systems: A Computed Torque Approach to H∞/ISS Control》初稿定位相对现有文献的理论贡献与风险。
**依据**：`main_extracted.txt`（§1 行 1–100、§5 行 331–745、§7 行 885–914、参考文献 1352–1397、附录 C 行 969–1273）+ 三篇参考文献 PDF 全文（`docx/P2_*.txt`、`docx/Ch20_*.txt`、`docx/P1_*.txt`）+ 2020–2025 web 调研。
**日期**：2025-08-16

---

## 1. 论文初稿核心主张速览（据 §1.2、§5.3、§7）

1. **TODQ 运动学（§3）**：以三项幂零代数 A₂（σ³=0，[Con25] 方向的最小三阶扩展）表示 x̄ = x̂ + σx̂̇ + ½σ²ẍ，串联链一次 O(n) 连乘同时输出 (x̂, ξ, vec6 ξ̇ = J̇q̇ + Jq̈)。
2. **HDQ 误差体系（§4）**：误差元素 x̃̆ = x̆(x̆d)* 一次乘法同时生成右不变位姿误差与几何一致 twist 误差（定理 1），闭式级联运动学 ėz = A(x̃)eξ（定理 2）；0 阶退化即 [P2] 的位姿误差。
3. **几何一致计算力矩律 + 混合证书（§5）**：A⊤-整形位姿反馈使交叉项精确相消，得**精确耗散等式** V̇ = −eξᵀKd eξ + eξᵀd（无任何放缩）。
   - 定理 3(b)：无扰水平集 Ωc 正向不变 + 渐近收敛 + 局部指数稳定（工作域局部，η̃=0 奇异与 SO(3) 双覆盖为拓扑障碍）。
   - 定理 3(c)：L2 扰动下性能目标 V̇ ≤ −(1/2κ)‖eξ‖² + (γa²/2)‖d‖² 的 **Schur 补当且仅当判据** Kd ⪰ ½(κ⁻¹+γa⁻²)I6（(c-1) 合并；(c-2) 旋转/平移逐分量拆分，量纲齐次，κω,γω,κv,γv 独立）。证书参数 κ, γa **不出现在控制律 (5.2) 中**。
   - 定理 3(d)：L∞ 扰动按 (5.1d) 代数分离 d = Θ(q)ufb + dex（乘性/外生），小增益条件 α < λmin(Kd)/λmax(Kd)，有效阻尼 λeff = λmin(Kd) − αλmax(Kd)，twist 误差有限时段均方界 ≤ D²/λeff² + 2V(0)/(λeff·T)，limsup RMS ≤ D/λeff。
   - 明确边界（§5.3 末尾、注记 2、§7 局限）：**不声称全状态 ISS**；strictification（附录 C.4）为未来工作；Ωc 前提事后数值核验；ez 稳态量级仅由线性化模型 (5.9) 工程估计。
4. **静态刚度标度律（§5.4）**：近恒等线性化两分量二阶模型，旋转块 A0 的 −½I3 因子使有效旋转刚度 ¼Kp,O；稳态残差只由静态刚度决定、与阻尼无关：(5.9)。§6 在 KUKA LBR4+ 力矩模式仿真中反演一致性 1.93%，水平集余度约 2.5 个数量级。

**关键结构事实（决定文献定位）**：扰动进入**加速度层**（ξ̇ 方程，相对阶 2），输出证书只约束 **twist 误差 eξ**；P2 的扰动进入**速度层**（ẋ 方程，相对阶 1），输出是**位姿误差**。

---

## 2. 三篇参考文献 PDF 逐篇分析（pdftotext 全文核实）

### 2.1 [P2] L.F.C. Figueredo, B.V. Adorno, J.Y. Ishihara, "Robust H∞ kinematic control of manipulator robots using dual quaternion algebra", Automatica 132 (2021) 109817
来源：https://www.sciencedirect.com/science/article/abs/pii/S000510982100337X （arXiv 版 https://ar5iv.labs.arxiv.org/html/1811.05436 ，zbMATH https://zbmath.org/1478.93146 ）

- **问题设置**：**纯运动学层**（速度级输入 q̇ = J⁺(·)，无惯量/科氏/重力/加速度参考；文内自述 "kinematic controller"、"kinematic control"）。
- **扰动模型**：① 加性 twist 扰动 vw 进入速度级运动学 ẋ = ½(vec6(Jq̇) + vw + vc)x（式 (6)）；② 乘法位姿不确定性 c ∈ Spin(3)⋉R³（x = xN·c，式 (5)），但经 v̄c = x*vc x 转写为**twist 级加性项**；③ 全部假设 vw, v′w, vc, v′c ∈ L2([0,∞), Hp)（式 (13) 上下文，Definition 1 第 (2) 条明确 "∀vw, vc, v′w, v′c ∈ L2"）。**无 L∞/持续偏差型扰动入口**。
- **证书类型**：
  - Lyapunov 函数 V = α1‖z̃‖² + α2‖z̃′‖²（位姿误差 1−x̃ 的两个 DQ 部分的范数二次型，式 (14)）；
  - 无扰**指数稳定**（V̇ ≤ −min{κO/2, 2κT}·V + Comparison Lemma，式 (15)–(16) 段）；
  - **H∞ L2 增益**（零初值）：被认证量是**位姿误差输出** O(z̃)=Im(z̃)、T(z̃)=p̃ 对扰动 (vw, vc) 的能量增益 γO、γT（Definition 1 的 (2)–(4) 式）；由 Schur 补导出**标量增益条件** κO ≥ 1/α1 + (α1/4)(γO1⁻²+γO2⁻²)、κT ≥ 2/α2 + (α2/8)(γT1⁻²+γT2⁻²)（式 (18)–(20)），再对 α1, α2 取最优以最小化瞬时控制能量（κO, κT 取最小）。
- **保守性来源**：(i) 导数界为**不等式**型（内积展开 + Young 型放缩，"M ≤ 0 蕴含条件"是充分化处理）；(ii) 扰动必须 L2（能量有限），持续有界偏差型扰动无证书；(iii) 只约束位姿误差输出，**twist 误差无 L2 界**；(iv) 位姿误差输出混 (rad)² 与 m² 量纲，无量纲齐次论证；(v) γ 越小 → κ 越大 → 控制能量/带宽需求越高，性能与实现强耦合。
- **保证不了**：动力学接口（无 M,C,g 层的加速度扰动入口；真实动力学耦合只能靠内环速度伺服吸收）；L∞ 扰动下任何稳态界；ISS；水平集工作域分析（unwinding 仅 Remark 1 提及，无定量工作域）；J J⁺ ≠ I 的奇异残差 vs 只并入 vw（脚注 6），无小增益条件。

### 2.2 [Ch20] R. Chandra, J.A. Corrales-Ramon, Y. Mezouar, "Resolved-Acceleration Control of Serial Robotic Manipulators Using Unit Dual Quaternions", IFAC-PapersOnLine 53(2) (2020) 8500–8505
来源：https://dumas.ccsd.cnrs.fr/UNIV-BPCLERMONT/hal-03206513v1 （ScienceDirect: IFAC-PapersOnLine 53-2, 2020）

- **问题设置**：**动力学层**（解算加速度/计算力矩，扭矩命令 Γ = H(q)Ĵ⁻¹(âcmd − Ĵ̇q̇) + C(q,q̇)q̇ + G(q)，式 (2)）；运动学用螺旋型 UDQ（Özgür–Mezouar）+ Featherstone 空间动力学的雅可比导数。
- **扰动模型**：**无**。明确假设 "Assuming perfect dynamic compensation"（式 (2) 前），全文无不确定性/扰动项、无 L2/L∞ 模型。
- **证书类型**：唯一稳定性论断是**无扰误差动力学** ω̂̇e + K̂v ω̂e + 2K̂p ln x̂e = 0（式 (36)）的**渐近稳定性**——直接**引自 Wang & Yu 2013**（"has been proven in Wang and Yu (2013)"），本文未构造 Lyapunov 函数、无导数不等式、无收敛率。双平衡点 (Î,0̂)/(−Î,0̂) 问题用 "乘 −1 使实部标量部分为正" 处理（引用 Wang & Yu 2013 的结论）。
- **保守性/缺口**：位姿反馈取**螺旋对数** ln x̂e（式 (35)），在大误差（转角 → π）处奇异、导数映射退化——正是初稿 §1.1 所指 "其导数映射在大误差处奇异"；**无任何鲁棒证书**（无扰动模型 ⇒ 无 H∞/ISS/UUB/L2 增益）；无水平集/局部指数论证；位姿误差与速度误差是分离的两套对象（对数位姿反馈 + twist 误差），非一次乘法生成的几何一致误差。

### 2.3 [P1] A. Cohen, M. Shoham, "Hyper Dual Quaternions representation of rigid bodies kinematics", Mechanism and Machine Theory 150 (2020) 103861
来源：https://www.sciencedirect.com/science/article/abs/pii/S0094114X20300823 （DOI: 10.1016/j.mechmachtheory.2020.103861）

- **问题设置**：**纯运动学表示层**——引入 HDQ（以超对偶数 HDN 为分量的四元数，两个幂零单位 ε, ε*），一个 HDQ 元素携带**位姿 + 一阶时间导数**（"hyper-dual part of Eq. (22) is a derivative with respect to time of the dual part"）；串联链一次链乘同时给出末端位姿与速度（式 (34) 及第 5 节），免去显式求导。**无控制、无动力学、无误差体系、无稳定性分析**。
- **关键原文（结论）**：*"For acceleration, jerk, snap or higher order derivative calculations, one may expand the HDQ to dual-quaternions of higher orders, taking advantage of the extension of Kotelnikov's Principle of transference to dual-numbers of order n"*——即 [P1] 自身明确把加速度及更高阶留给高阶扩展，且未做。初稿 §1.1 所述 "两个幂零单位均已占用，εε* 项不含二阶时间导数" 与原文一致（HDQ 中 ε 承载平移、ε* 承载一阶导数，εε* 项为平移一阶导数项）。
- **与本论文关系**：本论文 TODQ（A₂ 三阶）正是 [P1] 指出的 "dual-quaternions of higher orders" 方向的**最小三阶实现**（σ 幂次装 x̂, x̂̇, ẍ），并把该代数从"表示层"推进到"误差体系 + 控制证书层"。两文的贡献互补、不重叠；本论文对 [P1] 的引用（作为 TODQ 的代数源头）是恰当的，但应同时补引 Condurache [Con25] 及 Fike–Alonso HDN（已引 [3]）。

---

## 3. 外部文献调研（2020–2025 及经典支撑，7 条）

1. **X. Wang, C. Yu, "Unit dual quaternion-based feedback linearization tracking problem for attitude and position dynamics", Systems & Control Letters 62(3) (2013) 225–233**（zbMATH: https://zbmath.org/6157681 ；ANU: https://researchportalplus.anu.edu.au/en/publications/unit-dual-quaternion-based-feedback-linearization-tracking-proble ）
   相关性：**Ch20 稳定性论断的唯一真正来源**（其式 (36) 的渐近稳定性、双平衡点处理均出自此文）；本文 §1.1/§4 在批评 "螺旋对数位姿反馈 + 双平衡点" 时必须直接引它，否则 [Ch20] 的引用链不完整。
2. **E.D. Sontag, Y. Wang, "On characterizations of the input-to-state stability property", Systems & Control Letters 24(5) (1995) 351–359**（https://www.semanticscholar.org/paper/32639e79b7e6bb580996aa6b084b91c7695e9f07 ；zbMATH: https://zbmath.org/0877.93121 ）
   相关性：ISS ⇔ ISS-Lyapunov 函数的等价性定理，是附录 C.4 strictification 路线的理论依据（V̇ 无 −‖ez‖² 负项 ⇒ 需构造严格化 Lyapunov 函数才能得局部 ISS）。
3. **M. Malisoff, F. Mazenc, Constructions of Strict Lyapunov Functions, Communications and Control Engineering, Springer, London, 2009**（https://in2p3.hal.science/INRIA/hal-01389866 ）
   相关性：strictification（加 ϵe⊤z Kp A(x̃)eξ 型交叉项回收 −‖ez‖² 负定）的标准构造技术手册；本文附录 C.4 的 "W = V + ϵ…" 正是其标准套路，应引以表明方法是成熟的、只是本论文尚未完成验证。
4. **H.-L. Pham, "Robust Torque-Computed Control for a Robot Manipulator With Unit Dual Quaternion", IEEE Xplore（Early Access, 2025, 文献号 10945815；卷/期/DOI 待核）**（https://ieeexplore.ieee.org/abstract/document/10945815 ；作者页 https://ieeexplore.ieee.org/author/37087811844 ）
   相关性：**与本论文最接近的直接竞争者**——DQ + 计算力矩 + 鲁棒性（torque-computed control）的 2025 年新作；本文 §1.1/§5 必须引并明确差异（其证书形态、扰动模型、是否含几何一致误差体系与量纲齐次拆分）。
5. **B.V. Adorno, M.B. Marinho, "Kinematic screws and dual quaternion based motion controllers", Control Engineering Practice 118 (2022) 104709**（PII S0967066122001605: https://www.sciencedirect.com/science/article/abs/pii/S0967066122001605 ；卷/文号请按原文复核）
   相关性：DQ 运动学控制器的系统性推广框架（统一 [P2] 类律并推广至多臂/协作场景），本文 §1.1 定位 "DQ 控制谱系" 时引用。
6. **J. Arrizabalaga, M. Ryll, "Pose-Following with Dual Quaternions", Proc. 62nd IEEE Conf. on Decision and Control (CDC), Singapore, 2023**（arXiv:2308.09507: http://arxiv.org/abs/2308.09507 ；IEEE: https://ieeexplore.ieee.org/document/10383688 ）
   相关性：2023 年 DQ 位姿跟踪的新收敛证书（其自造误差函数 + 收敛证明），代表 "DQ 误差体系证书" 这一活跃竞争方向的最新进展，本文 §1.1 应引以表明研究前沿，并在 §5 与其证书形态对比。
7. **A.J. van der Schaft, "L2-gain analysis of nonlinear systems and nonlinear state feedback H∞ control", IEEE Trans. Automatic Control 37(6) (1992) 770–784**
   相关性：非线性 H∞/L2 增益的 Hamilton–Jacobi 经典框架——支持本文 "标准 H∞ 工具对非线性机器人需线性化/LPV 建模、丢失充要性与量纲齐次性" 的定位论断（与 ZDG96 的 LMI 框架互为补充）。
   （补充候选：N. Filipe, P. Tsiotras, "Rigid body motion tracking without linear and angular velocity feedback using dual quaternions", Proc. ECC 2013 —— DQ 误差系统证书的早期代表，可并入 §1.1 谱系句。）

---

## 4. 证书对比矩阵

行 = 证书维度；列 = [P2] / [Ch20] / Spong92 类鲁棒计算力矩 / 标准 LMI H∞（ZDG96 框架）/ 本论文初稿。填 是/否/部分 + 一行证据。

| 证书维度 | [P2] Figueredo 2021 | [Ch20] Chandra 2020 | Spong92 类鲁棒计算力矩 | 标准 LMI H∞（ZDG96） | 本论文初稿 |
|---|---|---|---|---|---|
| **1. 精确耗散等式（无放缩）** | 部分。V̇1 ≤ −α1⟨O z̃, κO O z̃+vw+vc⟩ 为不等式（含 Young 型处理与 α 最优化），非等式。 | 否。无任何耗散分析。 | 否。V̇ ≤ −λmin(Kd)‖e‖²+‖e‖μ，含界/放缩，仅 UUB。 | 否。Riccati/LMI 不等式，无"等式型"耗散。 | **是**。V̇ = −eξᵀKd eξ + eξᵀd，交叉项被 A⊤-整形精确相消，无任何放缩（定理 3 证明）。 |
| **2. L2 增益的充要 Schur 判据** | 部分。Schur 补导出 κO ≥ 1/α1+(α1/4)γO⁻² 等，是对其位姿误差耗散不等式的当且仅当；但为标量增益、输出为位姿误差，且 (18) 处 "M ≤ 0 蕴含" 具充分化色彩。 | 否。 | 否。 | 部分。对**线性化模型** bounded-real lemma 是充要，但模型非原系统 ⇒ 原系统只有局部近似/充分性。 | **是**。性能目标 V̇ ≤ −(1/2κ)‖eξ‖²+(γa²/2)‖d‖² ⇔ Kd ⪰ ½(κ⁻¹+γa⁻²)I6（Schur 补当且仅当，附录 C.3 明确 "无任何符号放缩"）。 |
| **3. 旋转/平移分量量纲齐次拆分** | 否。γO/γT 分开但位姿误差输出混 (rad)² 与 m²，无 twist 级逐分量拆分与量纲论证。 | 否。 | 否。关节空间，无任务空间分量概念。 | 否。线化坐标无几何量纲结构。 | **是**。(c-2) 对 Kd=diag(Kω,Kv)、Kp,T 各向同性给出逐分量独立判据与独立参数 (κω,γω),(κv,γv)，量纲齐次（附录 C.3 两处代数恒零 T×T=0、Tᵀ(T×ω̃)=0）。 |
| **4. 乘性/外生扰动分离 d=Θu+dex** | 否。有乘法位姿不确定性概念（x=xN·c）但并入 L2 加性 twist 扰动 v̄c，无代数分离、无小增益条件。 | 否。无扰动模型（perfect compensation）。 | 部分。乘性失配 ΔM 用 α<1 类条件处理（小增益思想），但无 Θufb 与外生偏差的显式分离 d=Θu+dex。 | 部分。LFT/Δ 块建模乘性不确定性，但需 μ/结构化奇异值分析，且在线化层。 | **是**。d = Θ(q)ufb + dex 由 (5.1d) 显式代数导出，小增益条件 α < λmin(Kd)/λmax(Kd)（各向同性时退化到 [Spo92] α<1）。 |
| **5. 持续 L∞ 扰动下的稳态 RMS 界** | 否。仅 L2（能量型），偏差型扰动无入口。 | 否。 | 部分。UUB 给出逐点稳态球 ‖e‖≤μ/λmin(Kd) 类，但非均方/RMS 界，且无 λeff=λmin(Kd)−αλmax(Kd) 型有效阻尼折减的显式分解。 | 部分。线性系统可算 H∞/H2 界，非线性偏差扰动下无 RMS 稳态标度。 | **是**。定理 3(d)：有限时段均方界 + limsup RMS(eξ) ≤ D/λeff，D 显式含 Dex 与 αλmax(Kp)‖ez‖；α→0 时趋于 Dex/λmin(Kd)。 |
| **6. 水平集不变性与局部指数稳定** | 部分。无扰全局指数衰减（Comparison Lemma），但无水平集/工作域分析（unwinding 仅 Remark 1）。 | 部分。无扰渐近稳定（引自 Wang-Yu 2013），无 Lyapunov 构造、无水平集/局部指数。 | 部分。无扰渐近收敛（关节空间），无水平集工作域论证。 | 部分。线性系统指数稳定，无非线性水平集不变性。 | **是**。Ωc 紧且正向不变、η̃≥η0>0 使 A 一致可逆、渐近收敛 + 局部指数稳定（定理 3(b)，附录 C.2/C.6）。 |
| **7. 位姿误差静态刚度标度律** | 否。 | 否。 | 否。 | 否。 | **是**。(5.9) ∥T∥ss=‖dv‖/kp,T、∥O∥ss=2‖dω‖/λ(Kp,O)（含 1/4 旋转折减），§6 双档反演一致 1.93%。 |
| **8. 逐点/全状态 ISS** | 否。 | 否。 | 否（UUB ≠ ISS，无增益函数 γ(‖d‖) 结构）。 | 否（线性框架）。 | **否（明确不声称）**。正文声明不声称全状态 ISS；扰动到 ez 相对阶 2，V̇ 无 −‖ez‖² 负项；strictification 列为未来工作（附录 C.4）。 |
| **9. 证书参数为纯分析参数（不进入控制律）** | 否。κO,κT 即控制律 (12) 中的增益，γ 直接决定 κ（最小控制能量）。 | 否（无证书）。 | 否。鲁棒项增益必须大于扰动界（进入控制律）。 | 部分。γ 迭代是分析参数，但 Riccati 解即控制器状态/输出（进入实现）。 | **是**。κ, γa 只出现在证书 (5.6a) 中，不出现在控制律 (5.2) 中（分量版可独立指定）。 |

**矩阵核心结论**：本论文在 9 个维度中 7 项"是"，其中 **第 1（精确耗散等式）、3（量纲齐次拆分）、5（L∞ RMS 稳态界）、7（刚度标度律）、9（证书参数不进入控制律）** 五项在对比文献中**全部为否/部分**，构成其相对优势的核心；第 8 项（全状态 ISS）是明确承认的缺口，也是最大的表述风险点。

---

## 5. 新颖性判断

### 5.1 真正新颖（相对现有文献）

1. **TODQ 代数用于"误差体系 + 控制证书"的闭环整合**。[Con25]/[P1] 只到表示层（[P1] 明确把加速度留给高阶扩展但未做）；把 A₂ 三阶代数与"一次乘法同时生成位姿+twist 误差"（定理 1）和级联标准形（定理 2）结合，文献中未见。
2. **精确耗散等式 + 证书参数与控制律分离**。[P2] 的 H∞ 证书是"增益即控制参数 + 不等式型导数界 + 只约束位姿误差输出"；本文的 A⊤-整形精确相消给出等式型 V̇，且 κ,γa 是纯分析参数（可事后认证、不改控制律）——这是证书形态上的结构性增强，且被认证输出为 twist 误差（加速度层相对阶 2），与 [P2] 的速度层位姿误差证书正交。
3. **旋转/平移量纲齐次逐分量拆分（(c-2)）**：依赖两处代数恒零（T×T=0、Tᵀ(T×ω̃)=0），文献中未见等价结果。
4. **乘性/外生分离 d=Θufb+dex 与 λeff 有效阻尼下的 L∞ RMS 稳态界**：相对 Spong92 的 UUB，新增了 (i) 乘性分量对有效阻尼的折减 λeff=λmin(Kd)−αλmax(Kd) 的显式公式、(ii) 外生偏差型扰动的 RMS 稳态标度 D/λeff、(iii) 任务空间位姿误差层（Spong 在关节空间）。
5. **静态刚度标度律与 1/4 旋转折减**（含仿真反演验证）：工程上可直接用于增益整定、可实验证伪，属新的实现级结果。

### 5.2 重叠风险（需在文中显式划界）

- **"H∞ + Schur 补"与 [P2] 重叠**：Schur 补技术 [P2] 已用。必须显式声明差异：层（速度 vs 加速度）、输出（位姿误差 vs twist 误差）、判据形态（标量增益 vs 矩阵 Kd ⪰ ½(κ⁻¹+γa⁻²)I6 当且仅当）、证书参数位置（控制律内 vs 外）、量纲拆分（无 vs 有）。
- **"计算力矩 + α<1 小增益"与 Spong92 重叠**：本文 (5.1f) 各向同性阻尼时**退化**为 Spong 条件 α<1——这是作者已意识到的继承关系（正文引 [Spo92]），但建议在 §5.3 更明确写一句 "本文 (5.1f) 是 Spong 条件的任务空间矩阵化推广" 以免审稿人视作无新意。
- **级联误差运动学 ėz=A(x̃)eξ 与 DQ 文献既有误差运动学结构相似**：P2 的 z̃ 动力学、Wang-Yu 2013 的 ω̂e 动力学均为同族对象；本文增量在于 HDQ 一次乘法生成 + A(x̃) 显式闭式 + A⊤-整形相消，需在 §4 明确与 Wang-Yu 2013 的误差动力学对比（现稿未引 Wang-Yu，必须补）。
- **"ISS" 命名风险（最重要的表述风险）**：标题与关键词含 "H∞/ISS"、关键词含 "输入-状态稳定"，但正文无 ISS 定理（只有 RMS 界 + 无扰局部指数稳定），且 strictification 未完成。这会被审稿人直接抓住为过度声明（见 §6）。

---

## 6. 审稿人可能的攻击点

1. **过度声明（最重）**：摘要/关键词/标题的 "H∞/ISS" 与正文 "不声称全状态 ISS"、"strictification（附录 C.4）未完成" 直接冲突。建议：标题/关键词改为 "H∞/L∞ 混合性能" 或保留 ISS 但明确 "ISS 为未来工作目标"；摘要中 "输入-状态稳定" 关键词删除或改为 "输入-状态稳定性（目标）"。同时 "Schur 补充分必要条件" 必须限定为"对给定存储函数 V 的耗散不等式"当且仅当，而非系统 L2 增益本身的充要（V 未搜索）。
2. **与 [P2] 重复之嫌**：H∞ + Schur 补已有先例；若 §1.1 不显式对比"速度层 vs 加速度层、位姿输出 vs twist 输出、增益位置"，审稿人会认为增量是修辞性的。现稿 §1.1 只说了 [P2] "带 L2 扰动衰减保证的运动学跟踪控制器"，定位不够尖锐。
3. **与 Spong92 重复之嫌**：(5.1f) 是 Spong α<1 的退化推广；需引 [Spo92]、[Abd91]、[Sag99] 并明确"本文把经典计算力矩鲁棒性推进到任务空间位姿 + 混合证书"的增量。现稿 §5.1 仅一句 "各向同性阻尼时退化为经典条件 [Spo92]"，§1 未给定位。
4. **定理 3(d) 的自我未闭合**：Ωc 前提（水平集滞留）靠事后数值核验，不是证明；strictification 未完成意味着 L∞ 界的适用范围未被严格界定。作者已承认（§7 局限 i），但审稿人会追问：能否给出 V 沿含扰轨迹的单调衰减界来闭合？或至少把 (5.7) 标注为"条件性结果"。
5. **局部性**：所有收敛声明都是水平集局部（unwinding 与 η̃=0 奇异）。符号翻转规则（注记 1）处理了 η̃<0，但需说明翻转与 A⊤-整形反馈的相容性及瞬态行为。
6. **仿真说服力**：§6.4 自认单次运行、1.5–2% 差异无统计显著性；C1 vs C2 数值等价（≤0.05%）被用作"证书分化"论据，但等价同样可被解读为"无实际精度优势"——需给出明确的"证书分化"判据（如：注入大扰动/奇异邻域后 C1 的界保持、C2 对数奇异暴露）。
7. **缺引**：Wang & Yu 2013（[Ch20] 证书来源）、Sontag & Wang 1995 / Malisoff–Mazenc 2009（strictification 理论）、van der Schaft 1992（非线性 H∞ 框架）、Pham 2025（直接竞争者）、Adorno–Marinho 2022、Arrizabalaga–Ryll 2023（前沿）均未引。
8. **量纲与符号**：κ（s）、γa（s^1/2）的量纲设定、γa√κ 无量纲的说法需在 §5.3 前给出统一量纲表，否则审稿人易质疑 (5.6a) 中 Kd ⪰ ½(κ⁻¹+γa⁻²)I6 各项量纲一致性（½(κ⁻¹+γa⁻²) 的量纲确为 s⁻¹，与 Kd 一致——建议明写）。

---

## 7. "相对文献定位"示例表述（供 §1.2 贡献列表与 §5.3 使用，3–5 段中文）

> **相对 [P2]（Figueredo et al., Automatica 2021）**：[P2] 的 H∞ 证书建立在速度层运动学上——扰动 vw, vc 进入位姿微分方程，被认证输出是位姿误差 O(z̃), T(z̃)，且扰动模型限定为 L2 加性类（其乘法位姿不确定性也被转写为 twist 级加性项）。因此 [P2] 没有动力学接口：加速度层扰动（惯量失配、执行器误差、接触外力）没有入口，twist 误差没有 L2 界，偏差型（L∞）不确定性不满足其 L2 假设。本文把证书推进到动力学接口的加速度层：扰动进入 ξ̇ 方程（相对阶 2），被认证量是几何一致 twist 误差 eξ；更关键的是证书形态——[P2] 的增益条件 κO, κT 本身就是控制律 (12) 的增益（γ 与 κ 绑定、需在性能与能量间折中），而本文的 Schur 补判据 Kd ⪰ ½(κ⁻¹+γa⁻²)I6 是当且仅当的，且 κ, γa 是纯分析参数、不出现在控制律 (5.2) 中，可实现"事后认证不改控制律"；此外 (c-2) 的旋转/平移逐分量拆分在 [P2] 中不存在（其位姿误差输出混合 (rad)² 与 m² 量纲）。

> **相对 [Ch20] 与 Wang–Yu（UDQ 解算加速度控制）**：[Ch20] 是动力学层 DQ 控制的代表性工作，但其位姿反馈取螺旋对数 ln x̂e（引自 Wang & Yu 2013 的误差动力学），在大误差处对数映射奇异；其稳定性论断依赖"完美动力学补偿"假设，全文无扰动模型、无任何鲁棒证书（无 Lyapunov 构造、无 L2/L∞ 增益、无 UUB/ISS）。本文与 [Ch20] 共享"几何一致 twist 误差 + 解析前馈"的信息集（§6 数值等价 ≤0.05% 证实），差异在：(i) 误差体系由一次 HDQ 乘法同时生成位姿与 twist 误差（定理 1），位姿反馈取 A⊤-整形而非螺旋对数，规避对数奇异（定理 3(b) 的水平集论证成立，对 ln 反馈不成立）；(ii) 本文提供 [Ch20] 完全不具备的三类证书（精确耗散等式、H∞ 充要判据、L∞ RMS 稳态界）。

> **相对 Spong92 类鲁棒计算力矩**：经典计算力矩鲁棒控制（Spong 1992；综述 [Abd91], [Sag99]）在关节空间处理乘性失配，其小增益条件 α<1 与一致最终有界（UUB）是本领域基线。本文的推进：(i) 乘性/外生扰动显式分离 d=Θ(q)ufb+dex（(5.1f) 在 Kd 各向同性时退化为 Spong 条件 α<1，是其任务空间矩阵化推广）；(ii) 给出 λeff=λmin(Kd)−αλmax(Kd) 的有效阻尼折减公式与持续 L∞ 扰动下 twist 误差的 RMS 稳态界 D/λeff——Spong 类只给逐点 UUB 球，且其扰动随 q̈ref 缩放、无外生偏差型扰动的稳态标度律；(iii) 结果在任务空间位姿误差层（含旋转/平移静态刚度标度律 (5.9)），而非关节空间。

> **相对标准 LMI H∞ 框架（ZDG96, van der Schaft 1992）**：标准 H∞ 工具对非线性机器人需在平衡点线性化或 LPV/LFT 建模，所得 bounded-real/Riccati 判据对模型是充要、对原非线性系统只有局部近似（充分性），且丢失位姿误差的几何结构与旋转/平移量纲齐次性；本文的 Schur 补判据直接作用于非线性闭环误差动态的耗散不等式，是当且仅当的（对给定存储函数 V），并保留 A(x̃) 的几何信息。代价是本文证书只约束 eξ（twist 能量）且为水平集局部结论——这是与 LMI 框架"全状态线性界"的互补取舍。

> **相对 2025 年最新 DQ 动力学控制（Pham 2025 等）**：近期工作（如 Pham, IEEE 2025 的 UDQ 计算力矩鲁棒控制）已把 DQ 与计算力矩结合，但据可检索文献，尚无同时具备 (i) 三阶运动学接口（TODQ）、(ii) 一次乘法生成的几何一致误差体系、(iii) 精确耗散等式 + 充要 Schur 判据 + 证书参数不进入控制律、(iv) 旋转/平移量纲齐次拆分、(v) L∞ 偏差扰动 RMS 稳态界的组合。本文的增量是"证书簇"而非单一指标。

---

## 8. 应补充引用清单（5 条 + 理由 + 建议位置）

| # | 文献（准确出处） | 补充理由 | 建议位置 |
|---|---|---|---|
| 1 | X. Wang, C. Yu, *Systems & Control Letters* **62**(3) (2013) 225–233, "Unit dual quaternion-based feedback linearization tracking problem for attitude and position dynamics" | [Ch20] 稳定性论断的真正来源（对数反馈 + 双平衡点）；批评对数奇异与双平衡点必须有此锚点；也是 DQ 误差动力学谱系的根 | §1.1（三个结构性缺口处）、§4 定理 1/2 对比 |
| 2 | E.D. Sontag, Y. Wang, *Systems & Control Letters* **24**(5) (1995) 351–359；M. Malisoff, F. Mazenc, *Constructions of Strict Lyapunov Functions*, Springer, 2009 | strictification 路线的理论依据（附录 C.4 的 W=V+ϵe⊤zKpAeξ 正是其标准构造）；引用表明该路线成熟、未完成只是工作量而非方法缺陷 | §5.3 注记 2、附录 C.4、§7 局限 (i) |
| 3 | H.-L. Pham, "Robust Torque-Computed Control for a Robot Manipulator With Unit Dual Quaternion", IEEE Xplore 2025（文献号 10945815；卷/DOI 待核） | 最接近的直接竞争者（DQ+计算力矩+鲁棒），不引则"文献空白"声明不成立 | §1.1 文献综述句、§5 对比段 |
| 4 | B.V. Adorno, M.B. Marinho, *Control Engineering Practice* **118** (2022) 104709, "Kinematic screws and dual quaternion based motion controllers" | DQ 运动学控制的推广框架/谱系定位（[P2] 的后继主线） | §1.1 背景段 |
| 5 | J. Arrizabalaga, M. Ryll, "Pose-Following with Dual Quaternions", Proc. IEEE CDC 2023（arXiv:2308.09507） | 2023 年 DQ 位姿跟踪证书新进展，证明该方向竞争活跃；本文证书形态（twist 输出 vs 位姿输出）与其对比 | §1.1 前沿句、§5.3 对比注记 |
| （6，可选） | A.J. van der Schaft, *IEEE Trans. Automatic Control* **37**(6) (1992) 770–784 | 非线性 H∞ 的 Hamilton–Jacobi 框架，支撑"标准 H∞ 需线性化"定位 | §5.3 前、定位段（ZDG96 之旁） |

---

## 9. 来源 URL 附录

- [P2] ScienceDirect: https://www.sciencedirect.com/science/article/abs/pii/S000510982100337X ；arXiv 全文: https://ar5iv.labs.arxiv.org/html/1811.05436 ；zbMATH: https://zbmath.org/1478.93146
- [Ch20] IFAC-PapersOnLine 53-2 (2020) 8500–8505；HAL 版: https://dumas.ccsd.cnrs.fr/UNIV-BPCLERMONT/hal-03206513v1
- [P1] ScienceDirect: https://www.sciencedirect.com/science/article/abs/pii/S0094114X20300823 （DOI 10.1016/j.mechmachtheory.2020.103861）
- Wang & Yu 2013: zbMATH https://zbmath.org/6157681 ；ANU 记录 https://researchportalplus.anu.edu.au/en/publications/unit-dual-quaternion-based-feedback-linearization-tracking-proble
- Sontag & Wang 1995: Semantic Scholar https://www.semanticscholar.org/paper/32639e79b7e6bb580996aa6b084b91c7695e9f07 ；zbMATH https://zbmath.org/0877.93121
- Malisoff & Mazenc 2009: HAL 记录 https://in2p3.hal.science/INRIA/hal-01389866
- Pham 2025: https://ieeexplore.ieee.org/abstract/document/10945815 ；作者页 https://ieeexplore.ieee.org/author/37087811844
- Adorno & Marinho 2022: https://www.sciencedirect.com/science/article/abs/pii/S0967066122001605
- Arrizabalaga & Ryll 2023: arXiv https://arxiv.org/abs/2308.09507 ；IEEE https://ieeexplore.ieee.org/document/10383688 ；TUM 记录 https://portal.fis.tum.de/en/publications/pose-following-with-dual-quaternions
- van der Schaft 1992: IEEE Xplore（doi 10.1109/9.256352）

**备注（需父 agent 复核的两处引用细节）**：① Pham 2025 的卷/期/DOI 未能从公开检索确认（IEEE 文献号 10945815，作者 H.-L. Pham，Early Access）；② Adorno–Marinho CEP 的卷/文号（118 (2022) 104709）系据 PII S0967066122001605 推断，建议按原文复核。
