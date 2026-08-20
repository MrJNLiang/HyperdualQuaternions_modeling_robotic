# 协调员第一手核验笔记（供整合专家使用）

> 本文件是协调员（主编）在等待 LIT/THEO/DATA 专家期间亲自核验的事实清单，
> 整合最终修改方案时应与三份专家报告交叉引用。

## A. 论文现状关键事实

1. §6 仿真章节（texdocs/sections/sec6-simulation.tex，79 行）目前只讲了：
   - V1 静态刚度标度律反演一致性（tuned/fast 两档，1.93%；base 档反例 12.3 倍 / 35–38% 偏离）
   - V2 极限工况证书分化（C1≡C2 ≤0.05%；C3 噪声 +2.2%、高速 +2.0%）
   - V3 水平集余度（V_peak^tuned ≤ 0.494 vs c*=160，约 2.5 个数量级）
2. **§6 完全没有使用 H∞ 证书数据**：`TNDQ_sim/results/gamma_sweep.csv`（22 行，
   组 A/B/C 对比 γa 分析参数 vs 综合参数）在论文任何地方（含附录）都没有被引用。
   这是"理论优势最直接的可视化素材"，被闲置。
3. **§6 没有把定理 3(d) 的理论界与实测 RMS 直接对照**：metrics CSV 里有现成列
   `rms_bound_5_7`（= D/λeff 理论界）与 `rms_margin`（实测 vs 理论界余量），论文未用。
4. 定理 3(c) 的充要 Schur 判据在 §6 中没有任何数据验证（gamma_sweep 的 hinf_lhs/hinf_rhs
   正是为此设计，同样闲置）。
5. §1.2 贡献列表只有 4 条，第 3 条把 H∞ 与 ISS 混在一句话里，未突出：
   充要性、量纲齐次分量拆分、乘性/外生扰动分离、纯分析参数（κ,γa 不进控制律）、
   以及"12 维状态只用 6 维 twist 误差获得全部能量证书"这一结构性取舍。
6. §5.3 定理 3(d) 只给 RMS 界（注记 2 称因 V̇ 无 −‖ez‖² 项只能给 RMS 界）。
   协调员推导出：由 (5.7d) V̇ ≤ −λeff‖eξ‖² + D‖eξ‖ 与 Ωc 内 ‖eξ‖² = 2V − ez⊤Kp ez ≥ 2V − 2c，
   可得 V̇ ≤ −λeff V + λeff c + D²/(2λeff)，
   从而逐点界 ‖eξ(t)‖ ≤ √(2V(0)e^{−λeff t} + 2c + D²/λeff²)。
   **即 eξ 存在逐点最终界（实用 ISS 界），论文未给出——这是零新增仿真的理论增益点。**
   （THEO 专家将严格核验此推导。）

### A-2. 协调员数值验证结果（miniconda numpy，对既有 npz 数据，零新仿真）

对 tndq_internal_line_bias / line_l2 / setpoint_mismatch 三个 npz（含 t, e_z, e_xi, V, K_d, k_p, dt）：
- 由 e_z 重构 A(x̃)（η̃ 正分支），差分重建 d̂ = ėξ + Kd eξ + A⊤Kp ez；
- **精确耗散等式** V̇ = −eξ⊤Kd eξ + eξ⊤d̂ 逐时刻成立：max|V̇−rhs| 为 2.8e-3 / 9.8e-3 / 3.8e-2（差分误差量级）；
- **逐点界验证**：取 λeff=λmin(Kd)=8（内部对象 α=0）、D=‖d̂‖∞、c=½max(ez⊤Kp ez)，
  检查 V(t) ≤ V(0)e^{−λeff t} + (c + D²/2λeff²) 与 ‖eξ(t)‖ ≤ √(2·bound)：
  三个运行**零违反**（max violation = 0；‖eξ‖ 峰值 9.5e-2/1.9e-1/2.6e-1 均低于界 √(2c+D²/λeff²)）。
- 结论：**逐点最终界推论不仅在数学上成立，而且在现有数据上严格满足且非平凡（非空洞）**。
  这为"把定理 3(d) 从 RMS 界升级为 eξ 逐点实用 ISS 界 + RMS 界双结论"提供了数据背书，
  且无需重跑任何仿真。

## B. 数据核验结果（协调员用 stdlib 独立复核）

1. §6.3.2 表格数字复核（grasp_metrics_summary.csv，law=tndq，gains=tuned，phase=circle，
   condition=none/highspeed/noise，带载行）：
   - none（带载）: exi_rms = 0.0009398918 → 0.940×10⁻³ ✓
   - highspeed: 0.002313821 → 2.314×10⁻³ ✓
   - noise: 0.00316441 → 3.164×10⁻³ ✓
   论文表格数字与数据一致。
2. 定理 3(d) 理论界 vs 实测（law=tndq, gains=tuned, phase=circle-ss）：
   - 带载 none: 实测 exi_rms=9.40e-4，界 rms_bound_5_7=1.47e-1，margin≈10.6×
   - 带载 highspeed: 实测 2.31e-3，界 1.47e-1，margin≈10.5×
   - 带载 noise: 实测 3.16e-3，界 1.34e-1，margin≈9.5×
   - 带载 coarse-dt: 实测 9.64e-4，界 8.92e-2，margin≈8.3×
   - 空载 none: 实测 1.53e-4，界 4.95e-2，margin≈58.8×
   即实测 RMS 处处严格低于理论界 8–59 倍 —— 定理 3(d) 的"保守但可用的界"有直接数据支撑，
   应做成柱状图 + 理论界线（现有 sec6 图未做此对照）。
   注意：performance.py 的 iss_ultimate_bound 注释说明该界按 α→0 情形计算（乐观侧），
   论文/图中必须注明。
3. gamma_sweep.csv 复核（精确版，据 run_gamma_sweep.py 源码）：
   - 组 A（固定 tuned 增益 Kd=24I，κ=1 读法扫 γa）：cert_level = ½(1 + γa⁻²)
     （κ=params.KAPPA=1），cert_ok 在 γa=0.15（level 22.72）→ 0.14（level 26.01）处翻转；
     **证书可行边界 γa* = 1/√(2λmin(Kd)−1) = 1/√47 ≈ 0.1459**。全组闭环轨迹位不变
     （measured_l2=0.1466 恒定、误差逐行相同）——"γa 是分析参数、不进控制律"的直接证据。
     族内最紧读法（κ=γa²）：tuned 档认证增益 1/λmin(Kd)=1/24≈0.0417。
   - 组 B（综合模式回写增益 Kd=γa⁻²I、κ=γa²）：certified_l2_gain=1/λmin(Kd)=γa²；
     注意 measured_l2_gain=√(E_exi/E_d) 在小 γa 行（0.25/0.20/0.177）**超过**零初值认证增益
     （0.163>0.0625、0.198>0.040、0.223>0.031），但**含 2V(0) 项的完整不等式 (5.6)
     hinf_lhs≤hinf_rhs 在全部 22 行成立**；γa=0.177 行 sat_steps=5（饱和）。
     ⇒ 组 B 的诚实表述：完整耗散不等式 (5.6) 全部满足；零初值增益界被"非零初值 V(0)
     项 + 极小 γa 档的指令饱和"挤破——这恰好演示了 (5.6) 中 2V(0) 项的作用，写论文时
     要把这个机制讲清楚，而不是回避。
   - 组 C（C3 旧 DQ H∞ 律）：无认证列（cert_ok=0，κ=nan），γ 直接改增益 kO=√2/γO：
     measured_l2 随 γ 减小而**升高**（0.081→0.157），γ=0.177 时 sat_steps=5（饱和）。
     ⇒ C3 的 γ 是综合参数："可调不可证"，且过小 γ 的增益放大被饱和/桥接吃掉。
   - 结论：gamma_sweep 是"分析参数 vs 综合参数"理论优势的直接实验证据，强烈建议写入 §6
     （新小节，如 §6.4 或附录 D），不用重跑仿真。
4. 内部仿真数据（未在论文 §6 使用但可支撑 L2 vs L∞ 叙事）：
   - TNDQ_sim/results/tndq_internal_line_{none,l2,bias}.csv、setpoint_{none,l2,noise,bias,
     mismatch,large-error}.csv、cup-circle_{none,l2,bias,contact,highspeed}.csv；
   - TNDQ_sim/results/tndq_coppeliasim_cup-circle_l2.npz（CoppeliaSim + L2 注入扰动——
     §6 目前只测了偏差型负载扰动，L2 型注入的 CoppeliaSim 数据已存在但未用）。
   - tndq_tracking_*_table.txt 含逐时刻 V、c0/c1/c2（H∞ 证书约束 c0,c1,c2 的逐时刻核验列）。
5. 现有论文图（sec6_*.png/pdf）四张已覆盖 V1–V3；缺少：
   定理 3(d) 界对照图、H∞ 证书验证图、静态刚度标度律斜率图（双对数）。

## C. 项目内既有分析文档（整合时引用）

- kimi分析文档.md：§5.3(c) Schur 补 vs Young 放缩的严谨性讨论（结论：Schur 补路径更少保守，
  已是论文采纳的路径）。
- docs/TNDQ论文_仿真验证章节.md、docx/S3抓杯实验结果分析_TNDQ增益整定与C3对比.md、
  docx/TNDQ动力学控制对比分析与实验设计方案.md、docx/TNDQ论文理论与TNDQ_sim代码对应_伪代码详解.md。
- **docs/TNDQ论文删除的内容.md（关键发现，整合时必须引用）**：
  1. 早期草稿曾有 **§6.7 γ 扫描协议（作为"后续实验设计"）**，如今该协议已被实现
     （TNDQ_sim/experiments/run_gamma_sweep.py + results/gamma_sweep.csv，组 A/B/C 与
     早期设计的 A/B/C 组一一对应），但**现行论文初稿完全没有引用这些结果**。
     ⇒ 修改方案应明确建议：把已实现的 γ 扫描结果作为新小节写回 §6（或附录），
     零新仿真、纯"数据再利用"，完全符合"不改模拟"约束。
  2. 早期草稿对定理 3(d) RMS 界的保守性有逐项归因分析：由 (5.9) 反演
     ‖dex‖≈√(0.389²+0.683²)=0.786，α→0 时界=0.786/24=3.27e-2 vs 实测 9.4e-4，
     比值 34.8（保守约 1.54 个数量级）；保守性来源：(5.7d) 中 ‖A‖₂≤1+‖T‖ 与
     ‖A₁₁‖₂=½ 的最坏方向取值、d 与 eξ 在圆周段近似正交、Young 等号条件远未达到。
     注意与 metrics CSV 的 rms_bound_5_7=1.47e-1（用重建 d̂_inf=3.54）口径不同，
     论文引用时需注明口径。
  3. 早期草稿区分 H∞ 证书参数两种读法：读法 A（可行性判定，κ=1、γa=0.5 ⇒
     Kd 需 ≥2.5，认证增益 γa√κ=0.5）与读法 B（族内最紧，κ=γa²、λmin(Kd)≥γa⁻² 取等，
     tuned 档 γa=1/√24≈0.204，认证 L2 增益=1/λmin(Kd)≈0.042）；两种读法不影响闭环轨迹
     （纯分析参数）。§6 若要引用 gamma_sweep，须先声明用哪种读法。
  4. 早期草稿的诚实边界清单（建议保留/回归）：空载 C1 不优于 C3（+1.98%）、单次运行
     无统计显著性、偏差型扰动不在 L2 前提内故 H∞ 紧性未获检验、runtime 含 RPC 不可比、
     base 档反演不一致不是理论失效、C1 vs C2 主张证书增益而非精度。
- LIT 专家已把三篇参考文献提取为 docx/P2_figueredo_automatica2021.txt、
  docx/Ch20_chandra_ifac2020.txt、docx/P1_cohen_shoham_mmt2020.txt（协调员已抽查确认：
  [P2] 是运动学层 L2 速度级扰动 H∞，γ 是综合参数直接定增益 κO=√2/γO；[Ch20] 是
  resolved-acceleration，无耗散证书）。

## D. 最终交付物约定
- 最终文档：docx/TNDQ论文修改方案_理论优势与仿真呈现.md（中文 Markdown）。
- 要求包含：①理论定位与贡献表述修改（§1.2/§5.3）；②§5.3 文本级改写建议
  （含逐点界新推论、证书对比表、增益整定反演公式、H∞ 术语精确化、ISS 表述）；
  ③§6 修改建议（利用 gamma_sweep、RMS 界对照、刚度标度律图；新增小节方案）；
  ④图件与数据表述建议（逐图清单+图注草稿）；⑤不动仿真的约束声明与可复现性说明；
  ⑥局限与审稿风险自检表。

## E. 最终文档建议结构（协调员拟，整合专家可调整）

0. 执行摘要（一段话 + 10 条最高优先级改动清单）
1. 理论价值与工程价值定位总述
   1.1 本文证书体系的独特性（与 LIT 证书矩阵挂钩）
   1.2 相对 [P2]/[Ch20]/Spong92/标准 LMI H∞ 的差异一句话对照
2. §1 引言/贡献修改建议（贡献条目改写、定位句、应补引用）
3. §5.3 主定理修改建议
   3.1 开篇"三缺口闭合"叙事改写
   3.2 定理 3(b)：水平集 + 无扰结论的表述强化（含 η̃ 分支工程处理）
   3.3 定理 3(c)：充要性/量纲齐次/纯分析参数卖点；H∞ 术语精确化；
       设计反演公式（能量预算→Kd 下界）；κ-γa 族与附录 C.3 关系
   3.4 定理 3(d)：新增"eξ 逐点实用 ISS 界"推论（推导 + 数值验证背书）；
       RMS 界保留；小增益条件定位；D/λeff 的工程解读
   3.5 边界说明升级（12 维状态/6 维证书的结构性取舍叙事）
   3.6 新增证书对比表（放 §5.3 末或 §5.4）
   3.7 strictification（附录 C.4）完成方案要点（可选/未来工作）
4. §6 仿真章节修改建议（不改仿真）
   4.1 叙事重构：从"验证三预言"升级为"定理→预言→数据→证书"闭环
   4.2 新增内容：H∞ 证书验证小节（gamma_sweep 组 A/B/C 的分析参数 vs 综合参数故事）
   4.3 新增内容：定理 3(d) 理论界 vs 实测 RMS 对照（rms_bound_5_7 柱状图）
   4.4 V1/V2/V3 现有小节的数据表述强化（含 4 阶段伪重复的统计诚实表述）
   4.5 静态刚度标度律双对数图建议
   4.6 局限小节改写
5. 图件与数据表述建议（逐图清单：图题、坐标轴、标注定理编号、图注草稿）
6. 执行约束与可复现性（声明：仅新增只读分析脚本，不改模拟/实验代码）
7. 审稿风险自检表（LIT/THEO/DATA 汇总的攻击点与对策）
8. 附录：专家团队分工与报告索引（LIT/THEO/DATA 报告文件路径）

## F. 专家进度与协作记录

- **LIT（350909ce）已完成**：docx/agent_LIT_report.md（167 行）。要点：9 维证书矩阵
  （本文 7/9 "是"，5 项独占：精确耗散等式、量纲齐次拆分、L∞ RMS 稳态界、刚度标度律、
  证书参数不进控制律；唯一"否"=全状态 ISS）；最重攻击点=标题"ISS"与正文冲突；
  补充引用 Wang&Yu 2013 / Sontag&Wang 1995 / Malisoff&Mazenc 2009 / Pham 2025 /
  Adorno&Marinho 2022 / Arrizabalaga&Ryll 2023 / van der Schaft 1992；
  5 段中文定位表述（报告 §7，可直接进论文）。
- 协调员已复核两处引用细节：Pham 2025 确认存在（IEEE 文献号 10945815，Early Access，
  https://ieeexplore.ieee.org/abstract/document/10945815）；Adorno&Marinho CEP 118 (2022)
  卷期存在（ScienceDirect CEP vol.118 2022），文号 104709 建议投稿前按原文复核。
- LIT→THEO 校准消息已发送（ISS 措辞风险、Schur"充要"限定、[P2] Schur 重叠、Spong 继承、
  补引清单），THEO 将在其报告追加"与文献结论的对齐"一节。
- THEO（a62dc625）、DATA（a108dbcc）仍在运行；DATA 已收到 numpy 环境提示
  （/home/ljn/miniconda3/bin/python，MPLCONFIGDIR=/tmp/mpl）与 read_image 不可用提示。
