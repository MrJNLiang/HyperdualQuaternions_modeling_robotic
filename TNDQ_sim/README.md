# TNDQ_sim — TNDQ/HDQ 几何一致控制仿真（内部数值 + CoppeliaSim 双后端）

基于论文《TNDQ论文初稿_运动学重构_误差体系与控制律》（`docs/` 目录）的完整 Python 实现：
TNDQ/HDQ/DQ 代数结构、7R 串联机械臂（KUKA LBR4+/LWR4+ 构型）TNDQ 链式正运动学、
几何一致误差体系（定理 1/2）、计算力矩控制律（式 5.2）与 H∞/ISS 性能保证（定理 3）；
并按 `docx/KUKALBR4p场景_定点与圆周扰动对比实验设计.md`（下称"场景篇"）与
`docx/TNDQ动力学控制对比分析与实验设计方案.md`（下称"总方案"）完成了
**CoppeliaSim（KUKALBR4+_sim.ttt 场景）力矩级对接**与 **E1–E8 扰动/γ 扫描实验矩阵**。

**默认纯数值输出（npz + CSV）**；仅 S3 实验的 `--plot` 选项可选生成对比图
（需 matplotlib，见下）。

## 目录结构

```
TNDQ_sim/
├── core/                        # FK 计算层
│   ├── dq_algebra.py            #   DQ 层：四元数/对偶四元数运算（§2, 式 2.1/2.2）
│   ├── tndq_algebra.py          #   TNDQ/HDQ 代数：式 3.1/3.2/2.3，截断塔（表 1），
│   │                            #   twist 公式 (3.5)，约束族 (3.8)，重投影（§3.4）
│   └── kinematics.py            #   TNDQ 串联链 FK：连乘法则 (3.4)，附录 B.1 关节因子，
│                                #   J̇q̇ 免构造读出，几何雅可比
├── control/                     # 反馈控制律层
│   ├── error_system.py          #   定理 1（式 4.1–4.4）+ 定理 2（式 4.5，A 矩阵）
│   ├── control_law.py           #   式 (5.2) 几何一致计算力矩律 + 引理 1 前馈
│   └── performance.py           #   定理 3：存储函数 V、(5.6a)/(5.6b) 增益条件、
│                                #   最紧 L2 增益 1/λmin(K_d)、ISS 极限球 (5.7)
├── simdata/
│   ├── trajectory_generator.py  #   期望轨迹的 TNDQ 表示（式 3.3a）：直线/圆/
│   │                            #   定点（S1）/绕杯圆周（S2）/趋近段/分段组合
│   └── input_simulation.py      #   L2 扰动、L∞ 偏差扰动、测量噪声（E2/E6）
├── config/
│   ├── params.py                #   DH 表、增益、场景/扰动/安全参数（全部可调项）
│   └── lbr4_dynamics.py         #   Gaz [11] LWR4+ 名义 RNEA 动力学（M̂/Ĉ/ĝ +
│                                #   电机折算惯量 B、限位/力矩上限、E3 失配开关）
├── interfaces/
│   └── coppeliasim_interface.py #   CoppeliaSim ZMQ Remote API 完整实现（中文注释）
├── output/
│   └── data_logger.py           #   npz + CSV + 定宽文本表 + 数值摘要
├── tests/
│   └── test_math_properties.py  #   14 项数学性质单元测试（全部通过，含忠实
│                                #   C2 [Ch20] 的符号/闭环 oracle T10–T12）
├── experiments/
│   ├── run_grasp_circle.py      #   S3 抓取-搬运实验：抓杯 + 带载圆周 +
│   │                            #   空载/带载 × 控制律（C1/C2/C2-abl/C3）全交叉
│   │                            #   对比 + 指标汇总 CSV（grasp_metrics_summary.csv）
│   └── run_gamma_sweep.py       #   γ 影响实验（E8）：新理论 γ_a 证书/综合双通道
│                                #   vs 旧理论 γ_O/γ_T 综合参数，图 + CSV
├── run_simulation.py            #   闭环仿真主程序（双后端、S1/S2、E1–E7、安全机制）
└── README.md
```

## 环境要求

- Python ≥ 3.8，核心仅依赖 `numpy`（测试可选 `pytest`）
- CoppeliaSim 对接（可选）：CoppeliaSim ≥ 4.4 + ZMQ Remote API 客户端
- S3 对比出图（可选）：`matplotlib`（仅 `run_grasp_circle.py --plot` 需要）

```bash
pip install numpy
pip install coppeliasim-zmqremoteapi-client   # 仅 --backend coppeliasim 需要
pip install matplotlib                        # 仅 S3 --plot 需要
```

## 运行方法（TNDQ_sim/ 目录下）

### 1. 单元测试

```bash
python3 -m tests.test_math_properties     # 11/11 通过
```

### 2. 内部后端闭环仿真

```bash
# 原版回归（加速度级理想对象，式 5.1）
python3 run_simulation.py                          # line 标称
python3 run_simulation.py --disturbance l2         # H∞ 实验（定理 3(c)）
python3 run_simulation.py --disturbance bias       # ISS 实验（定理 3(d)）

# 力矩级刚体对象（RNEA 正动力学，Gaz [11] 名义模型）+ 场景任务
python3 run_simulation.py --plant torque --scenario setpoint                  # S1 定点（E1）
python3 run_simulation.py --plant torque --scenario setpoint  --condition l2  # S1+E2
python3 run_simulation.py --plant torque --scenario cup-circle --condition highspeed  # S2+E5
python3 run_simulation.py --plant torque --scenario setpoint \
        --condition large-error --t-go 0 --t-end 10                           # E4 纯调节
python3 run_simulation.py --plant torque --scenario cup-circle \
        --condition contact --t-end 12                                        # E7 接触
```

### 3. CoppeliaSim 后端（KUKALBR4+_sim.ttt 场景）

```bash
# 先在 CoppeliaSim 中打开 KUKALBR4+_sim.ttt（无需手动点播放，程序自动 start）
python3 run_simulation.py --backend coppeliasim --scenario setpoint
python3 run_simulation.py --backend coppeliasim --scenario cup-circle --condition l2
```

### 4. S3 抓取-搬运实验（experiments/run_grasp_circle.py，仅 CoppeliaSim）

抓杯 → 提升 → 横移过渡 → 带载圆周的完整物理交互流程，并在**完全相同的
实验环境**（同轨迹/同力矩出口/同安全预算/同监控）下做两个维度的交叉对比：

- **负载维** `--mode noload|load`：同一条轨迹，唯一差别是是否刚性附着杯子
  （力传感器焊接 + 杯质量改写为 CUP_LOAD_MASS=0.25 kg）；控制器名义模型
  不含杯 → 负载即 ΔM/Δg 模型失配扰动，由定理 3(c)/(d) H∞/ISS 证书兜底。
- **控制律维** `--law tndq|dq-chandra|dq-ctc|dq-hinf`（总方案 §4/§5.2 同台对比）：
  - `tndq` = **C1** 几何一致计算力矩律（式 5.2，本文新理论）；
  - `dq-chandra` = **C2** 忠实 [Ch20] resolved-acceleration 律（原文式
    (32)–(35) + 式 (2) 逐项移植）：twist 误差取经伴随搬运的差
    ω_e = Ad(x̃)ξ_d − ξ（与定理 1 的 −e_ξ 同一），位姿反馈取螺旋对数
    −K_P·vec₆(2 ln x̃)（本项目右不变约定下的忠实换算，见
    `control/control_law.py::dq_chandra2020_law` 符号注释），ξ̇_d 与
    J̇q̇ 均为解析量（无差分）。公平性：CH20_K_V=24I、CH20_K_P=80I
    （params.py CH20_* 参数节），线性化 ℓ̈+K_vℓ̇+K_Pℓ=0 无折减，与
    C1(tuned)/C3 逐通道恒等（极点 {−4,−20}、DC 刚度 80）；
  - `dq-ctc` = **C2-abl** 朴素 twist 差消融基线（**非文献律**）：同一 DQ
    位姿误差 (O, T)，但速度层用朴素 twist 差 ξ_d−ξ（§4.1 伪项）、位姿
    反馈不经 Aᵀ 整形（无定理 3 证书）、前馈 ξ̇_d 与 J̇q̇ 均取数值差分
    （一拍滞后 + 差分噪声），接入**同一力矩出口**。注意：[Ch20] 原文式
    (32) 与 [P2] 前馈均含 Ad 搬运，朴素坐标差不对应任何已发表理论，本档
    仅用于消融 C1 结构属性的代价量化。公平性：DQC_K_D=24I、
    DQC_K_P=diag(160,80)（params.py DQC_* 参数节，1/2 折减配平），使其
    线性化 d→e 传递函数与 C1(tuned)/C3 逐通道恒等（极点 {−4,−20}、
    DC 刚度 80）；
  - `dq-hinf` = **C3** 一阶 DQ H∞ 运动学律（`hdq_hinf_coppeliasim` 原实现
    逐行移植，“之前理论”）：任务速度 = [k_O·O; −k_T·T] + vec₆(x̃ ξ_d x̃*)，
    经同一阻尼伪逆得 q̇_cmd，再由内环伺服 q̈_ref = Δq̇_cmd/dt +
    K_servo(q̇_cmd−q̇) 接入**同一力矩出口** τ = M̂q̈_ref + Ĉq̇ + ĝ。
    公平性：基线增益 k_T=4、k_O=8、K_servo=20（params.py DQH_* 参数节）。
- **增益维** `--gains base|tuned|fast`（仅对 `--law tndq` 有效，见 §4.1）：
  `base` 是出厂增益，只对齐了 C1/C3 的**主导极点**、没有对齐级联第二极点
  带来的**直流刚度**，因此不构成同预算对比；`tuned` 使 C1 与各基线（C2/
  C2-abl/C3）的线性化 d→e 传递函数**逐通道恒等**，才是公平对比点；
  `fast` 是同一安全约束下的可达上限档。
- **敏感条件维** `--condition none|highspeed|fast-transit|noise|coarse-dt`
  （层 3 结构敏感域对比）：准静态标准场景下各律同预算必然趋同（误差被
  DC 刚度垄断），敏感条件把结构差异（解析 vs 差分前馈、二阶通道有无、
  Aᵀ 整形）推到线性化失效/高频域使其可观测。全部只改时间/测量/控制
  周期参数，**路标几何与场景完全不变**（IK 限位验证仍有效、无穿模风险）：
  - `highspeed`：圆周 ω 1.0→2.5 rad/s（前馈/向心项 ×6.25，暴露解析 vs
    差分前馈差距）；
  - `fast-transit`：lift/retreat/transit/descend2 时长 ×0.5（快相位前馈
    精度；descend/hold 不动，附着时刻力学不变）；
  - `noise`：编码器级测量噪声 σ_q=5e-5 rad、σ_q̇=1e-3 rad/s（控制器只见
    带噪测量、安全检查用真值；差分放大 ∝1/dt）；
  - `coarse-dt`：控制周期 5→15 ms（非控制步 ZOH 保持力矩；差分前馈一拍
    滞后 ×3，解析前馈不受影响）。

  公平性：敏感条件下 C1 建议配 `--gains tuned`（与各基线同预算，残余
  差异纯属结构）。参数：params.py `GRASP_FAST_PHASE_SCALE`/
  `GRASP_CTRL_DECIM`/`CIRCLE_OMEGA_FAST`/`NOISE_SIGMA_*`。

```bash
# 先在 CoppeliaSim 中打开 KUKALBR4+_sim.ttt，以下均在 TNDQ_sim/ 目录下
python3 experiments/run_grasp_circle.py --mode noload               # C1 空载（base）
python3 experiments/run_grasp_circle.py --mode load                 # C1 带载（base）
python3 experiments/run_grasp_circle.py --gains tuned --mode noload # C1 空载（整定后）
python3 experiments/run_grasp_circle.py --gains tuned --mode load   # C1 带载（整定后）
python3 experiments/run_grasp_circle.py --gains fast  --mode load   # C1 带载（上限档）
python3 experiments/run_grasp_circle.py --law dq-chandra --mode noload # C2 空载（忠实 [Ch20]）
python3 experiments/run_grasp_circle.py --law dq-chandra --mode load   # C2 带载（忠实 [Ch20]）
python3 experiments/run_grasp_circle.py --law dq-ctc  --mode noload # C2-abl 空载
python3 experiments/run_grasp_circle.py --law dq-ctc  --mode load   # C2-abl 带载
python3 experiments/run_grasp_circle.py --law dq-hinf --mode noload # C3 空载
python3 experiments/run_grasp_circle.py --law dq-hinf --mode load   # C3 带载
python3 experiments/run_grasp_circle.py --compare-only              # 只重印各律对比 + 导出指标 CSV
python3 experiments/run_grasp_circle.py --compare-only --plot       # 对比 + 出图

# 敏感条件（每个条件跑齐各律；C1 用 tuned 保同预算公平），以 highspeed 为例：
python3 experiments/run_grasp_circle.py --mode load --gains tuned --condition highspeed
python3 experiments/run_grasp_circle.py --mode load --law dq-chandra --condition highspeed
python3 experiments/run_grasp_circle.py --mode load --law dq-ctc  --condition highspeed
python3 experiments/run_grasp_circle.py --mode load --law dq-hinf --condition highspeed
#（同理替换为 fast-transit / noise / coarse-dt；跑齐后 --compare-only 自动
# 输出各条件的对比表，--plot 另存条件分组柱状图）
```

相位时间线（路标经 IK 扫描验证，params.py S3 参数节）：descend
（指尖从杯口开口垂直伸入 30 mm，内撑式，全程走杯内空气）→ hold
（静止保持段中点 t=2.5 s 刚性附着）→ lift → retreat → transit →
descend2 → circle（R=0.06 m，ramp 2 s + 稳态 >1.5 圈，总时长 22.5 s）。

每记录步同时采样物理交互量：附着点力旋量（readForceSensor = 抓握力
直接测量）、杯子接触合力（getContactInfo）、机器人↔环境最小净距
（checkDistance，独立的“无穿模”审计量：零净距且邻域无接触力 = 幽灵
穿透 FAIL）。npz 齐备后自动打印：分相位（descend/hold/lift/
retreat/transit/descend2/circle/circle-ss）的 |T|/|O|/|e_ξ|/|τ| RMS、
抓握力均值、V 收敛特性（统一 base 权重折算，跨增益组可比）、饱和/治理
步数——空载↔带载、新律↔基线（C1/C2/C2-abl/C3）、整定前↔整定后三张交叉
对比表；`--plot` 另存 `results/grasp_compare_{errors,effort,interaction,
lyapunov}.png` 与敏感条件分组柱状图 `grasp_compare_conditions_{noload,
load}.png`（condition 组 × 各律柱，C1 优先取 tuned）。轨迹级输出：
`results/grasp_circle_[chandra_|dqhinf_|dqctc_|tuned_|fast_]{noload|load}
[_hspeed|_ftrans|_noise|_cdt].npz/.csv`（敏感条件加后缀，`none` 无后缀、
完全兼容已有结果文件）；指标级输出：每次对比后自动导出
`results/grasp_metrics_summary.csv`（law × gains × mode × condition ×
phase 全交叉，列含 T/O/e_ξ/τ 的 RMS 与峰值、抓握力均值/峰值、V 稳态/
峰值/回落时间、饱和/治理步数、单步耗时，供定量分析直接入表）。

### 4.1 C1 增益整定（`control/gain_design.py` + `experiments/tune_tndq_gains.py`）

出厂增益（`K_D=8I`、标量 `K_P=16`）**不是最优**。近单位误差处 A→
diag(−I/2, I)，(5.2) 的误差体系解耦为两条二阶通道

    旋转：  Ö + K_ω Ȯ + (p_O/4) O = −d_ω/2
    平移：  T̈ + K_v Ṫ +   p_T  T = +d_v

标量 `k_p`（p_O=p_T）因此让**旋转刚度只有平移的 1/4**：旋转极点退化为
{−0.54, −7.46}（ts=7.5 s，长于 lift/retreat/transit 各相位），直流误差
增益 0.125 —— 是 C3（外环×内环级联 k·K_servo=80）的 20 倍。这正是先前
“C3 稳态更优”的真实原因：早先的公平性只对齐了**主导极点**，没有对齐
级联第二极点带来的**直流刚度**。

把标量推广为对称正定矩阵 `K_p` 后定理 3 逐字成立（反馈取 −AᵀK_p e_z，
K_p 在 Aᵀ 内侧，V=½‖e_ξ‖²+½e_zᵀK_p e_z 的交叉项仍逐点相消；
`tune_tndq_gains.py` 阶段 0 用完整非线性 A(x̃) 随机抽样核验，残差
8.7e−16）。于是可在四个约束下做约束优化：λmin(K_d)≥2.5 (5.6a)、
max|p|·dt≤0.15（200 Hz 显式积分余量）、ζ≥1（接触任务禁过冲）、
指令峰值代理 ≤ QDDOT_MAX。`params.GAIN_SETS`：

| 组 | K_ω / K_v | p_O / p_T | 极点 | DC 增益 O/T | λmin(K_d) | 认证 L2 | 代价 J |
|---|---|---|---|---|---|---|---|
| `base` | 8 / 8 | 16 / 16 | {−0.54,−7.46} / {−4,−4} | 0.125 / 0.0625 | 8 | 0.125 | 16.26 |
| `tuned` | 24 / 24 | 320 / 80 | {−4,−20} 两通道 | 0.00625 / 0.0125 | 24 | 0.042 | 1.75 |
| `fast` | 36 / 36 | 720 / 180 | {−6,−30} 两通道 | 0.00278 / 0.00556 | 36 | 0.028 | 1.30 |

`tuned` 使 C1 两通道与 C3 的线性化 d→e 传递函数**逐通道恒等**，是真正的
同预算对比点；`fast` 是同一安全约束下的可达上限（J 最小可行点，但恰在
|p|dt=0.15 边界、指令峰值预算翻倍）。完整整定报告（含敏感性与证书表）：

```bash
python3 experiments/tune_tndq_gains.py          # 五阶段报告，无需 CoppeliaSim
python3 experiments/tune_tndq_gains.py --set tuned
python3 -m control.gain_design                  # 通道诊断自检
```

**S3 带载实测（圆周稳态段，与 C3 同环境）**——整定模型的外推被实测确认：

| 增益组 | \|T\|rms [m] | \|O\|rms | 预测/C3 | 实测/C3 | \|τ\|rms [N·m] | 饱和/治理 |
|---|---|---|---|---|---|---|
| `base` | 1.58e−2 | 5.26e−2 | 5.0 / 20.0 | 3.25 / 12.3 | 21.1 | 0 / 0 |
| `tuned` | 4.86e−3 | 4.27e−3 | 1.000 / 1.000 | 0.999 / 0.999 | 19.2 | 0 / 0 |
| `fast` | 2.20e−3 | 1.93e−3 | 0.444 / 0.444 | 0.453 / 0.453 | 19.2 | 0 / 0 |
| C3 基线 | 4.86e−3 | 4.27e−3 | — | 1 | 19.2 | 0 / 0 |

即：`tuned` 与 C3 在全部相位逐项落在 1.00–1.02 倍以内（增益分配成分被
彻底消除，剩余差异只反映结构）；`fast` 在 retreat/transit/circle 相位
把误差降到 C3 的 0.45 倍且力矩持平，代价是附着瞬态抓握力升高
（hold 段 21.6 N vs tuned 13.7 N vs base 9.2 N）——这是线性模型看不到的
物理代价，故 `tuned` 作为论文对比点、`fast` 作为上限档。

### 5. γ 影响实验（experiments/run_gamma_sweep.py，E8，无需 CoppeliaSim）

论文 §5.3 注记（γ_a 的角色）/附录 C.3 γ-κ 设计规则的实验化：新理论的
γ_a 是**分析参数**（只进证书 (5.6a)，不进控制律），旧理论的 γ_O/γ_T 是
**综合参数**（k=√2/γ 直接定增益）——三组实验在同一内部加速度级对象
（式 5.1）+ line 轨迹 + 同一 L2 扰动（seed=0）下把两者的差别做成可观测判别：

- **A 组（证书扫描）**：固定 tuned 增益（K_d=24I）扫 γ_a。实测：全部 γ_a
  行的误差逐位相同（T_rms=7.031e-4、O_rms=6.018e-4、meas_L2=0.1466），
  只有证书可判定性变化（γ_a=0.14 时 level=26.0 > λ_min=24，证书 FAIL 但
  行为不变）——“γ_a 不进控制律”的直接实验证据。
- **B 组（综合模式）**：γ-κ 规则回写增益：κ=γ_a²、K_d=γ_a⁻²I（(5.6a)
  取等号，认证能量增益=γ_a²）、K_p 临界阻尼。实测：稳态误差随 γ_a 单调
  下降（γ_a=1→0.25：T_rms 1.80e-1→9.46e-4），完整不等式 (5.6)（含 2V(0)
  项）逐点核验通过；代价是指令峰值随 γ_a⁻² 增长，screen() 四约束在
  γ_a≤0.2 处 FAIL（qdd 峰值 23→38 迫近 QDDOT_MAX=40），给出可达 γ_a 下界。
- **C 组（旧理论对照）**：复刻旧 H∞ 论文的 γ 实验（γ_O=γ_T=γ 扫描，
  kO=kT=√2/γ，经 K_servo=20 桥接）。实测：趋势与 B 组同构（γ=1→0.177：
  T_rms 1.02e-3→3.74e-4），但力矩接口下无证书可对照（“可调不可证”），
  且 γ 单参数锁死阻尼/刚度结构（γ=0.177 时已饱和 5 步）。

```bash
python3 experiments/run_gamma_sweep.py                 # A/B/C 三组，图 + CSV
python3 experiments/run_gamma_sweep.py --t-end 8 --no-plot
```

输出：`results/gamma_sweep.csv`（逐 γ 点的证书可行性/认证与实测 L2 增益/
稳态误差/能量/饱和计数 17 列）+ `results/gamma_sweep.png`（三面板：A 组
不变性与证书边界、B/C 组 L2 增益 vs γ、B/C 组稳态误差 vs γ）+ 终端表。
注：meas_L2=√(E_e/E_d) 未扣初始能量 V(0)，小 γ_a 时由初始瞬态主导而反弹，
非证书违例——判据是含 2V(0) 的完整 (5.6)（CSV 的 hinf_lhs ≤ hinf_rhs 列）。

### 命令行参数

| 参数 | 取值 | 说明 |
|---|---|---|
| `--backend` | `internal` / `coppeliasim` | 被控对象：内部积分器 / CoppeliaSim 动力学引擎 |
| `--plant` | `accel` / `torque` | internal 后端层级：式 (5.1) 理想加速度级 / RNEA 力矩级（coppeliasim 恒为力矩级） |
| `--scenario` | `line` `circle` `setpoint` `cup-circle` | 前两者为原版回归轨迹；后两者为场景篇 S1（杯口定点）/S2（绕杯圆周） |
| `--condition` | `none` `l2` `bias` `mismatch` `large-error` `highspeed` `noise` `contact` | 实验条件 E1–E7（见下） |
| `--t-end` / `--t-go` | 秒 | 时长 / S1-S2 平滑趋近段时长（`--t-go 0` = 纯调节压力测试） |
| `--trajectory` / `--disturbance` | | [兼容] 原版命令行的别名 |

输出：`results/tndq_{backend}_{scenario}_{condition}.npz`（全部原始数组）与同名 `.csv`。
CSV 列：`t, pos_err, ori_err_geodesic, e_xi_norm, e_z_O_norm, e_z_T_norm,
qddot_ref_norm, tau_norm, V, c0, c1, c2, runtime`——时间戳、‖p−p_d‖、
姿态测地距离 2·arccos|⟨r,r_d⟩|、‖τ‖、单步计算时间与约束残差 (3.8) 一应俱全。

## CoppeliaSim 场景设置（KUKALBR4+_sim.ttt，场景篇 §2）

| 场景对象 | 角色 |
|---|---|
| KUKA LBR4+ 机械臂（7R） | 被控对象；引擎负责真实动力学积分（Bullet/Newton，1 ms 物理子步） |
| **杯子** | 任务几何锚点：S1 定点目标 = 杯口上方 0.10 m 预抓取位姿（末端 z 轴竖直向下）；S2 圆心参照 = 杯口上方 0.15 m、半径 0.12 m 水平圆；E7 的"末端突加负载"来源（`attach_cup_to_tip`） |
| **椅子** | 工作空间静态障碍；E7 接触扰动来源（椅背擦碰，`sim.getContactInfo` 冲量监测） |
| 地板/桌面 | 静态环境，不参与控制回路 |

对接技术要点（`interfaces/coppeliasim_interface.py`，全部有中文注释与理论出处）：

- **同步步进**：`sim.setStepping(True)`，控制端每发一次指令步进一拍，杜绝时序竞态；
  控制周期取引擎 `sim.getSimulationTimeStep()`（建议场景设 2 ms ≈ 500 Hz，物理子步 1 ms）。
- **力矩模式**：`sim.setJointTargetForce(h, τ)` + 大速度目标（速控环饱和 → 纯力矩输出），
  即场景篇 §3 的名义 computed torque 出口 τ = M̂q̈_ref + Ĉq̇ + ĝ。
- **四元数顺序**：CoppeliaSim `getObjectQuaternion` 返回 `[x,y,z,w]`，
  内部 DQ 约定 `[w,x,y,z]`——接口层完成转换（`read_tip_pose_dq`）。
- **FK 对齐诊断**（场景篇 §1.2 第 9 项）：连接后对比 TNDQ FK 与引擎末端位姿，
  |Δp| < 1 mm、Δθ < 0.1° 才继续；不一致即提示 DH/POE 参数或坐标系错位。
- **关节路径探测**：依次尝试 `/lbr4p_joint_i`、`/LBR4p/jointi`、`/LBR4p/LBR4p_jointi`
  三组命名；`print_scene_inventory` 可存档场景对象清单（里程碑 0）。
- **错误处理**：连接失败抛 `CoppeliaSimConnectionError`；主循环 try/except/finally
  确保力矩清零 + 停止仿真 + 已采数据落盘；关节超限/奇异监控见下文安全机制。

## 机械臂参数（DH 法；Gaz [11] 名义动力学）

运动学采用**标准 DH 参数法**（附录 B.1 关节因子 A_i = Rz(θ)Tz(d)Tx(a)Rx(α)），
`config/params.py::KUKA_LBR4_DH`：

| i | a [m] | α [rad] | d [m] | θ₀ |
|---|---|---|---|---|
| 1 | 0 | +π/2 | 0.340 | 0 |
| 2 | 0 | −π/2 | 0 | 0 |
| 3 | 0 | −π/2 | 0.400 | 0 |
| 4 | 0 | +π/2 | 0 | 0 |
| 5 | 0 | +π/2 | 0.400 | 0 |
| 6 | 0 | −π/2 | 0 | 0 |
| 7 | 0 | 0 | 0.126 | 0 |

POE（旋量）参数法留作接口扩展位（params.py TODO）：DH 用于与 `core/kinematics.py`
的 TNDQ 关节因子严格一致；若场景模型采用 POE 描述，须先经 FK 对齐诊断校核。

动力学（`config/lbr4_dynamics.py`）：递归牛顿-欧拉（RNEA，Siciliano 式 7.107–7.114），
惯性参数按 Gaz–Flacco–De Luca LWR4+ 辨识模型（总方案文献 [11]）量级设定的**名义表**
（质量 2.7…0.3 kg 递减、连杆惯量、电机转子折算惯量 B = diag(1.2…0.15) kg·m²——
[11] 模型本身含电机惯量项；缺它会使末端接触力在小惯量腕关节产生非物理加速度）。
关节限位 ±170°/±120° 交替、额定速度 110–204 °/s、力矩上限 [176,176,100,100,100,38,38] N·m
均取 LWR4+ 手册值。模块自检（`python3 -m config.lbr4_dynamics`）：
RNEA vs M q̈+Cq̇+g 装配残差 2.7e-15；M 特征值 0.151–4.50 全正（性质 P1）；
g(q) vs 势能数值梯度残差 6.3e-9。
**正式对接实验前**须按场景篇 §1.2 第 5 项用 `sim.getShapeMass/Inertia` 回填引擎侧
参数——两者之差即受控的模型失配源（总方案 §5.1"失配即扰动"）。

## 实验条件 E1–E7（扰动来源，总方案 §5.3 + 场景篇 §6）

| 条件 | 内容 | 注入通道 |
|---|---|---|
| E1 `none` | 标称，无扰动 | —（定理 3(b) 指数收敛） |
| E2 `l2` | 指数衰减正弦（有限能量）| w_dyn 经 M̂ 折算入力矩通道 τ += M̂w（能量口径与加速度级一致，场景篇 §6.1）|
| `bias` | 常值+有界正弦（L∞ 持续）| 同上（定理 3(d) ISS 极限球） |
| E3 `mismatch` | 控制器名义质量/惯量整体高估 20%（MISMATCH_SCALE=1.2）| (M−M̂),(C−Ĉ),(g−ĝ) 折算 w_dyn |
| E4 `large-error` | 初始姿态误差 ~174°（近对拓）位形，纯调节 | unwinding 处置检验（定理 1(i)） |
| E5 `highspeed` | S2 圆周角速率 1.0 → 2.5 rad/s | J̇q̇/Cq̇ 不可忽略域 |
| E6 `noise` | q 噪声 σ=5e-5 rad（16bit 编码器）、q̇ 噪声 σ=1e-3 rad/s | 测量通道（经 FK 进入 d(t)）|
| E7 `contact` | 内部后端：5 N 级末端接触力半正弦脉冲（0.3 s），τ_ext = Jᵀ[0;F]；CoppeliaSim：椅背实体擦碰 + 接触冲量累计 | 对象端外力矩 |

**信息来源三分**（场景篇 §7）：①"真值" = CoppeliaSim 引擎状态（或内部 RNEA 对象积分）；
②控制器视角 = TNDQ FK/误差/控制律由测量 (q,q̇) 实时计算（E6 时含噪声）；
③理论预测 = 定理 3 证书（认证 L2 增益 1/λmin(K_d)、ISS 界 5.7）。
CSV 同时记录 ①-② 的误差与 ③ 的核验结果，三者闭环互证。

## 安全机制（场景篇 §6.3；全部触发即计数并写入汇总）

- 关节限位检查：越限安全终止并保留已采数据（`check_joint_limits`，2°margin）
- 力矩限幅 `clip_torque`（手册上限）+ q̈_ref 范数限幅（QDDOT_MAX=40 rad/s²）
- **关节级安全治理器**：额定速度限幅 + 限位预测制动（刹车距离 q̇²/2A_BRAKE 进入
  缓冲区即强制减速）——E4 大姿态纯调节防止伪逆路径高速冲过限位的必需项
- 奇异监控：σ_min(J) < 1e-3 时阻尼伪逆阻尼提至 5e-2 并告警
- 零空间冗余消解（7R n=7>m=6）：q̈_ref += (I−J⁺J)(K(q_c−q)−Dq̇) 关节居中推离限位，
  不改变任务空间动态（定理 1–3 误差体系不受影响）
- 全部限幅/治理/阻尼残差按定理 3 诚实条款计入 d(t)，由 H∞/ISS 证书兜底

## 实测数值结果（内部力矩级后端，dt=1 ms，2025 批量验证）

### S1 定点（杯口上方预抓取，t_go=3 s 趋近整形）

| 条件 | 稳态 \|O\| | 稳态 \|T\| | 稳态 \|e_ξ\| | 证书核验 |
|---|---|---|---|---|
| E1 标称 | 8.2e-7 | 3.7e-11 | 8.8e-7 | V→2.3e-12（指数型，定理 3(b)）|
| E2 L2 | 4.0e-3 | 2.5e-3 | 1.5e-2 | **实测 L2 增益 0.1079 ≤ 认证 0.125** |
| E3 失配 20% | 2.5e-1 | 8.4e-2 | 6.4e-3 | e_ξ 收敛、e_z 留稳态偏置（CTC 无积分作用的固有特性，被 ISS 界约束）|
| E4 174° 纯调节 | 1.1e-2 | 4.6e-8 | 1.2e-2 | V: 26.7→3.3e-4；力矩饱和 471/10000 步、治理 625/10000 步，仍指数收敛 |
| E6 噪声 | 7.0e-5 | 6.7e-5 | 2.8e-3 | 噪声地板量级，无发散 |

### S2 绕杯圆周（半径 0.12 m）

| 条件 | 稳态 \|O\| | 稳态 \|T\| | 证书核验 |
|---|---|---|---|
| E1 标称（ω=1.0）| 1.8e-6 | 3.6e-5 | 跟踪误差机器噪声量级 |
| E2 L2 | 4.3e-3 | 2.4e-3 | **实测 L2 增益 0.1078 ≤ 认证 0.125** |
| E5 高速（ω=2.5）| 3.2e-6 | 1.6e-4 | 引理 1 前馈 + J̇q̇ 免构造补偿在高速域仍精确 |
| E7 接触（5N/0.3s）| 9.4e-3 | 2.3e-5 | 冲击后 V 以设计速率指数恢复（12 s 时 1.8e-4）|
| bias L∞ | 1.1e-1 | 2.9e-2 | 稳态 \|e_ξ\| 0.0269 ≤ ISS 界 (5.7) 0.169 |

### 原版回归（加速度级，line/circle，10 s）

| 场景 | 稳态 \|e_ξ\| | 判据 |
|---|---|---|
| line 标称 | 9.7e-4 | 与历史基线一致 |
| line L2 | 5.3e-3 | 实测增益 0.1207 ≤ 0.125 |
| line bias | 3.2e-2 | ≤ ISS 界 0.165 |
| circle 标称 | 6.0e-1（10 s 均值）| ⚠ 该基线轨迹在 t≈4.7–5.5 s 穿越腕部奇异（σ_min(J)≈3e-4），奇异监控按设计提升阻尼、治理器抑制发散，t>8 s 恢复收敛——几何固有特性，历史 3 s 短表未触及 |

其他关键数字：约束残差 c₀,c₁,c₂ 全程 ≤ 6.3e-15（机器精度）；
控制器单步：加速度级 ~1.65 ms（FK+误差+控制律），力矩级 ~6.9 ms
（含 7 次 RNEA 组装 M̂ 与扰动折算，未做缓存优化——如实报告）。
单元测试 11/11 通过。

### S3 抓取-搬运（CoppeliaSim 力矩级，全交叉 law × mode，2026-07 实测）

带载圆周稳态段（circle-ss，舍 ramp 后）关键指标：

| 控制律 × 负载 | \|T\| RMS [m] | \|O\| RMS | \|e_ξ\| RMS | \|τ\| RMS [N·m] | V 稳态 |
|---|---|---|---|---|---|
| C1 空载 | 6.32e-4 | 1.72e-3 | 3.64e-4 | 18.1 | 2.7e-5 |
| C1 带载 | 1.58e-2（×25）| 5.26e-2（×31）| 6.41e-3 | 21.1 | 2.4e-2 |
| C3 空载 | 1.33e-4 | 8.85e-5 | 1.50e-4 | 18.1 | 2.1e-7 |
| C3 带载 | 4.86e-3（×37）| 4.27e-3（×48）| 9.55e-4 | 19.2 | 3.4e-4 |

- **负载效应**：两律带载稳态误差均放大 1–2 个数量级（名义模型不含杯，
  ΔM/Δg 失配无积分作用时的固有稳态偏置，被 ISS 界约束）；抓握力稳态
  均值 2.453 N = m·g（0.25 kg），力传感器直接验证负载真实作用在动力学链上。
- **控制律对比（带载 circle-ss，C3/C1）**：T_rms 0.31、O_rms 0.08、
  τ_rms 0.91。忠于实测的解读：本任务准静态（ω=1 rad/s），带载稳态误差由
  对常值重力失配的等效直流刚度决定——C3 外环×内环级联 k_T·K_servo ≈ 80/s²
  > C1 的 k_p=16，故 C3 稳态更小，属**增益分配效应而非结构优势**。
- **C1 的结构优势在另两处**：① 附着瞬态——C3 差分前馈一拍滞后放大冲击，
  hold 段抓握力均值 13.7 N vs C1 的 9.2 N（×1.5）；② 可证性——C1 持有
  H∞/ISS 证书（V 有界可预算，定理 3），C3 的速度→力矩桥接无证书，且差分
  前馈在大 dt/测量噪声（E6）下退化；高速域优势另见 E5（S2 表）。
- **无穿模审计**：四组全部通过——最小净距恒为正（C3 带载最紧 0.7 mm）；
  杯接触力仅出现在附着瞬态邻域（引擎焊接冲击，合法支撑），圆周段零接触。
- 控制器单步耗时：C1 ~9.5 ms，C3 ~10.1 ms（同一力矩出口，开销相当）。

## 五种控制理论对比（总方案 §4；C2/C2-abl/C3 已在 S3 同台实现，C4 为规划位）

| 控制器 | 误差定义 | 姿态处理 | 前馈结构 | 已知短板 |
|---|---|---|---|---|
| **C1 TNDQ 几何一致 CTC（本实现）** | HDQ 群误差 x̃=x x̂_d⁻¹（定理 1），e_ξ/e_z 同一几何对象 | 单位 DQ 无奇异；unwinding 由符号翻转处置（定理 1(i)）| 引理 1 解析前馈 + J̇q̇ 免构造读出 | 力矩层 RNEA 装配开销（6.9 ms，可缓存优化） |
| C2 忠实 [Ch20] resolved-acceleration 律（**已同台实现** `--law dq-chandra`）| twist 误差 ω_e=Ad(x̃)ξ_d−ξ（式 (32)，经伴随搬运，与定理 1 的 −e_ξ 同一）| 位姿反馈取螺旋对数 −K_P·vec₆(2 ln x̃) | ξ̇_d、J̇q̇ 均解析（原文式 (2)），无差分 | 螺旋对数导数映射在 φ→π 奇异（大误差弱点）；无对任意正定增益成立的耗散等式结构 → 无定理 3 类证书 |
| C2-abl 朴素 twist 差消融基线（**非文献律**；已同台实现 `--law dq-ctc`）| 同一 DQ 位姿误差 (O, T)，但速度层用朴素 twist 差 ξ_d−ξ（§4.1 伪项）| 单位 DQ，同 C1 符号翻转 | ξ̇_d 与 J̇q̇ 均数值差分（一拍滞后 + 差分噪声），位姿反馈不经 Aᵀ 整形 | 伪项随 ‖ξ_d‖ 线性增长（E6 对照组实证）；差分前馈噪声放大；Lyapunov 交叉项不相消 → 无定理 3 证书 |
| C3 一阶 DQ H∞ 运动学律（[P2] 系；**已同台实现** `--law dq-hinf`）| DQ 误差 x̃=x x_d*（与 C1 同约定）| 单位 DQ，unwinding 需额外处理 | 一阶 DQ 无二阶通道，速度→力矩需差分前馈 + 内环伺服桥接 | 缺 (3.5) 二阶免构造通道 → 前馈滞后（S3 实测：附着冲击 ×1.5）；桥接后一阶 H∞ 证书失效 |
| C4 关节空间 CTC | q̃ = q − q_d(逆解) | 依赖逆运动学采样 | 关节级前馈精确 | 任务空间误差不受直接调控；逆解在奇异/冗余下不适定 |

**TNDQ/HDQ 的实测优势**（对应上表数据）：
① 误差收敛性——E4 174° 近对拓初值仍指数收敛（忠实 C2 的螺旋对数导数
映射在 φ→π 处奇异，大误差域不适用）；
② 前馈精度——E5 高速域跟踪误差仅增大 ~4 倍且仍在 1e-4 量级（ξ̇_d 解析、J̇q̇ 免构造，
无数值差分噪声）；③ 证书可核验——实测 L2 增益 0.108 与认证 0.125 之差即为
理论保守度的直接度量，H∞/ISS 证书在力矩级+失配下依然成立；
④ 数值稳定性——约束残差 (3.8) 全程机器精度，重投影（§3.4）零漂移。
**各基线的同台数值对比已按总方案 §5.2（同一力矩出口、同一扰动、
同一预算：各律线性化 d→e 传递函数逐通道恒等）在 S3 抓取-搬运实验中支持**
（C2 = `dq-chandra` 忠实 [Ch20] 律，C2-abl = `dq-ctc` 朴素差分前馈消融档，
C3 = `hdq_hinf_coppeliasim` 原实现逐行移植。**数据口径**：`results/
grasp_metrics_summary.csv` 中既有 dq-ctc 数值为 C2-abl 实测，忠实 C2
（`grasp_circle_chandra_*.npz`）的 CoppeliaSim 测量在 `--law dq-chandra`
接入后补跑；见上文 S3 实测表与解读）；
γ 维度的新旧理论对照见 §5 γ 影响实验（E8）；C4 属后续实验里程碑。

## 实现要点与符号约定

- **DQ 存储**：8 维 `[w,x,y,z, dw,dx,dy,dz]`；x̂ = r + ε½pr（式 2.1）；
  twist ξ = 2ẋx*（式 2.2），vec₆ 顺序 `[ω; v]`
- **TNDQ 存储**（3×8）：ch[0]+σch[1]+½σ²ch[2]，乘法 c₂ = a₀b₂+2a₁b₁+a₂b₀（式 3.2）
- **HDQ 存储**（2×8）：x̆ = x̂ + ε*·dx̂/dt，乘法按式 (2.3)
- **J̇q̇ 免构造**：令 q̈=0 跑一次 TNDQ 链，式 (3.5) 的 vec₆(ξ̇) 即 J̇q̇
- **Unwinding**：HDQ 误差标量部 η̃<0 时两通道同时翻转符号（定理 1(i)）
- **趋近段整形**（goto_trajectory）：大误差调节转化为沿路小误差跟踪；
  相对旋转轴角取短路径（w<0 翻转，与 unwinding 同源）
