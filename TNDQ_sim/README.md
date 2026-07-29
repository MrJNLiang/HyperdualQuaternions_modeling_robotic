# TNDQ_sim — TNDQ/HDQ 几何一致控制仿真（内部数值 + CoppeliaSim 双后端）

基于论文《TNDQ论文初稿_运动学重构_误差体系与控制律》（`docs/` 目录）的完整 Python 实现：
TNDQ/HDQ/DQ 代数结构、7R 串联机械臂（KUKA LBR4+/LWR4+ 构型）TNDQ 链式正运动学、
几何一致误差体系（定理 1/2）、计算力矩控制律（式 5.2）与 H∞/ISS 性能保证（定理 3）；
并按 `docx/KUKALBR4p场景_定点与圆周扰动对比实验设计.md`（下称"场景篇"）与
`docx/TNDQ动力学控制对比分析与实验设计方案.md`（下称"总方案"）完成了
**CoppeliaSim（KUKALBR4+_sim.ttt 场景）力矩级对接**与 **E1–E7 扰动实验矩阵**。

**纯数值输出（npz + CSV），不含任何绘图。**

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
│   └── test_math_properties.py  #   11 项数学性质单元测试（全部通过）
├── run_simulation.py            #   闭环仿真主程序（双后端、S1/S2、E1–E7、安全机制）
└── README.md
```

## 环境要求

- Python ≥ 3.8，核心仅依赖 `numpy`（测试可选 `pytest`）
- CoppeliaSim 对接（可选）：CoppeliaSim ≥ 4.4 + ZMQ Remote API 客户端

```bash
pip install numpy
pip install coppeliasim-zmqremoteapi-client   # 仅 --backend coppeliasim 需要
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

## 四种控制理论对比（总方案 §4，对比实现规划位）

| 控制器 | 误差定义 | 姿态处理 | 前馈结构 | 已知短板 |
|---|---|---|---|---|
| **C1 TNDQ 几何一致 CTC（本实现）** | HDQ 群误差 x̃=x x̂_d⁻¹（定理 1），e_ξ/e_z 同一几何对象 | 单位 DQ 无奇异；unwinding 由符号翻转处置（定理 1(i)）| 引理 1 解析前馈 + J̇q̇ 免构造读出 | 力矩层 RNEA 装配开销（6.9 ms，可缓存优化） |
| C2 操作空间矩阵 CTC | R/p 分离误差（SO(3)×R³） | 旋转矩阵对数映射，π 处退化 | 需显式 J̇（数值差分或逐列构造） | 姿态/平移增益耦合缺几何一致性；J̇ 数值噪声 |
| C3 DQ 鲁棒 CTC（文献 DQ 系）| DQ 对数误差 | 单位 DQ，unwinding 需额外处理 | 一阶 DQ 无二阶通道，ξ̇_d 需数值差分 | 缺 (3.5) 二阶免构造通道 → 前馈滞后 |
| C4 关节空间 CTC | q̃ = q − q_d(逆解) | 依赖逆运动学采样 | 关节级前馈精确 | 任务空间误差不受直接调控；逆解在奇异/冗余下不适定 |

**TNDQ/HDQ 的实测优势**（对应上表数据）：
① 误差收敛性——E4 174° 近对拓初值仍指数收敛（C2 对数映射在 π 处病态）；
② 前馈精度——E5 高速域跟踪误差仅增大 ~4 倍且仍在 1e-4 量级（ξ̇_d 解析、J̇q̇ 免构造，
无数值差分噪声）；③ 证书可核验——实测 L2 增益 0.108 与认证 0.125 之差即为
理论保守度的直接度量，H∞/ISS 证书在力矩级+失配下依然成立；
④ 数值稳定性——约束残差 (3.8) 全程机器精度，重投影（§3.4）零漂移。
C2–C4 的同台数值对比按总方案 §5.2（同一力矩出口、同一扰动、同一预算）在
CoppeliaSim 场景中执行，属后续实验里程碑。

## 实现要点与符号约定

- **DQ 存储**：8 维 `[w,x,y,z, dw,dx,dy,dz]`；x̂ = r + ε½pr（式 2.1）；
  twist ξ = 2ẋx*（式 2.2），vec₆ 顺序 `[ω; v]`
- **TNDQ 存储**（3×8）：ch[0]+σch[1]+½σ²ch[2]，乘法 c₂ = a₀b₂+2a₁b₁+a₂b₀（式 3.2）
- **HDQ 存储**（2×8）：x̆ = x̂ + ε*·dx̂/dt，乘法按式 (2.3)
- **J̇q̇ 免构造**：令 q̈=0 跑一次 TNDQ 链，式 (3.5) 的 vec₆(ξ̇) 即 J̇q̇
- **Unwinding**：HDQ 误差标量部 η̃<0 时两通道同时翻转符号（定理 1(i)）
- **趋近段整形**（goto_trajectory）：大误差调节转化为沿路小误差跟踪；
  相对旋转轴角取短路径（w<0 翻转，与 unwinding 同源）
