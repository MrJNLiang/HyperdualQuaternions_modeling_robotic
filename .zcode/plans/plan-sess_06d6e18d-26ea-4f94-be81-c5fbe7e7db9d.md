## 目标

用论文定理3（§5.3）+ §5.4 标度律，为 reBot Arm B601-DM 计算任务空间刚度/阻尼增益 (K_p, K_d) 的认证可行界限，实机口径（dt=10 ms）为主、仿真口径（dt=2 ms）对照。

## 理论依据（论文 → 界限）

以极点配置规则 K_ω=K_v=(a+b)I₃、k_p,T=ab、k_p,O=4ab I₃（通道极点 {−a,−b}）为参数化，界限为：

1. **H∞ 下界 (5.6a)**：λ_min(K_d) ≥ ½(κ⁻¹+γ_a⁻²) = 2.5（κ=1.0, γ_a=0.5，即 params.py 的 KAPPA/GAMMA_A），另附 κ–γ_a 扫描表
2. **α 小增益条件 (5.1f)/(5.7a)**：α = sup‖J M⁻¹ΔM J⁺‖₂ < 1/cond₂(K_d)；各项同性 K_d 时要求 α<1，λ_eff = λ_min(K_d)−α·λ_max(K_d) > 0
3. **离散化上界（screen C-disc）**：max(a,b)·dt ≤ 0.15 → 实机 max(a,b) ≤ 15 → k_p,T = ab ≤ 225
4. **力矩/指令预算（C-eff + TAU_MAX）**：u_peak = λ_max(K_d)|e_ξ|+½λ_max(K_p)|e_z| ≤ QDDOT_MAX=30；工作空间采样验证 |M q̈+Cq̇+g| ≤ [27,27,27,7,7,7] N·m
5. **水平集 (5.5b)**：c* = ½λ_min(K_p,O) = 2ab，须大于初始误差能量 V(0)
6. **静态误差标度律 (5.9)**：e_T,ss = d_v/k_p,T、e_O,ss = d_ω/(2k_p,T)，d_v ≈ 0.24（GAIN_SETS grasp 注释实测）

## 实施步骤

1. 新建 `TNDQ_real/experiments/compute_gain_bounds.py`，复用现有模块：
   - `config/b601_dynamics.py` 的 `B601NominalDynamics.mass_matrix()` 采样工作空间 → M(q) 特征值范围
   - `core/kinematics.py` 的雅可比 + 数值场景计算 α：ΔM=εM（ε=10/20/30%，解析交叉验证 α=ε）、末端未建模 0.25 kg 负载、夹爪质量误差
   - `control/gain_design.py` 的 `c1_channels`/`screen` 做四约束（C-cert/C-disc/C-damp/C-eff）可行域扫描：(a,b) 网格 → 可行窗口边界
2. 输出：
   - k_d = a+b、k_p,T = ab、k_p,O = 4ab 的下界/上界数值表（实机 dt=10 ms 与仿真 dt=2 ms 两栏）
   - 既有 GAIN_SETS（base/tuned/small_arm/grasp）逐组 screen 验证结论
   - α 各场景数值 + λ_eff；V(0) vs c* 水平集检查；(5.9) 静态误差预测
3. 运行脚本，汇总成对话报告：界限表 + 每条界限对应的论文公式编号 + 推荐/排除的增益组（含 tuned 组实机 pole_dt=0.2>0.15 超界的定量解释）

## 交付物

- 脚本 `TNDQ_real/experiments/compute_gain_bounds.py`（可复跑）
- 对话中的完整数值报告（中文，含公式对应关系与推荐增益窗口）

不改任何现有控制参数文件；纯计算与报告。