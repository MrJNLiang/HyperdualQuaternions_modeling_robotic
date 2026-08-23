# TNDQ_real —— TNDQ 力矩控制真机部署包（reBot B601-DM，自包含）

本工程将 `TNDQ_b601` 中已在 Isaac Sim 验证的 TNDQ（截断四元数/对偶四元数）
力矩级控制律，部署到 **reBot B601-DM 真机**（Jetson + Ubuntu，达妙电机 MIT
模式直驱）。

**部署定义：Jetson 上只需拷贝 `TNDQ_real/` 这一个文件夹即可运行。**
算法层（core / control / config / simdata / run_lib）已作为 `TNDQ_b601` 的
同源副本内置于本包；总线驱动以裁剪版内置于 `vendor/`（meshes 可视化资产
未携带，不影响控制）。核心实验入口为 `experiments/exp1_real.py`（openhold /
grasp 两种模式）；`scripts/` 为标定工具链，作为上机可选参考保留。

```
控制线程 100 Hz（run_lib，CTRL_DT=10 ms）
    tau_TNDQ ──> apply_arm_torques(tau)
                        │  预分配缓冲 + 锁
总线线程 500 Hz（RebotArm.start_control_loop）
    读反馈 -> 电机系↔URDF系变换 -> 看门狗/斜坡/斜率限制 -> send_mit
                        │
   /dev/ttyACM0 (921600) -> 达妙串口-CAN 桥 -> CAN -> 7 电机
```

---

## 1. 目录结构

```
TNDQ_real/                          ★ 整个文件夹拷到 Jetson 即完成部署
├── config/
│   ├── paths.py          # sys.path 引导（REAL_ROOT + vendor）
│   ├── params.py         # 机器人/实验参数（TNDQ_b601 同源副本）
│   ├── b601_dynamics.py  # 标称动力学（TNDQ_b601 同源副本）
│   ├── params_real.py    # 真机参数覆写（monkey-patch config.params）
│   └── transforms.py     # ★ 标定常量：符号/零位/力矩标度/夹爪换算（阻塞项）
├── core/                 # TNDQ 代数/运动学（同源副本，见 core/README.md）
├── control/              # 控制律/误差体系/增益设计（同源副本）
├── simdata/              # 轨迹生成/pinocchio FK（同源副本裁剪版）
├── interfaces/
│   └── real_backend.py   # RealB601Backend：与仿真后端同签名的真机后端
├── experiments/
│   ├── run_lib.py        # 主控制循环（同源副本，后端可插拔）
│   └── exp1_real.py      # ★ 实机实验入口（openhold / grasp）
├── scripts/              # 标定与检查工具链（上机可选参考，00→03 顺序）
│   ├── 00_check_bus.py       # 总线连通性检查（不使能，零风险）
│   ├── 01_calib_signs.py     # 关节符号 + 零位标定（交互式）
│   ├── 02_gravity_hold.py    # URDF 系重力保持（符号终审 + TAU_SCALE 整定）
│   └── 03_calib_gripper.py   # 夹爪开度标定（卷尺回归）
├── tests/
│   └── test_wiring.py    # 无硬件接线冒烟测试（包内自洽）
├── vendor/
│   └── reBotArm_control_py/  # 总线驱动 uv 工程（源码+yaml+urdf 文本模型）
├── results/              # 实验 CSV 输出
└── core/README.md        # 副本清单与开发机同步纪律
```

---

## 2. 环境准备

### 2.1 Jetson（部署目标）

1. **拷贝部署包**：把整个 `TNDQ_real/` 拷到 Jetson（scp/rsync/U 盘均可），
   以下所有命令的工作目录均为该文件夹。
2. **安装驱动环境**（一次即可；驱动要求 Python 3.10）：
   ```bash
   cd vendor/reBotArm_control_py && uv sync && cd -
   ```
   `uv sync` 会安装 `motorbridge`（达妙串口-CAN 桥原生绑定，PyPI 分发，
   需网络；离线场景可提前 `uv sync --no-install-workspace` 打包 wheelhouse）。
3. **串口权限**：`sudo usermod -aG dialout $USER`（重新登录生效），
   或临时 `sudo chmod 666 /dev/ttyACM0`。
4. **pinocchio（建议）**：`uv pip install pin`（或 apt/conda）。安装后
   run_lib 走 C++ 快速 FK/pinocchio 轨迹路径；缺失时自动回退 HDQ 链 FK +
   kinematic 轨迹（打印告警，数学口径不变，10 ms 周期依然充裕）。
5. **部署自检**（无硬件可跑）：
   ```bash
   python3 tests/test_wiring.py     # 期望 === TNDQ_real WIRING TEST PASS ===
   ```

### 2.2 开发机（无硬件）

```bash
python3 TNDQ_real/tests/test_wiring.py     # 参数覆写/钩子/急停路径 PASS
```

> 硬件命令统一形式：`uv run --project vendor/reBotArm_control_py python <脚本>`
> （在 TNDQ_real/ 目录下执行；`--project` 指向驱动 uv 工程的虚拟环境）。

---

## 3. 标定：上机前的唯一阻塞项

`config/transforms.py` 四个常量决定电机系与 URDF 系的坐标约定。**未标定严禁
上闭环**：符号错 = 反馈极性反转 = 正反馈甩臂；零位错 = 限位/轨迹全错；力矩
标度错 = 重力补偿漂移。厂商代码出现过 `q_sim = -q_motor`，符号大概率为负，
必须实测。

| 常量 | 含义 | 由哪个脚本产出 |
|---|---|---|
| `JOINT_SIGN` | 编码器↔URDF 符号（每关节 ±1） | 01 |
| `JOINT_OFFSET` | 绝对零位偏差 [rad] | 01 |
| `TAU_SCALE` | 力矩标度（指令/反馈） | 02 |
| `GRIPPER_M_PER_RAD` / `GRIPPER_SIGN` | 夹爪角↔开度换算 | 03 |

---

## 4. 完整操作流程（严格按序，每步 PASS 才进下一步）

工作目录 = `TNDQ_real/`；`UVR = uv run --project vendor/reBotArm_control_py`。
全程手悬急停（Ctrl+C 立即失能）。

### 步骤 00 —— 总线检查（零风险）
```bash
UVR python scripts/00_check_bus.py
```
判据：`[PASS]`，7 关节反馈非全零。失败排查：USB/dmesg、桥接器供电、CAN 接线、dialout 权限。

### 步骤 01 —— 关节符号 + 零位标定（人机协同）
```bash
UVR python scripts/01_calib_signs.py
```
流程：厂商动力学电机系重力保持托臂 → 逐关节 +1 N·m 短脉冲，**目视运动方向**
对照屏幕给出的 URDF 正方向描述输入 y/n → 手动掰臂到 Q_INIT 姿态抓零位 →
打印 FK 预测 TCP 位置供卷尺复核（偏差应 < 1 cm，FK 呈镜像 = 某关节符号反）。
脚本末尾打印**回填块**，复制到 `config/transforms.py`。

### 步骤 02 —— URDF 系重力保持（符号终审 + TAU_SCALE 整定）
```bash
UVR python scripts/02_gravity_hold.py --duration 30
UVR python scripts/02_gravity_hold.py --factor 1,1.2,2.5,2.5,1,1   # 如需补偿系数
UVR python scripts/02_gravity_hold.py --pose armup                 # 前伸姿态交叉验证
```
判据：
- **符号终审**：前 2 s 斜坡混入期间无关节立即单向漂移（漂移 > 0.15 rad 脚本
  自动报警 → 回 01 重标）；
- **TAU_SCALE**：保持段每关节漂移速率 < 0.002 rad/s 为优（< 0.05 rad/30 s
  可接受——闭环 small_arm 增益对恒值残差有 p_T 抑制）。
- 整定：下垂（补偿不足）→ `TAU_SCALE[i] /= 1.2`；上漂 → `*= 1.2`，回填后重跑。

### 步骤 03 —— 夹爪开度标定
```bash
UVR python scripts/03_calib_gripper.py
```
对 3 个目标开度逐次卷尺实测指间距离并输入，脚本最小二乘回归后打印
`GRIPPER_M_PER_RAD` / `GRIPPER_SIGN` 回填块。

### 步骤 04 —— 回填后复验
把 01/02/03 的回填块写入 `config/transforms.py`，重跑无硬件接线测试确认
参数链仍通：
```bash
python3 tests/test_wiring.py
```

### 步骤 05 —— 就位（POS_VEL 位置模式，独立于闭环）
```bash
UVR python -c "import sys; sys.path.insert(0,'.'); import config.paths; \
import importlib.util as iu; s=iu.spec_from_file_location('rb','interfaces/real_backend.py'); \
rb=iu.module_from_spec(s); s.loader.exec_module(rb); \
from config.params import Q_INIT; rb.posvel_goto(Q_INIT)"
```
铁律：**闭环全程 MIT 力矩模式**；就位用独立进程 POS_VEL（驱动器位置环
pos_kp=150，绝不可与 TNDQ 力矩环混用）。

### 步骤 06 —— 空载基线（安全首跑）
```bash
UVR python experiments/exp1_real.py --mode openhold --traj pinocchio
```
验收：无 abort、CSV 落盘 `results/`、控制线程单步耗时 < 10 ms、
末端跟踪误差与仿真同口径量级一致。

### 步骤 07 —— 接触抓取（带载）
确认 openhold 稳定后：
```bash
UVR python experiments/exp1_real.py --mode grasp --traj pinocchio
```
（`--traj kinematic` / `--traj tndq` 为备选轨迹口径；`--csv` 可自定义输出。）

---

## 5. 安全机制一览（real_backend + run_lib 联合实现）

| 机制 | 参数（params_real.py） | 触发后行为 |
|---|---|---|
| 看门狗 | `WATCHDOG_TIMEOUT=0.05` s | 控制线程 50 ms 无心跳 → 总线线程降级为纯重力补偿 |
| 使能斜坡 | `TORQUE_RAMP_TIME=0.5` s | 0.5 s 内从 g(q) 线性混入到全控制力矩 |
| 力矩斜率限制 | `TORQUE_SLEW_MAX=200` N·m/s | 逐拍限制指令变化率 |
| 碰撞残差检测 | `CONTACT_RESID_TAU=8` N·m × 50 拍 | 实测-下发残差持续超限 → HardwareFault 急停 |
| 速度异常检测 | `VEL_SPIKE_MAX=6` rad/s | 单拍速度尖峰 → HardwareFault 急停 |
| 周期超时 | 落后 > 2 拍 | `step()` 抛 HardwareFault |
| 软限位 + 奇异阻尼 | run_lib 原生 | 限位越限记录 abort，奇异点阻尼常开 |
| 就位容差 | `Q_BRINGUP_TOL=0.02` rad | `reset_to` 发现构型偏离 → 拒绝启动 |

`HardwareFault` 由 run_lib 按类名捕获 → 记入 CSV abort 字段 → finally 触发
`backend.close()`（0.3 s 重力托臂 → 停总线 → 失能 → 断连）。

---

## 6. MIT 指令转换（力矩直驱 → 达妙电机）

达妙 MIT 模式驱动器内环执行 `τ_motor = kp(q_d−q) + kd(q̇_d−q̇) + τ_ff`。
纯力矩直驱 = pos/vel 目标填**实测值** + `MIT_KP=0` + 小 `MIT_KD`（安全网）+
`τ_ff = τ_TNDQ`（经 `JOINT_SIGN`/`TAU_SCALE` 标定逆变换）。与厂商重力前馈
保持同构。串口/电机 ID 配置在
`vendor/reBotArm_control_py/config/rebotarm_dm.yaml`（/dev/ttyACM0，921600，
rate 500；j1-3: DM4340P 0x01-0x03，j4-6: DM4310 0x04-0x06，gripper: 0x07）。

---

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| 反馈全零 | CAN 桥供电/接线；`dmesg \| grep ttyACM` |
| 重力保持立即单向漂移 | 符号/零位错误 → Ctrl+C 失能，回 01 |
| 重力保持缓慢下垂 | TAU_SCALE 偏大 → 该关节 `/= 1.2` 重跑 02 |
| FK 预测与卷尺呈镜像 | 某关节 JOINT_SIGN 反 → 回 01 |
| `run_lib` 打印 pinocchio 缺失告警 | 无碍；建议 `uv pip install pin` 走快速 FK |
| `reset_to` 拒绝启动 | 先用步骤 05 就位到 Q_INIT（容差 0.02 rad） |
| `uv sync` 安装 motorbridge 失败 | 需网络访问 PyPI；或离线 wheelhouse |

---

## 8. 开发机维护（同步纪律）

算法层唯一维护点是仓库内的 `TNDQ_b601/`。仿真侧若修改 core / control /
config(params,b601_dynamics) / simdata / run_lib，部署前按
`core/README.md` 的清单同步副本到本包并重跑 `tests/test_wiring.py`。
`transforms.py` / `params_real.py` 为真机专属，永不被同步覆盖。
