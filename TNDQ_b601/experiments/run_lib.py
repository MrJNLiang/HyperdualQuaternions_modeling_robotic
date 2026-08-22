"""
B601 实验共用库：TNDQ 力矩级控制主循环 + CSV 记录 + 摘要。

主循环口径与 TNDQ_sim/run_simulation.py 逐层一致：
    传感(q, q̇) -> 限位安全 -> FK 层(3.4)/(3.5) -> 期望轨迹(TNDQ 解析链)
    -> 误差层(4.1)-(4.5) -> 奇异监控(阻尼提升) -> 控制律(5.2)
    -> q̈_ref 范数限幅 -> 关节安全治理 -> 力矩层(§2.4) -> 力矩限幅
    -> 下发/物理步进 -> CSV 记录

与 TNDQ_sim 的适配差异（保持理论层零改动）：
  - B601 为 6R 非冗余（n = m = 6）：7R 零空间投影不移植（params 注释）；
  - 物理后端为 Isaac Sim 力矩级（interfaces/isaac_interface.py），
    真机部署时替换为同签名后端即可（传感/施力矩/步进三通道）；
  - CSV 列面向 B601 验证需求：关节/力矩明细 + 期望/实际末端位置
    （世界系）+ 立方体位姿 + 夹爪开度 + 约束残差 (3.8)。

运行方式（Isaac 官方运行时）：
    ~/isaacsim/python.sh TNDQ_b601/experiments/exp1_setpoint.py
"""

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import (
    B601_BASE_PREFIX, B601_DH_TABLE, B601_TOOL_ANGLE, CTRL_EVERY,
    DEFAULT_GAIN_SET, DT, ENABLE_DITHER, GAIN_SETS, GRIPPER_OPENING,
    LOG_EVERY,
    PINV_DAMPING, QDDOT_MAX, QDOT_MAX,
    Q_INIT, A_BRAKE, LIMIT_BUFFER, SINGULARITY_DAMPING, SINGULARITY_TOL,
    JOINT_LOWER, JOINT_UPPER, JOINT_DAMPING,
    R_TOOL_QUAT, SETPOINT_POS, GRASP_APPROACH_POS, LIFT_POS,
    GOTO_TOP_Z, GOTO_T_LIFT, GOTO_T_TRANS, GOTO_T_ROTDESC, GOTO_T_DESC,
    GOTO_GRASP_DWELL, GOTO_T_LIFTLOAD,
)
from config.b601_dynamics import (
    B601NominalDynamics, check_joint_limits, clip_torque,
)
from control.control_law import geometric_computed_torque_law
from control.error_system import full_error_state
from core.dq_algebra import dq_rot_z, dq_rotation, dq_translation
from core.kinematics import TNDQSerialChain
from core.tndq_algebra import TNDQ

# ---------------------------------------------------------------------------
# 失速 dither（PhysX TGS 组合锁对策）
#
# 实证（isaac_check/probe_combo_scan、probe_combo_v4、probe_breakaway）：
# 深折叠构型（q2≈-2.6~-2.8）下，闭环反馈力矩的 j2+j3+j4 三关节组合
# 会被 TGS 求解器锁死（开环施加 1 s 净位移 <0.01 rad），而任意单/双
# 关节子组合、同向 5x 放大都能动；锁与力矩方向签名（j2-、j3+、j4+，
# 即向立方体接近的任务方向）绑定，两次几何重设计（v3 侧抓 Q_F、
# v4 斜抓 Q_S）均复现。短脉冲（0.2 s、主分量加 ~2.5 N·m、与反馈
# 同向）可突破，突破后组合力矩持续驱动（probe_breakaway [2]：0.83 ->
# 1.57 rad）。真机无此病理（无 TGS 求解器），dither 仅在确证失速时
# 触发，幅值远低于 TAU_MAX，属仿真器兼容层。
# 实现（v3，多轮迭代）：
#   v1 离散脉冲：突破后速度衰减即复锁（pos_err 恒 0.086 m）；
#   v2 单关节零均值交变：臂在锁点附近振荡（pos_err 0.06~0.08 m），
#      推力均值为 0 无法穿越锁定区（probe_dither_bias 开环佐证），
#      且单关节 dither 未破坏 j2+j3+j4 组合签名、瞬时速度判据抖动；
#   v3 三关节有偏（3×4 N·m）：确证失速后激活也把臂踢飞（0.25 s
#      内 pos_err 0.07->0.41 m、j3 超限位）——推力过强 + 0.25 s 退出
#      滞后过长；
#   v3.1 单主分量（反馈幅值最大）：突破-关断-复锁循环 22 次无进展
#      （pos_err 恒 0.10 m）。根因（probe_combo_v4 TCP 位移定量）：
#      反馈主分量 j2(-) 的 TCP 位移与任务需求方向几乎反向
#      （cos=-0.998）——锁住的恰是"反馈组合签名"，而任务方向在
#      j4(+)/j3+j4 等解锁子组合一侧；
#   v3.2：进入 dither 前先做开环微分探针（每候选关节施 0.1 s 单位
#      力矩、测 TCP 位移，逐候选 reset 回停滞构型），选与任务方向
#      （p_d - p）余弦绝对值最大的解锁关节，按探针实测符号施加有
#      偏交变（Q_S 处选中 j4(+)，cos=0.97；j2(-) 与任务反向
#      cos=-0.998，按反馈幅值选必错）；
#   v3.3（exp1 dither7 后）：位置收敛到 2.3 cm 后姿态误差 0.30 rad
#      永不修正。诊断链：diag_wrist_stall（q̈_ref 腕分量 6/-23 但
#      M̂q̈_ref≈0 => 腕力矩 0.004 N·m）、probe_wrist_v4/wrist_inertia
#      （腕非锁非重惯量，M_eff≈名义；但 q_W 处 j5 滑行减速度
#      33.5 rad/s² vs Q_INIT 0.89 => 腕在组合锁区内被拖住）——锁
#      区内的腕修正同样需要 dither。改动：候选关节扩到 j2..j6；
#      失速判据与探针度量改用组合任务误差
#      e_comb = ‖p_d-p‖ + 0.05*ori_err（位置主导、姿态有权）；
#      控制律切动力学一致广义逆 J^bar=M^-1J^T(JM^-1J^T)^-1（力矩级
#      后端传 M，修正无加权 J^+ 向轻腕方向泄能的结构缺陷）。
#      结果：姿态收敛 0.017 rad，但位置卡 0.056 m；
#   v3.4（exp1 dither8 后）：根因链闭合——全位姿 IK 解
#      q*=[-0.252,-2.846,-1.718,-1.214,0,0] 位于深折叠锁区内，
#      闭环反馈签名（j2+/j3-/j4-）恰是锁定签名，dither 叠加式
#      （反馈+偏置）不改变签名故仍锁。probe_stall_dirs 决定性证据：
#      q_stall 处开环 j4+ 单关节 1 N·m×0.1 s 即把 pos_err
#      0.056 -> 0.0097（非运动学/接触约束，纯签名锁）。改动：
#      dither 激活期间把反馈净分量（锁定签名）替换为闭环单关节推：
#      任务误差投影到探针关节雅可比列施比例力矩（单关节签名=
#      解锁签名，闭环自适应方向与幅值防超调）。开环持续推实测
#      （dither11）振荡 0.039，闭环推为最终形态。
STALL_QDD_MIN = 0.5        # 指令加速度范数阈值 [rad/s^2]（应动）。
                            #   v3.4 dither10 实测：误差降到 ~2 cm 后
                            #   q̈_ref 随误差缩小降到 ~1，2.0 门限把
                            #   dither 永久拦截；配合速度/改善/持续/
                            #   floor 多重条件，放宽到 0.5 仍防误触发
STALL_QD_MAX = 0.25        # 速度判据阈值 [rad/s]（滑窗均值，防抖动）；
                            #   dither17 实测：卡滞期臂有 0.1~0.2 rad/s
                            #   蠕动振荡，0.15 拦截了大部分失速窗口；
                            #   goto 段误触发改由期望 twist 静止门限防
STALL_QD_SMOOTH = 0.15     # 速度滑窗时长 [s]
STALL_PE_WINDOW = 1.0      # e_comb 改善观察窗 [s]
STALL_PE_MIN_IMPROVE = 0.005   # 窗口内最小改善量 [m]（低于=无进展）
ORI_WEIGHT = 0.05        # e_comb 中姿态误差权重 [m/rad]
STALL_WIN_SUSTAIN = 2.0   # 失速持续判据滑窗 [s]；占比制：窗内判据成立
                            #   比例 ≥ STALL_FRACTION 才激活。dither19 实测：
                            #   卡滞期臂有蠕动振荡，速度/位移子条件间歇性
                            #   拦截，“连续 150 步”永远凑不满（t=4~10 卡
                            #   0.095 期间 dither 仅触发 2 次）
STALL_FRACTION = 0.5      # 滑窗内判据成立占比阈值
STALL_MIN_RUN = 10         # 另要求当前连续成立最少步数（0.1 s @ 控制
                            #   周期，防窗口尾部刚恢复时的惯性触发）
STALL_PE_FLOOR = 0.005     # e_comb 绝对下限：跟踪良好时不得激活。
                            #   v3.4 dither9 实测：floor=0.02 时位置收敛
                            #   到 0.014 m 后 dither 被 floor 拦截无法
                            #   继续修正；降为 5 mm（收敛后 q̈_ref 小，
                            #   判据其余条件天然防误触发）
STALL_DQ_MAX = 0.05        # 改善窗内关节位移上限 [rad]：锁定的定义是
                            #   关节物理上不动。dither15 实测：goto 末段
                            #   深折叠构型（q2≈-2.7）臂真实缓动（1 s 窗
                            #   max|Δq|≈0.12 rad、误差缓升）被误判失速，
                            #   探针把臂推出锁区后姿态漂到 1.7 rad、j3
                            #   超限位。真锁（q_stall）窗内位移 <0.02 rad，
                            #   取 0.05 分离两类（goto 段误触发主要由
                            #   STALL_XID_MAX 静止门限防）
STALL_XID_MAX = 0.005      # 期望 twist 范数上限：失速的定义是“指令静止
                            #   但臂不动”。goto 跟踪段期望仍在运动，误差
                            #   不降是跟踪问题不是失速（dither15/16 误触
                            #   发均发生在 goto 段）；只有 s≡1 定点阶段
                            #   （xi_d→ 0）才允许激活 dither。
DITHER_VEL_KILL = 1.5      # 停推滑行速度阈值 [rad/s]（腕/肘惯量极小）
DITHER_PUSH_MAX = 1.0      # 单关节推力限幅 [N·m]（probe_stall_dirs：
                            #   1 N·m×0.1 s 即 pos 0.056->0.0097）
DITHER_BURST_PER_ERR = 0.6  # 脉冲时长缩放 [s/m]：T_burst = 误差×本系数。
                            #   v6 重调：exp1 v6b（1.5）在 8.7 cm 锁深处
                            #   脉冲过长把臂甩出 0.44 m；v3.4 标定
                            #   0.1 s 降 4.6 cm，取 0.6 => e=0.087 处
                            #   脉冲 ~0.05 s，配合停推阈值 3 cm 分步
                            #   逼近；未达标由判据重新探针
DITHER_BURST_MAX = 0.10    # 脉冲时长上限 [s]（v6 收紧：短脉冲分步，
                            #   防单次过推甩动）
DITHER_LIMIT_MARGIN = 0.25  # 限位保护余量 [rad]：脉冲推力方向距软限位
                            #   不足本余量则立即退出交回反馈（dither15/16
                            #   两次撞限位均因脉冲把关节推向限位）
DITHER_REGRESS_MAX = 0.01  # 脉冲期间误差回升容忍 [m]：dither18 实测
                            #   探针边缘改善方向推后 pos 0.079->0.097
                            #   （推入更深锁区），回升超阈立即退出重探针
DITHER_STOP_ERR = 0.03     # 组合误差低于 3 cm 后不再推送（v6：v3.4 的
                            #   1 cm 在 8.7 cm 锁深下导致多轮探针反复
                            #   甩动；提高到 3 cm，推送进 3 cm 内交回
                            #   反馈收敛，防超调振荡）
DITHER_ACTIVATE_MIN_ERR = 0.05  # 激活下限 [m]（v6）：误差已降到 5 cm 内
                            #   时系统处于蠕变收敛相（非硬锁），探针
                            #   teleport 会扰动 PhysX 接触态导致步速骤降
                            #   （exp1 v6e 实测：t=20.9 pe=2.5 cm 误触发，
                            #   末段 1.3 s 仿真墙钟 12 min）；只对深锁
                            #   （>5 cm）激活逃逸
DITHER_N_JOINTS = 1        # 参与关节数（探针选出的任务方向最优）
DITHER_EXIT_MARGIN = 125   # （v3.4 保留备用）退出滞后判据恢复步数
DITHER_MAX_RUN = 2.0     # v3.4：单次推送最长时长 [s]，到期强制退出
                            #   重新探针（适应构型变化，防坏方向长推）
DITHER_PROBE_STEPS = 50    # 微分探针时长 [步]（0.1 s/关节，单位力矩）
DITHER_PROBE_DELTA_MIN = 0.002   # 探针改善下限 [m]（低于则退回反馈主分量）


def _comb_err(fk_x, des_x):
    """组合任务误差 e_comb = ‖p_d-p‖ + ORI_WEIGHT*ori_err [m]。"""
    dp = dq_translation(des_x) - dq_translation(fk_x)
    r, r_d = dq_rotation(fk_x), dq_rotation(des_x)
    ori = 2.0 * np.arccos(min(1.0, abs(float(r @ r_d))))
    return float(np.linalg.norm(dp)) + ORI_WEIGHT * ori


def _probe_dither_joint(backend, dyn, chain, q_stall, fk_x, des_x):
    """开环微分探针：选使组合任务误差下降最快的解锁关节。

    对 j2..j6 逐个施 0.1 s 单位力矩（重力补偿基线，每候选前
    reset_to 停滞构型保持一致），用物理层真实响应测组合误差变化
    （绕开被锁的反馈组合签名；probe_combo_v4 同法）。腕关节在锁
    区内同样被拖住（probe_wrist_inertia：q_W 处 j5 滑行减速度
    33.5 rad/s²），故候选含 j5/j6。返回 (j, delta, bias_sign)：
    delta = e_comb_after - e_comb_before（负=改善），bias_sign 为
    改善方向的力矩符号；结束后 reset 回停滞构型由反馈接管。
    """
    e0 = _comb_err(fk_x, des_x)
    results = []
    for jj in (1, 2, 3, 4, 5):
        backend.reset_to(q_stall)
        extra = np.zeros(6)
        extra[jj] = 1.0
        for _ in range(DITHER_PROBE_STEPS):
            q_now, _ = backend.get_joint_state()
            backend.apply_arm_torques(dyn.gravity_vector(q_now) + extra)
            backend.step()
        q1, _ = backend.get_joint_state()
        e1 = _comb_err(chain.fk_tndq(q1).ch[0], des_x)
        results.append((jj, e1 - e0))
    backend.reset_to(q_stall)   # 回停滞构型，dither 从原构型接管
    # 取改善最大者；若无改善（全 ≥0）取 |delta| 最小者反向
    j_p, d_p = min(results, key=lambda r: r[1])
    if d_p >= 0:
        j_p, d_p = min(results, key=lambda r: abs(r[1]))
        return j_p, -d_p, -1.0
    return j_p, d_p, 1.0


def w2dh(p_world):
    """世界 / base_link 系位置 -> DH 基座系（减基座前缀）。"""
    return np.asarray(p_world, dtype=float) - B601_BASE_PREFIX


# ---------------------------------------------------------------------------
# B601 控制链：DH 链 + TCP 尾变换（工具约定统一）
# ---------------------------------------------------------------------------

class B601TCPChain(TNDQSerialChain):
    """B601 控制链：DH 链 FK * 常值尾变换 E = Rz(B601_TOOL_ANGLE)。

    控制末端 = URDF gripper_link 原点/姿态（params.py 工具约定；
    isaac_interface.run_selfcheck [5] 已对账 Isaac gripper_link 世界
    位姿，dp < 2e-3 m）。E 为常值纯旋转且原点重合：右乘不改 twist
    （xi = 2 (x_dot E)(x E)* = 2 x_dot x*），故 J / Jdot_qdot / 约束
    残差 (3.8) 与裸 DH 链逐项一致，仅位姿通道（x / x_breve）进入
    工具系——实测 FK 与期望轨迹的任务姿态（R_TOOL_QUAT 等，ik_lib
    设计约定）由此在同一 TCP 约定下进入误差层 (4.1)。

    exp1 首跑发散根因（isaac_check 诊断链结论）：主循环曾直接用
    TNDQSerialChain 的裸 DH FK，其姿态与设计 TCP 姿态恒差
    B601_TOOL_ANGLE = 90 deg，goto 轨迹含幽灵旋转（|xi_d| 峰值
    0.75，旋转分量占 0.71），跟踪需大幅腕部运动而发散。理论层
    （core/）保持 TNDQ_sim 原样副本，尾变换在实验侧装配。
    """

    def fk_tndq(self, q, q_dot=None, q_ddot=None):
        e_bar = TNDQ.from_constant(dq_rot_z(B601_TOOL_ANGLE))
        return super().fk_tndq(q, q_dot, q_ddot) * e_bar


def build_setpoint_goto_trajectory():
    """exp1 goto 轨迹（v9 斜向下抓，用户指定运动形态）。

    零位（恒等姿态）-> 举高 0.28 m -> 恒等前移到接近退避位正上方 ->
    转腕+下降到退避位（斜上方外侧，沿接近轴 -x_g 距抓取位 8 cm）->
    沿接近轴斜下插入到抓取位（夹爪朝下对准方块）-> 静置（闭合夹持）
    -> 带载提升（沿 -x_g 退 2 cm + 竖直 8 cm）。
    几何依据：接触包络物理测绘（原点=指尖端，指垫在原点后侧
    15 mm）；diag_v9_grasp 功能验证 kinematic 逼近全程方块移位
    ≤1 mm（零碰撞）。
    返回 (traj, t_move, t_close)：t_move 为运动段总时长；t_close 为
    夹爪闭合指令时刻（抓取位静置段前段）。
    """
    from simdata.trajectory_generator import waypoint_sequence_trajectory
    chain = B601TCPChain(B601_DH_TABLE)
    x_init = chain.fkm(Q_INIT)
    r0 = dq_rotation(x_init)
    p0 = dq_translation(x_init)
    top0 = w2dh(np.array([p0[0], p0[1], GOTO_TOP_Z]))
    top = w2dh(np.array([GRASP_APPROACH_POS[0], GRASP_APPROACH_POS[1],
                         GOTO_TOP_Z]))
    legs = [
        (top0, r0, GOTO_T_LIFT, 0.0),
        (top, r0, GOTO_T_TRANS, 0.0),
        (w2dh(GRASP_APPROACH_POS), R_TOOL_QUAT, GOTO_T_ROTDESC, 0.0),
        (w2dh(SETPOINT_POS), R_TOOL_QUAT, GOTO_T_DESC, GOTO_GRASP_DWELL),
        (w2dh(LIFT_POS), R_TOOL_QUAT, GOTO_T_LIFTLOAD, 0.0),
    ]
    traj, t_move = waypoint_sequence_trajectory(x_init, legs)
    # v9 四跑归因：抓取位深折叠构型存在 ~1 cm/s 慢漂（重力残差），
    # 闭合必须在漂移发展前完成 -> 到位 +0.2 s 即发闭合指令
    t_close = GOTO_T_LIFT + GOTO_T_TRANS + GOTO_T_ROTDESC + GOTO_T_DESC + 0.2
    return traj, t_move, t_close


def build_setpoint_goto_trajectory_kinematic():
    """exp1 goto 轨迹（v11：笛卡尔运动学插值版，用户指定路线）。

    与 build_setpoint_goto_trajectory（TNDQ 解析链，保留作对比备份）
    路标/时序完全一致，但路径生成改用传统运动学插值
    （simdata.traj_kinematic：位置 quintic 样条 + 姿态 slerp，纯
    numpy，不依赖 TNDQ 代数构造）；TNDQ 仅在适配器边界把位姿导数
    表示为控制律输入格式。运动形态不变：举高 -> 前移 -> 转腕+下降
    到退避位 -> 沿接近轴插入抓取位 -> 静置闭合 -> 带载提升。
    返回 (traj, t_move, t_close)。
    """
    from simdata.traj_kinematic import (CartesianWaypointTrajectory,
                                        KinematicTrajectoryAdapter)
    chain = B601TCPChain(B601_DH_TABLE)
    x_init = chain.fkm(Q_INIT)
    r0 = dq_rotation(x_init)
    p0 = dq_translation(x_init)
    top0 = w2dh(np.array([p0[0], p0[1], GOTO_TOP_Z]))
    top = w2dh(np.array([GRASP_APPROACH_POS[0], GRASP_APPROACH_POS[1],
                         GOTO_TOP_Z]))
    legs = [
        (top0, r0, GOTO_T_LIFT, 0.0),
        (top, r0, GOTO_T_TRANS, 0.0),
        (w2dh(GRASP_APPROACH_POS), R_TOOL_QUAT, GOTO_T_ROTDESC, 0.0),
        (w2dh(SETPOINT_POS), R_TOOL_QUAT, GOTO_T_DESC, GOTO_GRASP_DWELL),
        (w2dh(LIFT_POS), R_TOOL_QUAT, GOTO_T_LIFTLOAD, 0.0),
    ]
    wp = CartesianWaypointTrajectory((p0, r0), legs)
    traj = KinematicTrajectoryAdapter(wp)
    t_move = wp.t_total
    t_close = GOTO_T_LIFT + GOTO_T_TRANS + GOTO_T_ROTDESC + GOTO_T_DESC + 0.2
    return traj, t_move, t_close


def joint_safety_governor(q, q_dot, qddot_ref):
    """关节级安全治理器（TNDQ_sim 场景篇 §6.3 的 B601 移植）。

    1) 速度限幅：|q̇_i| 超保守额定速度（QDOT_MAX）且指令同向加速时，
       改为按 A_BRAKE 制动；
    2) 限位预测制动：以最大制动减速度停车仍将进入限位缓冲区
       （刹车距离 q̇²/(2 A_BRAKE)）时强制反向减速。

    治理修正量经 (5.1) 归入 w_dyn -> d(t)，由定理 3(c)/(d) 证书兜底
    （诚实条款：触发即计数并写入汇总）。B601 限位非对称（j2/j3 上限
    为 0），soft 限按 JOINT_LOWER/UPPER 双侧内缩 LIMIT_BUFFER。
    返回 (修正后 q̈_ref, 是否触发)。
    """
    qdd = qddot_ref.copy()
    governed = False
    soft_lo = JOINT_LOWER + LIMIT_BUFFER
    soft_hi = JOINT_UPPER - LIMIT_BUFFER
    for i in range(q.shape[0]):
        # 1) 额定速度限幅
        if abs(q_dot[i]) > QDOT_MAX[i] and qdd[i] * q_dot[i] > 0.0:
            qdd[i] = -A_BRAKE * np.sign(q_dot[i])
            governed = True
        # 2) 限位预测制动（只对朝限位方向运动的关节生效）
        if q_dot[i] > 0.0:
            if q[i] + q_dot[i] ** 2 / (2.0 * A_BRAKE) > soft_hi[i]:
                qdd[i] = min(qdd[i], -A_BRAKE)
                governed = True
        elif q_dot[i] < 0.0:
            if q[i] - q_dot[i] ** 2 / (2.0 * A_BRAKE) < soft_lo[i]:
                qdd[i] = max(qdd[i], A_BRAKE)
                governed = True
    return qdd, governed


# ---------------------------------------------------------------------------
# CSV 记录器（B601 验证定制列）
# ---------------------------------------------------------------------------

_CSV_COLUMNS = [
    "t", "pos_err", "ori_err", "e_xi_norm", "e_z_O_norm", "e_z_T_norm",
    "qddot_ref_norm", "tau_norm", "sigma_min", "c0", "c1", "c2", "runtime",
    *[f"q{i + 1}" for i in range(6)],
    *[f"tau{i + 1}" for i in range(6)],
    "px", "py", "pz", "pd_x", "pd_y", "pd_z",
    "cube_x", "cube_y", "cube_z", "cube_yaw",
    "gripper_width", "tau_sat", "governed", "dither_on",
    *[f"meas{i + 1}" for i in range(6)],   # 实测力矩（诊断：指令 vs 执行）
]


class ExperimentCSV:
    """轻量 CSV 追加式记录器（pandas 友好，缺省字段记 nan）。"""

    def __init__(self, path):
        path = os.path.abspath(path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        self._f = open(path, "w", newline="")
        self._f.write(",".join(_CSV_COLUMNS) + "\n")
        self.n_rows = 0

    def row(self, **kw):
        def fmt(v):
            if isinstance(v, bool):
                return "1" if v else "0"
            return f"{float(v):.9e}"
        self._f.write(",".join(
            fmt(kw.get(c, float("nan"))) for c in _CSV_COLUMNS) + "\n")
        self.n_rows += 1

    def close(self):
        self._f.close()


# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------

def run_tndq_experiment(backend, trajectory, duration, csv_path,
                        gripper_schedule=None, label="", verbose=True):
    """运行一个 TNDQ 力矩级实验并写 CSV，返回摘要 dict。

    backend            已 setup() 的物理后端（Isaac 或未来真机）
    trajectory         evaluate(t) -> dict（x_breve_d/xi_d/xi_dot_d/x_d），
                       期望位置为 DH 基座系（生成时经 w2dh 转换），
                       期望姿态为 TCP 工具系（与 B601TCPChain 同约定）
    duration           实验时长 [s]
    csv_path           输出 CSV 路径
    gripper_schedule   t -> 指间开度目标 [m]；None = 恒 GRIPPER_OPENING
    """
    chain = B601TCPChain(B601_DH_TABLE)
    dyn = B601NominalDynamics()
    gains = GAIN_SETS[DEFAULT_GAIN_SET]
    K_d, k_p = gains["K_d"], gains["k_p"]
    _ctrl_dt = DT * CTRL_EVERY        # 实际控制周期 [s]（状态机时间基准）

    n_steps = int(round(duration / DT))
    backend.reset_to(Q_INIT)

    csv = ExperimentCSV(csv_path)
    sat_steps = gov_steps = 0
    aborted = None
    t_wall0 = time.perf_counter()
    log_t, log_pos_err, log_ori_err = [], [], []
    last_width = None
    # 失速 dither 状态机（任务级判据：pos_err 无改善 + 指令加速度大）
    dither_active = False
    dither_dirs = np.zeros((DITHER_N_JOINTS, 6))
    dither_count = 0
    dither_exit_hold = 0
    _pe_win_steps = int(round(0.1 / _ctrl_dt))    # 0.1 s 采样（控制周期）
    _pe_hist = []                                  # [(k, pos_err), ...]
    _q_hist = []                                   # [(k, max|q|), ...] 位移判据
    _qd_smooth = []                                # max|qd| 滑窗（防抖）
    _stall_run = 0                                 # 判据连续成立步数
    _stall_hist = []                               # 判据成立滑窗（占比制）

    try:
        for k in range(n_steps):
            t = k * DT
            # 控制降采样：CTRL_EVERY 个物理步控制一次（对齐真机
            # ~10 ms 反馈周期），中间步保持上一控制周期力矩
            if k % CTRL_EVERY == 0:
                k_ctrl = k // CTRL_EVERY

                # ---- 传感层 ----
                q, qd = backend.get_joint_state()

                # ---- 限位安全（安全终止，数据保存到当前步）----
                over = check_joint_limits(q)
                if over:
                    aborted = f"t={t:.3f}s 关节 {over} 超限位（安全终止）"
                    break

                # ---- 控制器（计时段）----
                tic = time.perf_counter()

                # [FK 层] 式 (3.4)/(3.5)：q̈=0 链给出 ξ 与 J̇q̇
                fk = chain.fk_outputs(q, qd, q_ddot=None, with_jacobian=True)

                # [期望] TNDQ 解析轨迹 -> HDQ 截断（命题 2 无损）
                des = trajectory.evaluate(t)

                # [误差层] 定理 1/2
                err = full_error_state(fk["x_breve"], des["x_breve_d"])

                # 奇异监控：σ_min 过小时提升阻尼（残差计入 d(t)，诚实条款 (i)）
                sig_min = float(np.linalg.svd(fk["J"], compute_uv=False)[-1])
                damping = PINV_DAMPING
                if sig_min < SINGULARITY_TOL:
                    damping = SINGULARITY_DAMPING

                # [控制层] 式 (5.2)（6R 非冗余，无零空间项）；力矩级被控
                # 对象传 M => 动力学一致广义逆（防向轻腕方向泄能）
                qddot_ref, _ = geometric_computed_torque_law(
                    err, des["xi_d"], des["xi_dot_d"],
                    fk["J"], fk["Jdot_qdot"], K_d, k_p, damping=damping,
                    M=dyn.mass_matrix(q))

                # 指令范数限幅（饱和残差计入 d(t)）
                qn = float(np.linalg.norm(qddot_ref))
                if qn > QDDOT_MAX:
                    qddot_ref = qddot_ref * (QDDOT_MAX / qn)

                # 关节级安全治理
                qddot_ref, governed = joint_safety_governor(q, qd, qddot_ref)
                gov_steps += int(governed)

                # [力矩层] τ = M̂ q̈_ref + Ĉ q̇ + ĝ（§2.4）+ 关节阻尼注入
                # （-D q̇：耗散项，抑制轻腕关节沿 task 反馈不敏感方向的
                #   残差漂移；物理对应真机电机阻尼/摩擦）+ 力矩限幅
                tau = dyn.computed_torque(q, qd, qddot_ref)
                tau = tau - JOINT_DAMPING * qd
                tau, sat = clip_torque(tau)
                sat_steps += int(sat)
                runtime = time.perf_counter() - tic

                # ---- 夹爪目标（仅在变化时下发）----
                width_cmd = (gripper_schedule(t)
                             if gripper_schedule is not None
                             else GRIPPER_OPENING)
                if width_cmd != last_width:
                    backend.set_gripper(width_cmd)
                    last_width = width_cmd

                # ---- 失速检测 + 交变 dither（PhysX TGS 组合锁对策）----
                dither_applied = False
                # 任务级失速判据：指令要求动（qddot_ref 大）、关节没真动
                # （速度低）、且 e_comb（位置+加权姿态）在 1 s 窗内无改善。
                # 组合锁下臂仍有 0.01~0.12 rad/s 蠕变，纯速度阈值会漏检。
                _pe_now = _comb_err(fk["x"], des["x_d"])
                if k_ctrl % _pe_win_steps == 0:
                    _pe_hist.append((k_ctrl, _pe_now))
                    _q_hist.append((k_ctrl, q.copy()))
                    if len(_pe_hist) > 64:
                        _pe_hist.pop(0)
                        _q_hist.pop(0)
                _stall = False
                _qd_smooth.append(float(np.max(np.abs(qd))))
                if len(_qd_smooth) > int(round(STALL_QD_SMOOTH / _ctrl_dt)):
                    _qd_smooth.pop(0)
                if (float(np.linalg.norm(qddot_ref)) > STALL_QDD_MIN
                        and float(np.linalg.norm(des["xi_d"])) < STALL_XID_MAX
                        and np.mean(_qd_smooth) < STALL_QD_MAX):
                    _win = [pe for kk, pe in _pe_hist
                            if k_ctrl - kk
                            <= int(round(STALL_PE_WINDOW / _ctrl_dt))]
                    _qw = [qq for kk, qq in _q_hist
                           if k_ctrl - kk
                           <= int(round(STALL_PE_WINDOW / _ctrl_dt))]
                    _dq_win = (float(np.max(np.abs(q - _qw[0])))
                               if len(_qw) >= 2 else float("inf"))
                    if (len(_win) >= 2
                            and (_pe_now - min(_win)
                                 > -STALL_PE_MIN_IMPROVE)
                            and _dq_win < STALL_DQ_MAX):  # 真不动才算锁
                        _stall = True
                # 门限：占比制持续成立 + 误差足够大（防 goto 段误触发）
                _stall_run = _stall_run + 1 if _stall else 0
                _stall_hist.append(_stall)
                if len(_stall_hist) > int(round(STALL_WIN_SUSTAIN / _ctrl_dt)):
                    _stall_hist.pop(0)
                _stall_enough = (
                    len(_stall_hist)
                    >= int(round(STALL_WIN_SUSTAIN / _ctrl_dt))
                    and sum(_stall_hist)
                    >= STALL_FRACTION * len(_stall_hist)
                    and _stall_run >= STALL_MIN_RUN)
                if not _stall_enough or _pe_now < STALL_PE_FLOOR:
                    _stall = False
                if not ENABLE_DITHER:
                    _stall = False          # 总开关关闭（params 注释）
                if _pe_now < DITHER_ACTIVATE_MIN_ERR:
                    _stall = False          # 蠕变收敛相不逃逸（见常量注释）
                if not dither_active and _stall:
                    # 进入：微分探针选组合误差下降最快的解锁关节（v3.3）；
                    # 探针改善过小则退回反馈主分量（j1 除外）
                    j_p, delta_p, bias_sgn = _probe_dither_joint(
                        backend, dyn, chain, q, fk["x"], des["x_d"])
                    if -delta_p < DITHER_PROBE_DELTA_MIN:
                        net_fb = tau - dyn.coriolis_plus_gravity(q, qd)
                        net_fb[0] = 0.0
                        j_p = int(np.argmax(np.abs(net_fb)))
                        bias_sgn = float(np.sign(net_fb[j_p]) or 1.0)
                    # 探针后重读状态（已 reset 回停滞构型）
                    q, qd = backend.get_joint_state()
                    dither_dirs = np.zeros((DITHER_N_JOINTS, 6))
                    dither_dirs[0, j_p] = bias_sgn
                    dither_active = True
                    dither_exit_hold = 0
                    dither_e_entry = _pe_now      # 回升护栏基准
                    dither_burst_left = int(round(min(
                        DITHER_BURST_MAX,
                        _pe_now * DITHER_BURST_PER_ERR) / _ctrl_dt))
                if dither_active:
                    # v3.4 定幅短脉冲：探针实测下降方向开环推，时长与
                    # 误差成正比（probe 标定：1 N·m×0.1 s 降 4.6 cm）；
                    # 脉冲到期交回反馈，未达标则判据再触发重新探针。
                    # dither14 实测闭环幅值自适应在锁区边缘振荡（proj
                    # 随构型变号），开环定幅标定脉冲为最终形态。
                    if _pe_now < DITHER_STOP_ERR:
                        dither_active = False          # 近收敛，防超调
                    elif dither_burst_left <= 0:
                        dither_active = False          # 脉冲到期交回反馈
                    elif _pe_now - dither_e_entry > DITHER_REGRESS_MAX:
                        dither_active = False          # 错推回升：退出重探针
                    if dither_active:
                        # 开环定幅脉冲：重力/科氏/阻尼基线 + 探针方向
                        # 单关节 DITHER_PUSH_MAX；超速即退出交回反馈
                        # （dither16 实测：超速滑行后速度回落又用旧方向
                        #   续推，构型已变仍同向推撞限位）
                        j_p = int(np.argmax(np.abs(dither_dirs[0])))
                        sgn = float(np.sign(dither_dirs[0, j_p]) or 1.0)
                        _lim_room = ((JOINT_UPPER[j_p] - LIMIT_BUFFER
                                      - q[j_p]) if sgn > 0 else
                                     (q[j_p] - JOINT_LOWER[j_p]
                                      - LIMIT_BUFFER))
                        if _lim_room < DITHER_LIMIT_MARGIN:
                            dither_active = False   # 限位保护：重探针
                        elif float(np.max(np.abs(qd))) <= DITHER_VEL_KILL:
                            tau = (dyn.coriolis_plus_gravity(q, qd)
                                   - JOINT_DAMPING * qd)
                            tau[j_p] += DITHER_PUSH_MAX * sgn
                            dither_burst_left -= 1
                            dither_applied = True
                        else:
                            dither_active = False   # 超速：交回反馈
                        dither_count += 1
                if dither_applied:
                    tau, _ = clip_torque(tau)   # dither 叠加后重新限幅

            # ---- 下发 + 物理步进（中间步保持上一控制周期力矩）----
            backend.apply_arm_torques(tau)
            backend.step()

            # ---- 记录（LOG_EVERY 步一行）----
            if k % LOG_EVERY == 0:
                p_w = dq_translation(fk["x"]) + B601_BASE_PREFIX
                pd_w = dq_translation(des["x_d"]) + B601_BASE_PREFIX
                r, r_d = dq_rotation(fk["x"]), dq_rotation(des["x_d"])
                pos_err = float(np.linalg.norm(p_w - pd_w))
                ori_err = float(2.0 * np.arccos(
                    min(1.0, abs(float(r @ r_d)))))
                cube_pos, cube_quat = backend.get_cube_pose()
                cube_yaw = float(np.arctan2(
                    2.0 * (cube_quat[0] * cube_quat[3]
                           + cube_quat[1] * cube_quat[2]),
                    1.0 - 2.0 * (cube_quat[2] ** 2 + cube_quat[3] ** 2)))
                # 实测力矩（诊断：指令 vs 物理层实际执行；纯仿真后端无此
                # 方法时填 NaN）
                _meas = (backend.get_measured_joint_efforts()
                         if hasattr(backend, "get_measured_joint_efforts")
                         else np.full(6, np.nan))
                csv.row(
                    t=t, pos_err=pos_err, ori_err=ori_err,
                    e_xi_norm=np.linalg.norm(err["e_xi"]),
                    e_z_O_norm=np.linalg.norm(err["e_z"][:3]),
                    e_z_T_norm=np.linalg.norm(err["e_z"][3:]),
                    qddot_ref_norm=np.linalg.norm(qddot_ref),
                    tau_norm=np.linalg.norm(tau),
                    sigma_min=sig_min,
                    c0=fk["c0"], c1=fk["c1"], c2=fk["c2"],
                    runtime=runtime,
                    **{f"q{i + 1}": q[i] for i in range(6)},
                    **{f"tau{i + 1}": tau[i] for i in range(6)},
                    px=p_w[0], py=p_w[1], pz=p_w[2],
                    pd_x=pd_w[0], pd_y=pd_w[1], pd_z=pd_w[2],
                    cube_x=cube_pos[0], cube_y=cube_pos[1],
                    cube_z=cube_pos[2], cube_yaw=cube_yaw,
                    gripper_width=backend.get_gripper_width(),
                    tau_sat=bool(sat), governed=bool(governed),
                    dither_on=bool(dither_active),
                    **{f"meas{i + 1}": float(_meas[i]) for i in range(6)})
                log_t.append(t)
                log_pos_err.append(pos_err)
                log_ori_err.append(ori_err)

            if verbose and k % int(round(5.0 / DT)) == 0 and k > 0:
                print(f"[{label}] t={t:6.2f}s  pos_err={log_pos_err[-1]:.4f} m"
                      f"  ori_err={log_ori_err[-1]:.4f} rad"
                      f"  ‖τ‖={np.linalg.norm(tau):.2f} N·m", flush=True)

    except KeyboardInterrupt:
        aborted = "用户中断（数据已保存到当前步）"
    finally:
        csv.close()

    # ---- 摘要 ----
    log_t = np.array(log_t)
    log_pos_err = np.array(log_pos_err)
    log_ori_err = np.array(log_ori_err)
    tail = log_t >= (duration - 1.0) if len(log_t) else np.array([], bool)
    summary = {
        "csv_path": csv.path,
        "n_rows": csv.n_rows,
        "duration": duration,
        "wall_time": time.perf_counter() - t_wall0,
        "sat_steps": sat_steps,
        "gov_steps": gov_steps,
        "dither_steps": dither_count,
        "aborted": aborted,
        "pos_err_final": float(np.mean(log_pos_err[tail])) if np.any(tail)
        else float("nan"),
        "ori_err_final": float(np.mean(log_ori_err[tail])) if np.any(tail)
        else float("nan"),
        "pos_err_peak": float(np.max(log_pos_err)) if len(log_pos_err)
        else float("nan"),
        "ori_err_peak": float(np.max(log_ori_err)) if len(log_ori_err)
        else float("nan"),
    }
    if verbose:
        print(f"[{label}] 完成：{csv.n_rows} 行 -> {csv.path}\n"
              f"    末端 1s 稳态窗: pos_err={summary['pos_err_final']:.4f} m"
              f"  ori_err={summary['ori_err_final']:.4f} rad\n"
              f"    全程峰值: pos_err={summary['pos_err_peak']:.4f} m"
              f"  ori_err={summary['ori_err_peak']:.4f} rad\n"
              f"    力矩饱和 {sat_steps} 步 / 治理触发 {gov_steps} 步 / "
              f"dither {dither_count} 步 / "
              f"墙钟 {summary['wall_time']:.1f}s"
              + (f"\n    [abort] {aborted}" if aborted else ""), flush=True)
    return summary
