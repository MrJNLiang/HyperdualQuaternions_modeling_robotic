"""
TNDQ_real 坐标/力矩标定常量 —— 电机系 <-> URDF/DH 系变换。

★ 上电前必须完成标定并回填本文件（scripts 01_calib_signs 流程）★

变换定义（arm 六关节，joint1..joint6 顺序）：
    q_urdf  = JOINT_SIGN * q_motor + JOINT_OFFSET
    qd_urdf = JOINT_SIGN * qd_motor
    tau_urdf = TAU_SCALE * (JOINT_SIGN * tau_motor)
    （下发为逆变换：tau_motor = JOINT_SIGN * tau_urdf / TAU_SCALE）

标定依据与步骤：
  [1] JOINT_SIGN（符号）：重力补偿（可掰动）状态下逐关节施加
      +1 N*m x 0.2 s 小脉冲，观察物理运动方向；与 URDF/DH 正方向
      一致记 +1，相反记 -1。厂商代码出现过 q_sim = -q_motor 注释
      （reBot-Isaacsim/reBotArm_Isaacsim/gravity_joint_sender.py），
      说明符号很可能为负，切勿凭假设上闭环。
  [2] JOINT_OFFSET（零位）：电机绝对编码器零位与 URDF 零位
      （verify_dh.py 对账的 DH 链零位）的偏差；掰臂到 2~3 个可
      量具/视觉测定 TCP 位置的姿态，用 FK 残差最小二乘拟合。
  [3] TAU_SCALE（力矩标度）：用 B601NominalDynamics.gravity_vector
      做重力保持（02_gravity_hold），按关节漂移反解补偿系数。
      厂商自带重力补偿示例存在 tau_g[j2]*=1.2 / [j3,j4]*=2.5 的
      硬编码修正（gravity_joint_sender.py L127-129），提示该链路
      力矩标度/动力学存在可观偏差，必须实测。

夹爪（gripper 组，单电机拉双指）：
    width_m = GRIPPER_M_PER_RAD * GRIPPER_SIGN * gripper_q_motor
    厂商两种换算并存：0.0073（gravity_joint_sender）/ 0.03
    （README 表格），且下发目标带负号（-1*gripper_q）——以本机
    实测开度回归为准。
"""
import numpy as np

# --- arm 六关节标定常量（恒等变换；2026-08-28 由 Borot-Arm_Mujoco 实机
#     运行证据回填，依据链见下）---
# 依据：Borot-Arm_Mujoco（DM 版，reBotArmController）在本机以恒等变换实机
#   跑通（位置控制/轨迹/网页模型显示均正确），且：
#   [1] 其 reBot-DevArm_fixend.urdf 与本包所用 reBot_B601_DM.urdf 六关节的
#       轴/原点/rpy/限位逐一对齐（仅 joint6 原点 x 差 4 mm，属末端帧定义）；
#   [2] 厂商 SDK 反馈链路无任何符号翻转（驱动源码全查）；
#   [3] Borot 硬件配置 joint_direction/tau_scale 均为默认恒等。
# 即：本机 电机系 == URDF 系。此前 [1,1,-1,-1,1,1] 结论（受被污染标定流程
#   影响）作废，勿再引用。
JOINT_SIGN = np.ones(6)                 # 恒等（Borot 实机证明）
JOINT_OFFSET = np.zeros(6)              # 恒等（Borot IK/轨迹定位正确 => 零位对齐）
TAU_SCALE = np.ones(6)                  # 先取 1；由 02_gravity_hold 漂移整定
                                        #   （下垂 => /=1.2，上漂 => *=1.2）

# --- 夹爪标定常量 ---
# 依据：Borot DATA_FLOW_ZH.md 夹爪链路表——全开 0.09 m 对应电机约 -5 rad，
#   闭合 0.00 m 对应 0 rad（rebotarm_hardware.yaml: open=-5.0, close=0.0）。
GRIPPER_M_PER_RAD = 0.018               # 0.09 m / 5 rad（Borot 实机口径）
GRIPPER_SIGN = -1.0                     # 开度增大对应电机负方向
GRIPPER_WIDTH_MAX = 0.14                # 指间开度上限 [m]（与仿真 GRIPPER_OPENING
                                        #   全开口径一致；物理行程实测后收紧）
