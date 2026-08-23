"""
TNDQ_real/scripts/01_calib_signs.py —— 关节符号/零位标定（交互式）。

阶段：
  A. 电机系重力保持（厂商动力学，臂稳定可掰动）；
  B. 逐关节 +1 N*m x 0.15 s 脉冲：打印 raw 位移，操作者对照 URDF 正
     方向描述判符号（人机协同，物理方向无法纯代码判定）；
  C. 手动掰臂到 Q_INIT 姿态 -> 抓零位 OFFSET；
  D. FK 输出当前 TCP 位置供卷尺复核，打印 transforms.py 回填块。

安全：全程重力保持托臂；Ctrl+C 立即失能。符号的最终裁决在 02 的
URDF 系重力保持：符号错则保持立即发散（2 s 短窗观察即可发现）。

运行：uv run python scripts/01_calib_signs.py（工作目录 = TNDQ_real/）
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from config.params import (B601_BASE_PREFIX, B601_DH_TABLE,  # noqa: E402
                           B601_TOOL_ANGLE, Q_INIT)

# URDF 正方向物理描述（右手定则，对照脉冲运动方向人工判符号）
URDF_POS_DIR = [
    "j1: 绕基座竖直轴；俯视逆时针为正",
    "j2: 肩部俯仰；抬臂（link2 远离基座方向抬起）为正——以 URDF 几何为准",
    "j3: 肘部俯仰；使腕部抬升的方向为正——以 URDF 几何为准",
    "j4: 绕小臂轴滚转；按右手定则沿 link4 轴向",
    "j5: 腕俯仰；按右手定则",
    "j6: 腕滚转（夹爪开合向旋转）；按右手定则",
]


def main():
    from reBotArm_control_py.actuator import RebotArm
    from reBotArm_control_py.dynamics import compute_generalized_gravity

    robot = RebotArm()
    robot.connect()
    extra = np.zeros(6)                 # 脉冲注入（主线程写，控制线程读）
    raw = np.zeros(6)                   # 最新 raw 关节角（电机系）

    def ctrl(r, dt):
        q = r.arm.get_positions(request_feedback=True)
        raw[:] = q[:6]
        tau = compute_generalized_gravity(q=q[:6]) + extra
        r.arm.send_mit(pos=q[:6], vel=np.zeros(6),
                       kp=np.ones(6), kd=np.full(6, 0.5), tau=tau)

    robot.arm.mode_mit()
    robot.enable_all()
    robot.start_control_loop(ctrl, rate=200)
    print("[01] 重力保持已启动：臂应稳定悬停（厂商模型，电机系）")

    sign = np.ones(6)
    try:
        # ---- 阶段 B：逐关节脉冲判符号 ----
        for i in range(6):
            input(f"\n[01-B] 即将脉冲关节 j{i + 1}（+1 N*m, 0.15 s）；"
                  f"请目视该关节，Ctrl+C 急停。回车开始...")
            time.sleep(0.3)
            q_before = raw.copy()
            extra[i] = 1.0
            time.sleep(0.15)
            extra[i] = 0.0
            time.sleep(0.3)
            dq = raw[i] - q_before[i]
            print(f"[01-B] j{i + 1}: raw 位移 {dq:+.4f} rad"
                  f"（{'raw 正方向' if dq > 0 else 'raw 负方向'}运动）")
            print(f"        URDF 正方向定义：{URDF_POS_DIR[i]}")
            ans = input("        观察到的物理运动与 URDF 正方向一致？[y/n] ")
            sign[i] = 1.0 if ans.strip().lower() == "y" else -1.0

        # ---- 阶段 C：掰臂到 Q_INIT 抓零位 ----
        print(f"\n[01-C] 现在请手动把臂掰到 Q_INIT 姿态："
              f"{np.round(Q_INIT, 3).tolist()} rad")
        print("       （近似零位：j2/j3 各向内 -0.10 rad≈-5.7°，其余 0；"
              "保持重力托住即可掰动）")
        input("       掰到位后回车抓取零位...")
        offset = Q_INIT - sign * raw.copy()
        q_now = sign * raw + offset
        print(f"[01-C] OFFSET 抓取完成：当前 URDF 系 q="
              f"{np.round(q_now, 4).tolist()}（应≈Q_INIT）")

        # ---- 阶段 D：FK 复核（卷尺测 gripper 原点 = 指尖端）----
        from core.dq_algebra import dq_mul, dq_rot_z, dq_translation
        from core.kinematics import TNDQSerialChain

        chain = TNDQSerialChain(B601_DH_TABLE)
        x = dq_mul(chain.fkm(q_now), dq_rot_z(B601_TOOL_ANGLE))
        p = np.asarray(dq_translation(x), float) + B601_BASE_PREFIX
        print(f"[01-D] FK 预测 gripper 原点（base_link 系）="
              f"{np.round(p, 4).tolist()} m")
        print("       请用卷尺复核实测指尖端位置；偏差 > 1 cm 说明符号"
              "或零位有误（FK 呈镜像 = 某关节符号反）")

        # ---- 回填块 ----
        print("\n========== 回填 TNDQ_real/config/transforms.py ==========")
        print(f"JOINT_SIGN = np.array({sign.tolist()})")
        print(f"JOINT_OFFSET = np.array({np.round(offset, 6).tolist()})")
        print("=========================================================")
        print("下一步：回填后运行 scripts/02_gravity_hold.py 做 URDF 系"
              "重力保持（符号终审 + TAU_SCALE 整定）")
    finally:
        robot.stop_control_loop()
        robot.disable_all()
        robot.disconnect()
        print("[01] 已失能断连")


if __name__ == "__main__":
    main()
