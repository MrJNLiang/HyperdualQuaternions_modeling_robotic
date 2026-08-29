"""
TNDQ_real/scripts/diag_wrist_map.py —— 腕部 5/6 电机 CAN ID 映射终审。

背景：yaml 维持出厂顺接（joint5=0x05 竖直腕偏航、joint6=0x06 黑筒滚转；
URDF 零位实算：joint5 轴 -z（偏航）、joint6 轴 +x（滚转），与 Studio
点动观测一致）。本脚本物理终审：使能后只手掰「黑筒滚转」（夹爪旋转
那个关节），看 raw 第 5 列还是第 6 列在动：
  只有 joint6（raw[5]）动 -> 顺接正确，保持 yaml；
  只有 joint5（raw[4]）动 -> ID 反接，需交换 yaml 两行。

安全：仅使能 + 重力保持（有界前馈），无主动运动指令；Ctrl+C 失能。
运行：python scripts/diag_wrist_map.py（工作目录 = TNDQ_real/）
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402


def main():
    from reBotArm_control_py.actuator import RebotArm
    from reBotArm_control_py.dynamics import compute_generalized_gravity

    robot = RebotArm()
    robot.connect()
    raw = np.zeros(6)
    # 同 01：raw->模型系暂定符号 + 逐关节重力标度；力矩须乘 SIGN（逆变换）
    SIGN_HOLD = np.array([1.0, 1.0, -1.0, -1.0, 1.0, 1.0])
    scale = np.array([1.0, 1.55, 1.55, 1.55, 1.0, 1.0])

    def ctrl(r, dt):
        q = r.arm.get_positions(request_feedback=True)
        raw[:] = q[:6]
        g = compute_generalized_gravity(q=SIGN_HOLD * q[:6])[:6]
        r.arm.send_mit(pos=q[:6], vel=np.zeros(6),
                       kp=np.ones(6), kd=np.full(6, 1.0),
                       tau=SIGN_HOLD * (g * scale))

    robot.arm.mode_mit()
    robot.enable_all()
    robot.start_control_loop(ctrl, rate=200)
    try:
        input("[map] 重力保持已启动。回车开始采样，随后 5 s 内只来回掰"
              "「黑筒滚转」（夹爪旋转那个关节），别碰其他关节...")
        lo = raw.copy()
        hi = raw.copy()
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 5.0:
            lo = np.minimum(lo, raw)
            hi = np.maximum(hi, raw)
            time.sleep(0.02)
        rng = hi - lo
        print(f"[map] 5 s 采样逐关节位移幅度 [rad]：{np.round(rng, 4).tolist()}")
        movers = [i + 1 for i in range(6) if rng[i] > 0.05]
        if movers == [6]:
            print("[map] 只有 joint6 动 -> 顺接正确（黑筒滚转=joint6=0x06），保持 yaml。")
        elif movers == [5]:
            print("[map] 只有 joint5 动 -> ID 反接！需交换 yaml 的 joint5/joint6 映射。")
        else:
            print(f"[map] 动的关节={movers}：混入其他关节或没动，"
                  f"重跑本测试，5 s 内只掰黑筒滚转。")
    finally:
        robot.stop_control_loop()
        robot.disable_all()
        robot.disconnect()
        print("[map] 已失能断连")


if __name__ == "__main__":
    main()
