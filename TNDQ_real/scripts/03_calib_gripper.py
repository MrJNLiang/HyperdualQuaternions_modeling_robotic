"""
TNDQ_real/scripts/03_calib_gripper.py —— 夹爪电机角<->指间开度标定。

流程：对 3 个目标开度下发 MIT 位置保持 -> 卷尺实测指间开度 -> 输入
-> 最小二乘回归 width = a * q_motor + b；回填 GRIPPER_M_PER_RAD=|a|、
GRIPPER_SIGN=sign(a)。

运行：uv run python scripts/03_calib_gripper.py（工作目录 = TNDQ_real/）
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from config.transforms import GRIPPER_M_PER_RAD, GRIPPER_SIGN  # noqa: E402


def main():
    from reBotArm_control_py.actuator import RebotArm

    robot = RebotArm()
    robot.connect()
    if not robot.has_gripper:
        sys.exit("[FAIL] 配置中无 gripper 组")
    robot.gripper.mode_mit()
    robot.gripper.enable()

    widths = [0.12, 0.08, 0.04]          # 目标指间开度 [m]
    qs, ws = [], []
    try:
        for w in widths:
            target_q = GRIPPER_SIGN * w / GRIPPER_M_PER_RAD
            for _ in range(30):          # 1 s @30Hz 位置保持
                robot.gripper.send_mit(pos=np.array([target_q]),
                                       vel=np.zeros(1), kp=np.array([4.0]),
                                       kd=np.array([0.5]), tau=np.zeros(1))
                time.sleep(1.0 / 30.0)
            gq = float(robot.gripper.get_positions(request_feedback=True)[0])
            meas = float(input(
                f"[03] 目标开度 {w * 1000:.0f} mm（电机角 {gq:+.3f} rad）"
                f"——卷尺实测指间开度 [mm]: ")) / 1000.0
            qs.append(gq)
            ws.append(meas)
    finally:
        robot.gripper.disable()
        robot.disconnect()

    a, b = np.polyfit(np.asarray(qs), np.asarray(ws), 1)
    resid_mm = float(np.max(np.abs(np.polyval([a, b], qs) - ws))) * 1000.0
    print(f"\n[03] 回归：width = {a:+.6f} * q_motor {b:+.6f}"
          f"（最大拟合残差 {resid_mm:.2f} mm）")
    print("========== 回填 TNDQ_real/config/transforms.py ==========")
    print(f"GRIPPER_M_PER_RAD = {abs(a):.6f}")
    print(f"GRIPPER_SIGN = {np.sign(a):+.1f}")
    print("=========================================================")


if __name__ == "__main__":
    main()
