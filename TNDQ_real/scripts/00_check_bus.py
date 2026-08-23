"""
TNDQ_real/scripts/00_check_bus.py —— 总线连通性检查（不使能，零风险）。

检查项：串口存在/权限 -> connect -> 7 电机反馈回读 -> 与 Q_INIT 对照。
运行：uv run python scripts/00_check_bus.py（工作目录 = TNDQ_real/）
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from config.params import Q_INIT  # noqa: E402


def main():
    ch = "/dev/ttyACM0"
    if not os.path.exists(ch):
        sys.exit(f"[FAIL] {ch} 不存在：检查 USB 连接，dmesg | grep ttyACM")
    if not os.access(ch, os.R_OK | os.W_OK):
        sys.exit(f"[FAIL] {ch} 无读写权限：sudo usermod -aG dialout $USER"
                 f"（重新登录后生效），或临时 sudo chmod 666 {ch}")

    from reBotArm_control_py.actuator import RebotArm
    r = RebotArm()
    r.connect()
    try:
        pos, vel, torq = r.get_state(request_feedback=True)
        print(f"[check] 反馈 pos ={[round(float(v), 4) for v in pos]}")
        print(f"[check] 反馈 vel ={[round(float(v), 4) for v in vel]}")
        print(f"[check] 反馈 torq={[round(float(v), 4) for v in torq]}")
        if pos.shape[0] < 7:
            sys.exit(f"[FAIL] 反馈关节数 {pos.shape[0]} < 7（arm6+gripper1）")
        if all(abs(float(v)) < 1e-12 for v in pos):
            sys.exit("[FAIL] 反馈全零：总线无通信（检查桥接器供电/CAN 接线）")
        print(f"[check] Q_INIT={Q_INIT.tolist()}（恒等变换下的对照参考；"
              f"符号/零位标定后本列才有物理意义）")
        print("[PASS] 总线连通 OK，可进入 01 标定")
    finally:
        r.disconnect()


if __name__ == "__main__":
    main()
