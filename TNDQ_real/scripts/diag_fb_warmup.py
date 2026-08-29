"""
TNDQ_real/scripts/diag_fb_warmup.py —— 带预热的反馈存活诊断（只读，不使能，零风险）。

背景：00_check_bus.py 单拍冷启动读取时，get_state 仅 poll 一次，
J5-J7 响应帧未及到达会被记为 0.0。本脚本连续读 100 拍（2 s）后
判定 7 关节反馈是否全部存活，用于区分"冷启动假象"与"真实通信缺失"。

运行：python scripts/diag_fb_warmup.py（rebot 环境；串口需空闲）
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

import numpy as np  # noqa: E402
from reBotArm_control_py.actuator import RebotArm  # noqa: E402


def main():
    r = RebotArm()
    r.connect()
    try:
        nz = np.zeros(7, dtype=bool)
        pos = vel = torq = None
        for _ in range(100):                      # 100 拍 x 20 ms = 2 s 预热
            pos, vel, torq = r.get_state(request_feedback=True)
            nz |= np.abs(pos) > 1e-9
            time.sleep(0.02)
        print("warmup 后 pos =", np.round(pos, 4).tolist())
        print("warmup 后 vel =", np.round(vel, 4).tolist())
        print("warmup 后 torq=", np.round(torq, 4).tolist())
        print("曾非零掩码   =", nz.tolist())
        if nz.all():
            print("[PASS] 7 关节反馈全部存活：单拍全零为冷启动假象")
        else:
            print("[FAIL] 预热后仍恒零的关节(mask)：", (~nz).tolist())
    finally:
        r.disconnect()


if __name__ == "__main__":
    main()
