"""
TNDQ_real/scripts/diag_model_probe.py —— 电机型号解码探针（只读，不使能，零风险）。

背景：新批次 5-7 号电机反馈帧在 model="4310" 下不解码；
native 库型号表含 4310P/4340P/... 而无裸 "4310"。
本脚本对指定 (motor_id, feedback_id) 逐个候选型号做
request_feedback + poll，报告哪个型号能解出反馈。

运行：python scripts/diag_model_probe.py（rebot 环境；串口需空闲）
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from motorbridge import Controller  # noqa: E402

CANDIDATES = ["4310P", "4310", "4340P", "10010L", "3510"]
TARGETS = [
    (0x05, 0x15, "joint5"),
    (0x06, 0x16, "joint6"),
    (0x07, 0x17, "gripper"),
    (0x04, 0x14, "joint4(旧批次对照)"),
]


def probe(motor_id: int, fb_id: int, model: str):
    c = Controller.from_dm_serial("/dev/ttyACM0", 921600)
    try:
        m = c.add_damiao_motor(motor_id, fb_id, model)
        for i in range(60):
            try:
                m.request_feedback()
            except Exception:  # noqa: BLE001
                break
            c.poll_feedback_once()
            st = m.get_state()
            if st is not None:
                return f"model={model}: 第{i}拍解出 pos={st.pos:.4f} " \
                       f"vel={st.vel:.4f} torq={st.torq:.4f}"
            time.sleep(0.005)
        return f"model={model}: 60 拍无反馈"
    finally:
        c.close()


def main():
    for mid, fid, label in TARGETS:
        print(f"== {label} (0x{mid:02X}/0x{fid:02X})")
        for model in CANDIDATES:
            print("  ", probe(mid, fid, model))


if __name__ == "__main__":
    main()
