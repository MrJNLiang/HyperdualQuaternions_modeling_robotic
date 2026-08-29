"""
TNDQ_real/scripts/diag_longpoll.py —— 长窗口反馈复检（只读，不使能，零风险）。

背景：Studio 显示夹爪(0x7)有活反馈而 5/6 全死；此前探针窗口太短
可能漏掉慢响应电机。本脚本单会话 15 s 慢轮询，统计 1/5/6/7
各电机首帧拍号与存活情况，给最终结论收尾。

运行：python scripts/diag_longpoll.py（rebot 环境；串口需空闲）
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from motorbridge import Controller  # noqa: E402


def open_with_retry(port="/dev/ttyACM0", baud=921600, tries=5):
    for k in range(tries):
        try:
            return Controller.from_dm_serial(port, baud)
        except Exception as exc:  # noqa: BLE001
            print(f"open 失败({k + 1}/{tries}): {exc}；1 s 后重试")
            time.sleep(1.0)
    raise RuntimeError("串口持续 busy")


def main():
    with open_with_retry() as c:
        motors = {
            1: c.add_damiao_motor(0x01, 0x11, "4340P"),
            5: c.add_damiao_motor(0x05, 0x15, "4310"),
            6: c.add_damiao_motor(0x06, 0x16, "4310"),
            7: c.add_damiao_motor(0x07, 0x17, "4310"),
        }
        first_hit = {}
        t0 = time.perf_counter()
        i = 0
        while time.perf_counter() - t0 < 15.0:
            i += 1
            for m in motors.values():
                try:
                    m.request_feedback()
                except Exception:  # noqa: BLE001
                    pass
            for _ in range(4):
                c.poll_feedback_once()
            for mid, m in motors.items():
                if mid in first_hit:
                    continue
                st = m.get_state()
                if st is not None:
                    first_hit[mid] = (i, st.pos, st.vel, st.torq)
            time.sleep(0.05)
        for mid in (1, 5, 6, 7):
            if mid in first_hit:
                n, p, v, tq = first_hit[mid]
                print(f"motor{mid}: 第{n}拍存活 pos={p:+.4f} "
                      f"vel={v:+.4f} torq={tq:+.4f}")
            else:
                print(f"motor{mid}: 15 s 长轮询仍无反馈帧")


if __name__ == "__main__":
    main()
