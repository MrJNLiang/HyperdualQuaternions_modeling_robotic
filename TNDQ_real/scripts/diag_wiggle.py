"""
TNDQ_real/scripts/diag_wiggle.py —— 手动摇动活性测试（电机失能态，手掰安全）。

单串口会话两段式（close 不释放 fd，进程内不可二次 open）：
  阶段 1（无需动作，5 s）：motor5 以 model="JH11" 做反馈帧探针；
  阶段 2（需人手摇关节 5，20 s）：每 0.5 s 打印
    motor1 反馈 pos / motor5 反馈 pos（JH11 句柄，兼测自主上报）/
    motor5 mechPos(0x7019) 寄存器；
    观察哪个通道随手摇变化 => 该通道为活信号。

安全前提：全程不使能、不下发任何控制指令，只读。
运行：python scripts/diag_wiggle.py（rebot 环境；串口需空闲；人在臂旁）
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
            print(f"   open 失败({k + 1}/{tries}): {exc}；1 s 后重试")
            time.sleep(1.0)
    raise RuntimeError("串口持续 busy：确认无其他进程/网关占用后重跑")


def main():
    print("== 阶段 1：JH11 反馈探针（5 s，无需动作）")
    with open_with_retry() as c:
        m1 = c.add_damiao_motor(0x01, 0x11, "4340P")
        m5 = c.add_damiao_motor(0x05, 0x15, "JH11")
        hit = False
        for i in range(100):
            try:
                m5.request_feedback()
            except Exception:  # noqa: BLE001
                break
            c.poll_feedback_once()
            st = m5.get_state()
            if st is not None:
                print(f"   JH11 第{i}拍解出 pos={st.pos:.4f} "
                      f"vel={st.vel:.4f} torq={st.torq:.4f}")
                hit = True
                break
            time.sleep(0.05)
        if not hit:
            print("   JH11 5 s 无反馈帧")

        print("== 阶段 2：手动摇动测试")
        print("   3 s 后开始采样；请用手缓慢来回转动关节 5（腕部）约 15 s。")
        print("   全程电机失能，手掰安全。")
        time.sleep(3.0)
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 20.0:
            m1.request_feedback()
            m5.request_feedback()
            c.poll_feedback_once()
            c.poll_feedback_once()
            s1 = m1.get_state()
            s5 = m5.get_state()
            try:
                r5 = m5.get_register_f32(0x7019, 200)
            except Exception:  # noqa: BLE001
                r5 = float("nan")
            p1 = s1.pos if s1 is not None else float("nan")
            p5 = s5.pos if s5 is not None else float("nan")
            print(f"   t={time.perf_counter() - t0:5.1f}s  "
                  f"m1_fb={p1:+.4f}  m5_fb={p5:+.4f}  m5_mechPos={r5:+.4f}")
            time.sleep(0.5)
    print("== 完成。判读：m5_fb 或 m5_mechPos 随手摇明显变化者为活通道。")


if __name__ == "__main__":
    main()
