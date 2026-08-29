"""
TNDQ_real/scripts/diag_fb_deep.py —— 逐关节聚焦反馈诊断（只读，不使能，零风险）。

对每个电机独立做 100 次 request_feedback + 全控制器 poll，
记录首次拿到非 None 状态的拍号；区分"总线整体问题"与"个别电机问题"。

运行：python scripts/diag_fb_deep.py（rebot 环境；串口需空闲）
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from reBotArm_control_py.actuator import RebotArm  # noqa: E402


def main():
    r = RebotArm()
    r.connect()
    try:
        import motorbridge
        print("motorbridge version:",
              getattr(motorbridge, "__version__", "unknown"))
        for name, m in r._motor_map.items():
            first = None
            for i in range(100):
                try:
                    m.request_feedback()
                except Exception as exc:  # noqa: BLE001
                    print(f"{name}: request 异常 {exc}")
                    break
                for c in r._ctrl_map.values():
                    try:
                        c.poll_feedback_once()
                    except Exception:  # noqa: BLE001
                        pass
                st = m.get_state()
                if st is not None:
                    first = i
                    print(f"{name}: 第 {i} 拍拿到反馈 "
                          f"pos={st.pos:.4f} vel={st.vel:.4f} torq={st.torq:.4f}")
                    break
                time.sleep(0.005)
            if first is None:
                print(f"{name}: 100 拍无任何反馈帧")
    finally:
        r.disconnect()


if __name__ == "__main__":
    main()
