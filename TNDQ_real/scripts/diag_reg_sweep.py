"""
TNDQ_real/scripts/diag_reg_sweep.py —— 寄存器全表扫描+摇动活性筛选（只读，零风险）。

背景：新批次 DM-J4310（5/6）无反馈帧；Studio 只读固定寄存器。
motorbridge 的 get_register_f32 可扫 0..255 任意 rid：
  阶段 1：扫全表，收集可读 rid 及静态值；
  阶段 2（手摇关节 5，约 12 s）：对可读 rid 循环复读，
    筛出随手摇变化的 rid => 新固件活位置寄存器候选。

安全前提：全程不使能、不下发控制指令，只读。
运行：python scripts/diag_reg_sweep.py（rebot 环境；串口需空闲；人在臂旁）
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
        m5 = c.add_damiao_motor(0x05, 0x15, "4310")

        print("== 阶段 1：rid 0..255 全表扫描（约数十秒）")
        alive = {}
        for rid in range(256):
            try:
                v = m5.get_register_f32(rid, 40)
                alive[rid] = v
            except Exception:  # noqa: BLE001
                continue
        print(f"   可读 rid 共 {len(alive)} 个：")
        for rid, v in alive.items():
            print(f"     rid {rid:3d} (0x{rid:02X}) = {v:+.4f}")

        print("== 阶段 2：手摇筛选（3 s 后开始，请来回转动关节 5 约 10 s）")
        time.sleep(3.0)
        span = {rid: [v, v] for rid, v in alive.items()}
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 12.0:
            for rid in list(span):
                try:
                    v = m5.get_register_f32(rid, 40)
                except Exception:  # noqa: BLE001
                    continue
                span[rid][0] = min(span[rid][0], v)
                span[rid][1] = max(span[rid][1], v)
        print("   随手摇变化的 rid（Δ>1e-3）：")
        found = False
        for rid, (lo, hi) in span.items():
            if hi - lo > 1e-3:
                found = True
                print(f"     rid {rid:3d} (0x{rid:02X}) 范围 [{lo:+.4f}, {hi:+.4f}]")
        if not found:
            print("     无。0..255 内不存在随运动变化的寄存器。")


if __name__ == "__main__":
    main()
