"""
TNDQ_real/scripts/diag_reg_pos.py —— 新批次电机寄存器位置读回探针（只读，零风险）。

背景：DM-J4310 新批次（5-7）不回传统反馈帧；Studio 网关走
mechPos(0x7019)/mechVel(0x701B) 寄存器读回。本脚本用
Motor.get_register_f32 通用路径对 7 个电机读这两个寄存器。

运行：python scripts/diag_reg_pos.py（rebot 环境；串口需空闲）
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from motorbridge import Controller  # noqa: E402


def main():
    c = Controller.from_dm_serial("/dev/ttyACM0", 921600)
    try:
        motors = {i: c.add_damiao_motor(i, 0x10 + i, "4310")
                  for i in range(1, 8)}
        for i, m in motors.items():
            for rid, label in [(0x7019, "mechPos"), (0x701B, "mechVel")]:
                try:
                    v = m.get_register_f32(rid, 500)
                    print(f"motor{i} {label}(0x{rid:04X}) = {v:.4f}")
                except Exception as exc:  # noqa: BLE001
                    print(f"motor{i} {label}(0x{rid:04X}) 读取失败: {exc}")
            time.sleep(0.05)
    finally:
        c.close()


if __name__ == "__main__":
    main()
