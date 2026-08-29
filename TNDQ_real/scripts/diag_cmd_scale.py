"""
TNDQ_real/scripts/diag_cmd_scale.py —— POS_VEL 命令通道验证 + xout 刻度标定（首次运动测试）。

原理：达妙 POS_VEL 命令单位确定为 rad（协议文档）。对关节 5 下发
目标 +0.6 rad（vlim=0.2 慢速），读回 xout(0x51)/p_m(0x50)：
  xout≈0.6      => 单位 rad、刻度 1:1，后端直接用；
  xout≈0.6/2π   => 单位 turns，后端乘 2π；
  其他比例      => 按实测比例换算。
随后回零并失能。

安全：单关节、慢速、有界；两段 input 确认；Ctrl+C 或异常自动失能；
      人手悬电源/随时 Ctrl+C。人在臂旁运行。
运行：python scripts/diag_cmd_scale.py（rebot 环境；串口需空闲）
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from motorbridge import Controller  # noqa: E402

TARGET = 0.6
VLIM = 0.2


def open_with_retry(port="/dev/ttyACM0", baud=921600, tries=5):
    for k in range(tries):
        try:
            return Controller.from_dm_serial(port, baud)
        except Exception as exc:  # noqa: BLE001
            print(f"open 失败({k + 1}/{tries}): {exc}；1 s 后重试")
            time.sleep(1.0)
    raise RuntimeError("串口持续 busy")


def read_xout(m):
    try:
        return m.get_register_f32(0x51, 60)
    except Exception:  # noqa: BLE001
        return float("nan")


def goto(m, target, secs=12.0):
    """周期重发 pos_vel（防 TIMEOUT 失能），直到 xout 稳定或超时。"""
    t0 = time.perf_counter()
    last = read_xout(m)
    stable = 0.0
    while time.perf_counter() - t0 < secs:
        m.send_pos_vel(target, VLIM)
        time.sleep(0.2)
        x = read_xout(m)
        print(f"   t={time.perf_counter() - t0:4.1f}s  xout={x:+.4f}")
        if abs(x - last) < 2e-3:
            stable += 0.2
            if stable > 1.0:
                return x
        else:
            stable = 0.0
        last = x
    return last


def main():
    with open_with_retry() as c:
        m5 = c.add_damiao_motor(0x05, 0x15, "4310")
        x_rest = read_xout(m5)
        print(f"静置 xout ≈ {x_rest:+.4f}")
        print("!! 首次运动测试：关节 5 将慢速转到 +0.6 rad 再回零。")
        print("!! 确认腕部运动范围无障碍、人手离开臂，随时 Ctrl+C（自动失能）。")
        input("   准备好后按回车使能 …")
        try:
            m5.enable()
            time.sleep(0.5)
            print(f"== 下发 pos_vel(+{TARGET}, vlim={VLIM}) …")
            x_hit = goto(m5, TARGET)
            print(f"到位 xout = {x_hit:+.4f}（命令 {TARGET:+.4f}）")
            ratio = x_hit / TARGET if abs(TARGET) > 1e-9 else float("nan")
            print(f"比例 xout/cmd = {ratio:.4f}")
            if abs(ratio - 1.0) < 0.05:
                print("判读：xout 单位 = rad，刻度 1:1 ✔")
            elif abs(ratio - 1.0 / (2 * 3.14159265)) < 0.01:
                print("判读：xout 单位 = turns，后端需 ×2π")
            else:
                print("判读：非常规比例，后端按实测比例换算")
            input("   观察无异常后按回车回零 …")
            x_zero = goto(m5, 0.0)
            print(f"回零后 xout = {x_zero:+.4f}")
        finally:
            m5.disable()
            print("已失能。")


if __name__ == "__main__":
    main()
