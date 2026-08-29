"""
TNDQ_real/scripts/diag_reg_hold.py —— 0x50/0x51 语义甄别 + 6 号复核 + RTT 测量（只读）。

阶段 0：测量 get_register_f32(0x50) 单次读回 RTT（决定后端可用频率）；
阶段 1（需配合）：静置 3 s → 手转关节 5 约 0.5 rad 并保持 5 s → 放手回静置 3 s；
    位置寄存器：保持窗均值明显偏移、回静复原；速度寄存器：保持窗仍约 0；
阶段 2（需配合）：手摇关节 6 约 5 s，复核 6 号同 rid 是否活。

安全前提：全程不使能、不下发控制指令，只读。
运行：python scripts/diag_reg_hold.py（rebot 环境；串口需空闲；人在臂旁）
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from motorbridge import Controller  # noqa: E402

R_POS_CAND = (0x50, 0x51)


def open_with_retry(port="/dev/ttyACM0", baud=921600, tries=5):
    for k in range(tries):
        try:
            return Controller.from_dm_serial(port, baud)
        except Exception as exc:  # noqa: BLE001
            print(f"open 失败({k + 1}/{tries}): {exc}；1 s 后重试")
            time.sleep(1.0)
    raise RuntimeError("串口持续 busy")


def sample(m, secs):
    """在 secs 秒内循环读 0x50/0x51，返回各 rid 的 [min, max, 末值]。"""
    acc = {r: [] for r in R_POS_CAND}
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < secs:
        for r in R_POS_CAND:
            try:
                acc[r].append(m.get_register_f32(r, 40))
            except Exception:  # noqa: BLE001
                pass
        time.sleep(0.01)
    out = {}
    for r, vs in acc.items():
        if vs:
            out[r] = (min(vs), max(vs), vs[-1])
    return out


def fmt(win):
    return "  ".join(
        f"rid{r:02X}=[{lo:+.4f},{hi:+.4f}] 末{last:+.4f}"
        for r, (lo, hi, last) in sorted(win.items()))


def main():
    with open_with_retry() as c:
        m5 = c.add_damiao_motor(0x05, 0x15, "4310")
        m6 = c.add_damiao_motor(0x06, 0x16, "4310")

        print("== 阶段 0：RTT 测量（50 次读 0x50）")
        t0 = time.perf_counter()
        for _ in range(50):
            m5.get_register_f32(0x50, 40)
        dt = (time.perf_counter() - t0) / 50 * 1000
        print(f"   单次 RTT ≈ {dt:.2f} ms")

        print("== 阶段 1：保持测试（关节 5）")
        print("   [1/3] 静置 3 s …")
        w_rest = sample(m5, 3.0)
        print("   静置:", fmt(w_rest))
        print("   [2/3] 现在用手把关节 5 转约 0.5 rad 并保持住，5 s …")
        w_hold = sample(m5, 5.0)
        print("   保持:", fmt(w_hold))
        print("   [3/3] 放手，回静置 3 s …")
        w_back = sample(m5, 3.0)
        print("   回静:", fmt(w_back))

        print("== 阶段 2：关节 6 复核")
        print("   请手摇关节 6 约 5 s …")
        lo6 = {r: 1e9 for r in R_POS_CAND}
        hi6 = {r: -1e9 for r in R_POS_CAND}
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 5.0:
            for r in R_POS_CAND:
                try:
                    v = m6.get_register_f32(r, 40)
                    lo6[r] = min(lo6[r], v)
                    hi6[r] = max(hi6[r], v)
                except Exception:  # noqa: BLE001
                    pass
            time.sleep(0.01)
        for r in R_POS_CAND:
            print(f"   motor6 rid{r:02X} 范围 [{lo6[r]:+.4f}, {hi6[r]:+.4f}]"
                  f"  Δ={hi6[r] - lo6[r]:.4f}")
        print("== 完成。")


if __name__ == "__main__":
    main()
