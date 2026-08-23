"""
TNDQ_real/scripts/02_gravity_hold.py —— URDF 系重力保持验证。

用 TNDQ_b601 自己的 B601NominalDynamics.gravity_vector（URDF 系）经
transforms 标定常量下发。判据：
  [符号终审] 前 2 s 斜坡混入；若某关节立即单向漂移 > 0.15 rad ->
             符号/零位错误，立即 Ctrl+C（失能）回 01 重标；
  [TAU_SCALE] 保持段逐关节漂移速率 < 0.002 rad/s（30 s < 0.06 rad）
             为优；< 0.05 rad/30s 可接受（闭环 small_arm 增益对恒值
             残差有 p_T 抑制，稳态误差 = 残差/p_T，params 已有预算）。

整定方法（迭代 2~3 轮）：某关节持续下垂（补偿不足）-> 该关节
TAU_SCALE 除以 factor（>1）；过补偿上漂 -> 乘以 factor。

运行（工作目录 = TNDQ_real/）：
    uv run python scripts/02_gravity_hold.py --duration 30
    uv run python scripts/02_gravity_hold.py --factor 1,1.2,2.5,2.5,1,1
    uv run python scripts/02_gravity_hold.py --pose armup
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from config.params import Q_INIT  # noqa: E402
from config.transforms import JOINT_OFFSET, JOINT_SIGN, TAU_SCALE  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--factor", type=str, default=None,
                    help="逐关节补偿系数（逗号分隔 6 值），乘在 g(q) 上")
    ap.add_argument("--pose", type=str, default="qinit",
                    choices=["qinit", "armup"],
                    help="保持姿态：qinit=Q_INIT；armup=前伸（j2=-0.8）"
                         "加大重力臂暴露标度误差")
    args = ap.parse_args()

    factor = (np.ones(6) if args.factor is None
              else np.array([float(v) for v in args.factor.split(",")]))
    q_target = (Q_INIT.copy() if args.pose == "qinit"
                else np.array([0.0, -0.8, -0.4, 0.0, 0.0, 0.0]))

    from config.b601_dynamics import B601NominalDynamics
    from reBotArm_control_py.actuator import RebotArm

    dyn = B601NominalDynamics()
    robot = RebotArm()
    robot.connect()
    robot.arm.mode_mit()
    robot.enable_all()

    dt = 0.01
    q_hist = []
    t0 = time.perf_counter()
    print(f"[02] URDF 系重力保持启动：pose={args.pose} "
          f"duration={args.duration}s factor={factor.tolist()}")
    print("[02] 前 2 s 斜坡混入——观察有无关节立即单向漂移（符号错征兆）")
    try:
        while True:
            tic = time.perf_counter()
            t = tic - t0
            if t > args.duration:
                break
            pos, vel, _ = robot.get_state(request_feedback=True)
            q = JOINT_SIGN * pos[:6] + JOINT_OFFSET      # 电机系 -> URDF 系
            g = dyn.gravity_vector(q) * factor            # URDF 系重力
            tau_motor = JOINT_SIGN * g / TAU_SCALE        # -> 电机系
            alpha = min(1.0, t / 2.0)                     # 2 s 使能斜坡
            tau_motor = alpha * tau_motor                 # （斜坡期欠补偿，
            robot.arm.send_mit(pos=pos[:6], vel=vel[:6],  #   臂缓落被托）
                               kp=np.zeros(6), kd=np.full(6, 0.5),
                               tau=tau_motor)
            q_hist.append(q.copy())
            # 早期发散保护：保持段单关节偏离起点 0.15 rad -> 报警
            if t > 2.0 and len(q_hist) > 20:
                dq = float(np.max(np.abs(q - q_hist[0])))
                if dq > 0.15:
                    raise RuntimeError(
                        f"t={t:.1f}s 关节偏离 {dq:.3f} rad：符号/零位错误"
                        f"或 TAU_SCALE 严重失配，立即回 01 重标")
            elapsed = time.perf_counter() - tic
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        print("\n[02] 用户中断")
    finally:
        robot.arm.disable()
        robot.disconnect()

    q_hist = np.asarray(q_hist)
    if len(q_hist) > 10:
        hold = q_hist[len(q_hist) // 2:]          # 后半段稳态
        drift = hold[-1] - hold[0]
        dur = len(hold) * dt
        rate = drift / max(dur, 1e-9)
        print(f"\n[02] 保持段 {dur:.1f} s 漂移结果：")
        for i in range(6):
            ok = "OK " if abs(rate[i]) < 0.002 else "调 "
            print(f"  j{i + 1}: 漂移 {drift[i]:+.4f} rad"
                  f"（{rate[i]:+.4f} rad/s）[{ok}]")
        print("[02] 判据：|速率| < 0.002 rad/s 优；< 0.05 rad/30s 可接受")
        print("     下垂关节 -> TAU_SCALE[i] /= 1.2 重试；上漂 -> *= 1.2")
    else:
        print(f"[02] 保持时间过短，无漂移统计；目标姿态参考 q={q_target.tolist()}")


if __name__ == "__main__":
    main()
