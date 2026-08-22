"""
TNDQ_real 接线冒烟测试 —— 无硬件、无 Isaac 可运行（开发机/Jetson 通用）。

验证内容：
  [1] 命名空间包合并：config.params（TNDQ_b601）与 config.params_real
      （TNDQ_real）共存可导入；
  [2] params_real 覆写生效：DT=CTRL_DT=10 ms、CTRL_EVERY=1、
      DEFAULT_GAIN_SET="small_arm"（run_lib 导入前 patch 语义）；
  [3] run_lib 真机钩子接线：mock 后端的 hardware_safety_check /
      update_gravity_snapshot 每控制步被调用，HardwareFault 路径可
      触发 abort 且数据保存；
  [4] 主循环在真机时间基准（DT=10 ms）下正常推进并写 CSV。

运行：python3 TNDQ_real/tests/test_wiring.py
"""
import importlib.util
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

REAL_ROOT = Path(__file__).resolve().parent.parent
REPO_ROOT = REAL_ROOT.parent
B601_ROOT = REPO_ROOT / "TNDQ_b601"
for _p in (str(REAL_ROOT), str(B601_ROOT)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import config.params_real  # noqa: F401,E402  （先于 run_lib：真机口径 patch）


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def main():
    # [1][2] 参数覆写检查
    import config.params as cp
    assert abs(cp.DT - cp.CTRL_DT) < 1e-12, "DT 未对齐 CTRL_DT"
    assert abs(cp.DT - 0.01) < 1e-12, "CTRL_DT 应为 10 ms"
    assert cp.CTRL_EVERY == 1, "CTRL_EVERY 应为 1"
    assert cp.DEFAULT_GAIN_SET == "small_arm"
    assert cp.ENABLE_DITHER is False
    print("[wiring][1][2] 参数覆写 OK：DT=10ms CTRL_EVERY=1 增益组=small_arm")

    run_lib = _load("run_lib", B601_ROOT / "experiments" / "run_lib.py")

    # [3][4] mock 后端（同签名契约 + 真机钩子）
    class MockBackend:
        def __init__(self):
            self.q = cp.Q_INIT.copy()
            self.qd = np.zeros(6)
            self.tau = np.zeros(6)
            self.n_check = 0          # hardware_safety_check 调用计数
            self.n_gsnap = 0          # update_gravity_snapshot 调用计数
            self.n_step = 0

        def reset_to(self, q, gripper_width=None):
            pass

        def get_joint_state(self):
            return self.q.copy(), self.qd.copy()

        def get_measured_joint_efforts(self):
            return self.tau.copy()

        def apply_arm_torques(self, tau):
            self.tau[:] = tau

        def set_gripper(self, w):
            pass

        def get_gripper_width(self):
            return 0.14

        def get_cube_pose(self):
            return np.full(3, np.nan), np.full(4, np.nan)

        def step(self, render=None):
            self.n_step += 1

        def hardware_safety_check(self):
            self.n_check += 1

        def update_gravity_snapshot(self, g):
            self.n_gsnap += 1

        def close(self):
            pass

    traj, t_move, _ = run_lib.build_setpoint_goto_trajectory_kinematic()
    duration = 0.3                     # 30 个控制步
    with tempfile.TemporaryDirectory() as td:
        csv = os.path.join(td, "wiring.csv")

        # 正常路径
        be = MockBackend()
        s = run_lib.run_tndq_experiment(be, traj, duration, csv,
                                        label="wiring", verbose=False)
        n_ctrl = int(round(duration / cp.DT))
        assert s["aborted"] is None, f"意外 abort: {s['aborted']}"
        assert be.n_check == n_ctrl, \
            f"安全检查 {be.n_check} 次 != 控制步 {n_ctrl}"
        assert be.n_gsnap == n_ctrl, \
            f"重力快照 {be.n_gsnap} 次 != 控制步 {n_ctrl}"
        assert be.n_step == n_ctrl
        assert s["n_rows"] > 0
        print(f"[wiring][3][4] 主循环 OK：{n_ctrl} 控制步、"
              f"钩子逐拍调用、CSV {s['n_rows']} 行、无 abort")

        # HardwareFault 路径（模拟通信超时 -> abort 记录 + 数据保存）：
        # 直接构造与真机同名的异常类验证 run_lib 捕获语义
        class HardwareFault(RuntimeError):
            pass

        class FaultBackend2(MockBackend):
            def hardware_safety_check(self):
                self.n_check += 1
                if self.n_check >= 5:
                    raise HardwareFault("模拟通信超时")

        be2 = FaultBackend2()
        csv2 = os.path.join(td, "wiring_fault.csv")
        s2 = run_lib.run_tndq_experiment(be2, traj, duration, csv2,
                                         label="wiring-fault", verbose=False)
        assert s2["aborted"] is not None and "硬件故障" in s2["aborted"], \
            f"abort 记录异常: {s2['aborted']}"
        assert be2.n_check == 5 and s2["n_rows"] > 0
        print(f"[wiring][3] 急停路径 OK：第 5 拍 HardwareFault -> "
              f"abort='{s2['aborted']}'，CSV 保存 {s2['n_rows']} 行")

    print("=== TNDQ_real WIRING TEST PASS ===")


if __name__ == "__main__":
    main()
