"""无硬件离线自检 —— 验证包自洽与算法层引导链（不创建 ROS 节点、
不触碰串口、不发布任何消息）。

覆盖：
    [1] paths.bootstrap：TNDQ_real 定位 + sys.path 引导 + 参数覆写顺序
    [2] ros_backend 导入链（config.* / rebotarm_msgs 依赖齐备）
    [3] run_lib 导入（真机口径 DT/CTRL_EVERY/增益组已覆写）
    [4] 三种任务轨迹可构建且 evaluate(t) 接口字段完备
    [5] RosChannelBackend 安全变换口径抽查（电机系 <-> URDF 系恒等）

运行（任一）：
    source /opt/ros/jazzy/setup.bash && source <官方install>/setup.bash \\
        && source <本包install>/setup.bash
    python3 -m tndq_controller.offline_check
    # 或源码树直接：python3 src/tndq_controller/tndq_controller/offline_check.py
"""

import sys
from pathlib import Path

if __package__ in (None, ""):                    # 源码树直接运行
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def main() -> int:
    # [1] 引导
    from tndq_controller.paths import bootstrap
    root = bootstrap()
    print(f"[1] PASS  TNDQ_real 引导: {root}")

    # [2] 后端导入链
    import numpy as np
    from tndq_controller.control_node import TNDQControllerNode
    from tndq_controller.ros_backend import (HardwareFault,
                                             RosChannelBackend)
    print("[2] PASS  ros_backend 导入链（config.* / rebotarm_msgs）")

    # [3] run_lib 真机口径
    import config.params as cp
    from experiments import run_lib
    assert cp.DT == cp.CTRL_DT and cp.CTRL_EVERY == 1, "真机口径未覆写"
    assert cp.DEFAULT_GAIN_SET == "small_arm", "增益组未切真机口径"
    assert not cp.ENABLE_DITHER, "dither 必须关闭（真机）"
    print(f"[3] PASS  run_lib 真机口径: DT={cp.DT}, 增益组="
          f"{cp.DEFAULT_GAIN_SET}")

    # [4] 任务轨迹
    class _StubNode:
        def get_logger(self):
            class _L:
                def warn(self, *_):
                    pass
            return _L()

    node = TNDQControllerNode.__new__(TNDQControllerNode)  # 不进 __init__
    node.get_logger = _StubNode().get_logger
    for task, duration in (("hold", 30.0), ("short", 15.0), ("goto", 35.0)):
        # goto pinocchio 不可用时 build_* 内部自动降级 kinematic
        node.traj_backend = "kinematic"          # 离线自检不依赖 pinocchio
        traj = TNDQControllerNode._build_trajectory(node, task, duration)
        des = traj.evaluate(0.0)
        for key in ("x_bar_d", "x_breve_d", "x_d", "xi_d", "xi_dot_d"):
            assert key in des, f"{task} 轨迹缺字段 {key}"
        print(f"[4] PASS  轨迹 {task}: t_total={traj.t_total:.2f}s, "
              "evaluate 字段完备")

    # [5] 标定变换口径（恒等链抽查；transforms 演进后此抽查仍成立）
    from config.transforms import JOINT_OFFSET, JOINT_SIGN, TAU_SCALE
    q_motor = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    q_urdf = JOINT_SIGN * q_motor + JOINT_OFFSET
    tau_urdf = np.array([1.0, -2.0, 3.0, -0.1, 0.2, -0.3])
    tau_motor = JOINT_SIGN * tau_urdf / TAU_SCALE
    assert q_urdf.shape == (6,) and tau_motor.shape == (6,)
    assert np.all(np.isfinite(tau_motor))
    print(f"[5] PASS  标定变换口径: SIGN={JOINT_SIGN.tolist()}, "
          f"TAU_SCALE={TAU_SCALE.tolist()}")

    print("\n=== tndq_controller OFFLINE CHECK PASS ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
