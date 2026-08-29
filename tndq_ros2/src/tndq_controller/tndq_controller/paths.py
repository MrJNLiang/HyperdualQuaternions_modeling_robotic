"""TNDQ_real 算法层引导 —— 与官方包 `_ensure_rebot_sdk_in_syspath` 同风格。

本 ROS2 包不复制 TNDQ_real 的 core/control/config/experiments 源码，而是
运行时把 TNDQ_real 根加入 sys.path 后按其文档化导入顺序加载（单一真相
源：transforms.py 标定常量与 params_real.py 安全参数随 TNDQ_real 演进
自动同步，绝不出现两份标定漂移）：

    import config.paths          # [1] TNDQ_real 根 + vendor 进 sys.path
    import config.params_real    # [2] 真机参数覆写（monkey-patch
                                 #     config.params；必须在 run_lib 前）
    from experiments import run_lib  # [3] 主循环（导入期读取已覆写参数）

TNDQ_real 定位（按序探测，首个命中者生效）：
    [1] 环境变量 TNDQ_REAL_ROOT
    [2] ~/Projects/HyperdualQuaternions_modeling_robotic/TNDQ_real
    [3] 本包源码树同级（tndq_ros2/src -> workspace 根 /TNDQ_real，
        供源码工作区直接 python 运行，不经 colcon 安装）
"""

import os
import sys
from pathlib import Path


def tndq_real_root() -> Path:
    candidates = []
    env = os.environ.get("TNDQ_REAL_ROOT", "")
    if env:
        candidates.append(Path(env).expanduser())
    candidates.append(
        Path.home() / "Projects" / "HyperdualQuaternions_modeling_robotic"
        / "TNDQ_real")
    # 本文件源码位置 .../tndq_ros2/src/tndq_controller/tndq_controller/paths.py
    # （colcon 安装后在 install/ 下，相对探测 [3] 自然落空，由 [2] 兜底）
    candidates.append(
        Path(__file__).resolve().parents[4] / "TNDQ_real")
    for root in candidates:
        if (root / "config" / "paths.py").is_file():
            return root.resolve()
    raise FileNotFoundError(
        "Cannot locate TNDQ_real (需含 config/paths.py)。请设置环境变量 "
        "TNDQ_REAL_ROOT 指向 TNDQ_real 目录；已探测：\n  "
        + "\n  ".join(str(c) for c in candidates))


def bootstrap() -> Path:
    """建立 sys.path 并按 TNDQ_real 铁律顺序完成参数覆写（幂等）。"""
    root = tndq_real_root()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    import config.paths          # noqa: F401,E402  vendor 路径引导
    import config.params_real    # noqa: F401,E402  真机口径覆写
    return root
