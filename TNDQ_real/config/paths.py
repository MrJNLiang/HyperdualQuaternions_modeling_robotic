"""
TNDQ_real 路径引导 —— 统一建立 sys.path（所有实机入口脚本第一步导入）。

约定：
  - TNDQ_b601 项目根加入 sys.path：复用 core / control / config /
    simdata / experiments.run_lib（理论层单一来源，见 core/README.md）；
  - reBotArm_control_py 驱动库加入 sys.path：真机总线驱动（uv 环境的
    motorbridge 依赖由运行环境保证，本模块不做 import）；
  - TNDQ_real 根加入 sys.path：本目录按 `config.` / `interfaces.` /
    `experiments.` 顶层包名引用。

注意导入顺序（入口脚本必须遵守）：
    import config.paths            # [1] 路径引导（本模块）
    import config.params_real      # [2] 真机参数覆写（monkey-patch
                                   #     config.params；必须在 run_lib 前）
    from experiments import run_lib  # [3] 主循环（导入期读取已覆写参数）
"""
import sys
from pathlib import Path

REAL_ROOT = Path(__file__).resolve().parent.parent      # TNDQ_real/
REPO_ROOT = REAL_ROOT.parent                            # 仓库根
B601_ROOT = REPO_ROOT / "TNDQ_b601"                     # 仿真/理论层根
DRIVER_ROOT = (REPO_ROOT / "reBot-Isaacsim" / "third_party"
               / "reBotArm_control_py")                 # 真机驱动库根

for _p in (str(B601_ROOT), str(DRIVER_ROOT), str(REAL_ROOT)):
    if _p not in sys.path:
        sys.path.insert(0, _p)
