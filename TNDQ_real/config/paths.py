"""
TNDQ_real 路径引导 —— 统一建立 sys.path（所有实机入口脚本第一步导入）。

自包含部署约定（Jetson 上仅部署 TNDQ_real 这一个文件夹）：
  - TNDQ_real 根加入 sys.path：顶层包 config / core / control / simdata /
    interfaces / experiments 均在本包内（算法层为 TNDQ_b601 同源副本，
    见 core/README.md 的同步清单）；
  - vendor/reBotArm_control_py（驱动 uv 工程根）加入 sys.path：真机总线
    驱动 reBotArm_control_py（motorbridge 原生绑定由该工程 uv sync
    安装保证，本模块不做 import）。

注意导入顺序（入口脚本必须遵守）：
    import config.paths            # [1] 路径引导（本模块）
    import config.params_real      # [2] 真机参数覆写（monkey-patch
                                   #     config.params；必须在 run_lib 前）
    from experiments import run_lib  # [3] 主循环（导入期读取已覆写参数）
"""
import sys
from pathlib import Path

REAL_ROOT = Path(__file__).resolve().parent.parent      # TNDQ_real/
VENDOR_ROOT = REAL_ROOT / "vendor" / "reBotArm_control_py"  # 驱动工程根

for _p in (str(REAL_ROOT), str(VENDOR_ROOT)):
    if _p not in sys.path:
        sys.path.insert(0, _p)
