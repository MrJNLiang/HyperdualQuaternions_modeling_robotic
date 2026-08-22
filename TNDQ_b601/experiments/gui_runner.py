"""
Isaac Sim GUI 挂载模式运行器 —— 在已启动的 GUI 里（Window -> Script Editor）
单行执行本文件即可跑 exp1 定点实验：

    exec(open("/home/liang/Documents/code/HyperdualQuaternions_modeling_robotic/TNDQ_b601/experiments/gui_runner.py").read())

与 standalone（python.sh）模式的三处关键差异：
  1. SimulationApp 桩替换：GUI 进程内 Kit 已启动，不能再拉起第二个实例
     （isaacsim 6.0 simulation_app.py 的 __init__ 会无条件 load_plugins +
     app.startup）；
  2. close() 猴子补丁：World.reset() / SimulationContext.reset() 官方注明
     "not intended to be used in the Isaac Sim's Extensions workflow"，在
     GUI 主线程同步调用会死锁，且 reset() 内部 stop()+play() 会把时间轴留
     在 playing 状态——控制循环结束后无人施加力矩（drive 已清零），机械臂
     垮塌自交使 PhysX 求解器空转，整窗表现为"卡死、视角拖不动"。故改为
     仅停止时间轴，场景保留供视角检查；
  3. sys.argv / sys.path 手动设置：runpy.run_path 不自动加脚本目录，且
     argparse 会误解析 GUI 自身启动参数。

注意：控制循环同步占用主线程，实验期间窗口不响应属正常现象；循环结束、
时间轴停止后 GUI 恢复交互，可自由移动视角查看最终场景。
"""

import runpy
import sys

_REPO = "/home/liang/Documents/code/HyperdualQuaternions_modeling_robotic"

# --- [1] SimulationApp 桩（必须先于任何 isaacsim 实验模块 import 生效）---
import isaacsim


class _GUIAttachApp:
    """代替 SimulationApp：不拉起新 Kit 实例，close 为空操作。"""

    def __init__(self, *a, **k):
        pass

    def close(self):
        pass

    def update(self):
        pass


isaacsim.SimulationApp = _GUIAttachApp

# --- [3] 路径：TNDQ_b601 包根 + experiments（run_lib 等）---
for _p in (_REPO + "/TNDQ_b601", _REPO + "/TNDQ_b601/experiments"):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# --- [2] close() 补丁：只停时间轴，不 world.reset() ---
import interfaces.isaac_interface as _ii


def _gui_close(self):
    try:
        import omni.timeline
        omni.timeline.get_timeline_interface().stop()
        print("[gui] 时间轴已停止，GUI 交还交互；场景保留供视角检查。")
    except Exception as exc:  # noqa: BLE001
        print(f"[gui] 停止时间轴失败: {exc}")


_ii.IsaacB601Backend.close = _gui_close

# --- 运行 exp1 ---
sys.argv = ["exp1_setpoint.py", "--headless", "0"]
runpy.run_path(_REPO + "/TNDQ_b601/experiments/exp1_setpoint.py",
               run_name="__main__")
print("[gui] exp1 完成。")
