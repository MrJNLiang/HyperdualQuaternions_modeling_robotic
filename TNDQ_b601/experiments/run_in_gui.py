"""在 Isaac Sim GUI 的 Script Editor 中运行 B601 实验（结束不关窗口）。

用法（两步）：
    1) 终端启动 GUI：      ~/isaacsim/isaac-sim.sh
    2) GUI 中 Window -> Script Editor：
       点文件夹图标 Load 本文件（或整段粘贴），点 Run。

行为：
    - 实验脚本驱动逐步物理步进（world.step(render=True)），viewport
      中同步看到机械臂运动全过程（比实时慢，11 s 物理时长墙钟约
      2~4 分钟，取决于渲染负载）；
    - Kit 主进程已在运行，本脚本把 SimulationApp 桩替换掉（防止重复
      启动 Kit / 结束时关窗口）；实验结束后场景与最终姿态保留在
      GUI 中，窗口常驻，可自由转视角观察；
    - CSV 仍照常写入 results/（与命令行运行一致）。

切换实验：改下面 SCRIPT 指向 exp1_setpoint.py / exp2_*.py 即可。
"""
import runpy
import sys

SCRIPT = ("/home/liang/Documents/code/HyperdualQuaternions_modeling_robotic"
          "/TNDQ_b601/experiments/exp1_setpoint.py")
PROJECT = ("/home/liang/Documents/code/HyperdualQuaternions_modeling_robotic"
           "/TNDQ_b601")

# --- 桩替换 SimulationApp（Kit 已在运行）---
# isaac_interface.setup() 里是 `from isaacsim import SimulationApp` 动态
# 取属性，替换模块属性即生效；close() 变 no-op => 窗口不关闭。
import isaacsim


class _AppStub:
    def __init__(self, *args, **kwargs):
        print("[gui-runner] Kit 已在运行，SimulationApp 已桩替换", flush=True)

    def close(self):
        print("[gui-runner] 实验结束：GUI 窗口常驻，场景保留", flush=True)

    def update(self):
        pass


isaacsim.SimulationApp = _AppStub

# --- cv2 加载器修复（GUI 环境专用）---
# Isaac 自带 opencv 的 config.py 正常由 loader 以 exec 注入 LOADER_DIR /
# BINARIES_PATHS 后执行；GUI 环境下某条 import 链以普通 importlib 方式
# import cv2.config，注入缺失 -> NameError: LOADER_DIR。预执行真 config
# 并登记 sys.modules，后续 import 命中缓存即跳过出错路径。
try:
    import importlib.util
    if "cv2.config" not in sys.modules:
        _cv2_dir = "/home/liang/isaacsim/exts/omni.pip.compute/pip_prebundle/cv2"
        import os as _os
        _cfg_path = _os.path.join(_cv2_dir, "config.py")
        if _os.path.isfile(_cfg_path):
            _spec = importlib.util.spec_from_file_location(
                "cv2.config", _cfg_path)
            _mod = importlib.util.module_from_spec(_spec)
            _mod.__dict__.update({
                "LOADER_DIR": _cv2_dir,
                "BINARIES_PATHS": [],
                "PYTHON_EXTENSIONS_PATHS": [],
            })
            _spec.loader.exec_module(_mod)
            sys.modules["cv2.config"] = _mod
            print("[gui-runner] cv2.config 预加载完成", flush=True)
except Exception as _e:  # noqa: BLE001（修复失败不阻断，报原始错）
    print("[gui-runner] cv2.config 预加载跳过:", _e, flush=True)

# --- 实验脚本参数：--headless 0 => 每物理步渲染，viewport 同步可见 ---
sys.argv = [SCRIPT, "--headless", "0"]
if PROJECT not in sys.path:
    sys.path.insert(0, PROJECT)

print("[gui-runner] 开始运行:", SCRIPT, flush=True)
runpy.run_path(SCRIPT, run_name="__main__")
print("[gui-runner] 完成。窗口保持打开，可继续观察场景。", flush=True)
