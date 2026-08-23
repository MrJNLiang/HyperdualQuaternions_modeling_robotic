# core/ —— TNDQ 核心算法（自包含副本）说明

自本版起，`TNDQ_real` 为**独立部署包**：本目录及同级 `config/`、`control/`、
`simdata/`、`experiments/run_lib.py` 均为 `TNDQ_b601` 算法层的**同源副本**，
Jetson 上仅需部署 `TNDQ_real/` 这一个文件夹即可运行，不再通过 `sys.path`
回链仓库内的 `TNDQ_b601/`。

## 副本清单（与 TNDQ_b601 的对应关系）

    TNDQ_real/core/          <-  TNDQ_b601/core/
        __init__.py / dq_algebra.py / tndq_algebra.py / kinematics.py
    TNDQ_real/control/       <-  TNDQ_b601/control/
        control_law.py / error_system.py / gain_design.py / performance.py
    TNDQ_real/config/        <-  TNDQ_b601/config/（合并，无重名文件）
        params.py / b601_dynamics.py
        + 真机专属：params_real.py（覆写）/ transforms.py（标定）/ paths.py
    TNDQ_real/simdata/       <-  TNDQ_b601/simdata/（裁剪，去掉仿真激励）
        trajectory_generator.py / fk_pinocchio.py
        traj_pinocchio.py / traj_kinematic.py
    TNDQ_real/experiments/run_lib.py  <-  TNDQ_b601/experiments/run_lib.py
    TNDQ_real/vendor/reBotArm_control_py/
        <-  reBot-Isaacsim/third_party/reBotArm_control_py/
        （裁剪：仅保留 Python 源码、config/*.yaml、pyproject 与
         urdf/reBot_B601_DM/urdf/ 文本模型；meshes 仅可视化用，未携带）

## 开发机上的同步纪律（重要）

算法层在**开发仓库的唯一维护点是 `TNDQ_b601/`**（仿真实验持续使用它）。
若仿真侧修复/改进了上述任一文件，部署前必须把改动同步到本包对应副本：

    cp TNDQ_b601/core/{dq_algebra,tndq_algebra,kinematics}.py   TNDQ_real/core/
    cp TNDQ_b601/control/*.py                                     TNDQ_real/control/
    cp TNDQ_b601/config/{params,b601_dynamics}.py                 TNDQ_real/config/
    cp TNDQ_b601/simdata/{trajectory_generator,fk_pinocchio,traj_pinocchio,traj_kinematic}.py TNDQ_real/simdata/
    cp TNDQ_b601/experiments/run_lib.py                           TNDQ_real/experiments/

同步后重跑 `python3 TNDQ_real/tests/test_wiring.py` 确认接线仍 PASS。
注意：`config/transforms.py`、`config/params_real.py` 为真机专属，**永不**
被上述同步覆盖。

## 真机特有内容（本包独有，不存在于 TNDQ_b601）

    config/params_real.py    真机参数覆写（DT=10 ms、small_arm 增益组等）
    config/transforms.py     标定常量（符号/零位/力矩标度/夹爪换算）
    config/paths.py          sys.path 引导（REAL_ROOT + vendor）
    interfaces/real_backend.py  RebotArm 真机后端（同签名替换 isaac_interface）
    experiments/exp1_real.py 实机入口（openhold / grasp 两模式）
    scripts/                 标定与检查工具链（00~03，上机可选参考）
    vendor/                  总线驱动 uv 工程（motorbridge 经 uv sync 安装）
