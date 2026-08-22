# core/ —— TNDQ 核心算法复用说明

本目录**不存放核心算法的代码副本**。TNDQ 数学核心（dq_algebra /
tndq_algebra / kinematics）与实验库（config / control / simdata）通过
`sys.path` 直接引用同仓库的 `TNDQ_b601/`，保证仿真与真机共享**同一份**
理论实现——任何数学修复只改一处，两条链同时生效，杜绝副本分叉。

引用关系（由 `TNDQ_real/config/paths.py` 与各入口脚本建立）：

    TNDQ_real（实机部署层）
        │ sys.path += <repo>/TNDQ_b601
        ▼
    TNDQ_b601/
        core/        dq_algebra.py / tndq_algebra.py / kinematics.py
        control/     control_law.py / error_system.py / gain_design.py
        config/      params.py（被 real_config 覆写）/ b601_dynamics.py
        simdata/     traj_pinocchio.py / traj_kinematic.py / fk_pinocchio.py
        experiments/ run_lib.py（主控制循环，后端可插拔）

真机特有的内容全部收敛在 TNDQ_real 内：

    config/          params_real.py（参数覆写）/ transforms.py（标定常量）/ paths.py
    interfaces/      real_backend.py（RebotArm 真机后端，同签名替换 isaac_interface）
    experiments/     exp1_real.py（实机入口）

## Jetson 侧部署拷贝（可选优化）

若希望 Jetson 上不携带 Isaac Sim 诊断资产，可按以下清单裁剪拷贝
（保持相对目录结构，本目录 README 一并拷贝到 TNDQ_b601/core/ 位置即可）：

    TNDQ_b601/{core, control, config, simdata}/
    TNDQ_b601/experiments/run_lib.py
    reBot-Isaacsim/third_party/reBotArm_control_py/

其中 `simdata/fk_pinocchio.py` 与 `traj_pinocchio.py` 依赖 pinocchio
（aarch64 可用官方源/apt 安装）；缺失时 run_lib 自动回退 HDQ 链 FK 与
kinematic 轨迹（打印告警，数学口径不变）。
