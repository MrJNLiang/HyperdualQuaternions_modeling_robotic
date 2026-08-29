"""
TNDQ_real/scripts/web_preview_short.py —— short 短轨迹 web 可视化预演（v5）。

在开源项目 reBotArm_simulator-DM（three.js 网页模型）上预演实机 short
轨迹的运动形态：本节点发布 /rebotarm/joint_states（sensor_msgs/
JointState，网页经 rosbridge ws://localhost:9090 订阅并镜像显示），
关节轨迹与 run_lib.build_short_trajectory 完全同参数——j1 从 Q_INIT
按 quintic smoothstep 转 +SHORT_SWING_RAD，SHORT_T_GO 去程，SHORT_T_BACK
原路缓回，GOTO_RETURN_DWELL 静稳，默认循环播放。

纯运动学预演（无力矩/无硬件）：验证运动幅度、速度观感与回零形态；
物理行为由 TNDQ_sim 仿真与实机安全链兜底。

运行（ROS2 环境，非 conda rebot；三终端）：
    # 终端 1：web 模拟器（Borot 开源项目原样）
    cd Borot-Arm_Mujoco/reBotArm_simulator-DM && npm start
    # 终端 2：rosbridge（网页 <-> ROS2 桥）
    source /opt/ros/jazzy/setup.bash && ros2 launch rosbridge_server \
        rosbridge_websocket_launch.xml port:=9090
    # 终端 3：本预演（TNDQ_real 目录）
    source /opt/ros/jazzy/setup.bash && python scripts/web_preview_short.py

浏览器打开 http://localhost:3001 -> 连接 ROS（ws://localhost:9090）
-> 勾选"镜像"即可看到臂按 short 轨迹往返。
"""
import sys
from pathlib import Path

REAL_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REAL_ROOT))

from config.params import (GOTO_RETURN_DWELL, Q_INIT,  # noqa: E402
                           SHORT_SWING_RAD, SHORT_T_BACK, SHORT_T_GO)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

PUB_RATE = 50.0            # 发布频率 [Hz]（网页订阅节流 80 ms，足够）
TOPIC = "/rebotarm/joint_states"   # rebot-ros-ui.js REQUIRED_TOPICS.jointStates
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]  # normalizeJointName 口径


def smoothstep(u):
    """quintic smoothstep s(u) = 10u^3 - 15u^4 + 6u^5（与轨迹构建器同款：
    位置弧长按此插值，j1 角度与 TCP 弧长成正比，时间参数一致）。"""
    u = min(1.0, max(0.0, u))
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))


def joint_angle_at(t):
    """short 轨迹 j1 角（其余关节恒 Q_INIT）：去程 0->1，回程 1->0。"""
    if t < SHORT_T_GO:
        s = smoothstep(t / SHORT_T_GO)
    elif t < SHORT_T_GO + SHORT_T_BACK:
        s = 1.0 - smoothstep((t - SHORT_T_GO) / SHORT_T_BACK)
    else:
        s = 0.0                       # 回零静稳（GOTO_RETURN_DWELL）
    return Q_INIT[0] + SHORT_SWING_RAD * s


def main():
    import argparse

    ap = argparse.ArgumentParser(description="short 轨迹 web 可视化预演")
    ap.add_argument("--once", action="store_true",
                    help="单次播放后退出（默认循环，Ctrl+C 停）")
    args = ap.parse_args()

    rclpy.init()
    node = Node("tndq_web_preview_short")
    pub = node.create_publisher(JointState, TOPIC, 10)
    period = 1.0 / PUB_RATE
    t_cycle = SHORT_T_GO + SHORT_T_BACK + GOTO_RETURN_DWELL
    print(f"[web-preview] 发布 {TOPIC} @ {PUB_RATE:.0f} Hz"
          f"（{'单次' if args.once else '循环'}）")
    print(f"[web-preview] 轨迹：j1 {Q_INIT[0]:+.3f} -> "
          f"{Q_INIT[0] + SHORT_SWING_RAD:+.3f} rad（{SHORT_T_GO:.0f}s）-> "
          f"回（{SHORT_T_BACK:.0f}s）-> 静稳 {GOTO_RETURN_DWELL:.0f}s，"
          f"循环 {t_cycle:.1f}s")
    print("[web-preview] 浏览器 localhost:3001 -> 连接 ROS -> 勾选镜像；"
          "Ctrl+C 退出")

    k = 0
    k_max = (int(round(t_cycle * PUB_RATE)) + 1) if args.once else None
    try:
        while rclpy.ok():
            if k_max is not None and k >= k_max:
                break                   # 单次模式：一个完整周期后退出
            t = (k * period) % t_cycle
            q = [joint_angle_at(t)] + [float(v) for v in Q_INIT[1:]]
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = q
            pub.publish(msg)
            k += 1
            if k % int(PUB_RATE) == 0:
                print(f"[web-preview] t={t:5.2f}s  j1={q[0]:+.4f} rad",
                      flush=True)
            rclpy.spin_once(node, timeout_sec=period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[web-preview] 结束")


if __name__ == "__main__":
    main()
