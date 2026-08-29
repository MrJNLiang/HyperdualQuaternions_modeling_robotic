#!/usr/bin/env python3
"""数字孪生桥：实机 UDP 遥测 -> /rebotarm/joint_states（web 镜像）。

链路（v6）：
    real_backend._bus_cycle（500 Hz 总线线程，每 10 拍 50 Hz）
      --struct '<6d'/tobytes（48 B）--> UDP 127.0.0.1:47470
      --> 本节点 --sensor_msgs/JointState--> /rebotarm/joint_states
      --> rosbridge(9090) --> reBotArm_simulator-DM 网页（勾选"镜像"）。

单向只读：本节点不向实机写任何东西；实机未跑时 UDP 无数据，
    网页停在最后姿态（真实反映，无假数据）。

运行环境：系统 python3 + ROS2 jazzy（非 conda rebot，同 web_preview）：
    env PATH=/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin bash -c \
      'source /opt/ros/jazzy/setup.bash \
       && cd TNDQ_real && exec python3 scripts/ros_twin_bridge.py'

端口/频率与 params_real.TWIN_UDP_PORT / TWIN_RATE_HZ 对应（改动需同步）。
"""
import math
import socket
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

UDP_PORT = 47470                     # = params_real.TWIN_UDP_PORT
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]


def main():
    rclpy.init()
    node = Node("tndq_real_twin_bridge")
    pub = node.create_publisher(JointState, "/rebotarm/joint_states", 10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", UDP_PORT))
    sock.settimeout(0.5)
    print(f"[twin-bridge] UDP 127.0.0.1:{UDP_PORT} -> /rebotarm/joint_states")
    print("[twin-bridge] 网页 localhost:3001 -> 连接 ROS -> 勾选镜像；"
          "实机跑起来后即实时镜像；Ctrl+C 退出")

    n_msg = 0
    try:
        while rclpy.ok():
            try:
                data, _ = sock.recvfrom(128)
            except socket.timeout:
                continue             # 实机未跑：无数据继续等
            if len(data) != 48:
                continue             # 包长校验失败丢弃
            q = struct.unpack("<6d", data)
            if any(not math.isfinite(v) for v in q):
                continue
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = [float(v) for v in q]
            pub.publish(msg)
            n_msg += 1
            if n_msg % (50 * 5) == 1:            # ~5 s 一行心跳
                node.get_logger().info(
                    "q=[" + " ".join(f"{v:+.3f}" for v in q) + "]")
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()
        print("[twin-bridge] 已退出")


if __name__ == "__main__":
    main()
