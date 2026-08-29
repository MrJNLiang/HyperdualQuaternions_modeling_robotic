"""tndq_controller 启动文件。

前置：官方 bringup 已运行（rebotarmcontroller 占用串口），例如：
    ros2 launch rebotarm_bringup rebotarm.launch.py

启动本包：
    ros2 launch tndq_controller tndq_control.launch.py task:=short duration:=15.0

操作流程（安全顺序，全程手悬急停 Ctrl+C）：
    1. ros2 service call /tndq_controller/bringup   std_srvs/srv/Trigger
       （POS_VEL 慢速就位 Q_INIT，闭环前置）
    2. ros2 service call /tndq_controller/start     std_srvs/srv/Trigger
       （反馈健康闸门 -> 切 MIT -> 保持帧接力 -> 100 Hz 闭环）
    3. ros2 service call /tndq_controller/stop      std_srvs/srv/Trigger
       （可选，优雅终止实验，回保持期）
    4. ros2 service call /tndq_controller/shutdown  std_srvs/srv/Trigger
       （收尾：官方 pos_vel 位置环托臂，全程不失能）
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_DEFAULT_PARAMS = Path(__file__).resolve().parent.parent / 'config' / \
    'tndq_controller.yaml'


def generate_launch_description():
    task = LaunchConfiguration('task')
    duration = LaunchConfiguration('duration')

    return LaunchDescription([
        DeclareLaunchArgument('task', default_value='hold',
                              description='hold | short | goto'),
        DeclareLaunchArgument('duration', default_value='30.0',
                              description='实验时长 [s]'),
        Node(
            package='tndq_controller',
            executable='tndq_controller',
            name='tndq_controller',
            output='screen',
            parameters=[
                # 包内默认 yaml（share/tndq_controller/config/）
                str(_DEFAULT_PARAMS),
                {
                    'task': task,
                    'duration': duration,
                },
            ],
        ),
    ])
