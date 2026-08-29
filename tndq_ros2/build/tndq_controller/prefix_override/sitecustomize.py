import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ljn/Projects/HyperdualQuaternions_modeling_robotic/tndq_ros2/install/tndq_controller'
