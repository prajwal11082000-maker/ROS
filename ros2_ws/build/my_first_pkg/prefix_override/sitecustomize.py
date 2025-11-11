import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/enmac/ROS/ROS/ros2_ws/install/my_first_pkg'
