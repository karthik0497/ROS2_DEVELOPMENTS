import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/karthik/Documents/ROS2_DEVELOPMENTS/ros2_workspace_1/install/my_py_pkg'
