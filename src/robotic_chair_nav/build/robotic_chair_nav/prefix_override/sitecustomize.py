import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ilyas/ros2_ws/src/robotic_chair_nav/install/robotic_chair_nav'
