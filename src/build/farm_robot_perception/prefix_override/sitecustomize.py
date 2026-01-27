import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rutha/Projects/class_projects/robotics_final/src/install/farm_robot_perception'
