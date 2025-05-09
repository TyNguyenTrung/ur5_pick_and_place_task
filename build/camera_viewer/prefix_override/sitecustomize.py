import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/trungty/Downloads/moveit2_UR5/install/camera_viewer'
