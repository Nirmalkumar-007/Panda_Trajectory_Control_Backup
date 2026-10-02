import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/iit-rain/panda_ws/src/panda_trajectory/install/panda_trajectory'
