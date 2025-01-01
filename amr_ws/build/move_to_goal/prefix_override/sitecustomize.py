import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/viator/ws/walleva_ws/amr_ws/install/move_to_goal'
