import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/viator/ws/walleva_ws/b3_ws/install/dual_bot'
