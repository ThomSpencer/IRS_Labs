import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/ros_ws/IRS_Labs/install/hand_solo_virtual_nav'
