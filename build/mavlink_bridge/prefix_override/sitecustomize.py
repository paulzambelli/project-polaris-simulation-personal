import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/polaris_pz/project-polaris-simulation-personal/install/mavlink_bridge'
