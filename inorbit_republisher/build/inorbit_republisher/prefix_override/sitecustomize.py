import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/ros_inorbit_samples.code-workspace/inorbit_republisher/install/inorbit_republisher'
