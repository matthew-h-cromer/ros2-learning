import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/matthew-cromer/Documents/ros2-pubsub-from-scratch/install/py_pubsub'
