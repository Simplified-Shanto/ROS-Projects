import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shanto/Github/wall_bot_ws/src/wall_follower/install/wall_follower'
