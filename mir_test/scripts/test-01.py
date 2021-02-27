#!/usr/bin/env python
from __future__ import print_function
import roslibpy

ros = roslibpy.Ros(host='192.168.2.22', port=9090)
ros.on_ready(lambda: print('Is ROS connected?', ros.is_connected))
ros.get_nodes(errback=None)
