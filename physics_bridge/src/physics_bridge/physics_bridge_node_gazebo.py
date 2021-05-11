#!/usr/bin/env python

import rospy
from engines.gazebo import GazeboBridge

if __name__ == '__main__':

    rospy.init_node('physics_bridge', log_level=rospy.DEBUG)

    gb = GazeboBridge()

    rospy.spin()
