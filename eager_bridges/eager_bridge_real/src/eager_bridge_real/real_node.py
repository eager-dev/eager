#!/usr/bin/env python

import rospy
from eager_bridge_real.real_bridge import RealBridge

if __name__ == '__main__':

    rospy.init_node('physics_bridge')

    rb = RealBridge()

    rospy.spin()
