#!/usr/bin/env python

import rospy
from engines.real import RealBridge

if __name__ == '__main__':

    rospy.init_node('physics_bridge')

    rb = RealBridge()

    rospy.spin()
