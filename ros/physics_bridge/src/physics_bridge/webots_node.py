#!/usr/bin/env python

import rospy
from engines.webots.webots import WeBotsBridge

if __name__ == '__main__':

    rospy.init_node('physics_bridge')

    wb = WeBotsBridge()

    rospy.spin()