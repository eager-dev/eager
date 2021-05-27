#!/usr/bin/env python

import rospy
from eager_bridge_webots.webots import WeBotsBridge

if __name__ == '__main__':

    rospy.init_node('physics_bridge')

    wb = WeBotsBridge()

    rospy.spin()
