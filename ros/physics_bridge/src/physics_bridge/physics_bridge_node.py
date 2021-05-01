#!/usr/bin/env python3

import rospy
from engines.webots import WeBotsBridge



if __name__ == '__main__':

    rospy.init_node('physics_bridge')

    wb = WeBotsBridge()

    rospy.spin()