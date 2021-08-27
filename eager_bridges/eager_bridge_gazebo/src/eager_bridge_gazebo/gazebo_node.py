#!/usr/bin/env python

import rospy
from eager_bridge_gazebo.gazebo_bridge import GazeboBridge

if __name__ == '__main__':

    rospy.init_node("physics_bridge")

    gb = GazeboBridge()

    rospy.spin()
