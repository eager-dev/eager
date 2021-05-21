#!/usr/bin/env python

import rospy
from ros_gym_bridge_gazebo.gazebo import GazeboBridge

if __name__ == '__main__':

    rospy.init_node('physics_bridge')

    gb = GazeboBridge()

    rospy.spin()