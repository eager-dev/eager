#!/usr/bin/env python
from __future__ import print_function
import sys

import rospy
from ros_gym_bridge_pybullet.pybullet import PyBulletBridge

if __name__ == '__main__':

    rospy.init_node('physics_bridge')

    wb = PyBulletBridge()

    rospy.spin()