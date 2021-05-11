#!/usr/bin/env python
from __future__ import print_function
import sys

import rospy
from engines.pybullet import PyBulletBridge

if __name__ == '__main__':

    rospy.init_node('physics_bridge')

    wb = PyBulletBridge()

    rospy.spin()