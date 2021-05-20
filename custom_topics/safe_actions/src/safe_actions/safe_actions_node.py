#!/usr/bin/env python3

import rospy
from safe_actions import SafeActions
import sys


if __name__ == '__main__':

    rospy.init_node('safe_actions')

    sa = SafeActions()

    rospy.spin()
