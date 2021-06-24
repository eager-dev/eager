#!/usr/bin/env python

import rospy
from safe_actions import SafeActions


if __name__ == '__main__':

    rospy.init_node('safe_actions')

    sa = SafeActions()

    rospy.spin()
