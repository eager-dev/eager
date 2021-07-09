#!/usr/bin/env python

import rospy
from eager_sensor_multisense_s21.rgbd_from_disparity import RgbdFromDisparity

if __name__ == '__main__':

    rospy.init_node("rgbd_from_disparity")

    rfd = RgbdFromDisparity()

    rospy.spin()
