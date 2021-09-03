#!/usr/bin/env python3

from __future__ import print_function

import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import numpy as np


class RenderNode(object):
    def __init__(self, name):
        self.name = '/'.join(name.split('/')[:2])
        self.bridge = CvBridge()
        self.topic_name = rospy.get_param('render_node/topic_name')
        self.fps = rospy.get_param('render_node/fps')
        self.image_sub = rospy.Subscriber('/' + self.topic_name, Image, self.callback)
        self.image_ros = None
        # cv2.startWindowThread()  # todo: needed?
        cv2.namedWindow(self.name + "/render")

    def render(self):
        if self.image_ros is None:
            return
        try:
            # Related issue: https://github.com/ros-perception/vision_opencv/issues/207
            cv_image = np.frombuffer(self.image_ros.data, dtype=np.uint8).reshape(self.image_ros.height, self.image_ros.width, -1)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        except CvBridgeError as e:
            print(e)
            return
        cv2.imshow(self.name + "/render", cv_image)
        cv2.waitKey(1)

    def run(self):
        rospy.init_node('physics_bridge')
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            self.render()
            rate.sleep()

    def callback(self, img):
        self.image_ros = img


if __name__ == '__main__':

    n = RenderNode(name=rospy.get_name())

    n.run()
