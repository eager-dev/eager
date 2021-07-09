import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage


class RgbdFromDisparity(object):
    def __init__(self):
        self.bridge = CvBridge()

        left_image_topic = rospy.get_param("~left_image_topic","left/image_rect_color")
        right_image_topic = rospy.get_param("~right_image_topic", "right/image_rect_color")
        disparity_topic = rospy.get_param("~disparity_topic", "disparity")

        self.left_image = None
        self.right_image = None
        self.disparity = None
        self.rgbd = Image()

        rospy.Subscriber(left_image_topic, Image, self.left_image_callback)
        rospy.Subscriber(right_image_topic, Image, self.right_image_callback)
        rospy.Subscriber(disparity_topic, DisparityImage, self.disparity_callback)

        self.pub_left = rospy.Publisher("left_rgbd", Image, queue_size=3)
        self.pub_right = rospy.Publisher("right_rgbd", Image, queue_size=3)
        self.publish_rgbd()

    def disparity_callback(self, msg):
        try:
            self.disparity = self.bridge.imgmsg_to_cv2(msg.image, "passthrough")
        except CvBridgeError as e:
            rospy.logwarn(e)

    def left_image_callback(self, msg):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logwarn(e)

    def right_image_callback(self, msg):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logwarn(e)

    def publish_rgbd(self):
        while not rospy.is_shutdown():
            if self.left_image is not None and self.right_image is not None and self.disparity is not None:
                depth = cv2.normalize(self.disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                left_msg = self.bridge.cv2_to_imgmsg(np.concatenate((self.left_image, depth[:, :, np.newaxis]), 2))
                right_msg = self.bridge.cv2_to_imgmsg(np.concatenate((self.right_image, depth[:, :, np.newaxis]), 2))
                self.pub_left.publish(left_msg)
                self.pub_right.publish(right_msg)
            self.rate.sleep()
