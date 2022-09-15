#!/usr/bin/env python3

import rospy
import mediapipe as mp
import cv2, cv_bridge
from sensor_msgs.msg import Image
from __future__ import print_function
from cv_bridge import CvBridge, CvBridgeError


class Mediapipe:

    def __init__(self):
        # setting mediapipe holistic model
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic

        self.bridge = CvBridge()
        cv2.namedWindow("mediapipe", 1)
        self.image_sub = rospy.Subscriber('camera/frontleft/camera/image', Image, self.image_callback)


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding= 'bgr8')
        cv2.imshow("mediapipe", image)
        cv2.waitKey(3)


def main(args):
    rospy.init_node('ros_stream')
    ros_stream = Mediapipe()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
