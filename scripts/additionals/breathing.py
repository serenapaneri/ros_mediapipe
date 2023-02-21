#!/usr/bin/env python3

from __future__ import print_function

import rospy
import cv2
import mediapipe as mp
import numpy as np
import matplotlib.pyplot as plt
import cv_bridge
import yaml
import time
import os
import math
import sys

# needed to use cv_bridge with python 3
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/home/spot/cv_bridge_ws/install/lib/python3/dist-packages') # controlla sia giusto

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class mediapipe:

    def __init__(self):
        # setting the cv_bridge
        self.bridge = CvBridge()

        self.camera_sub = rospy.Subscriber("/kinect2/hd/image_color", Image, self.camera_callback)
        self.depth_sub = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self.depth_callback)


    def camera_callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        cv2.imshow("mediapipe_image", cv_image)
        cv2.waitKey(3)

    def depth_callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, '16UC1')
        except CvBridgeError as e:
            print(e)

        cv2.imshow("depth_image", cv_image)
        cv2.waitKey(3)


def main(args):
    global landmarks
    rospy.init_node("mediapipe_stream", anonymous = True)
    medpipe = mediapipe()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

