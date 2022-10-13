#!/usr/bin/env python3

from __future__ import print_function

import rospy
import cv2
import mediapipe as mp
import numpy as np
import matplotlib.pyplot as plt
import cv_bridge
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

        # setting mediapipe pose model
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic

        # subscribers to the camera topic
        self.camera_sub = rospy.Subscriber("/spot/camera/frontleft/image", Image, self.camera_callback)
        

    def camera_callback(self, data):
        """
          Callback function of the rgb camera 
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            # rotate the image of 90°
            rot_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
        except CvBridgeError as e:
            print(e)

        cv2.imshow("mediapipe_image", mph)
        cv2.waitKey(3)

def main(args):
    rospy.init_node("holistic", anonymous = True)
    medpipe = mediapipe()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
