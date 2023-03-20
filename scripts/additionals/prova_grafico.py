#!/usr/bin/env python3

import rospy
import cv2
import mediapipe as mp
import numpy as np
import cv_bridge
import time
import os
import sys

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class mediapipe:

    def __init__(self):
        # setting the cv_bridge
        self.bridge = CvBridge()

        # camera subscriber
        self.camera_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)


    def camera_callback(self, data):

        try:            
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)


        cv2.imshow("image", cv_image)
        cv2.waitKey(3)





def main(args):
    rospy.init_node("mediapipe_stream", anonymous = True)
    med = mediapipe()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
