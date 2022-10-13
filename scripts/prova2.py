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
sys.path.append('/home/spot/cv_bridge_ws/install/lib/python3/dist-packages')

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

cv_image = None

def camera_callback(data):
    """
      Callback function of the rgb camera 
    """
    global cv_image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')

def main():
    rospy.init_node('prova2', anonymous = False)
    camera_sub = rospy.Subscriber('/spot/camera/frontleft/image', Image, camera_callback, queue_size = 1)
    time.sleep(1)
    image = cv_image
    rotate_image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    cv2.imshow("image", rotate_image)
    cv2.waitKey(3)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

