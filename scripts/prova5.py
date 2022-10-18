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

rospy.init_node("holistic", anonymous = True)

bridge = CvBridge()

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

def show_image(img):
    cv2.imshow("mediapipe image", img)
    cv2.waitKey(3)

def camera_callback(img_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    except CvBridgeError, e 
        rospy.logerr("CvBridge Error: {0}".format(e))

    cv_image = cv2.transpose(cv_image)
    cv_image = cv2.flip(cv_image, 1)

    with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
        # in order to work with mediapipe we need the format RGB
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        rgb_image.flags.writeable = False

        # make detections
        results = holistic.process(rgb_image)

        # in order to work with opencv we need the BGR format
        rgb_image.flags.writeable = True
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

        mp_drawing.draw_landmarks(rgb_image, results.face_landmarks, mp_holistic.FACE_CONNECTIONS,
                                     mp_drawing.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1),
                                     mp_drawing.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1)
                                     )

         # body pose model
        mp_drawing.draw_landmarks(rgb_image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS,
                                    mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=4),
                                    mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                    )
        show_image(rgb_image)



camera_sub = rospy.Subscriber("/spot/camera/frontleft/image", Image, camera_callback)

cv2.namedWindow("mediapipe image", 1)

while not rospy.is_shutdown():
    rospy.spin()
