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

class mediapipe:

    def __init__(self):
        # setting the cv_bridge
        self.bridge = CvBridge()

        # setting mediapipe pose model
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic

        # subscribers to the camera topic
        self.camera_sub = rospy.Subscriber("/spot/camera/back/image", Image, self.camera_callback)
        

    def camera_callback(self, data):
        """
          Callback function of the rgb camera 
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            # rotate the image of 90°
            rot_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE) # cambia questo
        except CvBridgeError as e:
            print(e)

        with self.mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
            # in order to work with mediapipe we need the format RGB
            rgb_image = cv2.cvtColor(rot_image, cv2.COLOR_BGR2RGB)
            rgb_image.flags.writeable = False

            # make detections
            results = holistic.process(rgb_image)

            # in order to work with opencv we need the BGR format
            rgb_image.flags.writeable = True
            rgb_image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)

            self.mp_drawing.draw_landmarks(rgb_image, results.face_landmarks, self.mp_holistic.FACE_CONNECTIONS,
                                     self.mp_drawing.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1),
                                     self.mp_drawing.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1)
                                     )

            # body pose model
            self.mp_drawing.draw_landmarks(rgb_image, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                                     self.mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=4),
                                     self.mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                     )

        

            cv2.imshow("mediapipe_image", rgb_image)
            cv2.waitKey(3)

def main(args):
    rospy.init_node("mediapipe_stream", anonymous = True)
    medpipe = mediapipe()
    print(medpipe)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
