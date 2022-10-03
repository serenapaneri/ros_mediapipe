#!/usr/bin/env python3

from __future__ import print_function

import rospy
import cv2
import mediapipe as mp
import cv_bridge
import time
import os
import sys

sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/home/dls_user/cv_bridge_ws/install/lib/python3/dist-packages') # controlla sia giusto

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Mediapipe:

    def __init__(self):
        # setting the cv_bridge
        self.bridge = CvBridge()

        # setting mediapipe holistic model
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic

        # subscribers to camera topics 
        self.camera_sub = rospy.Subscriber("/spot/camera/frontleft/image", Image, self.camera_callback)
        self.depth_sub = rospy.Subscriber("/spot/depth/frontleft/image", Image, self.depth_callback)


    def holistic_2d(self, data):
        # initiate holistic model 
        with self.mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
            # in order to work with mediapipe we need the format RGB
            rgb_image = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
            rgb_image.flags.writeable = False

            # make detections
            results = holistic.process(rgb_image)

            # in order to work with opencv we need the BGR format
            rgb_image.flags.writeable = True
            rgb_image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)

            # face mask model 
            self.mp_drawing.draw_landmarks(rgb_image, results.face_landmarks, self.mp_holistic.FACE_CONNECTIONS,
                                     mp_drawing.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1),
                                     mp_drawing.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1)
                                     )

            # body pose model
            self.mp_drawing.draw_landmarks(rgb_image, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                                     mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=4),
                                     mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                     )


    def camera_callback(self, data):
        """
          Callback function of the rgb camera 
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            # rotate the image of 90°
            rot_image = cv2.rotate(cv_image, cv2.cv2.ROTATE_90_CLOCKWISE)
        except CvBridgeError as e:
            print(e)
        
        mph = self.holistic_2d(rot_image)
        cv2.imshow("mediapipe_image", mph)
        cv2.waitKey(3)


    def depth_callback(self, data):
        """
          Callback function of the depth camera
        """
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
        except CvBridgeError as e:
            print(e)

        
def main(args):
    mpipe = Mediapipe()
    rospy.init_node("holistic", anonymous = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

