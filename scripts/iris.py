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

sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/home/spot/cv_bridge_ws/install/lib/python3/dist-packages') # controlla sia giusto

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from matplotlib.animation import FuncAnimation

class iris:

    def __init__(self):
        # setting the cv_bridge
        self.bridge = CvBridge()

        # setting mediapipe pose model
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_face_mesh = mp.solutions.face_mesh

        # subscribers to the camera topic
        self.camera_sub = rospy.Subscriber("/spot/camera/frontleft/image", Image, self.camera_callback)

    def distance(self, point1, point2):
        point1 = self.np.array(point1)
        point2 = self.np.array(point2)
        self.dist = np.linalg.norm(point1, point2)
        return dist


    def iris_2d(self, data):
        # initiate face_mesh model 
        with self.mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5) as face_mesh:
            # in order to work with mediapipe we need the format RGB
            rgb_image = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
            rgb_image.flags.writeable = False

            # make detections
            results = face_mesh.process(rgb_image)

            # in order to work with opencv we need the BGR format
            rgb_image.flags.writeable = True
            rgb_image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)

            self.mp_drawing.draw_landmarks(rgb_image, landmark_list=face_landmarks, connections=mp_face_mesh.FACEMESH_IRISES,
          landmark_drawing_spec=None,
          connection_drawing_spec=mp_drawing_styles
          .get_default_face_mesh_iris_connections_style())

            # iris model
            self.mp_drawing.draw_landmarks(rgb_image, results.face_mesh_landmarks, self.mp_face_mesh.FACEMESH_IRISES,
                                     mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=4),
                                     mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                     )

       # se la distanza tra i landmark dell'occhio è sotto una certa threshold allora l'occhio
       # si considera chiuso, altrimenti aperto


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
        
        mph = self.iris_2d(rot_image)
        cv2.imshow("iris_image", mph)
        cv2.waitKey(3)

def main(args):
    ir = iris()
    rospy.init_node("iris", anonymous = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
