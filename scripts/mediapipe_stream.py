#!/usr/bin/env python3

import rospy
import cv2
import mediapipe as mp
import numpy as np
import cv_bridge
import time
import os
import sys
import sensor_msgs.point_cloud2 as pc2

# needed to use cv_bridge with python 3
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/home/spot/cv_bridge_ws/install/lib/python3/dist-packages')

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class mediapipe:

    def __init__(self):
        # setting the cv_bridge
        self.bridge = CvBridge()

        # setting mediapipe holistic model
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic

        # mediapipe variable for detection
        self.results = None

        self.rgb_image = None

        self.landmarks = []
        # self.visibilities = []

        # real coordinates from point_cloud2
        self.x_coord = []
        self.y_coord = []
        self.z_coord = []

        # camera subscriber
        self.camera_sub = rospy.Subscriber("/kinect2/hd/image_color", Image, self.camera_callback)
        # point cloud subscriber
        self.camera_sub = rospy.Subscriber("/kinect2/hd/points", pc2.PointCloud2, self.cloud_callback)


    def camera_callback(self, data):

        try:            
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        with self.mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
            # in order to work with mediapipe we need the format RGB
            self.rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.rgb_image.flags.writeable = False

            ### make detections ###
            self.results = holistic.process(self.rgb_image)

            # in order to work with opencv we need the BGR format
            self.rgb_image.flags.writeable = True
            self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)

            ### face mask model ### 
            self.mp_drawing.draw_landmarks(self.rgb_image, self.results.face_landmarks, self.mp_holistic.FACE_CONNECTIONS,
                                     self.mp_drawing.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1),
                                     self.mp_drawing.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1)
                                     )

            ### body pose model ###
            self.mp_drawing.draw_landmarks(self.rgb_image, self.results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                                     self.mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=4),
                                     self.mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                     )


            cv2.imshow("mediapipe_image", self.rgb_image)
            cv2.waitKey(3)


    def cloud_callback(self, point_data):
        
        assert isinstance(point_data, PointCloud2)
        ### converting the marker in pixels ###
        if self.results is not None and self.results.pose_landmarks:
            pts = self.results.pose_landmarks.landmark

            # taking the nose coordinates
            nose = pts[-33]
            h, w, c = self.rgb_image.shape
            imgx, imgy= int(nose.x*w), int(nose.y*h)

            ### converting pixels into x,y,z point cloud ###
            points3D = list(pc2.read_points(point_data, field_names=('x', 'y', 'z'), skip_nans=True, uvs = [(imgx, imgy)]))
            for p in points3D:
                # these coordinates are w.r.t the kinect_rgb_optical_frame
                self.x_coord.append(p[0])
                self.y_coord.append(p[1])
                self.z_coord.append(p[2])


def main(args):
    rospy.init_node("mediapipe_stream", anonymous = True)
    med = mediapipe()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
