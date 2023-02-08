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
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray

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
        self.landmarks = []

        # markers
        self.spheres = Marker()
        self.linelist = Marker()
        self.marker_array = MarkerArray()

        # camera subscriber
        self.camera_sub = rospy.Subscriber("/kinect2/hd/image_color", Image, self.camera_callback)
        # point cloud subscriber
        self.camera_sub = rospy.Subscriber("/kinect2/hd/points", pc2.PointCloud2, self.cloud_callback)
        # marker publisher
        self.marker_pub = rospy.Publisher('markers', MarkerArray, queue_size = 10)


    def camera_callback(self, data):

        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        with self.mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
            # in order to work with mediapipe we need the format RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            rgb_image.flags.writeable = False

            # make detections
            self.results = holistic.process(rgb_image)

            # in order to work with opencv we need the BGR format
            rgb_image.flags.writeable = True
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            # face mask model 
            self.mp_drawing.draw_landmarks(rgb_image, self.results.face_landmarks, self.mp_holistic.FACE_CONNECTIONS,
                                     self.mp_drawing.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1),
                                     self.mp_drawing.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1)
                                     )

            # body pose model
            self.mp_drawing.draw_landmarks(rgb_image, self.results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                                     self.mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=4),
                                     self.mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                     )

            if self.result is not None and self.result.pose_landmarks:
                pts = self.result.pose_landmarks.landmark
                for id_,lm in enumerate(pts):
                    h, w, c = rgb_image.shape
                    imgx, imgy = int((lm.x*w), int(lm.y*h))
                    self.landmarks.append((imgx,imgy))

            cv2.imshow("mediapipe_image", rgb_image)
            cv2.waitKey(3)


    def cloud_callback(self, point_data):
        self.create_linelist(point_data)
        self.create_spheres(point_data)
        self.marker_array.markers.extend([self.spheres, self.linelist])
        self.marker_pub.publish(self.marker_array)



    def create_linelist(self, point_data):

        try:
            body = [['shoulder_line', [self.landmarks[11], self.landmarks[12]]], ['waist_line', [self.landmarks[23], self.landmarks[24]]], ['left_shoulder_waist', [self.landmarks[11],  self.landmarks[23]]], ['right_shoulder_waist', [self.landmarks[12], self.landmarks[24]]], ['right_thigh', [self.landmarks[24], self.landmarks[26]]], ['left_thigh',[self.landmarks[23], self.landmarks[25]]], ['right_leg', [self.landmarks[26], self.landmarks[28]]], ['left_leg', [self.landmarks[25], self.landmarks[27]]], ['right_forearm', [self.landmarks[14], self.landmarks[16]]], ['left_forearm', [self.landmarks[13], self.landmarks[15]]], ['right_bicep', [self.landmarks[12], self.landmarks[14]]], ['left_bicep', [self.landmarks[11], self.landmarks[13]]]]

            self.linelist.points = []
            self.linelist.header.frame_id = "kinect2_rgb_optical_frame"
            self.linelist.header.stamp = rospy.Time.now()
            self.linelist.type = Marker.LINE_LIST

            self.linelist.id = 1
            self.linelist.action = Marker.ADD
            self.linelist.scale.x = 0.05

            self.linelist.color.g = 1.0
            self.linelist.color.a = 1.0

            for _,point1 in body:
                for pt in point1:
                   ptl_x, ptl_y, ptl_z = list(pc2.read_points(point_data, skip_nans=True, field_names = ('x', 'y', 'z'), uvs = [(pt[0], pt[1])]))[0]

                   self.linelist_point = Point()
                   self.linelist_point.x = ptl_x
                   self.linelist_point.y = ptl_y
                   self.linelist_point.z = ptl_z
                   self.linelist.points.append(self.linelist_point)
        except:
            pass


    def create_spheres(self, point_data):

        try:
            points = [self.landmarks[0], self.landmarks[15], self.landmarks[13], self.landmarks[11], self.landmarks[16], self.landmarks[14], self.landmarks[12], self.landmarks[23], self.landmarks[25], self.landmarks[27], self.landmarks[24], self.landmarks[26], self.landmarks[28]]

            self.spheres.points = []
            self.spheres.header.frame_id = "kinect2_rgb_optical_frame"
            self.spheres.header.stamp = rospy.Time.now()

            self.spheres.id = 0
            self.spheres.action = Marker.ADD

            self.spheres.type = Marker.SPHERE_LIST
            self.spheres.color.r = 1.0
            self.spheres.color.a = 1.0

            self.spheres.scale.x = 0.08
            self.spheres.scale.y = 0.08
            self.spheres.scale.z = 0.01

            for p in points:
               pts_x, pts_y, pts_z = list(pc2.read_points(point_data, skip_nans=True, field_names = ('x', 'y', 'z'), uvs = [(pt[0], pt[1])]))[0]

               self.spheres_point = Point()
               self.spheres_point.x = pts_x
               self.spheres_point.y = pts_y
               self.spheres_point.z = pts_z
               self.spheres.points.append(self.spheres_point)
        except:
            pass



def main(args):
    rospy.init_node("mediapipe_stream", anonymous = True)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
