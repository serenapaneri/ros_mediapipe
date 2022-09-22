#!/usr/bin/env python3
#!coding=utf-8

"""
It takes in estimated 2d anatomical landmarks in image coordinate(u,v) and convert them to world coordinate(x,y,z).
Also,these projected landmarks are displayed in rviz with the help of sphere markers(joints) that are joined 
together with line markers(links). Here I have used two approaches to estimate the 3d landmarks.

1.To get world coordinate(x,y,z) from camera intrinsic parameters:
Z = depth_image[v,u]
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy

where,
cx=X-axis optical center in pixels
cy=Y-axis optical center in pixels.
fx=X-axis focal length in meters
fy=X-axis focal length in meters
"""

import rospy
import cv2
import mediapipe as mp
import numpy as np
import cv_bridge
import sensor_msgs.point_cloud2 as pc2
import struct
import message_filters as mf 
import time
import os
import sys

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
# from cv_bridge.boost.cv_bridge_boost import getCvType
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class Mediapipe:

    def __init__(self):
        """
          Init function of the class in which we set the package cv_bridge and the mediapipe holistic model.
          Moreover we have two subscribers used to communicate with the camera and depth camera topics and
          a publisher that allows to draw with markers on the opencv image.
        """
        # setting the cv_bridge
        self.bridge = CvBridge()

        # setting mediapipe holistic model
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic()

        # message_filter allows to chain messages together, in this case subscription
        #subscribers to the camera and depth topic
        self.camera_sub = mf.Subscriber("/spot/camera/frontleft/camera/image", Image)
        self.depth_sub = mf.Subscriber("/spot/depth/frontleft/camera/image", Image)
        self.depth_info_sub = mf.Subscriber("/spot/depth/frontleft/camera_info", CameraInfo)

        # message_filter syncronizes messages and associate a callback 
        syn = mf.ApproximateTimeSynchronizer([self.camera_sub, self.depth_sub, self.depth_info_sub], queue_size = 10, slop = 0.1)
        syn.registerCallback(self.image_callback)

        # publisher that allows to draw markers and we use a MarkerArray 
        # since we have a lots of markes to display in Rviz
        self.marker_pub = rospy.Publisher("markers", MarkerArray, queue_size = 10)
        # in this case we will visualize spheres in the joints and lines for the links
        self.spheres, self.linelist = Marker(), Marker()
        self.marker_array = MarkerArray()


    def holistic_2d(self, image):
        """
          This function gets the 2d pose from a opencv image
        """
        # initializing a list 
        pts = []
        # converting from BGR format into RGB one cause is the one feasible with mediapipe
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # all detections are stored inside this variable results
        results = self.holistic.process(rgb_image)
        # connecting the face landmarks
        self.mp_drawing.draw_landmarks(image, results.face_landmarks, self.mp_holistic.FACE_CONNECTIONS)
        if results.pose_landmarks:
            # connecting the pose landmarks with links
            self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS)
            # count is the current iterarion and value is the value of the item at the current iteration 
            for id, value in enumerate(results.pose_landmarks.landmark):
                # with shape we are getting the image size
                # h = height, w = width, c = channel
                h,w,c = image.shape
                # convert the string into an integer 
                imgx, imgy = int(value.x*w), int(value.y*h)
                # store the values in the list
                pts.appen((imgx, imgy))
        return pts, image


    def create_line_list(self, depth_arr, rpts):
      #It creates lines markers between two points. They represent the links of the skeleton
        try:
            # storing on the list the corresponding mediapipe connections between markers
            # it does not include all the markers and the connections but only the ones of the scheleton
            body=[['shoulder',[rpts[11],rpts[12]]],
            ['waist',[rpts[23],rpts[24]]],
            ['left_shoulder_waist',[rpts[11],rpts[23]]],
            ['right_shoulder_waist',[rpts[12],rpts[24]]],
            ['right_thigh',[rpts[24],rpts[26]]],
            ['left_thigh',[rpts[23],rpts[25]]],
            ['right_leg',[rpts[26],rpts[28]]],
            ['left_leg',[rpts[25],rpts[27]]],
            ['right_forearm',[rpts[14],rpts[16]]],
            ['left_forearm',[rpts[13],rpts[15]]],
            ['right_bicep',[rpts[12],rpts[14]]],
            ['left_bicep',[rpts[11],rpts[13]]]]

            ## setting Marker fields https://nu-msr.github.io/me495_site/lecture07_visual.html
            # points is a marker type
            self.linelist.points = []
            # header.frame_id is what the pose is relative to
            self.linelist.header.frame_id = "kinect_frame"
            # header contains the time at which the marker comes into existence
            self.linelist.header.stamp = rospy.Time.now()
            # LINE_LIST creates lines between pair of points
            self.linelist.type = Marker.LINE_LIST
            # we only set the w member of the pose
            self.linelist.pose.orientation.w = 1.0
            # id to identify the marker which is uniquw
            self.linelist.id = 1
            # action is what we want to do with the marker, and in this case we want to add it
            self.linelist.action = Marker.ADD 
            # it is used to set the size of the marker that by default is 1x1x1 meters
            self. linelist.scale.x = 0.05
            # we set in green the links between the sphere markers
            self.linelist.color.g = 1.0
            # setting alpha to 1 avoids to have a trasparent marker
            self.linelist.color.a = 1.0

            # for all the markers in the body list 
            for _,pointl in body: 
                # for each elements in pointl
                for pt in pointl:
                    depth_val = float(depth_arr[pt[1], pt[0]])
                    ptl_x,ptl_y,ptl_z = self.depth_to_xyz(pt[0],pt[1],depth_val)
                    
                    # setting point methods
                    self.linelist_point = Point()
                    self.linelist_point.x = ptl_x
                    self.linelist_point.y = ptl_y
                    self.linelist_point.z = ptl_z
                    # storing the values in linelist.point
                    self.linelist.points.append(self.linelist_point)
                
        except:
            pass


    def create_spheres(self,depth_arr,rpts):
    """
      It creates sphere markers that are positioned in the joints of the body
    """
        try:
            #points=[nose,left_wrist,right,wrist,left_ankle,right ankle]
            points=[rpts[0],rpts[15],rpts[16],rpts[27],rpts[28]]
            self.spheres.points = []
            self.spheres.header.frame_id = "kinect_frame"
            self.spheres.header.stamp = rospy.Time.now()
                                
            self.spheres.id = 0
            self.spheres.action = Marker.ADD
                
            # points
            self.spheres.type = Marker.SPHERE_LIST
            self.spheres.pose.orientation.w = 1.0
            self.spheres.color.r = 1.0
            self.spheres.color.a = 1.0
                    
            self.spheres.scale.x = 0.08
            self.spheres.scale.y = 0.08
            self.spheres.scale.z = 0.01
            for p in points:
                depth_val=float(depth_arr[p[1], p[0]])
                pts_x,pts_y,pts_z=self.depth_to_xyz(p[0],p[1],depth_val)
                
                self.sphere_point = Point()
                self.sphere_point.x = pts_x
                self.sphere_point.y = pts_y
                self.sphere_point.z = pts_z
                self.spheres.points.append(self.sphere_point)
                    
        except:
            pass


    def depth_to_xyz(self,u,v,depth_val):
        """  
          This extract x, y, z coordinates from u, v image coordinates.
          u represents the x image coordinate
          v represents the y image coordinate
          depth_val is the value taken from the depth image that is used for z coordinate 
        """

        # focal length x axis
        fx = self.cam_intrin[0]
        # focal length y axis
        fy = self.cam_intrin[4]
        # optical centre x axis
        cx = self.cam_intrin[2]
        # optical centre y axis
        cy = self.cam_intrin[5]

        # converting into x, y and z 
        z = float(depth_val)
        x = float((u - cx)/fx)*z
        y = float((v - cy)/fy)*z

        result = [x, y, z]
        return result

            
    def image_callback(self,rgb_data,depth_data,camera_data):
    """
      Callback function of the subscribers to the camera topic
    """
        try:
            # converting a sensor_msgs into an opencv image for both rgb and depth data
            img = self.bridge.imgmsg_to_cv2(rgb_data,"bgr8")
            d_img = self.bridge.imgmsg_to_cv2(depth_data,"passthrough")
            # print the dimensions of the two images
            print(d_img.shape,img.shape)
            # using holistic_sd function that sets the mediapipe environment
            rpts,rimg = self.holistic_2d(img)
            depth_arr = np.array(d_img)
            self.cam_intrin = list(camera_data.K)
            self.create_line_list(depth_arr,rpts)
            self.create_spheres(depth_arr,rpts)
            self.marker_array.markers.extend([self.spheres,self.linelist])
            self.marker_pub.publish(self.marker_array)

            
        except CvBridgeError as e:
            print(e)
        
        cv2.startWindowThread()
        cv2.namedWindow('image')   
        cv2.imshow('image', rimg)
        cv2.waitKey(0)


def main(args):
    rospy.init_node('ros_stream5', anonymous=True)
    mediapipe = Mediapipe()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

