#!/usr/bin/env python3

import rospy
import mediapipe as mp
import numpy as np
import cv2
import cv_bridge
import sys

# needed to use cv_bridge with python 3
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/home/spot/cv_bridge_ws/install/lib/python3/dist-packages')

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from mediapipe_stream import *
from spot_mediapipe.srv import Pose 
from visualization_msgs.msg import MarkerArray,Marker

# subscriber to the depth camera
depth_sub = None
# pose client
pose_client = None
# marker publisher
marker_pub = None

# for the visualization_msgs
marker_array = MarkerArray()
spheres = Marker()
linelist = Marker()
 
# camera matrix coefficients
camera_matrix = [1081.3720703125, 0.0, 959.5, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 1.0]

# depth array
depth_arr = []

landmarks = []

def depth_callback(depth_data):
    global depth_arr
    bridge = CvBridge()
    depth_img= bridge.imgmsg_to_cv2(depth_data,"passthrough")
    depth_arr = np.array(depth_img)
    return depth_arr


def create_line_list(depth_arr_, landmarks_):
    global linelist, landmarks
    try:
        body=[['shoulder_line', [landmarks_[11], landmarks_[12]]], ['waist_line', [landmarks_[23], landmarks_[24]]], ['left_shoulder_waist', [landmarks_[11], landmarks_[23]]],
            ['right_shoulder_waist', [landmarks_[12], landmarks_[24]]], ['right_thigh', [landmarks_[24], landmarks_[26]]], ['left_thigh', [landmarks_[23], landmarks_[25]]],
            ['right_leg', [landmarks_[26], landmarks_[28]]], ['left_leg', [landmarks_[25], landmarks_[27]]], ['right_forearm', [landmarks_[14], landmarks_[16]]],
            ['left_forearm', [landmarks_[13], landmarks_[15]]], ['right_bicep', [landmarks_[12], landmarks_[14]]], ['left_bicep', [landmarks_[11], landmarks_[13]]]]
        linelist.points = []
        linelist.header.frame_id = "kinect2_rgb_optical_frame"
        linelist.header.stamp = rospy.Time.now()
        linelist.type = Marker.LINE_LIST
        linelist.pose.orientation.w = 1.0
        linelist.id = 1
        linelist.action = Marker.ADD 
        linelist.scale.x = 0.05

        linelist.color.g = 1.0
        linelist.color.a = 1.0

        for _,pointl in body:
            for pt in pointl:
                depth_val = float(depth_arr_[pt[1], pt[0]])
                ptl_x,ptl_y,ptl_z = depth_to_xyz(pt[0],pt[1],depth_val)
                   
                linelist_point = Point()
                linelist_point.x = ptl_x
                linelist_point.y = ptl_y
                linelist_point.z = ptl_z
                linelist.points.append(linelist_point)
                
    except:
        pass


def create_spheres(depth_arr_, landmarks_):
    global spheres
    try:
        points = [landmarks_[0], landmarks_[15], landmarks_[13], landmarks_[11], landmarks_[23], landmarks_[25], landmarks_[27], landmarks_[16], landmarks_[14], landmarks_[12], landmarks_[24], landmarks_[26], landmarks_[28]]
        spheres.points = []
        spheres.header.frame_id = "kinect2_rgb_optical_frame"
        # spheres.header.frame_id = "odom"
        spheres.header.stamp= rospy.Time.now()
                                
        spheres.id = 0
        spheres.action = Marker.ADD
                
        #points
        spheres.type = Marker.SPHERE_LIST
        spheres.pose.orientation.w = 1.0
        spheres.color.r = 1.0
        spheres.color.a = 1.0
                    
        spheres.scale.x = 0.08
        spheres.scale.y = 0.08
        spheres.scale.z = 0.01

        for p in points:
            depth_val = float(depth_arr_[p[1], p[0]])
            pts_x, pts_y, pts_z = depth_to_xyz(p[0], p[1], depth_val)
                
            sphere_point = Point()
            sphere_point.x = pts_x
            sphere_point.y = pts_y
            sphere_point.z = pts_z
            spheres.points.append(sphere_point)
                    
    except:
        pass


def depth_to_xyz(u,v,depth_val):
    global camera_matrix

    # these are the intrinsic parameter of the camera
    fx = camera_matrix[0]
    fy = camera_matrix[4]
    cx = camera_matrix[2]
    cy = camera_matrix[5]

    z = float(depth_val)
    x = float((u - cx)/fx)*z
    y = float((v - cy)/fy)*z

    result = [x, y, z]

    return result
   

def main():

    global depth_sub, info_sub, pose_client, marker_pub, spheres, linelist, depth_arr
    # initializing the node
    rospy.init_node('markers', anonymous=True)

    # depth camera subscriber
    depth_sub = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, depth_callback)
    # pose service
    pose_client = rospy.ServiceProxy('pose', Pose)
    # marker publisher
    marker_pub = rospy.Publisher('marker_display', MarkerArray, queue_size = 10)

    create_line_list(depth_arr, rpts)
    create_spheres(depth_arr, rpts)
    marker_array.markers.extend(spheres)
    marker_array.markers.extend(linelist)
    marker_pub.publish(marker_array)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
if __name__ == '__main__':
    main()
