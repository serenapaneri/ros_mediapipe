#!/usr/bin/env python

import rospy
import mediapipe as mp
import cv2, cv_bridge
from sensor_msgs.msg import Image

class Mediapipe:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window",1)
        self.image_sub = rospy.Subscriber('camera/frontleft/camera/image', Image, self.image_callback)


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow("window",image)
        cv2.waitKey(3)

rospy.init_node('ros_stream4')
mediapipe = Mediapipe()
rospy.spin()
