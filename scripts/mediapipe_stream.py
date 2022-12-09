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

# needed to use cv_bridge with python 3
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/home/spot/cv_bridge_ws/install/lib/python3/dist-packages') # controlla sia giusto

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

results = None
mpipe = None
landmarks = None
threshold = 0.5 # change this parameter with trial and error
stop = 200 # change this looking on the x of the graphs
time = np.arange(0, stop, 1) # it should give an x axis long stop and with the step at every sencond

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
        global mpipe, landmarks, results
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        dst = undistorted(cv_image)

        with self.mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
            # in order to work with mediapipe we need the format RGB
            rgb_image = cv2.cvtColor(dst, cv2.COLOR_BGR2RGB)
            rgb_image.flags.writeable = False

            # make detections
            results = holistic.process(rgb_image)
            if results.pose_landmarks: 
                mpipe = True
            else:
                mpipe = False

            # in order to work with opencv we need the BGR format
            rgb_image.flags.writeable = True
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            # face mask model 
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


def calculate_angle(a, b, c):
        """
          Function used to compute an angle between 3 joints of interest.
        """
        a = np.array(a) # first joint
        b = np.array(b) # mid joint
        c = np.array(c) # end joint
    
        # calculate the radians between 3 joints and then convert it in angles
        radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
        angle = np.abs(radians*180.0/np.pi)
    
        # the angle should be beween 0 and 180
        if angle > 180.0:
            angle = 360 - angle
        return angle

def undistorted(image):
    with open (r'/home/spot/ros_ws/src/spot_mediapipe/scripts/calibration_matrix.yaml') as file:
        document = yaml.load(file)
        camera_matrix = np.array(document["camera_matrix"])
        dist_coeff = np.array(document["dist_coeff"])
        h, w = image.shape[:2]
        newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeff[0], (w,h), 1, (w,h))
        mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeff[0], None, newcameramatrix, (w,h), 5)
        dst = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        return dst
        

def main(args):
    global landmarks
    rospy.init_node("mediapipe_stream", anonymous = True)
    medpipe = mediapipe()

    # try: 
    #     landmarks = results.pose_landmarks.landmark
    # except:
    #     pass

    # left arm
    # left_pinky = [landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].x, landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].y]
    # left_wrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
    # left_elbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
    # left_shoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]

    # right arm
    # right_pinky = [landmarks[mp_pose.PoseLandmark.RIGHT_PINKY.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_PINKY.value].y]
    # right_wrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]
    # right_elbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
    # right_shoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]

    # left leg
    # left_hip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x, landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
    # left_knee = [landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x, landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y]
    # left_ankle = [landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].x, landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].y]
    # left_foot_index = [landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].y]

    # right leg
    # right_hip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
    # right_knee = [landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].y]
    # right_ankle = [landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].y]
    # right_foot_index = [landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].y]

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

