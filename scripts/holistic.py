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

mpipe = None
landmarks = None
movement = None

class mediapipe:

    def __init__(self):
        # setting the cv_bridge
        self.bridge = CvBridge()

        # setting mediapipe holistic model
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic

        # subscribers to the camera topic
        self.camera_sub = rospy.Subscriber("/spot/camera/frontleft/image", Image, self.camera_callback)


    def calculate_angle(self, a, b, c):
        """
          Function used to compute an angle between 3 joints of interest.
        """
        a = self.np.array(a) # first joint
        b = self.np.array(b) # mid joint
        c = self.np.array(c) # end joint
    
        # calculate the radians between 3 joints and then convert it in angles
        self.radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
        self.angle = np.abs(radians*180.0/np.pi)
    
        # the angle should be beween 0 and 180
        if angle > 180.0:
            angle = 360-angle
        return angle


    def plot_init():
        
    


    def holistic_2d(self, data):
        global mpipe. landmarks
        # initiate holistic model 
        with self.mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
            # in order to work with mediapipe we need the format RGB
            rgb_image = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
            rgb_image.flags.writeable = False

            # make detections
            results = holistic.process(rgb_image)
            if result.pose_landmarks:
                mpipe = True
            else:
                mpipe = False

            # in order to work with opencv we need the BGR format
            rgb_image.flags.writeable = True
            rgb_image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)

            # face mask model 
            # self.mp_drawing.draw_landmarks(rgb_image, results.face_landmarks, self.mp_holistic.FACEMESH_IRISES, 
            #                                landmark_drawing_spec=None, connection_drawing_spec=mp_drawing_styles
            #                                .get_default_face_mesh_iris_connections_style())

            self.mp_drawing.draw_landmarks(rgb_image, results.face_landmarks, self.mp_holistic.FACE_CONNECTIONS,
                                     mp_drawing.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1),
                                     mp_drawing.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1)
                                     )

            # body pose model
            self.mp_drawing.draw_landmarks(rgb_image, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                                     mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=4),
                                     mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                     )

            # extract landmarks
            try:
                landmarks = result.pose_landmarks.landmark
            except:
                pass

            # left arm
            left_pinky = [landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].x, landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].y]
            left_wrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
            left_elbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
            left_shoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]

            # right arm
            right_pinky = [landmarks[mp_pose.PoseLandmark.RIGHT_PINKY.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_PINKY.value].y]
            right_wrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]
            right_elbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
            right_shoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]

            # left leg
            left_hip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x, landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
            left_knee = [landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x, landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y]
            left_ankle = [landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].x, landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].y]
            left_foot_index = [landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].y]

            # right leg
            right_hip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
            right_knee = [landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].y]
            right_ankle = [landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].y]
            right_foot_index = [landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].y]

            # calculate angles
            l_wrist = calculate_angle(left_pinky, left_wrist, left_elbow)
            r_wrist = calculate_angle(right_pinky, right_wrist, right_elbow)
            l_elbow = calculate_angle(left_wrist, left_elbow, left_shoulder)
            r_elbow = calculate_angle(right_wrist, right_elbow, right_shoulder)
            l_shoulder = calculate_angle(left_elbow, left_shoulder, left_hip)
            r_shoulder = calculate_angle(right_elbow, right_shoulder, right_hip)
            l_hip = calculate_angle(left_shoulder, left_hip, left_knee)
            r_hip = calculate_angle(right_shoulder, right_hip, right_knee)
            l_knee = calculate_angle(left_hip, left_knee, left_ankle)
            r_knee = calculate_angle(right_hip, right_knee, right_ankle)
            l_ankle = calculate_angle(left_knee left_ankle, left_foot_index)
            r_ankle = calculate_angle(right_knee, right_ankle, right_foot_index)

            angles = np.array[l_wrist, r_wrist, l_elbow, r_elbow, l_shoulder, r_shoulder, l_hip, r_hip, l_knee, r_knee, l_ankle, r_ankle]
            # evaluate if the values of the array change during time and plot the data
            # to plot data maybe is better to do two different figures (upper part vs lower part) with 3 subpot each

            # plot movemets

            # upper body 
            fig_up = plt.figure(1)
            fig_up.suptitle("Upper body: left vs right", fontsize = 16)
            ax1 = fig_up.add_subplot(3,2,1)
            ax1.set_title("left wrist")
            ax1.set(xlabel = 'Time', ylabel = 'Amplitude')
            ax2 = fig_up.add_subplot(3,2,2)
            ax2.set_title("right wrist")
            ax2.set(xlabel = 'Time', ylabel = 'Amplitude')
            ax3 = fig_up.add_subplot(3,2,3)
            ax3.set_title("left elbow")
            ax3.set(xlabel = 'Time', ylabel = 'Amplitude')
            ax4 = fig_up.add_subplot(3,2,4)
            ax4.set_title("right elbow")
            ax4.set(xlabel = 'Time', ylabel = 'Amplitude')
            ax5 = fig_up.add_subplot(3,2,5)
            ax5.set_title("left shoulder")
            ax5.set(xlabel = 'Time', ylabel = 'Amplitude')
            ax6 = fig_up.add_subplot(3,2,6)
            ax6.set_title("right shoulder")
            ax6.set(xlabel = 'Time', ylabel = 'Amplitude')

            plot1, = ax1.plot([], [], linewidth = 5.0)
            plot2, = ax2.plot([], [], linewidth = 5.0)
            plot3, = ax3.plot([], [], linewidth = 5.0)
            plot4, = ax4.plot([], [], linewidth = 5.0)
            plot5, = ax5.plot([], [], linewidth = 5.0)
            plot6, = ax6.plot([], [], linewidth = 5.0)


            anim1 = FuncAnimation(fig_up, update_plot, init_func=plot_init)
            fig_up.show()


            # lower body
            fig_low = plt.figure(2)
            fig_up.suptitle("Lower body: left vs right", fontsize = 16)
            ax7 = fig_low.add_subplot(3,2,1)
            ax7.set_title("left hip")
            ax7.set(xlabel = 'Time', ylabel = 'Amplitude')
            ax8 = fig_low.add_subplot(3,2,2)
            ax8.set_title("right hip")
            ax8.set(xlabel = 'Time', ylabel = 'Amplitude')
            ax9 = fig_low.add_subplot(3,2,3)
            ax9.set_title("left knee")
            ax9.set(xlabel = 'Time', ylabel = 'Amplitude')
            ax10 = fig_low.add_subplot(3,2,4)
            ax10.set_title("right knee")
            ax10.set(xlabel = 'Time', ylabel = 'Amplitude')
            ax11 = fig_low.add_subplot(3,2,5)
            ax11.set_title("left ankle")
            ax11.set(xlabel = 'Time', ylabel = 'Amplitude')
            ax12 = fig_low.add_subplot(3,2,6)
            ax12.set_title("right ankle")
            ax12.set(xlabel = 'Time', ylabel = 'Amplitude')

            plot7, = ax7.plot([], [], linewidth = 5.0)
            plot8, = ax8.plot([], [], linewidth = 5.0)
            plot9, = ax9.plot([], [], linewidth = 5.0)
            plot10, = ax10.plot([], [], linewidth = 5.0)
            plot11, = ax11.plot([], [], linewidth = 5.0)
            plot12, = ax12.plot([], [], linewidth = 5.0)

            anim2 = FuncAnimation(fig_low, update_plot, init_func=plot_init)
            fig_low.show()

            # plot the skeleton
            self.mp_drawing.plot_landmarks(results.pose_world_landmarks, mp_holistic.POSE_CONNECTIONS)



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

        
def main(args):
    mpipe = mediapipe()
    rospy.init_node("holistic", anonymous = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
