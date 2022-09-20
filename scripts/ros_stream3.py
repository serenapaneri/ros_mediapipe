#!/usr/bin/env python3

from __future__ import print_function

import rospy
import mediapipe as mp
import cv2, cv_bridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cv_image = None

def image_callback(msg):
    global cv_image
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, 'passthrough')
        rospy.loginfo('Access to the frontleft camera')
    except CvBridgeError as e:
        print(e)


def main():
    rospy.init_node('ros_stream')

    # setting mediapipe holistic model
    mp_drawing = mp.solutions.drawing_utils
    mp_holistic = mp.solutions.holistic

    image_sub = rospy.Subscriber('camera/frontleft/camera/image', Image, image_callback)    
    
    with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:

        while not rospy.is_shutdown():
            try:
                image = cv2.cvtColor(cv_image,cv2.COLOR_GRAY2RGB)
                image.flags.writeable = False

                result = holistic.process(image)

                image.flags.writeable = True
                image = cv2.cvtColor(cv_image,cv2.COLOR_RGB2BGR)

                # Draw face landmarks
                # mp_drawing.draw_landmarks(image, results.face_landmarks, mp_holistic.FACE_CONNECTIONS)

                # Pose Detections
                # mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)

                cv2.imshow("mediapipe", image)
                cv2.waitKey(10)

            except KeyboardInterrupt:
                print("shutting down")
                cv2.destroyAllWindows()


if __name__ == '__main__':
    main()


