#!/usr/bin/env python
import cv2
import mediapipe as mp
import time
import rospy
from std_msgs.msg import String

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic


def status(input):
    pub.publish(input)

# For webcam input:
cap = cv2.VideoCapture(1)
pub = rospy.Publisher('status_holistic', String, queue_size=10)
rospy.init_node('ros_stream2', anonymous=True)
rate = rospy.Rate(10) # 10
with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
  while cap.isOpened():
    success, image = cap.read()


    # Flip the image horizontally for a later selfie-view display, and convert
    # the BGR image to RGB.
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_GREY2RGB)
    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    results = holistic.process(image)

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    # Draw face landmarks
    # mp_drawing.draw_landmarks(image, results.face_landmarks, mp_holistic.FACE_CONNECTIONS)

    # Pose Detections
    # mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)

        
    cv2.imshow('mediapipe', image)
    if cv2.waitKey(5) & 0xFF == 27:
      break

cap.release()
cv2.destroyAllWindows()
