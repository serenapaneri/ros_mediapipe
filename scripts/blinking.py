#!/usr/bin/env python3

import rospy
import math
from mediapipe_stream import *

# to see the markers of the face model with mediapipe
# https://github.com/tensorflow/tfjs-models/blob/master/face-landmarks-detection/mesh_map.jpg

right_eye = [33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161 , 246]
left_eye = [362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385,384, 398]

def main():
    rospy.init_node("blinking", anonymous = True)
    medpipe = mediapipe()

    rate = rospy.Rate(1)
    if medpipe.results is not None and medpipe.results.face_landmarks:
        f_landmarks = medpipe.results.face_landmarks.landmark
        print(f_landmarks)
        # print(math.dist(p1, p2))
        rate.sleep()        
    else:
        print("blinking")
        rate.sleep() 
if __name__ == '__main__':
    main()
