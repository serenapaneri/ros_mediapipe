#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import yaml

def main():
    with open (r'/home/spot/ros_ws/src/spot_mediapipe/scripts/calibration_matrix.yaml') as file:
        document = yaml.load(file)
        print(document)
        print('---------------')
        print(document['camera_matrix'])
        print('---------------')
        print(document['dist_coeff'])

if __name__ == '__main__':
    main()
