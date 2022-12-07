#!/usr/bin/env python3

from __future__ import print_function

import rospy
import cv2
import numpy as np
import yaml
import time

def main():
    rospy.init_node("prova_yaml", anonymous = True)
    with open ("calibration_matrix.yaml") as file:
        document = yaml.full_load(file)
        print(document)
        print('-----------')
        print(document["camera_matrix"])
        print('-----------')
        print(document["dist_coeff"])

if __name__ == '__main__':
    main()
