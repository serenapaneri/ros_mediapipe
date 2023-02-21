#!/usr/bin/env python3

import rospy
import numpy as np
from spot_mediapipe.srv import Movement
from spot_mediapipe.srv import Blink

def main():

    rospy.init_node('movement', anonymous = True)

    rospy.wait_for_service('movement')
    motion_client = rospy.ServiceProxy('movement', Movement)

    
    rate = rospy.Rate(0.5)
    while True:
        res = motion_client()
        if res.mot == True:
            print('The person is moving')
        else:
            print('The person is not moving')

        rate.sleep()

if __name__ == '__main__':
    main()
