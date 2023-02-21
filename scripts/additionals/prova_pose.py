#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Pose

pub_body = None

default = 0.0
look_up = - 0.03
look_down1 = 0.08 
look_down2 = 0.12

def pose_body(pose):

    global pub_body
    pose_msg = Pose()
    pose_msg.position.x = 0.0
    pose_msg.position.y = 0.0
    pose_msg.position.z = 0.0
    pose_msg.orientation.x = 0.0  
    pose_msg.orientation.y = pose
    pose_msg.orientation.z = 0.0
    pose_msg.orientation.w = 1.0

    pub_body.publish(pose_msg)
    time.sleep(5)
    print('Done')


def main():

    global pub_body, default, lookdown2
    rospy.init_node('prova_pose')
    
    pub_body = rospy.Publisher('/spot/body_pose', Pose, queue_size = 1)

    try:
        pose_body(lookdown2)
        print('Down')
        time.sleep(10)
        pose_body(default)
        print('Down')
    except:
        print('Error')

    rospy.spin()


if __name__ == '__main__':
    main()
