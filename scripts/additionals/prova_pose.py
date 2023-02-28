#!/usr/bin/env python

import rospy
import time
import actionlib
import actionlib.msg
import spot_msgs.msg
# action_client = None

import geometry_msgs.msg 
from geometry_msgs.msg import Pose

# pub_body = None

default = 0.0
look_up = - 0.03
look_down1 = 0.08 
look_down2 = 0.12


####### STRUCTURE OF THE POSEBODY.ACTION #####
'''
# The pose the body should be moved to. Only the orientation and the z component (body height) of position is considered.
# If this is unset, the rpy/body height values will be used instead.

geometry_msgs/Pose body_pose

# Alternative specification of the body pose with rpy (in degrees). These values are ignored if the body_pose is set

int8 roll
int8 pitch
int8 yaw
float32 body_height
---
bool success
string message
---
string feedback
'''

def pose_body(pitch):

    global pub_body
    pose_msg = Pose() 
    pose_msg.orientation.y = pitch
    pose_msg.orientation.w = 1.0
    rospy.sleep(1)
    return pose_msg

#     pub_body.publish(pose_msg)
#     time.sleep(5)
#     print('Done')


# def pose_client():

    # action_client = actionlib.SimpleActionClient('/spot/pose_body', spot_msgs.msg.PoseBodyAction)
    # action_client = actionlib.SimpleActionClient('/spot/pose_body', spot_msgs.msg.PoseBodyActionGoal)

#     print('1')
    # action_client.wait_for_server()

    # goal = spot_msgs.msg.PoseBodyActionGoal()
#     goal = spot_msgs.msg.PoseBodyGoal()
#     print('2')
#     goal.orientation.y = 0.12
#     goal.orientation.w = 0.12
#     print('3')

#     action_client.send_goal(goal)
#     print('4')
#     action_client.wait_for_result()
#     print('5')


def main():

    global look_down2
    # pub_body = rospy.Publisher('/spot/body_pose', Pose, queue_size = 1)

    # try:
    #     pose_body(lookdown2)
    #     print('Down')
    #     time.sleep(10)
    #     pose_body(default)
    #     print('Down')
    # except:
    #     print('Error')

    # rospy.spin()

    try:
        rospy.init_node('prova_pose')
        # print('6')
        pub_body = rospy.Publisher('/spot/body_pose', Pose, queue_size = 1)
        # print('7')
        # pose_msg = Pose()
        # pose_msg.orientation.y = 0.0
        # print('8')
        # pose_msg.orientation.w = 1.0
        # print('9')
        # rospy.sleep(1)
        prova = pose_body(default)
        pub_body.publish(prova)
        # rospy.sleep(5)
        print('done')
    except rospy.ROSInterruptException:
        print('stopped')
        

if __name__ == '__main__':
    main()
