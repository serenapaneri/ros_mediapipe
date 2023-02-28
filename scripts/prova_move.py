#!/usr/bin/env python

import rospy
import time
import actionlib
import actionlib.msg
import spot_msgs.msg

action_client = None

# from geometry_msgs.msg import Pose

# pub_body = None


# def pose_body(pose):

#     global pub_body
#     pose_msg = Pose()
#     pose_msg.position.x = 0.0
#     pose_msg.position.y = 0.0
#     pose_msg.position.z = 0.0
#     pose_msg.orientation.x = 0.0  
#     pose_msg.orientation.y = pose
#     pose_msg.orientation.z = 0.0
#     pose_msg.orientation.w = 1.0

#     pub_body.publish(pose_msg)
#     time.sleep(5)
#     print('Done')


####### STRUCTURE OF THE TRAJECTORY.ACTION #####
'''
geometry_msgs/PoseStamped target_pose

# After this duration, the command will time out and the robot will stop. Must be non-zero

std_msgs/Duration duration

# If true, the feedback from the trajectory command must indicate that the robot is
# at the goal position. If set to false, the robot being near the goal is equivalent to
# it being at the goal. This is based on the feedback received from the boston dynamics
# API call at
# https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference.html?highlight=status_near_goal#se2trajectorycommand-feedback-status

bool precise_positioning
---
bool success
string message
---
string feedback
'''


def move_client():

    # action_client = actionlib.SimpleActionClient('/spot/trajectory', spot_msgs.msg.TrajectoryActionGoal)
    action_client = actionlib.SimpleActionClient('/spot/trajectory', spot_msgs.msg.TrajectoryAction)
    action_client.wait_for_server()

    goal = spot_msgs.msg.TrajectoryActionGoal()
    # goal.target_pose.header.frame_id = 'body'
    goal.goal.target_pose.header.frame_id = 'body'
    goal.goal.target_pose.pose.orientation.z = 1.0
    goal.goal.target_pose.pose.orientation.w = 1.0
    goal.goal.duration.data.sec = 5
    

    action_client.send_goal(goal)
    action_client.wait_for_result()


def main():

    global action_client
    
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
        rospy.init_node('prova_move')
        result = move_client()
    except rospy.ROSInterruptException:
        print('stopped')
        

if __name__ == '__main__':
    main()
