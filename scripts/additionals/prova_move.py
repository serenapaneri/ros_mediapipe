#!/usr/bin/env python

import rospy
import time
import actionlib
import actionlib.msg
from spot_msgs.msg import *


def main():
  
    rospy.init_node('prova_move')  
    client = actionlib.SimpleActionClient('/spot/trajectory', spot_msgs.msg.TrajectoryAction)
  
    goal = spot_msgs.msg.TrajectoryGoal()

    goal.target_pose.header.frame_id = 'body'
    # goal.goal_id.id = 1
    goal.target_pose.pose.position.x = 1.2
    goal.target_pose.pose.position.y = - 0.8
    # goal.target_pose.pose.position.y = 0.0
    # goal.target_pose.pose.orientation.x = 0.0
    # goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z =  - 0.3
    goal.target_pose.pose.orientation.w = 1.0
    goal.duration.data.secs = 10
    rospy.loginfo("dopo dichiarazione target")
    time.sleep(1)
    client.send_goal(goal)
    client.wait_for_result()

    print('finish')
    
    rospy.spin()

if __name__ == '__main__':
    main()
