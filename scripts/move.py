#!/usr/bin/env python3

from __future__ import print_function

import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf import transformations


# publishers for the topics /cmd_vel and /body_pose
pub_cmd = None
pub_body = None
sub_odom = None

position_ = Point()
position_ = 0
yaw_ = 0
state_ = 0

lin_vel = 0.3 # you should set this parameter
ang_vel = 30 # you should set this parameter
tilt = 1

def odom_callback(msg):
    global position_, yaw_
    position_ = msg.pose.pose.position
    quaternion = (msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y, 
                  msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    """
      Function that switch between different states
    """
    global state_
    state_ = state


def normalize_angle(angle):
    """
    This function is used to normalized the angle
    It always stays from o to 180°
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def go_forward(distance):
    """
      Function to go on a straight line 
    """
    twist_msg = Twist()
    twist_msg.linear.x = lin_vel
    twist_msg.angular.z = 0

    start_time = rospy.Time.now().to_sec()
    current_distance = 0

    while (current_distance < distance):
        pub_cmd.publish(twist_msg)
        time = rospy.Time.now().to_sec()
        current_distance = lin_vel*(time - start_time)
    twist_msg.linear.x = 0
    pub_cmd.publish(twist_msg)
    print('The robot is moving forward')
    # change_state(0)


def yaw(angle):
    """
      Always cointerclockwise
    """
    ang_vel_rad = ang_vel*(2*math.pi)/360
    relative_angle = angle*(2*math.pi)/360

    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = ang_vel_rad

    start_time = rospy.Time.now().to_sec()
    current_angle = 0

    while (current_angle < relative_angle):
        pub_cmd.publish(twist_msg)
        time = rospy.Time.now().to_sec()
        current_angle = ang_vel_rad*(time - start_time)
    twist_msg.angular.z = 0
    pub_cmd.publish(twist_msg)
    print('The robot is rotating')
    # change_state(1)


def look_up():
    """
      Function to make the robot tilt upward
      To control the body position relative to the feet
    """
    pose_msg = Pose()
    pose_msg.position.x = 0
    pose_msg.position.y = 0
    pose_msg.position.z = 0
    pose_msg.orientation.x = 0  
    pose_msg.orientation.y = 1
    pose_msg.orientation.z = 0
    pose_msg.orientation.w = 1

    pub_body.publish(pose_msg)
    print('The robot is looking up')
    # change_state(2)

def look_up2(tilt):

    pose_msg = Pose()
    quaternion = (pose_msg.orientation.x 
                  pose_msg.orientation.y
                  pose_msg.orientation.z
                  pose_msg.orientation.w)

    euler = transformations.euler_from_quaternion(quaternion)
    pitch_ = euler[1]
    pitch_ = tilt # should be an angle in degree 

    pub_body.publish(pose_msg)
    print('The robot is looking up')
    # change_state(2)


def stop():
    """
      Function used to stop the robot's behavior
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_cmd.publish(twist_msg)
    print('The robot is stopping')


def main():
    """
      This should be a state machine that has only two possible states
      and moreover the two function above are limited to a chosen distance.
      In addition there could be a third state in which the robot tilt up.
    """
    global pub_cmd, pub_body, sub_odom
    rospy.init_node('move')

    # publishers for /cmd_vel and /body_pose
    pub_cmd = rospy.Publisher('/spot/cmd_vel', Twist, queue_size = 1)
    pub_body = rospy.Publisher('/spot/body_pose', Pose, queue_size = 1)
    sub_odom = rospy.Subscriber('/spot/odometry', Odometry, odom_callback)

    print('The robot is moving within the environment')

    # change_state(0)
    while True:
        go_forward(2) # in meters
        yaw(45) # in degree
        go_forward(3)
        yaw(360)
        # look_up()
        stop()

    rospy.spin()

if __name__ == "__main__":
    main()
