#!/usr/bin/env python3

import rospy
import time
from visualization_msgs.msg import Marker, MarkerArray

def main():

    rospy.init_node('prova_marker')
    sphere_marker_pub = rospy.Publisher('/sphere_marker', Marker, queue_size = 0)
    text_marker_pub = rospy.Publisher('/text_marker', Marker, queue_size = 0)


    ##### SPHERE #####
    sphere_marker = Marker()
    sphere_marker.header.frame_id = 'odom'
    sphere_marker.header.stamp = rospy.Time.now()

    sphere_marker.id = 0
    sphere_marker.type = 2 # sphere
    sphere_marker.action = Marker.ADD

    # Set the scale of the marker
    sphere_marker.scale.x = 0.8
    sphere_marker.scale.y = 0.8
    sphere_marker.scale.z = 0.8

    # Set the color
    sphere_marker.color.r = 1.0
    sphere_marker.color.g = 0.0
    sphere_marker.color.b = 0.0
    sphere_marker.color.a = 1.0

    # Set the pose of the marker
    sphere_marker.pose.position.x = 0
    sphere_marker.pose.position.y = 0
    sphere_marker.pose.position.z = 1.0
    sphere_marker.pose.orientation.x = 0.0
    sphere_marker.pose.orientation.y = 0.0
    sphere_marker.pose.orientation.z = 0.0
    sphere_marker.pose.orientation.w = 1.0


    ##### TEXT #####
    text_marker = Marker()
    text_marker.header.frame_id = 'odom'
    text_marker.header.stamp = rospy.Time.now()

    text_marker.id = 1
    text_marker.type = 9
    text_marker.action = Marker.ADD

    # Set the scale of the marker
    text_marker.scale.z = 0.5

    # Set the color
    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0

    # Set the pose of the marker
    # marker.pose.position.x = 0
    # marker.pose.position.y = 0
    text_marker.pose.position.z = 1.5
    # marker.pose.orientation.x = 0.0
    # marker.pose.orientation.y = 0.0
    # marker.pose.orientation.z = 0.0
    text_marker.pose.orientation.w = 1.0

    text_marker.text = 'ID0001'

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        sphere_marker_pub.publish(sphere_marker)
        text_marker_pub.publish(text_marker)
        rate.sleep()

if __name__ == '__main__':
    main()

    
