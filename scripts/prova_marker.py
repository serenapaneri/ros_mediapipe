#!/usr/bin/env python3

import rospy
import time
from visualization_msgs.msg import Marker, MarkerArray

def sphere(x, y, z, i):
    ##### SPHERE #####
    sphere_marker = Marker()
    sphere_marker.header.frame_id = 'odom'
    sphere_marker.header.stamp = rospy.Time.now()

    sphere_marker.id = i
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
    sphere_marker.pose.position.x = x
    sphere_marker.pose.position.y = y
    sphere_marker.pose.position.z = z
    sphere_marker.pose.orientation.x = 0.0
    sphere_marker.pose.orientation.y = 0.0
    sphere_marker.pose.orientation.z = 0.0
    sphere_marker.pose.orientation.w = 1.0

    return sphere_marker


def text(x, y, z, n):
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
    text_marker.pose.position.x = x
    text_marker.pose.position.y = y
    text_marker.pose.position.z = z + 0.5
    text_marker.pose.orientation.w = 1.0

    text_marker.text = 'ID000' + str(n)

    return text_marker
    

def main():

    rospy.init_node('prova_marker')
    sphere_marker_pub = rospy.Publisher('/sphere_marker', Marker, queue_size = 0)
    text_marker_pub = rospy.Publisher('/text_marker', Marker, queue_size = 0)

    x = 0.0
    y = 0.0
    z = 1.0
    n = 1
    i = 0

    sphere_marker = sphere(x, y, z, i)
    text_marker = text(x, y, z, n)    

    #### IMPLEMENTARE MARKER ARRAY #####

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        sphere_marker_pub.publish(sphere_marker)
        text_marker_pub.publish(text_marker)
        rate.sleep()

if __name__ == '__main__':
    main()

    
