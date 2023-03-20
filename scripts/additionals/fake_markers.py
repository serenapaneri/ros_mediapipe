#!/usr/bin/env python3

import rospy
import time
from visualization_msgs.msg import Marker, MarkerArray

markers = {}
markers_list = []


##
# function to display on rviz a sphere that has coordinates specified in the parameters
def sphere(x, y, z, id_):
    sphere_marker = Marker()
    sphere_marker.header.frame_id = 'map'
    sphere_marker.header.stamp = rospy.Time.now()

    sphere_marker.id = id_
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


##
# function to display on rviz a text that has coordinates specified in the parameters
def text(x, y, z, n):
    text_marker = Marker()
    text_marker.header.frame_id = 'map'
    text_marker.header.stamp = rospy.Time.now()

    text_marker.id = 100 + n
    text_marker.type = 9 # text
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
    # to publish the position and ID of the individuals
    marker_pub = rospy.Publisher('/people_marker', MarkerArray, queue_size = 0)

    # w.r.t rviz this is constant and irrelevant from the kinect 
    x = 1.7
    y = 2.45
    z = 1.0

    k = 0
    n = 1

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        if k >= n:
            marker_pub.publish(markers_list)
            rate.sleep()
        else:
            k += 1
            markers["sphere_marker" + str(k)] = sphere(x, y, z, k)
            markers["text_marker" + str(k)] = text(x, y, z, k)
            for values in markers.values():
                markers_list.append(values)
            marker_pub.publish(markers_list)
            rate.sleep()


if __name__ == '__main__':
    main()

    

