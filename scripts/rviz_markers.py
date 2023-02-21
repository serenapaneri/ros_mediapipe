#!/usr/bin/env python3

import rospy
import time
from visualization_msgs.msg import Marker, MarkerArray
from spot_mediapipe.srv import Found
from spot_mediapipe.srv import XYZ

# numbers of found individuals
found_client = None
# coordinates of the found individual
xyz_client = None

markers = {}
markers_list = []

n = 0

def sphere(x, y, z, id_):
    sphere_marker = Marker()
    sphere_marker.header.frame_id = 'odom'
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


def text(x, y, z, n):
    text_marker = Marker()
    text_marker.header.frame_id = 'odom'
    text_marker.header.stamp = rospy.Time.now()

    text_marker.id = 100*n
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

    global found_client, xyz_client, markers, markers_list, n
    rospy.init_node('prova_marker')

    # rospy.wait_for_service('foundpeople')
    # found_client = rospy.ServiceProxy('foundpeople', Found)
    rospy.wait_for_service('coordinates')
    xyz_client = rospy.ServiceProxy('coordinates', XYZ)
    marker_pub = rospy.Publisher('/people_marker', MarkerArray, queue_size = 0)

    # w.r.t rviz this is constant and independent from the kinect 
    z_ = 1.0

    # people counter
    k = 0

    # n = 1

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # client to retrieve people coordinates
        res_xyz = xyz_client()
        x_coord = res_xyz.x
        y_coord = res_xyz.y
        z_coord = res_xyz.z

        # client that retrieves IDs
        res_found = found_client()
        n = res_found.peoplefound

        if len(x_coord) > 1 and len(z_coord) > 1:
            # here should go the x coordinates (?)
            if x_coord[0] < 3.0:
                y_ = x_coord[0]
            else:
                y_ = x_coord[1]

            # here should go the z coordinates (?)
            if z_coord[0] < 3.5:
                x_ = z_coord[0]
            else:
                x_ = z_coord[1]

            print('x: {}'.format(x_))
            print('y: {}'.format(y_))
            
            if k >= n:
                marker_pub.publish(markers_list)
                rate.sleep()
            else:
                k = n
                markers["sphere_marker" + str(k)] = sphere(x_, y_, z_, k)
                markers["text_marker" + str(k)] = text(x_, y_, z_, k)
                for values in markers.values():
                    markers_list.append(values)
                    # print(markers_list)
                marker_pub.publish(markers_list)
                rate.sleep()

        else:
            rate.sleep()

if __name__ == '__main__':
    main()

    

