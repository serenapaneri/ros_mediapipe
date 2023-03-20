#!/usr/bin/env python3

import rospy
import time
from visualization_msgs.msg import Marker, MarkerArray
from spot_mediapipe.srv import Found
from spot_mediapipe.srv import XYZ
from spot_mediapipe.srv import GlobalCoord
from spot_mediapipe.srv import XYCoord, XYCoordResponse

# nose in real coordinates
xy_nose_service = None
# numbers of found individuals
found_client = None
# global cordinates client
global_client = None
# coordinates of the found individual
xyz_client = None

markers = {}
markers_list = []

n = 0

x_ = 0
y_ = 0

def coord_handle(req):
    global x_, y_
    res = XYCoordResponse()
    res.x_nose = x_
    res.y_nose = y_
    return res

##
# function to find within a list the first value under a certain threshold and return it
def first_threshold(list_, thresh1, thresh2):
    for value in list_:
        if thresh1 < value and value < thresh2:
            return value
    return None

##
# function to display on rviz a sphere that has coordinates specified in the parameters
def sphere(x, y, z, id_):
    sphere_marker = Marker()
    sphere_marker.header.frame_id = 'odom'
    # sphere_marker.header.frame_id = 'kinect_rgb_optical_frame'
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
    text_marker.header.frame_id = 'odom'
    # text_marker.header.frame_id = 'kinect_rgb_optical_frame'
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

    global xy_nose_service, global_client, found_client, xyz_client, markers, markers_list, n, x_, y_
    rospy.init_node('prova_marker')

    # client that counts the number of people
    # rospy.wait_for_service('foundpeople')
    # found_client = rospy.ServiceProxy('foundpeople', Found)
    xy_nose_service = rospy.Service('xy_coord', XYCoord, coord_handle)
    # client that recieves the position of the individual w.r.t. the camera
    rospy.wait_for_service('coordinates')
    xyz_client = rospy.ServiceProxy('coordinates', XYZ)
    # global coordinates client
    rospy.wait_for_service('globalcoord')
    global_client = rospy.ServiceProxy('globalcoord', GlobalCoord)
    # to publish the position and ID of the individuals
    marker_pub = rospy.Publisher('/people_marker', MarkerArray, queue_size = 0)

    # w.r.t rviz this is constant and irrelevant from the kinect 
    z_ = 1.0

    # people counter
    k = 0

    n = 1

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        time.sleep(1)
        # client to retrieve people coordinates
        res_xyz = xyz_client()
        x_coord = res_xyz.x
        y_coord = res_xyz.y
        z_coord = res_xyz.z

        # client that retrieves IDs
        # res_found = found_client()
        # n = res_found.peoplefound

        # taking the first coordinates that is not affected by an error
        if len(x_coord) > 5 and len(z_coord) > 5:

            # here should go the x coordinates (?)
            y_ = first_threshold(x_coord, 1.0, 3.5)

            # here should go the z coordinates (?)
            x_ = first_threshold(z_coord, 1.0, 3.5)

            print('x: {}'.format(x_))
            print('y: {}'.format(y_))
            
            if k >= n:
                marker_pub.publish(markers_list)
                rate.sleep()
            else:
                time.sleep(3)
                res_global = global_client()
                x_global = res_global.x_global
                y_global = res_global.y_global
                k += 1
                markers["sphere_marker" + str(k)] = sphere(x_global, y_global, z_, k)
                markers["text_marker" + str(k)] = text(x_global, y_global, z_, k)
                for values in markers.values():
                    markers_list.append(values)
                marker_pub.publish(markers_list)
                rate.sleep()

        else:
            rate.sleep()

if __name__ == '__main__':
    main()

    

