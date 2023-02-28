#!/usr/bin/env python

# This script is simplified since the exploration part is not done with the rrt navigation but it
# is only done with a Twist and Odometry message. And the assuntion here is single person in a room.

import rospy
import random
import sys
import time
import smach
import smach_ros
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from spot_mediapipe.srv import Detection
from spot_mediapipe.srv import Audio
from spot_mediapipe.srv import Trauma
from spot_mediapipe.srv import Blink
from spot_mediapipe.srv import Movement
from spot_mediapipe.srv import HumanPose
from spot_mediapipe.srv import Docu
from spot_mediapipe.srv import Found, FoundResponse
from spot_mediapipe.srv import GoOn
from spot_mediapipe.srv import XYCoord

# people service
people_srv = None
# audio client to start the trauma behavior
audio_client = None 
# trauma client to recieve the answers
trauma_client = None 
# motion client to recieve information about the movement
motion_client = None 
# motion client to recieve information about the blinking
blink_client = None
# client to recieve information about the pose 
pose_client = None
# detection client
detection_client = None
# document client 
docu_client = None
# go_on client
goon_client = None
# xy real nose coordinates
xy_nose_client = None
# velocity publisher
pub_goal = None
# body pose publisher
pub_body = None

# robot state variables
actual_position = Point()
actual_position.x = 0 
actual_position.y = 0 
actual_yaw = 0

# list of landmarks collected, len(landmarks) is always 33
landmarks = []

# poses in which the person can be found 
pose = 0 
detected = False

# people found
found_people = 0 

stop = False
finish = False

##### PARAMETERS #####

# velocity parameters
lin_vel = 0.5 # you should set this parameter
ang_vel = 0.5 # you should set this parameter

# body pose parameters
#### DEVI SETTARE NUOVI PARAMETRI
default = 0.0
look_up = - 0.03
look_down1 = 0.08 
look_down2 = 0.12

# proxemics parameters
distance_standup = 1.8 # meters
distance_sitdown = 1.2 # meters
distance_laydown = 1.2 # meters

# point where to position the robot
goal_position_x = 0
goal_position_y = 0

xy_global_pos = []
ext_point_xy = []
actual_pos = []

start_time = 0

def people_found(req):
    global found_people
    res = FoundResponse()
    res.peoplefound = found_people
    return res

##
# This is the callback function of the subscriber to the topic odom and it is used to know the actual position
# and orientation of the robot in the space.
def odom_callback(msg):
    global actual_position, actual_yaw
    
    # actual position
    actual_position = msg.pose.pose.position
    # actual yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    actual_yaw = euler[2]


##
# Function used to rotate the robot or make it stop rotating
def rotate(ang):

    twist_msg = Twist()
    twist_msg.angular.z = ang
    rospy.sleep(1)
    return twist_msg 
    

def pose_body(pitch):

    pose_msg = Pose() 
    pose_msg.orientation.y = pitch
    pose_msg.orientation.w = 1.0
    rospy.sleep(1)
    return pose_msg


def calculate_angle(a, b, c):

    a = np.array(a) # first joint
    b = np.array(b) # mid joint
    c = np.array(c) # end joint
    
    # calculate the radians between 3 joints and then convert it in angles
    radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
    angle = np.abs(radians*180.0/np.pi)
    
    # the angle should be beween 0 and 180
    if angle > 180.0:
        angle = 360 - angle
    return angle



##
# In this class the robot should simulate the exploration part, so it navigates randomly in the 
# environment, while building a map and searching for people to rescue.
# When someone is detected then the state change and the robot computes the pose of the person
# to better approach it and then starts all the computations.
class Motion(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion','compute_pose'])
        
    def execute(self, userdata):
     
       global detection_client, pub_goal, landmarks, detected, found_people, start_time

       # retrieving, if there are any the mediapipe markers
       res_detection = detection_client()
       detected = res_detection.detected

       # if a person is detected
       if detected:
           print('A person has been detected')
           # turn = rotate(0.0)
           # pub_goal.publish(turn)
           found_people += 1
           start_time = rospy.Time.from_sec(time.time())
           return 'compute_pose'

       # if there is nobody
       else:
           # here wuold go the exploration part, it is simplify as
           ### DOES NOT WORK BUT THE IDEA IS THAT ONE
           
           # print(actual_yaw)
           # turn = rotate(0.5)
           # pub_goal.publish(turn)
           time.sleep(1)
           
           return 'motion'

##
# In this class there is the evaluation of the pose of the person detected in 
# order to better approach him/her.
class ComputePose(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['go_to'])
        
    def execute(self, userdata):

        global pose_client, pose

        time.sleep(3)
        res_pose = pose_client()
        pose = res_pose.pose_

        # stand up
        if pose == 1:
            print('The person is standing up')
        # sit down
        elif pose == 2:
            print('The person is sitting down')
        # lay down
        elif pose == 3:
            print('The person is laying down')
        else:
            print('No pose detected')
   
        return 'go_to'


##
# In this class the robot approaches the person, knowing in advance its pose, to better compute the
# position of the robot where all the computations are done.
class GoTo(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['adjust'])
        
    def execute(self, userdata):

        global xy_nose_client, pose, distance_standup, distance_sitdown, distance_laydown, actual_position, actual_yaw, goal_position_x, goal_position_y, xy_global_pos, ext_point_xy, actual_pos, pub_goal
       

        ##### GUARDA X E Y RISPETTO A ODOM CHE CREDO SIANO TUTTI DVERSI #######

        time.sleep(5)

        # implement move base
        # nose coordinates with respect to the odom frame
        res_nose_xy = xy_nose_client()
        y_nose_pos = res_nose_xy.x_nose
        x_nose_pos = - res_nose_xy.y_nose

        # computing the person position in global coordinates
        x_global_pos = x_nose_pos + actual_position.x
        y_global_pos = y_nose_pos + actual_position.y
        xy_global_pos.append(x_global_pos)
        xy_global_pos.append(y_global_pos)

        ext_point_x = actual_position.x
        ext_point_y = actual_position.y + 1.0
        ext_point_xy.append(ext_point_x)
        ext_point_xy.append(ext_point_y)

        actual_pos.append(actual_position.x)
        actual_pos.append(actual_position.y)

        print(ext_point_xy)
        print(actual_pos) 
        print(xy_global_pos)

        misalignment = calculate_angle(ext_point_xy, actual_pos, xy_global_pos)
        print(misalignment)


        


        # if the person is standing up
        if pose == 1:
            goal_position_x = x_global_pos
            goal_position_y = y_global_pos - distance_standup
            print(goal_position_x)
            print(goal_position_y)

        #     while ((actual_position.x - goal_position_x)*(actual_position.x - goal_position_x) + (actual_position.y - goal_position_y)*(actual_position.y - goal_position_y)) > distance_standup:
        #         time.sleep(0.1)    


        # if the person is sitting down
        elif pose == 2:
            goal_position_x = x_global_pos
            goal_position_y = y_global_pos - distance_sitdown
            print(goal_position_x)
            print(goal_position_y)
        #     while ((actual_position.x - goal_position_x)*(actual_position.x - goal_position_x) + (actual_position.y - goal_position_y)*(actual_position.y - goal_position_y)) > distance_sitdown:
        #         time.sleep(0.1) 

        # if the person is laying down
        elif pose == 3:
            goal_position_x = x_global_pos
            goal_position_y = y_global_pos - distance_laydown
            print(goal_position_x)
            print(goal_position_y)
        #     while ((actual_position.x - goal_position_x)*(actual_position.x - goal_position_x) + (actual_position.y - goal_position_y)*(actual_position.y - goal_position_y)) > distance_laydown:
        #         time.sleep(0.1) 
        

        else:
           print('NO MOTION') 
        time.sleep(5)
    
        return 'adjust'

##
# In this class the robot adjust its own position to have an optimal view of the person in order to
# starts all the analysis to be done. In the adjustment the robot should also assume a specific pose
# on the basis of the pose of the person and then stop every kind of motion. 
class Adjust(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['evaluation'])
        
    def execute(self, userdata):
    
        global look_up, look_down1, look_down2, default, pub_body

        ## THE ROBOT HERE SHOULD MOVE IN ORDER TO ADJUST ITSELF AROUND THE PERSON

        if pose == 1:
            robot_pose = pose_body(look_up)
            pub_body.publish(robot_pose)   
        elif pose == 2:
            robot_pose = pose_body(look_down1)
            pub_body.publish(robot_pose)     
        elif pose == 3:
            robot_pose = pose_body(look_down2)
            pub_body.publish(robot_pose)
        else:
            robot_pose = pose_body(default)
            pub_body.publish(robot_pose)
        # probabilmente bisongna mettere uno sleep aspettando che il robot si sitemi
        print('Adjusting the pose')
        # time.sleep(5)
                 
        return 'evaluation'



class Evaluation(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['waiting'])
        
    def execute(self, userdata):
    
        global audio_client

        print('Starting the evaluation part')
        audio_client('start')                
        return 'waiting'


class Waiting(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['analysis', 'waiting'])
        
    def execute(self, userdata):
    
        global goon_client, finish

        res_ok = goon_client()
        finish = res_ok.goon

        if finish == True:
            return 'analysis'
        else:
            return 'waiting'


##
# In this class the robot verify both the consciousness of the people and the trauma computation. These
# two kinds of analisy are done simultaneosly in order to reduce the rescue time, which is crucial for
# these kind of applications. Finally the robot should   
class Analysis(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['everything_checked'])
        
    def execute(self, userdata):

        global audio_client, trauma_client, motion_client, blink_client, docu_client, pose, stop, start_time, default

        # trauma
        res_trauma = trauma_client()

        ### CONTROLLARE PERCHE MI PUZZA
        q1 = res_trauma.question1
        q2 = res_trauma.question2
        q3a = res_trauma.question3a
        q3b = res_trauma.question3b
        

        # verifica se prende le domande
        docu_client('starting')

        # motion client
        res_motion = motion_client()
        motion = res_motion.mot
        print('Motion: {}'.format(motion))

        # blink client
        res_blink = blink_client()
        blink = res_blink.blinking
        print('Eye-blinking: {}'.format(blink))

        # talk
        if q1 == 1 or q1 == 2 or q2 == 1 or q2 == 2 or q3a == 1 or q3a == 2:
            print('Response: True')
        else:
            print('Response: False')

        # first question
        if q1 == 1:
            print('Question1: Clear answer')
        elif q1 == 2:
            print('Question1: Confused answer')
        elif q1 == 3:
            print('Question1: No answer')

        # second question
        if q2 == 1:
            print('Question2: Clear answer')
        elif q2 == 2:
            print('Question2: Confused answer')
        elif q2 == 3:
            print('Question2: No answer')

        # third question
        if q3a == 1:
            print('Question3: Clear answer')
        elif q3a == 2:
            print('Question3: Confused answer')
        elif q3a == 3:
            print('Question3: No answer')

        # trauma question
        if q3b == True:
            print('Trauma: True')
        elif q3b == False:
            print('Trauma: False')

        # stop audio
        audio_client('stop')
        print('Stop audio part')

        docu_client('stopping')

        stop_time = rospy.Time.from_sec(time.time())
        time_needed = stop_time - start_time
        required_time = time_needed.to_sec()
        print('Rescue time: {}'.format(time_needed))
  

        robot_pose = pose_body(default)
        pub_body.publish(robot_pose)

        return 'everything_checked' 
        

def main():

    global people_srv, audio_client, trauma_client, motion_client, pose_client, detection_client, blink_client, docu_client, goon_client, xy_nose_client, pub_goal, pub_body
    rospy.init_node('state_machine')
    sm = smach.StateMachine(outcomes=['everything_checked'])

    # people service
    people_srv = rospy.Service('foundpeople', Found, people_found)
    # detection client
    rospy.wait_for_service('detection')
    detection_client = rospy.ServiceProxy('detection', Detection)
    rospy.wait_for_service('xy_coord')
    print('Wait for xy_nose service')
    xy_nose_client = rospy.ServiceProxy('xy_coord', XYCoord)
    # audio client
    rospy.wait_for_service('trauma_audio')
    audio_client = rospy.ServiceProxy('trauma_audio', Audio)
    # trauma client
    trauma_client = rospy.ServiceProxy('trauma_questions', Trauma)
    # motion client
    rospy.wait_for_service('movement')
    motion_client = rospy.ServiceProxy('movement', Movement)
    # pose client
    rospy.wait_for_service('human_pose')
    pose_client = rospy.ServiceProxy('human_pose', HumanPose)
    # blink client
    rospy.wait_for_service('blink')
    blink_client = rospy.ServiceProxy('blink', Blink)
    # document client
    rospy.wait_for_service('docum')
    docu_client = rospy.ServiceProxy('docum', Docu)
    # go_on client
    goon_client = rospy.ServiceProxy('go', GoOn)
    # cmd_vel publisher
    pub_goal = rospy.Publisher('/goaltospot', TrajectoryActionGoal, queue_size = 1) #????
    # body_pose publisher
    pub_body = rospy.Publisher('/spot/body_pose', Pose, queue_size = 1)

    # CLIENT FOR MOVEBASE


    with sm:
        smach.StateMachine.add('Motion', Motion(), 
                               transitions={'compute_pose':'ComputePose',
                                            'motion':'Motion'})
        smach.StateMachine.add('ComputePose', ComputePose(), 
                               transitions={'go_to':'GoTo'})
        smach.StateMachine.add('GoTo', GoTo(), 
                               transitions={'adjust':'Adjust'})
        smach.StateMachine.add('Adjust', Adjust(), 
                               transitions={'evaluation':'Evaluation'})
        smach.StateMachine.add('Evaluation', Evaluation(), 
                               transitions={'waiting':'Waiting'})
        smach.StateMachine.add('Waiting', Waiting(), 
                               transitions={'analysis':'Analysis',
                                             'waiting':'Waiting'})
        smach.StateMachine.add('Analysis', Analysis(), 
                               transitions={'everything_checked':'everything_checked'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    main()
