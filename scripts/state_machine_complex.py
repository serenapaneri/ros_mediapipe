#! /usr/bin/env python3

import rospy
import random
import smach
import smach_ros
import time

 
class Motion(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['detected'])
        
    def execute(self, userdata):
     
       
       return 'detected'


   
class Detected(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion'])
        
    def execute(self, userdata):
    
        return 'motion'

     
class Oracle(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion','game_finished'])
        
    def execute(self, userdata):
    

        return 'motion' 
        

def main():

    global armor_interface, oracle_client, comm_client, hint_sub
    rospy.init_node('state_machine_simple')
    sm = smach.StateMachine(outcomes=['everything_checked'])

    # oracle client
    oracle_client = rospy.ServiceProxy('winning_hypothesis', Winhypothesis)
    # command client
    comm_client = rospy.ServiceProxy('comm', Command)
    # hint subscriber
    hint_sub = rospy.Subscriber('hint', Hint, hint_callback)

    with sm:
        smach.StateMachine.add('Motion', Motion(), 
                               transitions={'enter_room':'Room', 
                                            'go_oracle':'Oracle',
                                            'motion': 'Motion'})
        smach.StateMachine.add('Room', Room(), 
                               transitions={'motion':'Motion'})
        smach.StateMachine.add('Oracle', Oracle(), 
                               transitions={'motion':'Motion', 
                                            'game_finished':'game_finished'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('state_machine_complex', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    main()


