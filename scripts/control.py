#!/usr/bin/env python3

"""
  This node should arrange the motion of the robot once a person is detected. Of course
  there are several ways in which the robot can approach the person.
  However the final position in which the robot should do all the investigations is near the 
  head (mantaing a certain distance), framing both the face and the body, in order to see if
  the person is moving its own body and to see if it is blinking. (Also to have the microphone
  as close as possible to the person's mouth). Instead the position of the person can be computed 
  a priori. 

  - LAY DOWN : - if the person is seen by side the robot should compute the middle point between
                 the distance from the nose to the foot. Set a safe distance between the person and 
                 the robot (using depth cameras), and then move near the head facing also the body

               - if the person is seen from the foot the robot should compute the distance between itself
                 and the foot (depth camera), translate to the left or to the right, rotates of 90° 
                 (orario o antiorario dipende come ha traslato il robot) and traslate again. In that case
                 we come back to the case aforementioned

               - if the person is seen from the head the robot should just adjust its angle in a way
                 that it can see both nose and feet landmarks.

  - SIT DOWN : this case is similar to the previous (you can differentiate from the previos taking
               into account the y position of the nose)

  - STAND UP : - if the person is seen from the front --> ok
               - if the person is seen by side the robot should rotate until it visualize (greater than
                 a treshold) the face landmarks.
               - if the person is seen from the back same as before.  
"""


