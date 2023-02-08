#!/usr/bin/env python3

import rospy
import numpy as np
import math
# from mediapipe_stream_init import *
from mediapipe_stream import *
from spot_mediapipe.srv import Movement, MovementResponse
from spot_mediapipe.srv import Pose, PoseResponse
from spot_mediapipe.srv import Blink, BlinkResponse

motion_left_elbow = []
motion_left_shoulder = []
motion_left_hip = []
motion_left_knee = []
motion_right_elbow = []
motion_right_shoulder = []
motion_right_hip = []
motion_right_knee = []

landmarks = []

motion_le = False
motion_ls = False
motion_lh = False
motion_lk = False
motion_re = False
motion_rs = False
motion_rh = False
motion_rk = False

motion = False
blink = False

open_counter = 0
close_counter = 0
blink_counter = 0

blink_list = []

v = 0.95

def motion_handle(req):
    global motion
    res = MovementResponse()
    res.mot = motion
    return res

def pose_handle(req):

    global landmarks
    res = PoseResponse()
    res.landmarks = landmarks
    return res   


def blink_handle(req):
    global blink
    res = BlinkResponse()
    res.blinking = blink
    return res


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


def calculate_distance(p1, p2):

    distance = np.linalg.norm(p1 - p2)
    return distance


def main():

    global motion, open_counter, close_counter, blink_counter, blink_list, blink, landmarks, v, motion_le, motion_ls, motion_lh, motion_lk, motion_re, motion_rs, motion_rh, motion_rk

    rospy.init_node("motion", anonymous = True)
    medpipe = mediapipe()

    motion_srv = rospy.Service('movement', Movement, motion_handle)
    pose_srv = rospy.Service('pose', Pose, pose_handle)
    blink_srv = rospy.Service('blink', Blink, blink_handle)

    # rate = rospy.Rate(5)
    # rate = rospy.Rate(1)
    rate = rospy.Rate(10)
    while True:
        if medpipe.results is not None and medpipe.results.pose_landmarks and medpipe.results.face_landmarks:
            landmarks = medpipe.results.pose_landmarks.landmark
            f_landmarks = medpipe.results.face_landmarks.landmark

            # nose
            nose = [landmarks[0].x, landmarks[0].y, landmarks[0].z]
            nose_v = landmarks[0].visibility
            nose_ = [landmarks[medpipe.mp_holistic.PoseLandmark.NOSE.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.NOSE.value].y]


            # left arm
            left_wrist = [landmarks[15].x, landmarks[15].y, landmarks[15].z]
            left_wrist_v = landmarks[15].visibility
            left_wrist_ = [landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_WRIST.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_WRIST.value].y]

            left_elbow = [landmarks[13].x, landmarks[13].y, landmarks[13].z]
            left_elbow_v = landmarks[13].visibility
            left_elbow_ = [landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_ELBOW.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_ELBOW.value].y]

            left_shoulder = [landmarks[11].x, landmarks[11].y, landmarks[11].z]
            left_shoulder_v = landmarks[11].visibility
            left_shoulder_ = [landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_SHOULDER.value].y]


            # right arm
            right_wrist = [landmarks[16].x, landmarks[16].y, landmarks[16].z]
            right_wrist_v = landmarks[16].visibility
            right_wrist_ = [landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_WRIST.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_WRIST.value].y]

            right_elbow = [landmarks[14].x, landmarks[14].y, landmarks[14].z]
            right_elbow_v = landmarks[14].visibility
            right_elbow_ = [landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_ELBOW.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_ELBOW.value].y]

            right_shoulder = [landmarks[12].x, landmarks[12].y, landmarks[12].z]
            right_shoulder_v = landmarks[12].visibility
            right_shoulder_ = [landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_SHOULDER.value].y]


            # left leg
            left_hip = [landmarks[23].x, landmarks[23].y, landmarks[23].z]
            left_hip_v = landmarks[23].visibility
            left_hip_ = [landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_HIP.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_HIP.value].y]

            left_knee = [landmarks[25].x, landmarks[25].y, landmarks[25].z]
            left_knee_v = landmarks[25].visibility
            left_knee_ = [landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_KNEE.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_KNEE.value].y]

            left_ankle = [landmarks[27].x, landmarks[27].y, landmarks[27].z]
            left_ankle_v = landmarks[27].visibility
            left_ankle_ = [landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_ANKLE.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.LEFT_ANKLE.value].y]


            # right leg
            right_hip = [landmarks[24].x, landmarks[24].y, landmarks[24].z]
            right_hip_v = landmarks[24].visibility
            right_hip_ = [landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_HIP.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_HIP.value].y]

            right_knee = [landmarks[26].x, landmarks[26].y, landmarks[26].z]
            right_knee_v = landmarks[26].visibility
            right_knee_ = [landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_KNEE.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_KNEE.value].y]

            right_ankle = [landmarks[28].x, landmarks[28].y, landmarks[28].z]
            right_ankle_v = landmarks[28].visibility
            right_ankle_ = [landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_ANKLE.value].x, landmarks[medpipe.mp_holistic.PoseLandmark.RIGHT_ANKLE.value].y]
            

            ## CALCULATE ANGLES
            # left arm angles
            if left_shoulder_v > v and left_elbow_v > v and left_wrist_v > v:
                left_elbow_angle = calculate_angle(left_shoulder_ , left_elbow_, left_wrist_)
                motion_left_elbow.append(left_elbow_angle)

            if left_hip_v > v and left_shoulder_v > v and left_elbow_v > v:
                left_shoulder_angle = calculate_angle(left_hip_, left_shoulder_, left_elbow_)
                motion_left_shoulder.append(left_shoulder_angle)


            # left body angles
            if left_knee_v > v and left_hip_v > v and left_shoulder_v > v:
                left_hip_angle = calculate_angle(left_knee_, left_hip_, left_shoulder_)
                print('left_hip_angle : {}'.format(left_hip_angle))
                motion_left_hip.append(left_hip_angle)

            if left_ankle_v > v and left_knee_v > v and left_hip_v > v:
                left_knee_angle = calculate_angle(left_ankle_, left_knee_, left_hip_)
                motion_left_knee.append(left_knee_angle)


            # right arm angles
            if right_shoulder_v > v and right_elbow_v > v and right_wrist_v > v:
                right_elbow_angle = calculate_angle(right_shoulder_, right_elbow_, right_wrist_)
                motion_right_elbow.append(right_elbow_angle)

            if right_hip_v > v and right_shoulder_v > v and right_elbow_v > v:
                right_shoulder_angle = calculate_angle(right_hip_, right_shoulder_, right_elbow_)
                motion_right_shoulder.append(right_shoulder_angle)


            # right body angles
            if right_knee_v > v and right_hip_v > v and right_shoulder_v > v:
                right_hip_angle = calculate_angle(right_knee_, right_hip_, right_shoulder_)
                print('right_hip_angle : {}'.format(right_hip_angle))
                motion_right_hip.append(right_hip_angle)

            if right_ankle_v > v and right_knee_v > v and right_hip_v > v:
                right_knee_angle = calculate_angle(right_ankle_, right_knee_, right_hip_)
                motion_right_knee.append(right_knee_angle)

            # verify left elbow motion
            if len(motion_left_elbow) < 2:
                rate.sleep()
            else:
                le = (motion_left_elbow[-1] - motion_left_elbow[-2])
                if abs(le) > 3 and left_shoulder_v > v and left_elbow_v > v and left_wrist_v > v:
                    motion_le = True
                else:
                    motion_le = False
                    rate.sleep()

            # verify left shoulder motion
            if len(motion_left_shoulder) < 2:
                rate.sleep()
            else:
                ls = (motion_left_shoulder[-1] - motion_left_shoulder[-2])
                if abs(ls) > 3 and left_hip_v > v and left_shoulder_v > v and left_elbow_v > v:
                    motion_ls = True
                else:
                    motion_ls = False 
                    rate.sleep()

            # verify left hip motion
            if len(motion_left_hip) < 2:
                rate.sleep()
            else:
                lh = (motion_left_hip[-1] - motion_left_hip[-2])
                if abs(lh) > 3 and left_knee_v > v and left_hip_v > v and left_shoulder_v > v:
                    motion_lh = True
                else:
                    motion_lh = False 
                    rate.sleep()

            # verify left knee motion
            if len(motion_left_knee) < 2:
                rate.sleep()
            else:
                lk = (motion_left_knee[-1] - motion_left_knee[-2])
                if abs(lk) > 5 and left_ankle_v > v and left_knee_v > v and left_hip_v > v:
                    motion_lk = True
                else:
                    motion_lk = False 
                    rate.sleep()


            # verify right elbow motion
            if len(motion_right_elbow) < 2:
                rate.sleep()
            else:
                re = (motion_right_elbow[-1] - motion_right_elbow[-2])
                if abs(re) > 3 and right_shoulder_v > v and right_elbow_v > v and right_wrist_v > v:
                    motion_re = True
                else:
                    motion_re = False 
                    rate.sleep()

            # verify right shoulder motion
            if len(motion_right_shoulder) < 2:
               rate.sleep()
            else:
                rs = (motion_right_shoulder[-1] - motion_right_shoulder[-2])
                if abs(rs) > 3 and right_hip_v > v and right_shoulder_v > v and right_elbow_v > v:
                    motion_rs = True
                else:
                    motion_rs = False 
                    rate.sleep()

            # verify right hip motion
            if len(motion_right_hip) < 2:
                rate.sleep()
            else:
                rh = (motion_right_hip[-1] - motion_right_hip[-2])
                if abs(rh) > 3 and right_knee_v > v and right_hip_v > v and right_shoulder_v > v:
                    motion_rh = True
                else:
                    motion_rh = False 
                    rate.sleep()

            # verify right knee motion
            if len(motion_right_knee) < 2: 
                rate.sleep()
            else:
                rk = (motion_right_knee[-1] - motion_right_knee[-2])
                if abs(rk) > 5 and right_ankle_v > v and right_knee_v > v and right_hip_v > v:
                    motion_rk = True
                else:
                    motion_rk = False 
                    rate.sleep()

            if motion_le or motion_ls or motion_lh or motion_lk or motion_re or motion_rs or motion_rh or motion_rk:
                motion = True
                print('The person is moving')
            else:
                motion = False
                print('The person is not moving')        


            # left eye
            left_eye_up = np.array((f_landmarks[386].x, f_landmarks[386].y))
            left_eye_down = np.array((f_landmarks[373].x, f_landmarks[373].y))
            left_eye_left = np.array((f_landmarks[263].x, f_landmarks[263].y))
            left_eye_right = np.array((f_landmarks[362].x, f_landmarks[362].y))

            # right eye
            right_eye_up = np.array((f_landmarks[159].x, f_landmarks[159].y))
            right_eye_down = np.array((f_landmarks[145].x, f_landmarks[145].y))
            right_eye_left = np.array((f_landmarks[133].x, f_landmarks[133].y))
            right_eye_right = np.array((f_landmarks[33].x, f_landmarks[33].y))


            left_vert = calculate_distance(left_eye_up, left_eye_down)
            left_hor = calculate_distance(left_eye_left, left_eye_right)
            right_vert = calculate_distance(right_eye_up, right_eye_down)
            right_hor = calculate_distance(right_eye_left, right_eye_right)

            lratio = int((left_hor/left_vert)*100)
            rratio = int((right_hor/right_vert)*100)

            ratio = (lratio + rratio)/2

            if ratio < 160 and blink_counter <= 2:
                open_counter += 1
                print('The eyes are open')
                blink = False
                rate.sleep()
            elif ratio < 160 and blink_counter > 2:
                rate.sleep()
            elif ratio > 160 and open_counter <= 2:
                close_counter += 1
                print('The eyes are closed')
                blink = False
                rate.sleep()
            elif ratio > 160 and open_counter > 2:
                blink_counter += 1
                blink_list.append(blink_counter)
                if len(blink_list) < 2:
                    rate.sleep()
                elif blink_list[-2] == blink_list[-1]:
                    rate.sleep()
                elif blink_list[-2] != blink_list[-1]:
                    blink = True
                    print(blink_list[-1])
                    rate.sleep()

            nose_prova = np.array((landmarks[0].x, landmarks[0].y))
            piede_destro_prova = np.array((landmarks[28].x, landmarks[28].y))
            piede_sinistro_prova = np.array((landmarks[27].x, landmarks[27].y))

            if left_ankle_v > v:
                print('Distance between nose and left ankle: {}'.format(calculate_distance(nose_prova, piede_sinistro_prova)))
            elif right_ankle_v > v:
                print('Distance between nose and right ankle: {}'.format(calculate_distance(nose_prova, piede_destro_prova)))
            else:
                print('distance not visible')

            print('Nose coordinates: {}'.format(nose))
            print('Left ankle coordinates: {}'.format(left_ankle))
            print('Right ankle coordinates: {}'.format(right_ankle))

            
            print('Distance between ankles: {}'.format(calculate_distance(piede_destro_prova, piede_sinistro_prova)))
        


        # if there is not someone detected
        else:
            print('No signal')
            rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
