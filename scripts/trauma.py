#!/usr/bin/env python3

import rospy
import speech_recognition as sr
import time
import subprocess
from gtts import gTTS
from trauma.srv import Audio, AudioResponse
from trauma.srv import Trauma, TraumaResponse

# audio service
audio_srv = None
# trauma service
trauma_srv = None

start = False
go_on = False

# answers to the questions
question_1 = 0
question_2 = 0
question_3a = 0
question_3b = False

trauma_list = ['ache', 'aching', 'air', 'ambulance', 'ankle', 'arm', 'back', 'breath', 'breathe', 'broke', 'broken', 'bruising', 'burn', 'burning', 'cannnot', 'chest', 'cramping', 'discomfort', 'dizzy', 'doctor', 'dull', 'elbow', 'eyes', 'feel', 'finger', 'foot', 'hand', 'hands', 'head', 'headache', 'help', 'hip', 'hospital', 'hurt', 'hurts', 'injury', 'knee', 'laceration', 'leg', 'legs', 'lot', 'medical', 'move', 'neck', 'numbness', 'pain', 'painful', 'pinching', 'pressure', 'really', 'relief', 'see', 'sharp', 'shooting', 'shoulder', 'sick', 'sore', 'soreness', 'spot', 'stabbing', 'stiffness', 'stomach', 'stomachache', 'suffering', 'tender', 'throbbing', 'tightness', 'tingling', 'torment', 'torture', 'torturing', 'wrist', 'yes']

##
# \brief Callback function of audio_srv
# \param req, AudioRequest
# \return start
#
# This callback function allows the client to start and stop the behavior
def audio(req):
    if (req.audio == 'start'):
        start = True
    elif (req.audio == 'stop'):
        start = False
    return start


##
# \brief Callback function of trauma_srv
# \param req, TraumaRequest
# \return None
#
# This callback function that send information to the client relying on the 
# answers of the user
def questions(req):
    global question_1, question_2, question_3, go_on
    res = TraumaResponse()
    res.question1 = question_1
    res.question2 = question_2
    res.question3a = question_3a
    res.question3b = question_3b
    res.ok = go_on


##
# \brief Function that convert a string into a list
# \param ans, string
# \return list_, list
#
# This function allows to convert a string into a list, separating the items
def convert(ans):
    list_ = list(ans.split(" "))
    return list_


##
# \brief Function that checks if elements of list2 exist in list1.
# \param: list1, list2
# \return: True, False
#
# This function is used to check if the elements in the list2 are present or not in the list1.
def search_list(list1, list2):

    result = any(item in list1 for item in list2)
    if result:
        return True
    else:
        return False


##
# \brief Function that returns the indixes in a list.
# \param: list1, list2
# \return: index, []
#
# This function is used to search the elements of a list2 in a list1 and return their indexes.  
def list_index(list1, list2):

    check = search_list(list1, list2)
    if check == True:
        index = [i for i,item in enumerate(list1) if item in list2]
        return index
    else:
        return []


def main():

    global audio_srv, trauma_srv, start, question_1, question_2, question_3a, question_3b, go_on
    rospy.init_node('trauma')

    audio_srv = rospy.Service('trauma_audio', Audio, audio)
    trauma_srv = rospy.Service('trauma_questions', Trauma, questions)

    q1 = """ What is your name? """
    q2 = """ Do you know where you are? """
    q3 = """ Does anything hurt? """

    if start == True:
        # get audio from the microphone
        r = sr.Recognizer()

        ## NAME    
        tts = gTTS(text = q1, lang = 'en', tld = 'us', slow = True)
        tts.save('q1.mp3')
        process1 = subprocess.Popen(["audacious", '--quit-after-play', 'q1.mp3'])
        process1.wait()
        process1.terminate()

        with sr.Microphone() as source:
    
            r.adjust_for_ambient_noise(source, duration = 0.5)

            try:
                audio1 = r.listen(source, timeout = 5)
                answer1 = r.recognize_google(audio1)
                answer1.lower()
                print(answer1)
                question_1 = 1

            except sr.UnknownValueError:
                print("Could not understand audio")
                question_1 = 2
            except sr.WaitTimeoutError:
                print("Timeout")
                question_1 = 3
            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))
            time.sleep(3)


        ## WHERE
        tts = gTTS(text = q2, lang = 'en', tld = 'us', slow = True)
        tts.save('q2.mp3')
        process2 = subprocess.Popen(["audacious", '--quit-after-play', 'q2.mp3'])
        process2.wait()
        process2.terminate()

        with sr.Microphone() as source:

            r.adjust_for_ambient_noise(source, duration = 0.5)

            try:
                audio2 = r.listen(source, timeout = 5)
                answer2 = r.recognize_google(audio2)
                answer2.lower()
                print(answer2)
                question_2 = 1

             except sr.UnknownValueError:
                print("Could not understand audio")
                question_2 = 2
            except sr.WaitTimeoutError:
                print("Timeout")
                question_2 = 3
            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))
            time.sleep(3)


        ## TRAUMA
        tts = gTTS(text = q3, lang = 'en', tld = 'us', slow = True)
        tts.save('q3.mp3')
        process3 = subprocess.Popen(["audacious", '--quit-after-play', 'q3.mp3'])
        process3.wait()
        process3.terminate()  

        with sr.Microphone() as source:

            r.adjust_for_ambient_noise(source, duration = 0.5)

            try:
                audio3 = r.listen(source, timeout = 5)
                answer3 = r.recognize_google(audio3)
                answer3.lower()
                print(answer3)
                print(convert(answer3))
                list_ = convert(answer3)

                question_3a = 1
                question_3b = search_list(list_, trauma_list)

                print(question_3b)

             except sr.UnknownValueError:
                print("Could not understand audio")
                question_3a = 2
            except sr.WaitTimeoutError:
                print("Timeout")
                question_3a = 3
            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))
            time.sleep(3)

            go_on = True

    elif start == False:
        go_on = False

    rospy.spin()

if __name__ == '__main__':
    main()


