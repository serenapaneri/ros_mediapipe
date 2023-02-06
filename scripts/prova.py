#!/usr/bin/env python3

import speech_recognition as sr
import time
import subprocess
from gtts import gTTS

# QUESTIONS
q1 = """ What is your name? """
q2 = """ Do you know where you are? """
q3 = """ Does anything hurt? """

trauma_list = ['ache', 'aching', 'air', 'ambulance', 'ankle', 'arm', 'back', 'breath', 'breathe', 'broke', 'broken', 'bruising', 'burn', 'burning', 'cannnot', 'chest', 'cramping', 'discomfort', 'dizzy', 'doctor', 'dull', 'elbow', 'eyes', 'feel', 'finger', 'foot', 'hand', 'hands', 'head', 'headache', 'help', 'hip', 'hospital', 'hurt', 'hurts', 'injury', 'knee', 'laceration', 'leg', 'legs', 'lot', 'medical', 'move', 'neck', 'numbness', 'pain', 'painful', 'pinching', 'pressure', 'really', 'relief', 'see', 'sharp', 'shooting', 'shoulder', 'sick', 'sore', 'soreness', 'spot', 'stabbing', 'stiffness', 'stomach', 'stomachache', 'suffering', 'tender', 'throbbing', 'tightness', 'tingling', 'torment', 'torture', 'torturing', 'wrist', 'yes']


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

r = sr.Recognizer()

## NAME
tts = gTTS(text = q1, lang = 'en', tld = 'us', slow = True)
tts.save('q1.mp3')
process1 = subprocess.Popen(["audacious", '--quit-after-play', 'q1.mp3'])
process1.wait()
process1.terminate()

with sr.Microphone() as source:

    r.adjust_for_ambient_noise(source)
    
    try:
        audio1 = r.listen(source, timeout = 5)
        answer1 = r.recognize_google(audio1)
        answer1.lower()
        print(answer1)

    except sr.UnknownValueError:
        print("Could not understand audio")
    except sr.WaitTimeoutError:
        print("Timeout")
    except sr.RequestError as e:
        print("Could not request results; {0}".format(e))


## WHERE
tts = gTTS(text = q2, lang = 'en', tld = 'us', slow = True)
tts.save('q2.mp3')
process2 = subprocess.Popen(["audacious", '--quit-after-play', 'q2.mp3'])
process2.wait()
process2.terminate()

with sr.Microphone() as source:

    r.adjust_for_ambient_noise(source)

    try:
        audio2 = r.listen(source, timeout = 5)
        answer2 = r.recognize_google(audio2)
        answer2.lower()
        print(answer2)

    except sr.UnknownValueError:
        print("Could not understand audio")
    except sr.WaitTimeoutError:
        print("Timeout")
    except sr.RequestError as e:
        print("Could not request results; {0}".format(e))


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
        print('-------------')
        list_ = convert(answer3)

        trauma = search_list(list_, trauma_list)
        print(trauma)

    except sr.UnknownValueError:
        print("Could not understand audio")
    except sr.WaitTimeoutError:
        print("Timeout")
    except sr.RequestError as e:
        print("Could not request results; {0}".format(e))

