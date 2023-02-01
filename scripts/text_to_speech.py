#!/usr/bin/env python3

from gtts import gTTS

q1 = """ What is your name? """
q2 = """ Do you know where you are? """
q3 = """ Does anything hurt? """

tts = gTTS(text = q3, lang = 'en', tld = 'us', slow = True)
tts.save('q3.mp3')

print('File saved!')
