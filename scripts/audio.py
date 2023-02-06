#!/usr/bin/env python3

import pygame

pygame.init()
pygame.mixer.init()

pygame.mixer.music.load("q2.mp3")
pygame.mixer.music.play()

while pygame.mixer.music.get_busy():
    pygame.time.Clock().tick(10)

pygame.quit()
