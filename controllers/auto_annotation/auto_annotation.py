#!/usr/bin/env python3

import os
import sys
from controller import Robot, CameraRecognitionObject, Display, Supervisor
import random
import cv2
import numpy as np

deviceImagePath = os.getcwd()
#robot = Robot()
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep()*10)
camera = supervisor.getDevice('camera_sensor')
camera.enable(timestep)
camera.recognitionEnable(timestep)
camera.enableRecognitionSegmentation()
number = 0

print("hasRecognition(): " + str(camera.hasRecognition()))
print("hasRecognitionSegmentation(): " + str(camera.hasRecognitionSegmentation()))

cv2.startWindowThread()
cv2.namedWindow("preview")

ball_color   = (  0, 0  , 255, 255)
enemy1_color = (  0, 255,   0, 255)
enemy2_color = (255,   0,   0, 255)
enemy3_color = (255, 255,   0, 255)

while supervisor.step(timestep) != -1:
    supervisor.getFromDef('BALL').getField('translation').setSFVec3f([random.uniform(0.0, 4.0), random.uniform(-1.0, 1.0), 0.1])
    supervisor.getFromDef('ENEMY1').getField('rotation').setSFRotation([0, 0, 1, random.uniform(-3.14, 3.14)])
    for i in range(10):
        supervisor.step(timestep)
    camera.saveImage(deviceImagePath + '/images/image' + str(number) + '.jpg', 80)
    camera.saveRecognitionSegmentationImage(deviceImagePath + '/images/segmentation_image' + str(number) + '.jpg', 80)
    number += 1
    seg_img = camera.getRecognitionSegmentationImage()
    img = np.frombuffer(seg_img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    ball = cv2.inRange(img, ball_color, ball_color)
    x, y, width, height = cv2.boundingRect(ball)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=ball_color, thickness=2)
    print("ball x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    enemy1 = cv2.inRange(img, enemy1_color, enemy1_color)
    x, y, width, height = cv2.boundingRect(enemy1)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=enemy1_color, thickness=2)
    print("enemy1 x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    enemy2 = cv2.inRange(img, enemy2_color, enemy2_color)
    x, y, width, height = cv2.boundingRect(enemy2)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=enemy2_color, thickness=2)
    print("enemy2 x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    enemy3 = cv2.inRange(img, enemy3_color, enemy3_color)
    x, y, width, height = cv2.boundingRect(enemy3)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=enemy3_color, thickness=2)
    print("enemy3 x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))

    cv2.imshow("preview", img)
    cv2.waitKey(timestep)

