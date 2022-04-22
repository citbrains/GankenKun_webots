#!/usr/bin/env python3

import os
import sys
import cv2
import numpy as np

deviceImagePath = os.getcwd()
number = 0

cv2.startWindowThread()
cv2.namedWindow("preview")

ball_color        = (   0,   0, 255)
robot11_color     = ( 255,   0,   0)
robot12_color     = ( 229,   0,   0)
robot13_color     = ( 204,   0,   0)
robot21_color     = ( 255, 255,   0)
robot22_color     = ( 229, 255,   0)
robot23_color     = ( 204, 255,   0)
goal1_left_color  = (   0, 255,   0)
goal1_right_color = (   0, 229,   0)
goal2_left_color  = (   0, 204,   0)
goal2_right_color = (   0, 178,   0)

while True:
    img = cv2.imread(deviceImagePath + "/image" + str(number) + "r.png")
    f = open(deviceImagePath + "/image" + str(number) + ".txt", mode = "wt")
    img_height, img_width, _ = img.shape

    ball = cv2.inRange(img, ball_color, ball_color)
    x, y, width, height = cv2.boundingRect(ball)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=ball_color, thickness=2)
    print("ball x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    if width > 0 and height > 0:
        f.write("0 "+str((x+width/2)/img_width)+" "+str((y+height/2)/img_height)+" "+str(width/img_width)+" "+str(height/img_height)+"\r\n")

    #robot11 = cv2.inRange(img, robot11_color, robot11_color)
    #x, y, width, height = cv2.boundingRect(robot11)
    #cv2.rectangle(img, (x, y), (x + width, y + height), color=robot11_color, thickness=2)
    #print("robot11 x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    #if width > 0 and height > 0:
    #    f.write("2 "+str((x+width/2)/img_width)+" "+str((y+height/2)/img_height)+" "+str(width/img_width)+" "+str(height/img_height)+"\r\n")

    robot12 = cv2.inRange(img, robot12_color, robot12_color)
    x, y, width, height = cv2.boundingRect(robot12)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=robot12_color, thickness=2)
    print("robot12 x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    if width > 0 and height > 0:
        f.write("2 "+str((x+width/2)/img_width)+" "+str((y+height/2)/img_height)+" "+str(width/img_width)+" "+str(height/img_height)+"\r\n")

    robot13 = cv2.inRange(img, robot13_color, robot13_color)
    x, y, width, height = cv2.boundingRect(robot13)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=robot13_color, thickness=2)
    print("robot13 x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    if width > 0 and height > 0:
        f.write("2 "+str((x+width/2)/img_width)+" "+str((y+height/2)/img_height)+" "+str(width/img_width)+" "+str(height/img_height)+"\r\n")

    robot21 = cv2.inRange(img, robot21_color, robot21_color)
    x, y, width, height = cv2.boundingRect(robot21)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=robot21_color, thickness=2)
    print("robot21 x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    if width > 0 and height > 0:
        f.write("2 "+str((x+width/2)/img_width)+" "+str((y+height/2)/img_height)+" "+str(width/img_width)+" "+str(height/img_height)+"\r\n")

    robot22 = cv2.inRange(img, robot22_color, robot22_color)
    x, y, width, height = cv2.boundingRect(robot22)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=robot22_color, thickness=2)
    print("robot22 x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    if width > 0 and height > 0:
        f.write("2 "+str((x+width/2)/img_width)+" "+str((y+height/2)/img_height)+" "+str(width/img_width)+" "+str(height/img_height)+"\r\n")

    robot23 = cv2.inRange(img, robot23_color, robot23_color)
    x, y, width, height = cv2.boundingRect(robot23)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=robot23_color, thickness=2)
    print("robot23 x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    if width > 0 and height > 0:
        f.write("2 "+str((x+width/2)/img_width)+" "+str((y+height/2)/img_height)+" "+str(width/img_width)+" "+str(height/img_height)+"\r\n")

    goal1_left = cv2.inRange(img, goal1_left_color, goal1_left_color)
    x, y, width, height = cv2.boundingRect(goal1_left)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=goal1_left_color, thickness=2)
    print("goal1_left x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    if width > 0 and height > 0:
        f.write("1 "+str((x+width/2)/img_width)+" "+str((y+height/2)/img_height)+" "+str(width/img_width)+" "+str(height/img_height)+"\r\n")

    goal1_right = cv2.inRange(img, goal1_right_color, goal1_right_color)
    x, y, width, height = cv2.boundingRect(goal1_right)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=goal1_right_color, thickness=2)
    print("goal1_right x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    if width > 0 and height > 0:
        f.write("1 "+str((x+width/2)/img_width)+" "+str((y+height/2)/img_height)+" "+str(width/img_width)+" "+str(height/img_height)+"\r\n")

    goal2_left = cv2.inRange(img, goal2_left_color, goal2_left_color)
    x, y, width, height = cv2.boundingRect(goal2_left)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=goal2_left_color, thickness=2)
    print("goal2_left x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    if width > 0 and height > 0:
        f.write("1 "+str((x+width/2)/img_width)+" "+str((y+height/2)/img_height)+" "+str(width/img_width)+" "+str(height/img_height)+"\r\n")

    goal2_right = cv2.inRange(img, goal2_right_color, goal2_right_color)
    x, y, width, height = cv2.boundingRect(goal2_right)
    cv2.rectangle(img, (x, y), (x + width, y + height), color=goal2_right_color, thickness=2)
    print("goal2_right x: " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
    if width > 0 and height > 0:
        f.write("1 "+str((x+width/2)/img_width)+" "+str((y+height/2)/img_height)+" "+str(width/img_width)+" "+str(height/img_height)+"\r\n")

    f.close()

    cv2.imshow("preview", img)
    cv2.waitKey(1000)

    number += 1

