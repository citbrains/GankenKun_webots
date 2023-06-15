# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#from gamestate import GameState
from importlib import import_module
from re import S
from turtle import left
from field import Field
#from forceful_contact_matrix import ForcefulContactMatrix

from controller import Supervisor, AnsiCodes, Node

import copy
import json
import math
import numpy as np
import os
import random
import socket
import subprocess
import sys
import time
import traceback
# import transforms3d
import random
import numpy as np
import csv

from scipy.spatial import ConvexHull

from types import SimpleNamespace
import datetime

def append_solid(solid, solids):  # we list only the hands and feet
    if solid.getField('name'):
        solids.append(solid)
    children = solid.getProtoField('children') if solid.isProto() else solid.getField('children')
    for i in range(children.getCount()):
        child = children.getMFNode(i)
        if child.getType() in [Node.ROBOT, Node.SOLID, Node.GROUP, Node.TRANSFORM, Node.ACCELEROMETER, Node.CAMERA, Node.GYRO, Node.TOUCH_SENSOR]:
            append_solid(child, solids)
            continue
        if child.getType() in [Node.HINGE_JOINT, Node.HINGE_2_JOINT, Node.SLIDER_JOINT, Node.BALL_JOINT]:
        # if child.getType() in [Node.HINGE_JOINT]:
            endPoint = child.getProtoField('endPoint') if child.isProto() else child.getField('endPoint')
            solid = endPoint.getSFNode()
            if solid.getType() == Node.NO_NODE or solid.getType() == Node.SOLID_REFERENCE:
                continue
            append_solid(solid, solids)  # active tag is reset after a joint

def frange(start, end, step):
    if step == 0:
        raise ValueError('step must not be zero')

    start = float(start)
    end = float(end)
    step = float(step)
    if abs(step) >= abs(start - end):
        return [start]

    exp = len(str(step).split('.')[1])  # ステップ値から整数化に使用する値を得る
    start = int(start * 10 ** exp)
    end = int(end * 10 ** exp)
    step = int(step * 10 ** exp)

    result = [round(val * 10 ** -exp, exp) for val in range(start, end, step)]
    return result

def getXY(r, degree):
    # 度をラジアンに変換
    rad = math.radians(degree)
    x = r * math.cos(rad)
    y = r * math.sin(rad)
    #print(x, y)
    lis = [x,y]
    return lis

# start the webots supervisor
supervisor = Supervisor()
time_step = 8 
#time_step = int(supervisor.getBasicTimeStep())

field = Field("kid")
children = supervisor.getRoot().getField('children')
# children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 3 0 0.08 size 1 }}')
children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.81 0.018 0.446 rotation 0 1 0 0 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
# children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
player = supervisor.getFromDef('ENEMY1')
solids = []
read_val = [0]*13
memo_ball = [3,0,0.08]
append_solid(player, solids)

ball = supervisor.getFromDef('BALL')
player_translation = supervisor.getFromDef('ENEMY1').getField('translation')
player_rotation = supervisor.getFromDef('ENEMY1').getField('rotation')
player_controller = supervisor.getFromDef('ENEMY1').getField('controller')
ball_translation = supervisor.getFromDef('BALL').getField('translation')
ball_rotation = supervisor.getFromDef('BALL').getField('rotation')
ram = 0
x_list=[]
y_list=[]

for x in frange(2.81,2.85,0.01):
    x_list.append(x)
for y in frange(-0.050,0.200,0.002):
    y_list.append(y)

x30 = [2.868, 2.877, 2.851]
y30 = [0.060, 0.055, 0.070]

#x : 0.005*13 y : -0.00866*13
x_30 = [2.81366, 2.79634, 2.78768]
y_30 = [0.028660, 0.018660, 0.013660]

#memo x : 2.85 ->2.85259(+0.00259) -> 2.88367(0.03367) y : 0.05 -> 0.0596587(+0.0096587) -> 0.175565(0.125565)
x15=[2.85966, 2.86932, 2.84034]
y15=[0.0474099, 0.0448198, 0.0525901]

# x:2.79482 -> 2.8 -> 2.80259(+0.00259) -> 2.82849(+0.02849)  y:0.0536174 -> 0.0343 -> 0.0246413 -> -0.0719458
x_15 = [2.80448, 2.81414, 2.8238]
y_15 = [0.0562075, 0.0587976, 0.0613877]

try:
    while(1):
        aaa = 0
        print(aaa)
        print("cross")
        for m in range(0,3):
            for n in range(0,100):
                aaa +=1
                count = 0
                player.remove()

                children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation {x15[m]+0.0003367*(n+1)} {y15[m]+0.00125565*(n+1)} 0.446 rotation 0 0 1 -0.262  controller "play_motion" controllerArgs "./kick_motion_cross.csv"}}')
                #print("x : ",x_list[x], "y : ",y_list[y])
                player = supervisor.getFromDef('ENEMY1')
                ball.resetPhysics()
                ball_translation.setSFVec3f([3,0,0.08])
                ball_rotation.setSFRotation([0, 0, 1, 0])
                fg = False

                while supervisor.step(time_step) != -1:
                    count += 1

                    player = supervisor.getFromDef('ENEMY1')
                    solids = []
                    read_val = [0]*13
                    append_solid(player, solids)

                    memo_ball= ball.getPosition()
                    # print("memo ball : ",memo_ball)
                    #if memo_ball[0] <= 3:

                    for solid in solids:
                        if str(solid.getField('name').getSFString()) == "camera_sensor":
                            read_val[0] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left upper [arm]":
                            read_val[1] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left lower [arm]":
                            read_val[3] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left [hand]":
                            read_val[5] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right upper [arm]":
                            read_val[2] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right lower [arm]":
                            read_val[4] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right [hand]":
                            read_val[6] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
                            read_val[7] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
                            read_val[9] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
                            read_val[11] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
                            read_val[8] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
                            read_val[10] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
                            read_val[12] = solid.getPosition()

                    if count < 160:
                        with open('data/P15_3d_pose_cross_2.txt', 'a') as f:
                            for d in read_val:
                                f.write("%s\n" % d)
                    elif fg == False:
                        print("kick end")
                        fg = True
                        cnt_memo = count

                    #with open('3d_pose_X.txt', 'a') as f:
                    #    for d in read_val:
                    #        f.write("%s\n" % d)

                    #print(count)
                    if count > 210 or memo_ball[0] >= 4.4:
                        #print(memo_ball)
                        memo = str(memo_ball)
                        cnt = str(cnt_memo)
                        with open('data/Ball_15_cross_2.txt', 'a') as f:
                            f.write(memo)
                            f.write("\n")
                            #f.write(cnt)
                            #f.write("\n")
                        break

        print(aaa)
        print("cross_2")    

        for m in range(0,3):
            for n in range(0,100):
                aaa +=1
                count = 0
                player.remove()

                children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation {x30[m]+0.0007*(n+1)} {y30[m]+0.0001204*(n+1)} 0.446 rotation 0 0 1 -0.524  controller "play_motion" controllerArgs "./kick_motion_cross.csv"}}')
                #print("x : ",x_list[x], "y : ",y_list[y])
                player = supervisor.getFromDef('ENEMY1')
                ball.resetPhysics()
                ball_translation.setSFVec3f([3,0,0.08])
                ball_rotation.setSFRotation([0, 0, 1, 0])
                fg = False

                while supervisor.step(time_step) != -1:
                    count += 1

                    player = supervisor.getFromDef('ENEMY1')
                    solids = []
                    read_val = [0]*13
                    append_solid(player, solids)

                    memo_ball= ball.getPosition()
                    # print("memo ball : ",memo_ball)
                    #if memo_ball[0] <= 3:
                    
                    for solid in solids:
                        if str(solid.getField('name').getSFString()) == "camera_sensor":
                            read_val[0] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left upper [arm]":
                            read_val[1] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left lower [arm]":
                            read_val[3] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left [hand]":
                            read_val[5] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right upper [arm]":
                            read_val[2] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right lower [arm]":
                            read_val[4] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right [hand]":
                            read_val[6] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
                            read_val[7] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
                            read_val[9] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
                            read_val[11] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
                            read_val[8] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
                            read_val[10] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
                            read_val[12] = solid.getPosition()

                    if count < 160:
                        with open('data/P30_3d_pose_cross_2.txt', 'a') as f:
                            for d in read_val:
                                f.write("%s\n" % d)
                    elif fg == False:
                        fg = True
                        cnt_memo = count


                    #with open('3d_pose.txt', 'a') as f:
                    #    for d in read_val:
                    #        f.write("%s\n" % d)

                    #print(count)
                    if count > 210 or memo_ball[0] >= 4.4:
                        #print(memo_ball)
                        memo = str(memo_ball)
                        cnt = str(cnt_memo)
                        with open('data/Ball_30_cross_2.txt', 'a') as f:
                            f.write(memo)
                            f.write("\n")
                            #f.write(cnt)
                            #f.write("\n")
                        break


        for x in range(0,4):
            for y in range(0,125):
                aaa +=1
                count = 0
                player.remove()

                children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation {x_list[x]} {y_list[y]} 0.446 rotation 0 1 0 0 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
                #print("x : ",x_list[x], "y : ",y_list[y])
                player = supervisor.getFromDef('ENEMY1')
                ball.resetPhysics()
                ball_translation.setSFVec3f([3,0,0.08])
                ball_rotation.setSFRotation([0, 0, 1, 0])
                fg = False

                while supervisor.step(time_step) != -1:
                    count += 1

                    player = supervisor.getFromDef('ENEMY1')
                    solids = []
                    read_val = [0]*13
                    append_solid(player, solids)

                    memo_ball= ball.getPosition()
                    # print("memo ball : ",memo_ball)
                    #if memo_ball[0] <= 3:
                    for solid in solids:
                        if str(solid.getField('name').getSFString()) == "camera_sensor":
                            read_val[0] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left upper [arm]":
                            read_val[1] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left lower [arm]":
                            read_val[3] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left [hand]":
                            read_val[5] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right upper [arm]":
                            read_val[2] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right lower [arm]":
                            read_val[4] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right [hand]":
                            read_val[6] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
                            read_val[7] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
                            read_val[9] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
                            read_val[11] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
                            read_val[8] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
                            read_val[10] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
                            read_val[12] = solid.getPosition()

                    if count < 200:
                        with open('data/Front_3d_poseX_2.txt', 'a') as f:
                            for d in read_val:
                                f.write("%s\n" % d)
                    elif fg == False:
                        print("kick end")
                        fg = True
                        cnt_memo = count


                    #with open('3d_pose.txt', 'a') as f:
                    #    for d in read_val:
                    #        f.write("%s\n" % d)


                    #print(count)
                    if count > 250 or memo_ball[0] >= 4.4:
                        #print(memo_ball)
                        memo = str(memo_ball)
                        cnt = str(cnt_memo)
                        with open('data/Ball_frontx_2.txt', 'a') as f:
                            f.write(memo)
                            f.write("\n")
                            #f.write(cnt)
                            #f.write("\n")
                        break
        print(aaa)
        print("30")    

        for m in range(0,3):
            for n in range(0,100):
                aaa +=1
                count = 0
                player.remove()

                children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation {x30[m]+0.0007*(n+1)} {y30[m]+0.0001204*(n+1)} 0.446 rotation 0 0 1 -0.524  controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
                #print("x : ",x_list[x], "y : ",y_list[y])
                player = supervisor.getFromDef('ENEMY1')
                ball.resetPhysics()
                ball_translation.setSFVec3f([3,0,0.08])
                ball_rotation.setSFRotation([0, 0, 1, 0])
                fg = False

                while supervisor.step(time_step) != -1:
                    count += 1

                    player = supervisor.getFromDef('ENEMY1')
                    solids = []
                    read_val = [0]*13
                    append_solid(player, solids)

                    memo_ball= ball.getPosition()
                    # print("memo ball : ",memo_ball)
                    #if memo_ball[0] <= 3:
                    
                    for solid in solids:
                        if str(solid.getField('name').getSFString()) == "camera_sensor":
                            read_val[0] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left upper [arm]":
                            read_val[1] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left lower [arm]":
                            read_val[3] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left [hand]":
                            read_val[5] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right upper [arm]":
                            read_val[2] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right lower [arm]":
                            read_val[4] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right [hand]":
                            read_val[6] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
                            read_val[7] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
                            read_val[9] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
                            read_val[11] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
                            read_val[8] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
                            read_val[10] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
                            read_val[12] = solid.getPosition()

                    if count < 200:
                        with open('data/P30_3d_poseX_2.txt', 'a') as f:
                            for d in read_val:
                                f.write("%s\n" % d)
                    elif fg == False:
                        fg = True
                        cnt_memo = count


                    #with open('3d_pose.txt', 'a') as f:
                    #    for d in read_val:
                    #        f.write("%s\n" % d)

                    #print(count)
                    if count > 250 or memo_ball[0] >= 4.4:
                        #print(memo_ball)
                        memo = str(memo_ball)
                        cnt = str(cnt_memo)
                        with open('data/Ball_30x_2.txt', 'a') as f:
                            f.write(memo)
                            f.write("\n")
                            #f.write(cnt)
                            #f.write("\n")
                        break

#        print(aaa)
#        print("-30")
#
#        for m in range(0,3):
#            for n in range(0,100):
#                aaa +=1
#                count = 0
#                player.remove()
#
#                children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation {x_30[m]+0.0006*(n+1)} {y_30[m]-0.001039*(n+1)} 0.446 rotation 0 0 1 0.524  controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
#                #print("x : ",x_list[x], "y : ",y_list[y])
#                player = supervisor.getFromDef('ENEMY1')
#                ball.resetPhysics()
#                ball_translation.setSFVec3f([3,0,0.08])
#                ball_rotation.setSFRotation([0, 0, 1, 0])
#                fg = False
#
#                while supervisor.step(time_step) != -1:
#                    count += 1
#
#                    player = supervisor.getFromDef('ENEMY1')
#                    solids = []
#                    read_val = [0]*13
#                    append_solid(player, solids)
#
#                    memo_ball= ball.getPosition()
#                    # print("memo ball : ",memo_ball)
#                    #if memo_ball[0] <= 3:
#                    for solid in solids:
#                        if str(solid.getField('name').getSFString()) == "camera_sensor":
#                            read_val[0] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left upper [arm]":
#                            read_val[1] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left lower [arm]":
#                            read_val[3] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left [hand]":
#                            read_val[5] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right upper [arm]":
#                            read_val[2] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right lower [arm]":
#                            read_val[4] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right [hand]":
#                            read_val[6] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
#                            read_val[7] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
#                            read_val[9] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
#                            read_val[11] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
#                            read_val[8] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
#                            read_val[10] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
#                            read_val[12] = solid.getPosition()
#
#                    if count < 185:
#                        with open('data/P-30_3d_poseX.txt', 'a') as f:
#                            for d in read_val:
#                                f.write("%s\n" % d)
#                    elif fg == False:
#                        fg = True
#                        cnt_memo = count
#
#                    #with open('3d_pose.txt', 'a') as f:
#                    #    for d in read_val:
#                    #        f.write("%s\n" % d)
#
#                    #print(count)
#                    if count > 250 or memo_ball[0] >= 4.4:
#                        #print(memo_ball)
#                        memo = str(memo_ball)
#                        cnt = str(cnt_memo)
#                        with open('data/Ball_-30x.txt', 'a') as f:
#                            f.write(memo)
#                            f.write("\n")
#                            #f.write(cnt)
#                            #f.write("\n")
#                        break

        print(aaa)
        print("15")
        for m in range(0,3):
            for n in range(0,100):
                aaa +=1
                count = 0
                player.remove()

                children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation {x15[m]+0.0003367*(n+1)} {y15[m]+0.00125565*(n+1)} 0.446 rotation 0 0 1 -0.262  controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
                #print("x : ",x_list[x], "y : ",y_list[y])
                player = supervisor.getFromDef('ENEMY1')
                ball.resetPhysics()
                ball_translation.setSFVec3f([3,0,0.08])
                ball_rotation.setSFRotation([0, 0, 1, 0])
                fg = False

                while supervisor.step(time_step) != -1:
                    count += 1

                    player = supervisor.getFromDef('ENEMY1')
                    solids = []
                    read_val = [0]*13
                    append_solid(player, solids)

                    memo_ball= ball.getPosition()
                    # print("memo ball : ",memo_ball)
                    #if memo_ball[0] <= 3:

                    for solid in solids:
                        if str(solid.getField('name').getSFString()) == "camera_sensor":
                            read_val[0] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left upper [arm]":
                            read_val[1] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left lower [arm]":
                            read_val[3] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left [hand]":
                            read_val[5] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right upper [arm]":
                            read_val[2] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right lower [arm]":
                            read_val[4] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right [hand]":
                            read_val[6] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
                            read_val[7] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
                            read_val[9] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
                            read_val[11] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
                            read_val[8] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
                            read_val[10] = solid.getPosition()

                        if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
                            read_val[12] = solid.getPosition()

                    if count < 200:
                        with open('data/P15_3d_poseX_2.txt', 'a') as f:
                            for d in read_val:
                                f.write("%s\n" % d)
                    elif fg == False:
                        fg = True
                        cnt_memo = count


                    #with open('3d_pose.txt', 'a') as f:
                    #    for d in read_val:
                    #        f.write("%s\n" % d)

                    #print(count)
                    if count > 250 or memo_ball[0] >= 4.4:
                        #print(memo_ball)
                        memo = str(memo_ball)
                        cnt = str(cnt_memo)
                        with open('data/Ball_15x_2.txt', 'a') as f:
                            f.write(memo)
                            f.write("\n")
                            #f.write(cnt)
                            #f.write("\n")
                        break

#        print(aaa)
#        print("-15")
#        for m in range(0,3):
#            for n in range(0,100):
#                aaa +=1
#                count = 0
#                player.remove()
#
#                children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation {x_15[m]+0.0003367*(n+1)} {y_15[m]-0.001255632*(n+1)} 0.446 rotation 0 0 1 0.262  controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
#                #print("x : ",x_list[x], "y : ",y_list[y])
#                player = supervisor.getFromDef('ENEMY1')
#                ball.resetPhysics()
#                ball_translation.setSFVec3f([3,0,0.08])
#                ball_rotation.setSFRotation([0, 0, 1, 0])
#                fg = False
#
#                while supervisor.step(time_step) != -1:
#                    count += 1
#
#                    player = supervisor.getFromDef('ENEMY1')
#                    solids = []
#                    read_val = [0]*13
#                    append_solid(player, solids)
#
#                    memo_ball= ball.getPosition()
#                    # print("memo ball : ",memo_ball)
#                    #if memo_ball[0] <= 3:
#
#                    for solid in solids:
#                        if str(solid.getField('name').getSFString()) == "camera_sensor":
#                            read_val[0] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left upper [arm]":
#                            read_val[1] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left lower [arm]":
#                            read_val[3] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left [hand]":
#                            read_val[5] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right upper [arm]":
#                            read_val[2] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right lower [arm]":
#                            read_val[4] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right [hand]":
#                            read_val[6] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
#                            read_val[7] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
#                            read_val[9] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
#                            read_val[11] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
#                            read_val[8] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
#                            read_val[10] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
#                            read_val[12] = solid.getPosition()
#
#                    if count < 185:
#                        with open('data/P-15_3d_poseX.txt', 'a') as f:
#                            for d in read_val:
#                                f.write("%s\n" % d)
#                    elif fg == False:
#                        fg = True
#                        cnt_memo = count
#
#                    #with open('3d_pose_X.txt', 'a') as f:
#                    #    for d in read_val:
#                    #        f.write("%s\n" % d)
#
#                    #print(count)
#                    if count > 250 or memo_ball[0] >= 4.4:
#                        #print(memo_ball)
#                        memo = str(memo_ball)
#                        cnt = str(cnt_memo)
#                        with open('data/Ball_-15x.txt', 'a') as f:
#                            f.write(memo)
#                            f.write("\n")
#                            #f.write(cnt)
#                            #f.write("\n")
#                        break


#
#        print(aaa)
#        print("30")
#        for r in frange(0.010,0.060,0.001):
#            aaa += 1
#            count = 0
#            player.remove()
#            pos_list = getXY(r,30)
#            #print(pos_list)
##memo x + 0.030, y + 0.120
##memo2 
##x : 2.860 -> 2.865 (+0.005), y : 0.0650 -> 0.0736 (+0.0086)
##x : 2.860 -> 2.93 (+0.07) ,  y : 0.0650 -> 0.1862 (+0.1212)
##back x : 2.85134 , y : 0.07
#            children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation {2.860+pos_list[0]} {0.0650+pos_list[1]} 0.446 rotation 0 0 1 -0.524 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
#            pos_list[0] = pos_list[0] + 2.86
#            pos_list[1] = pos_list[1] + 0.065
#            #print("x : ",pos_list[0], "y : ",pos_list[1])
#            player = supervisor.getFromDef('ENEMY1')
#            ball.resetPhysics()
#            ball_translation.setSFVec3f([3,0,0.08])
#            ball_rotation.setSFRotation([0, 0, 1, 0])
#            fg = False
#
#            while supervisor.step(time_step) != -1:
#                count += 1
#
#                player = supervisor.getFromDef('ENEMY1')
#                solids = []
#                read_val = [0]*13
#                append_solid(player, solids)
#
#                memo_ball= ball.getPosition()
#                # print("memo ball : ",memo_ball)
#                if count < 72:
#                    for solid in solids:
#                        if str(solid.getField('name').getSFString()) == "camera_sensor":
#                            read_val[0] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left upper [arm]":
#                            read_val[1] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left lower [arm]":
#                            read_val[3] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left [hand]":
#                            read_val[5] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right upper [arm]":
#                            read_val[2] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right lower [arm]":
#                            read_val[4] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right [hand]":
#                            read_val[6] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
#                            read_val[7] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
#                            read_val[9] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
#                            read_val[11] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
#                            read_val[8] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
#                            read_val[10] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
#                            read_val[12] = solid.getPosition()
#
#                    with open('test_file_left.txt', 'a') as f:
#                        for d in read_val:
#                            f.write("%s\n" % d)
#                elif fg == False:
#                    fg = True
#                    cnt_memo = count
#
#                if count > 250 or memo_ball[0] >= 4.4:
#                    #print(memo_ball)
#                    memo = str(memo_ball)
#                    cnt = str(cnt_memo)
#                    with open('ball_pos_left.txt', 'a') as f:
#                        f.write(memo)
#                        f.write("\n")
#                        #f.write(cnt)
#                        #f.write("\n")
#                    break
#        
#
#        print(aaa)
#        print("-30")
#        for r in frange(0.010,0.060,0.001):
#            aaa += 1
#            count = 0
#            player.remove()
#            pos_list = getXY(r,-30)
##memo x + 0.04 y - 0.085
##x : 2.810 -> 2.815(+0.005)  -> 2.870 (+0.06)  y : 0.01500 -> 0.00634 (-0.00866) -> -0.0889 (--.1039)
#
##memo  center x:2.805 y:0.02366 
##x : 0.005*13 y : -0.00866*13
#x_30 = [2.805, 2.81366, 2.79634, 2.78768]
#y_30 = [0.02366, 0.028660, 0.018660, 0.013660]
#
#            children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation {2.81+pos_list[0]} {0.015+pos_list[1]} 0.446 rotation 0 0 1 0.524 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
#            pos_list[0] = pos_list[0] + 2.81
#            pos_list[1] = pos_list[1] + 0.015
#            #print("x : ",pos_list[0], "y : ",pos_list[1])
#            player = supervisor.getFromDef('ENEMY1')
#            ball.resetPhysics()
#            ball_translation.setSFVec3f([3,0,0.08])
#            ball_rotation.setSFRotation([0, 0, 1, 0])
#            fg = False
#
#            while supervisor.step(time_step) != -1:
#                count += 1
#
#                player = supervisor.getFromDef('ENEMY1')
#                solids = []
#                read_val = [0]*13
#                append_solid(player, solids)
#
#                memo_ball= ball.getPosition()
#                # print("memo ball : ",memo_ball)
#                if count < 72:
#                    for solid in solids:
#                        if str(solid.getField('name').getSFString()) == "camera_sensor":
#                            read_val[0] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left upper [arm]":
#                            read_val[1] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left lower [arm]":
#                            read_val[3] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left [hand]":
#                            read_val[5] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right upper [arm]":
#                            read_val[2] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right lower [arm]":
#                            read_val[4] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right [hand]":
#                            read_val[6] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
#                            read_val[7] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
#                            read_val[9] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
#                            read_val[11] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
#                            read_val[8] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
#                            read_val[10] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
#                            read_val[12] = solid.getPosition()
#
#                    with open('test_file_right.txt', 'a') as f:
#                        for d in read_val:
#                            f.write("%s\n" % d)
#                elif fg == False:
#                    fg = True
#                    cnt_memo = left
#
#                if count > 250 or memo_ball[0] >= 4.4:
#                    #print(memo_ball)
#                    memo = str(memo_ball)
#                    cnt = str(cnt_memo)
#                    with open('ball_pos_right.txt', 'a') as f:
#                        f.write(memo)
#                        f.write("\n")
#                        #f.write(cnt)
#                        #f.write("\n")
#                    break
#
#        print(aaa)
#        print("15")
#        for r in frange(0.010,0.060,0.001):
#            aaa += 1
#            count = 0
#            player.remove()
#            pos_list = getXY(r,15)
##memo x + 0.01 , y + 0.1 
##memo x : 2.85 ->2.85259(+0.00259) -> 2.88367(0.03367) y : 0.05 -> 0.0596587(+0.0096587) -> 0.175565(0.125565)
#x15=[2.85, 2.85966, 2.86932, 2.84034]
#y15=[0.05, 0.0474099, 0.0448198, 0.525901]
#            children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation {2.85+pos_list[0]} {0.05+pos_list[1]} 0.446 rotation 0 0 1 -0.262 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
#            pos_list[0] = pos_list[0] + 2.85
#            pos_list[1] = pos_list[1] + 0.0475
#            #print("x : ",pos_list[0], "y : ",pos_list[1])
#            player = supervisor.getFromDef('ENEMY1')
#            ball.resetPhysics()
#            ball_translation.setSFVec3f([3,0,0.08])
#            ball_rotation.setSFRotation([0, 0, 1, 0])
#            fg = False
#
#            while supervisor.step(time_step) != -1:
#                count += 1
#
#                player = supervisor.getFromDef('ENEMY1')
#                solids = []
#                read_val = [0]*13
#                append_solid(player, solids)
#
#                memo_ball= ball.getPosition()
#                # print("memo ball : ",memo_ball)
#                if count < 72:
#                    for solid in solids:
#                        if str(solid.getField('name').getSFString()) == "camera_sensor":
#                            read_val[0] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left upper [arm]":
#                            read_val[1] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left lower [arm]":
#                            read_val[3] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left [hand]":
#                            read_val[5] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right upper [arm]":
#                            read_val[2] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right lower [arm]":
#                            read_val[4] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right [hand]":
#                            read_val[6] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
#                            read_val[7] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
#                            read_val[9] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
#                            read_val[11] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
#                            read_val[8] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
#                            read_val[10] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
#                            read_val[12] = solid.getPosition()
#
#                    with open('test_file_left_15.txt', 'a') as f:
#                        for d in read_val:
#                            f.write("%s\n" % d)
#                elif fg == False:
#                    fg = True
#                    cnt_memo = left
#
#                if count > 250 or memo_ball[0] >= 4.4:
#                    #print(memo_ball)
#                    memo = str(memo_ball)
#                    cnt = str(cnt_memo)
#                    with open('ball_pos_left_15.txt', 'a') as f:
#                        f.write(memo)
#                        f.write("\n")
#                        #f.write(cnt)
#                        #f.write("\n")
#                    break
#        print(aaa)
#        print("-15")
#        for r in frange(0.010,0.060,0.001):
#            aaa += 1
#            count = 0
#            player.remove()
#            pos_list = getXY(r,15)
##memo x + 0.06 y -0.1057
## x:2.79482 -> 2.8 -> 2.80259(+0.00259) -> 2.82849(+0.02849)  y:0.0536174 -> 0.0343 -> 0.0246413 -> -0.0719458
#x_15 = [2.79482, 2.80448, 2.81414, 2.8238]
#y_15 = [0.0536174, 0.0562075, 0.0587976, 0.0613877]
#            children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation {2.80+pos_list[0]} {0.0343+pos_list[1]} 0.446 rotation 0 0 1 0.262 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
#            pos_list[0] = pos_list[0] + 2.80
#            pos_list[1] = pos_list[1] + 0.0343
#            #print("x : ",pos_list[0], "y : ",pos_list[1])
#            player = supervisor.getFromDef('ENEMY1')
#            ball.resetPhysics()
#            ball_translation.setSFVec3f([3,0,0.08])
#            ball_rotation.setSFRotation([0, 0, 1, 0])
#            fg = False
#
#            while supervisor.step(time_step) != -1:
#                count += 1
#
#                player = supervisor.getFromDef('ENEMY1')
#                solids = []
#                read_val = [0]*13
#                append_solid(player, solids)
#
#                memo_ball= ball.getPosition()
#                # print("memo ball : ",memo_ball)
#                if count < 72:
#                    for solid in solids:
#                        if str(solid.getField('name').getSFString()) == "camera_sensor":
#                            read_val[0] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left upper [arm]":
#                            read_val[1] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left lower [arm]":
#                            read_val[3] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left [hand]":
#                            read_val[5] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right upper [arm]":
#                            read_val[2] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right lower [arm]":
#                            read_val[4] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right [hand]":
#                            read_val[6] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
#                            read_val[7] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
#                            read_val[9] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
#                            read_val[11] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
#                            read_val[8] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
#                            read_val[10] = solid.getPosition()
#
#                        if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
#                            read_val[12] = solid.getPosition()
#
#                    with open('test_file_right_15.txt', 'a') as f:
#                        for d in read_val:
#                            f.write("%s\n" % d)
#                elif fg == False:
#                    fg = True
#                    cnt_memo = left
#
#                if count > 250 or memo_ball[0] >= 4.4:
#                    #print(memo_ball)
#                    memo = str(memo_ball)
#                    cnt = str(cnt_memo)
#                    with open('ball_pos_right_15.txt', 'a') as f:
#                        f.write(memo)
#                        f.write("\n")
#                        #f.write(cnt)
#                        #f.write("\n")
#                    break

        print(aaa)
        print("get_finish\n")
        sys.exit()


except Exception:
    error(f"Unexpected exception in main referee loop: {traceback.format_exc()}", fatal=True)
