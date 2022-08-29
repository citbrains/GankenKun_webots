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

def calculation_of_waypoints(test):
    list = [0,0,0]
    x = (test[0] +test[3]) / 2
    y = (test[1] +test[4]) / 2
    z = (test[2] +test[5]) / 2
    list[0] = x
    list[1] = y
    list[2] = z
    return list

# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

field = Field("kid")
children = supervisor.getRoot().getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 0.1 size 1 }}')
children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.86 0.0883 0.445 rotation 0.979 -0.186 0.0884 0.0217 controller "play_motion" controllerArgs "./kick_motion1.csv"}}')
# children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
player = supervisor.getFromDef('ENEMY1')
solids = []
read_val = [0]*13
append_solid(player, solids)

ball = supervisor.getFromDef('BALL')
player_translation = supervisor.getFromDef('ENEMY1').getField('translation')
player_rotation = supervisor.getFromDef('ENEMY1').getField('rotation')
player_controller = supervisor.getFromDef('ENEMY1').getField('controller')
ball_translation = supervisor.getFromDef('BALL').getField('translation')
ball_rotation = supervisor.getFromDef('BALL').getField('rotation')
ram = 0

try:
    while(1):
        count = 0
        player.remove()

        # children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.81 0.024 0.4457 rotation 0.0039 -0.012 0.9999 0.2618 controller "play_motion" controllerArgs "./kick_motion1.csv"}}')
        #ram = random.randrange(5)
        if ram == 0:
            print("front ")
            children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.86 0.0627 0.444 rotation 0.966 -0.168 0.195 0.0461 controller "play_motion" controllerArgs "./kick_motion1.csv"}}')
            # children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.83 0.0854 0.445 rotation -0.512 0.0707 -0.856 0.00899 controller "play_motion" controllerArgs "./kick_motion1.csv"}}')
            # children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.83 0.0753 0.445 rotation -0.831 0.311 -0.462 0.0122 controller "play_motion" controllerArgs "./kick_motion1.csv"}}')
            # children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.83 0.0555 0.445 rotation 0.911 -0.339 0.233 -0.0194 controller "play_motion" controllerArgs "./kick_motion1.csv"}}')
            # children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.83 0.0648 0.446 rotation -0.957 0.228 -0.181 -0.0133 controller "play_motion" controllerArgs "./kick_motion1.csv"}}')
            # children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.83 0.0642 0.446 rotation 0.0039 -0.012 0.9999 0.2618 controller "play_motion" controllerArgs "./kick_motion1.csv"}}')
            # ram += 1
        elif ram == 1:
        #     print("left near")
        #     children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.81 0.024 0.4457 rotation 0.0039 -0.012 0.9999 0.2618 controller "play_motion" controllerArgs "./kick_motion1.csv"}}')
        #     ram += 1
        # elif ram == 2:
            print("left")
            children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.82 -0.0281 0.445 rotation 0.0039 -0.012 0.9999 0.2618 controller "play_motion" controllerArgs "./kick_motion1.csv"}}')
            ram += 1
        elif ram == 2:
            print("right")
            children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.86 0.15 0.445 rotation 0.0386 0.0185 0.999 -0.518 controller "play_motion" controllerArgs "./kick_motion1.csv"}}')
            ram = 0

        player = supervisor.getFromDef('ENEMY1')
        ball.resetPhysics()
        ball_translation.setSFVec3f([3, 0, 0.1])
        ball_rotation.setSFRotation([0, 0, 1, 0])
        while supervisor.step(time_step) != -1:
            count += 1
            # print(count)

            player = supervisor.getFromDef('ENEMY1')
            solids = []
            read_val = [0]*14
            append_solid(player, solids)
            # print(len(solids))

            for solid in solids:
                if str(solid.getField('name').getSFString()) == "camera_sensor":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[0] = solid.getPosition()
                    #read_val[0] = [0,0,0]

                # if str(solid.getField('name').getSFString()) == "left_shoulder_link":
                #     # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                #     read_val[1] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left upper [arm]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    # left_arm = float(solid.getPosition())
                    # left_test[0] = solid.getPosition()
                    read_val[1] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left lower [arm]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    # left_arm += float(solid.getPosition())
                    # left_test[1] = solid.getPosition()
                    read_val[3] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left [hand]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[5] = solid.getPosition()

                # if str(solid.getField('name').getSFString()) == "right_shoulder_link":
                #     # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                #     read_val[2] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right upper [arm]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    # right_arm = float(solid.getPosition())
                    read_val[2] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right lower [arm]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    # right_arm += float(solid.getPosition())
                    read_val[4] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right [hand]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[6] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[7] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[9] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[11] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[8] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[10] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[12] = solid.getPosition()

            # ut = datetime.datetime.now()
            read_val[13] = ball.getPosition()
            with open('test_file.txt', 'a') as f:
                # f.write("%s" % ut)
                for d in read_val:
                    f.write("%s\n" % d)

            if count > 300:
                break
            #if count > 800 - 1:
                #pos = ball_translation.getSFVec3f()
                #print(str(x)+", "+str(y)+", "+str(pos[0])+", "+str(pos[1]))
                #with open('result.csv', 'a', newline='') as f:
                    #writer = csv.writer(f)
                    #writer.writerow([x, y, pos[0], pos[1]])
except Exception:
    error(f"Unexpected exception in main referee loop: {traceback.format_exc()}", fatal=True)
