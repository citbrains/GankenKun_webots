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
children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.783 0.015 0.4465 rotation 0.0039 -0.012 0.9999 0.2618 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
# children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
player = supervisor.getFromDef('ENEMY1')
solids = []
read_val = [0]*14
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

        children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.783 0.015 0.4465 rotation 0.0039 -0.012 0.9999 0.2618 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
        #ram = random.randrange(5)
        # if ram == 0:
        #     print("000000000000000000000000000")
        #     children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.783 0.015 0.4465 rotation 0.0039 -0.012 0.9999 0.2618 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
            # children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.783 0.015 0.4465 rotation 0.0039 -0.012 0.9999 0.2618 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
            # children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.8 0.15 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
            #children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.8 0.15 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
            # ram += 1
        # elif ram == 1:
        #     print("11111111111111111111111")
        #     children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.783 0.015 0.4465 rotation 0.0039 -0.012 0.9999 0.2618 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
        #     ram += 1
        # elif ram == 2:
        #     print("2222222222222222222222222")
        #     children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.8 0.14 0.4467 rotation 0.0006 -0.0066 -0.9999 0.2618 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
        #     ram += 1
        # elif ram == 3:
        #     print("333333333333333333333333333333333")
        #     children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.807 -0.028 0.446 rotation 0.001 -0.007 0.999 0.523 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
        #     ram += 1
        # elif ram == 4:
        #     print("44444444444444444444444444444")
        #     children.importMFNodeFromString(-1, f'DEF ENEMY1 GankenKun_Keypoints {{translation 2.8845 0.1461 0.4470 rotation -0.0045 0.0058 0.999 -0.5242 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
        #     ram = 0

        player = supervisor.getFromDef('ENEMY1')
        ball.resetPhysics()
        ball_translation.setSFVec3f([3, 0, 0.1])
        ball_rotation.setSFRotation([0, 0, 1, 0])
        while supervisor.step(time_step) != -1:
            count += 1

            player = supervisor.getFromDef('ENEMY1')
            solids = []
            read_val = [0]*15
            append_solid(player, solids)
            # print(len(solids))

            for solid in solids:
                if str(solid.getField('name').getSFString()) == "camera_sensor":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    # read_val[0] = solid.getPosition()
                    read_val[0] = [0,0,0]

                if str(solid.getField('name').getSFString()) == "left_shoulder_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[1] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left upper [arm]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    # left_arm = float(solid.getPosition())
                    # left_test[0] = solid.getPosition()
                    read_val[3] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left lower [arm]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    # left_arm += float(solid.getPosition())
                    # left_test[1] = solid.getPosition()
                    read_val[4] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left [hand]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[7] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right_shoulder_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[2] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right upper [arm]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    # right_arm = float(solid.getPosition())
                    read_val[5] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right lower [arm]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    # right_arm += float(solid.getPosition())
                    read_val[6] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right [hand]":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[8] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left_waist_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[9] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left_knee_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[11] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "left_ankle_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[13] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right_waist_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[10] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right_knee_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[12] = solid.getPosition()

                if str(solid.getField('name').getSFString()) == "right_ankle_pitch_link":
                    # print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))
                    read_val[14] = solid.getPosition()

            # ut = datetime.datetime.now()
            with open('test_file.txt', 'a') as f:
                # f.write("%s" % ut)
                for d in read_val:
                    f.write("%s\n" % d)

            if count > 400:
                break
            #if count > 800 - 1:
                #pos = ball_translation.getSFVec3f()
                #print(str(x)+", "+str(y)+", "+str(pos[0])+", "+str(pos[1]))
                #with open('result.csv', 'a', newline='') as f:
                    #writer = csv.writer(f)
                    #writer.writerow([x, y, pos[0], pos[1]])
except Exception:
    error(f"Unexpected exception in main referee loop: {traceback.format_exc()}", fatal=True)
