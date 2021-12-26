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
import transforms3d
import random
import numpy as np
import csv

from scipy.spatial import ConvexHull

from types import SimpleNamespace

def append_solid(solid, solids):  # we list only the hands and feet
    if solid.getField('name'):
        solids.append(solid)
    children = solid.getProtoField('children') if solid.isProto() else solid.getField('children')
    for i in range(children.getCount()):
        child = children.getMFNode(i)
        if child.getType() in [Node.ROBOT, Node.SOLID, Node.GROUP, Node.TRANSFORM, Node.ACCELEROMETER, Node.CAMERA, Node.GYRO,
                               Node.TOUCH_SENSOR]:
            append_solid(child, solids)
            continue
        if child.getType() in [Node.HINGE_JOINT, Node.HINGE_2_JOINT, Node.SLIDER_JOINT, Node.BALL_JOINT]:
            endPoint = child.getProtoField('endPoint') if child.isProto() else child.getField('endPoint')
            solid = endPoint.getSFNode()
            if solid.getType() == Node.NO_NODE or solid.getType() == Node.SOLID_REFERENCE:
                continue
            append_solid(solid, solids)  # active tag is reset after a joint

# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

field = Field("kid")
children = supervisor.getRoot().getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 0.1 size 1 }}')
children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
player = supervisor.getFromDef('PLAYER')
solids = []
append_solid(player, solids)
print(len(solids))
for solid in solids:
    print(str(solid.getField('name').getSFString())+": "+str(solid.getPosition()))

ball = supervisor.getFromDef('BALL')
player_translation = supervisor.getFromDef('PLAYER').getField('translation')
player_rotation = supervisor.getFromDef('PLAYER').getField('rotation')
player_controller = supervisor.getFromDef('PLAYER').getField('controller')
ball_translation = supervisor.getFromDef('BALL').getField('translation')
ball_rotation = supervisor.getFromDef('BALL').getField('rotation')

try:
    for x in np.arange(-0.35, -0.1, 0.01):
        for y in np.arange(0, 0.25, 0.01):
            count = 0
            player.remove()
            children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation {x} {y} 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
            player = supervisor.getFromDef('PLAYER')
            ball.resetPhysics()
            ball_translation.setSFVec3f([0, 0, 0.1])
            ball_rotation.setSFRotation([0, 0, 1, 0])
            while supervisor.step(time_step) != -1:
                count += 1
                if count > 800:
                    break
                if count > 800 - 1:
                    pos = ball_translation.getSFVec3f()
                    print(str(x)+", "+str(y)+", "+str(pos[0])+", "+str(pos[1]))
                    with open('result.csv', 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([x, y, pos[0], pos[1]])
except Exception:
    error(f"Unexpected exception in main referee loop: {traceback.format_exc()}", fatal=True)
