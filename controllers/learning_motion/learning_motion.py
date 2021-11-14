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
from skopt import gp_minimize

# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

field = Field("kid")
children = supervisor.getRoot().getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 0.1 size 1 }}')
children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "motion.csv"}}')
player = supervisor.getFromDef('PLAYER')
ball = supervisor.getFromDef('BALL')
player_translation = supervisor.getFromDef('PLAYER').getField('translation')
player_rotation = supervisor.getFromDef('PLAYER').getField('rotation')
player_controller = supervisor.getFromDef('PLAYER').getField('controller')
ball_translation = supervisor.getFromDef('BALL').getField('translation')
ball_rotation = supervisor.getFromDef('BALL').getField('rotation')

def func(param):
    global player, ball, children, ball_translation, ball_rotation, supervisor

    with open("../play_motion/kick_motion.csv", "r") as f:
        read_data = csv.reader(f, delimiter=",", lineterminator="\r\n")
        data = [row for row in read_data]
        data[6][0] = str(int(param[0]))
        data[7][0] = str(int(param[1]))
        data[7][3] = str(int(param[2]))
        data[7][4] = str(int(param[3]))
    with open("../play_motion/motion.csv", "w") as f:
        writer = csv.writer(f)
        writer.writerows(data)
            
    count = 0
    player.remove()
    children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.2 0.1 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "motion.csv"}}')
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
            with open('result.csv', 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([param[0], param[1], param[2], param[3], pos[0], pos[1]])
    return -pos[0]

x0 = (1,50)
x1 = (1,50)
x2 = (0,80)
x3 = (0,80)
x = (x0, x1, x2, x3)
result = gp_minimize(func, x, n_calls=100, noise=0.0, model_queue_size=1, verbose=True)

