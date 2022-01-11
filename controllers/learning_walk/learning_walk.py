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
children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.450 rotation 0 0 1 0 controller "generate_walking_pattern" controllerArgs "0.05"}}')
player = supervisor.getFromDef('PLAYER')
#player_translation = supervisor.getFromDef('PLAYER').getField('translation')
#player_rotation = supervisor.getFromDef('PLAYER').getField('rotation')
#player_controller = supervisor.getFromDef('PLAYER').getField('controller')

def func(param):
    global player, children, supervisor
    count = 0
    player.remove()
    condition = f'DEF PLAYER RoboCup_GankenKun {{translation -0.2 0.1 0.450 rotation 0 0 1 0 controller "generate_walking_pattern" controllerArgs "'+str(param[0])+'"}}'
    print(condition)
    children.importMFNodeFromString(-1, condition)
    player = supervisor.getFromDef('PLAYER')
    player_translation = supervisor.getFromDef('PLAYER').getField('translation')
    while supervisor.step(time_step) != -1:
        count += 1
        if count > 1000:
            break
        if count > 1000 - 1:
            pos = player_translation.getSFVec3f()
            with open('result.csv', 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([param[0], pos[0], pos[1]])
    return -pos[0]

x0 = (0.01,0.3)
x = (x0,)
result = gp_minimize(func, x, n_calls=20, noise=0.0, model_queue_size=1, verbose=True)

