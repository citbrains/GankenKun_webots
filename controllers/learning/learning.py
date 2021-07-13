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

from scipy.spatial import ConvexHull

from types import SimpleNamespace

# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())
time_count = 0

field = Field("kid")
children = supervisor.getRoot().getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 0.5 size 1 }}')
children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.439 rotation 0 0 1 0 controller "GankenKun_controller"}}')
player = supervisor.getFromDef('PLAYER')
player_translation = supervisor.getFromDef('PLAYER').getField('translation')
player_rotation = supervisor.getFromDef('PLAYER').getField('rotation')

try:
    previous_real_time = time.time()
    count = 0
    while supervisor.step(time_step) != -1:
        time_count += time_step
        count += 1
        if count > 100:
            count = 0
            x = -0.8+random.uniform(-0.5,0.5)
            y = random.uniform(-0.5,0.5)
            the = random.uniform(-3.14,3.14)
            player.resetPhysics()
            player_translation.setSFVec3f([x, y, 0.439])
            player_rotation.setSFRotation([0, 0, 1, the])
except Exception:
    error(f"Unexpected exception in main referee loop: {traceback.format_exc()}", fatal=True)
