#!/usr/bin/env python3

from controller import Supervisor, AnsiCodes, Node, Emitter, Receiver

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

# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

children = supervisor.getRoot().getField('children')
children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 0.1 size 1 }}')
children.importMFNodeFromString(-1, f'DEF BLUE_PLAYER1 GankenKun_simple {{translation -0.3 0 0.450 rotation 0 0 1 0 name "blue player 1" jerseyTexture "textures/GankenKun_blue1.png" jerseyColor 0, 0, 1 channel 1 controller "GankenKun_soccer"}}')
children.importMFNodeFromString(-1, f'DEF BLUE_PLAYER2 GankenKun_simple {{translation -2 -1 0.450 rotation 0 0 1 0 name "blue player 2" jerseyTexture "textures/GankenKun_blue2.png" jerseyColor 0, 0, 1 channel 2 controller "GankenKun_soccer"}}')
children.importMFNodeFromString(-1, f'DEF BLUE_PLAYER3 GankenKun_simple {{translation -2 1 0.450 rotation 0 0 1 0 name "blue player 3" jerseyTexture "textures/GankenKun_blue3.png" jerseyColor 0, 0, 1 channel 3 controller "GankenKun_soccer"}}')
children.importMFNodeFromString(-1, f'DEF RED_PLAYER1 GankenKun_simple {{translation 1 0 0.450 rotation 0 0 1 3.14 name "red player 1" jerseyTexture "textures/GankenKun_red1.png" jerseyColor 1, 0, 0 channel 4 controller "GankenKun_soccer"}}')
children.importMFNodeFromString(-1, f'DEF RED_PLAYER2 GankenKun_simple {{translation 2 -1 0.450 rotation 0 0 1 3.14 name "red player 2" jerseyTexture "textures/GankenKun_red2.png" jerseyColor 1, 0, 0 channel 5 controller "GankenKun_soccer"}}')
children.importMFNodeFromString(-1, f'DEF RED_PLAYER3 GankenKun_simple {{translation 2 1 0.450 rotation 0 0 1 3.14 name "red player 3" jerseyTexture "textures/GankenKun_red3.png" jerseyColor 1, 0, 0 channel 6 controller "GankenKun_soccer"}}')

emitter = supervisor.getDevice('emitter')

num = 0
count = 0
while supervisor.step(time_step) != -1:
    count += 1
    if count > 50:
        if num  < 10:
            message = "walk,0.1,0.0,0.0".encode('utf-8')
            print("forward")
        else:
            message = "walk,0.0,0.1,0.0".encode('utf-8')
            print("side")
        num += 1;
        if num > 20:
            num = 0
        emitter.send(message)
        count = 0

#blue_player1 = supervisor.getFromDef('BLUE_PLAYER1')
#ball = supervisor.getFromDef('BALL')
#player_translation = supervisor.getFromDef('PLAYER').getField('translation')
#player_rotation = supervisor.getFromDef('PLAYER').getField('rotation')
#player_controller = supervisor.getFromDef('PLAYER').getField('controller')
#ball_translation = supervisor.getFromDef('BALL').getField('translation')
#ball_rotation = supervisor.getFromDef('BALL').getField('rotation')
#
#try:
#    for x in np.arange(-0.35, -0.1, 0.01):
#        for y in np.arange(0, 0.25, 0.01):
#            count = 0
#            player.remove()
#            children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation {x} {y} 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "./kick_motion0.csv"}}')
#            player = supervisor.getFromDef('PLAYER')
#            ball.resetPhysics()
#            ball_translation.setSFVec3f([0, 0, 0.1])
#            ball_rotation.setSFRotation([0, 0, 1, 0])
#            while supervisor.step(time_step) != -1:
#                count += 1
#                if count > 800:
#                    break
#                if count > 800 - 1:
#                    pos = ball_translation.getSFVec3f()
#                    print(str(x)+", "+str(y)+", "+str(pos[0])+", "+str(pos[1]))
#                    with open('result.csv', 'a', newline='') as f:
#                        writer = csv.writer(f)
#                        writer.writerow([x, y, pos[0], pos[1]])
#except Exception:
#    error(f"Unexpected exception in main referee loop: {traceback.format_exc()}", fatal=True)
