#!/usr/bin/env python3

from controller import Supervisor
from gymnasium.spaces import Box, Discrete, Sequence
from gymnasium.utils import EzPickle, seeding

from pettingzoo import AECEnv
from pettingzoo.utils import wrappers
from pettingzoo.utils.agent_selector import agent_selector
import numpy as np
import math
import time

def rotation_to_euler(rotation):
    x, y, z, angle = rotation
    s = math.sin(angle)
    c = math.cos(angle)
    t = 1 - c
    if (x*y*t + z*s) > 0.998:
        yaw = 2*math.atan2(x*math.sin(angle/2), math.cos(angle/2))
        pitch = math.pi/2
        roll = 0
        return yaw, pitch, roll
    if (x*y*t + z*s) < -0.998:
        yaw = -2*math.atan2(x*math.sin(angle/2), math.cos(angle/2))
        pitch = -math.pi/2
        roll = 0
        return yaw, pitch, roll
    yaw = math.atan2(y * s - x * z * t , 1 - (y**2 + z**2 ) * t)
    pitch = math.asin(x * y * t + z * s)
    roll = math.atan2(x * s - y * z * t , 1 - (x**2 + y**2) * t)
    return yaw, pitch, roll

class raw_env(AECEnv, EzPickle):
    metadata = {
        "render_modes": ["human", "rgb_array"],
        "name": "soccer_v0",
        "is_parallelizable": True,
        "has_manual_policy": True,
    }
    def __init__(self, max_cycles=900):
        EzPickle.__init__(self, max_cycles=max_cycles)
        self.max_cycle = max_cycles
        self.out_agent = []
        self.agent_name_mapping = {}
        self.agents = ["blue1", "blue2", "blue3", "red1", "red2", "red3"]
        for i in range(len(self.agents)):
            self.agent_name_mapping[self.agents[i]] = i
        obs_space = Box(low=-5, high=5, shape = ([21]), dtype=np.float16)
        self.observation_spaces = dict(
            zip(
                self.agents,
                [obs_space for _ in enumerate(self.agents)]
            )
        )
        self.action_spaces = dict(
            zip(self.agents, [Discrete(8) for _ in enumerate(self.agents)])
        )
        self.possible_agents = self.agents
        self._agent_selector = agent_selector(self.agents)
        #self.reinit()

        # start the webots supervisor
        self.supervisor = Supervisor()
        self.time_step = int(self.supervisor.getBasicTimeStep())

        children = self.supervisor.getRoot().getField('children')
        children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 0.1 size 1 }}')
        children.importMFNodeFromString(-1, f'DEF BLUE_PLAYER1 GankenKun_simple {{translation -0.3 0 0.450 rotation 0 0 1 0 name "blue player 1" jerseyTexture "textures/GankenKun_blue1.png" jerseyColor 0, 0, 1 channel 1 controller "GankenKun_soccer"}}')
        children.importMFNodeFromString(-1, f'DEF BLUE_PLAYER2 GankenKun_simple {{translation -2 -1 0.450 rotation 0 0 1 0 name "blue player 2" jerseyTexture "textures/GankenKun_blue2.png" jerseyColor 0, 0, 1 channel 2 controller "GankenKun_soccer"}}')
        children.importMFNodeFromString(-1, f'DEF BLUE_PLAYER3 GankenKun_simple {{translation -2 1 0.450 rotation 0 0 1 0 name "blue player 3" jerseyTexture "textures/GankenKun_blue3.png" jerseyColor 0, 0, 1 channel 3 controller "GankenKun_soccer"}}')
        children.importMFNodeFromString(-1, f'DEF RED_PLAYER1 GankenKun_simple {{translation 1 0 0.450 rotation 0 0 1 3.14 name "red player 1" jerseyTexture "textures/GankenKun_red1.png" jerseyColor 1, 0, 0 channel 4 controller "GankenKun_soccer"}}')
        children.importMFNodeFromString(-1, f'DEF RED_PLAYER2 GankenKun_simple {{translation 2 -1 0.450 rotation 0 0 1 3.14 name "red player 2" jerseyTexture "textures/GankenKun_red2.png" jerseyColor 1, 0, 0 channel 5 controller "GankenKun_soccer"}}')
        children.importMFNodeFromString(-1, f'DEF RED_PLAYER3 GankenKun_simple {{translation 2 1 0.450 rotation 0 0 1 3.14 name "red player 3" jerseyTexture "textures/GankenKun_red3.png" jerseyColor 1, 0, 0 channel 6 controller "GankenKun_soccer"}}')

        self.blue1_emitter = self.supervisor.getDevice('blue1_emitter')
        self.blue2_emitter = self.supervisor.getDevice('blue2_emitter')
        self.blue3_emitter = self.supervisor.getDevice('blue3_emitter')
        self.red1_emitter = self.supervisor.getDevice('red1_emitter')
        self.red2_emitter = self.supervisor.getDevice('red2_emitter')
        self.red3_emitter = self.supervisor.getDevice('red3_emitter')

        self.ball_pos = self.supervisor.getFromDef('BALL').getField('translation')
        self.blue1_pos = self.supervisor.getFromDef('BLUE_PLAYER1').getField('translation')
        self.blue1_rot = self.supervisor.getFromDef('BLUE_PLAYER1').getField('rotation')
        self.blue2_pos = self.supervisor.getFromDef('BLUE_PLAYER2').getField('translation')
        self.blue2_rot = self.supervisor.getFromDef('BLUE_PLAYER2').getField('rotation')
        self.blue3_pos = self.supervisor.getFromDef('BLUE_PLAYER3').getField('translation')
        self.blue3_rot = self.supervisor.getFromDef('BLUE_PLAYER3').getField('rotation')
        self.red1_pos = self.supervisor.getFromDef('RED_PLAYER1').getField('translation')
        self.red1_rot = self.supervisor.getFromDef('RED_PLAYER1').getField('rotation')
        self.red2_pos = self.supervisor.getFromDef('RED_PLAYER2').getField('translation')
        self.red2_rot = self.supervisor.getFromDef('RED_PLAYER2').getField('rotation')
        self.red3_pos = self.supervisor.getFromDef('RED_PLAYER3').getField('translation')
        self.red3_rot = self.supervisor.getFromDef('RED_PLAYER3').getField('rotation')

    def observation_space(self, agent):
        return self.observation_spaces[agent]

    def action_space(self, agent):
        return self.action_spaces[agent]

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)

    def observe(self, agent):
        ball_x, ball_y, _ = self.ball_pos.getSFVec3f()
        blue1_x, blue1_y, _ = self.blue1_pos.getSFVec3f()
        blue1_yaw, _, _ = rotation_to_euler(self.blue1_rot.getSFRotation())
        blue2_x, blue2_y, _ = self.blue2_pos.getSFVec3f()
        blue2_yaw, _, _ = rotation_to_euler(self.blue2_rot.getSFRotation())
        blue3_x, blue3_y, _ = self.blue3_pos.getSFVec3f()
        blue3_yaw, _, _ = rotation_to_euler(self.blue3_rot.getSFRotation())
        red1_x, red1_y, _ = self.red1_pos.getSFVec3f()
        red1_yaw, _, _ = rotation_to_euler(self.red1_rot.getSFRotation())
        red2_x, red2_y, _ = self.red2_pos.getSFVec3f()
        red2_yaw, _, _ = rotation_to_euler(self.red2_rot.getSFRotation())
        red3_x, red3_y, _ = self.red3_pos.getSFVec3f()
        red3_yaw, _, _ = rotation_to_euler(self.red3_rot.getSFRotation())
        state = [ball_x, ball_y, 0, blue1_x, blue1_y, blue1_yaw, blue2_x, blue2_y, blue2_yaw, blue3_x, blue3_y, blue3_yaw, red1_x, red1_y, red1_yaw, red2_x, red2_y, red2_yaw, red3_x, red3_y, red3_yaw]
        return state

    def step(self, action):
        if self.terminations[self.agent_selection] or self.truncations[self.agent_selection]:
            self._was_dead_step(action)
            return
        pass

if __name__ == "__main__":
    env = raw_env()
    num = 0
    while env.supervisor.step(env.time_step) != 1:
        if num > 100:
            print(env.observe("blue1"))
            num = 0
        num += 1





#num = 0
#count = 0
#while supervisor.step(time_step) != -1:
#    count += 1
#    if count > 50:
#        if num  < 10:
#            message = "walk,0.1,0.0,0.0".encode('utf-8')
#            print("forward")
#        else:
#            message = "walk,0.0,0.1,0.0".encode('utf-8')
#            print("side")
#        num += 1;
#        if num > 20:
#            num = 0
#        blue1_emitter.send(message)
#        blue2_emitter.send(message)
#        blue3_emitter.send(message)
#        red1_emitter.send(message)
#        red2_emitter.send(message)
#        red3_emitter.send(message)
#        count = 0

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
