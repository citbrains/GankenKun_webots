#!/usr/bin/env python3

import numpy as np
import math
import random

from controller import Supervisor

from gymnasium.spaces import Box, Discrete, Sequence
from gymnasium.utils import EzPickle, seeding

from pettingzoo import AECEnv
from pettingzoo.utils import wrappers
from pettingzoo.utils.agent_selector import agent_selector
from pettingzoo.utils.conversions import parallel_wrapper_fn

from player import Player

__all__ = ["env", "parallel_env", "raw_env"]

def env(**kwargs):
    env = raw_env(**kwargs)
    env = wrappers.AssertOutOfBoundsWrapper(env)
    env = wrappers.OrderEnforcingWrapper(env)
    return env

parallel_env = parallel_wrapper_fn(env)

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
        "name": "soccer_v0",
        "is_parallelizable": True,
    }
    def __init__(self, max_cycles=1000):
        EzPickle.__init__(self, max_cycles=max_cycles)
        self.frames = 0
        self._seed()
        self.max_cycles = max_cycles
        self.out_agent = []
        self.agent_name_mapping = {}
        self.agent_dict = {}
        self.agent_list = []
        self.agents = ["blue1", "blue2", "blue3", "red1", "red2", "red3"]
        for i in range(len(self.agents)):
            self.agent_name_mapping[self.agents[i]] = i
            self.agent_list.append(Player(agent_name = self.agents[i]))
        obs_space = Box(low=-5, high=5, shape = ([21]), dtype=np.float16)
        self.observation_spaces = dict(zip(self.agents, [obs_space for _ in enumerate(self.agents)]))
        self.action_spaces = dict(zip(self.agents, [Discrete(6) for _ in enumerate(self.agents)]))
        self.actions = ["walk,1,0,0", "walk,-1,0,0", "walk,0,1,0", "walk,0,-1,0", "walk,0,0,1", "walk,0,0,-1", "motion,left_kick", "motion,right_kick"]
        self.state_space = Box(low=-5, high=5, shape = ([21]), dtype=np.float16)

        self.possible_agents = self.agents
        self._agent_selector = agent_selector(self.agents)

        self.supervisor = Supervisor()
        self.time_step = int(self.supervisor.getBasicTimeStep())

        self.reinit()

    def observation_space(self, agent):
        return self.observation_spaces[agent]

    def action_space(self, agent):
        return self.action_spaces[agent]

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)

    def observe(self, agent):
        return self.state()
    
    def state(self):
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
        self._cumulative_rewards[self.agent_selection] = 0
        agent = self.agent_list[self.agent_name_mapping[self.agent_selection]]
        agent.score = 0
        
        message = self.actions[action].encode('utf-8')
        if self.agent_selection == "blue1":
            self.blue1_emitter.send(message)
        elif self.agent_selection == "blue2":
            self.blue2_emitter.send(message)
        elif self.agent_selection == "blue3":
            self.blue3_emitter.send(message)
        elif self.agent_selection == "red1":
            self.red1_emitter.send(message)
        elif self.agent_selection == "red2":
            self.red2_emitter.send(message)
        elif self.agent_selection == "red3":
            self.red3_emitter.send(message)
        
        if self._agent_selector.is_last():
            self.frames += 1

        terminate = False
        truncate = self.frames >= self.max_cycles
        self.terminations = {a: terminate for a in self.agents}
        self.truncations = {a: truncate for a in self.agents}
        
        if len(self._agent_selector.agent_order):
            self.agent_selection = self._agent_selector.next()
        
        self._clear_rewards()
        self.rewards[self.agent_selection] = 10

        self._accumulate_rewards()
        self._deads_step_first()
    
    def reinit(self):
        self.score = 0
        self.run = True
        children = self.supervisor.getRoot().getField('children')

        try:
            self.ball
        except:
            pass
        else:
            self.ball.remove()
            self.blue1.remove()
            self.blue2.remove()
            self.blue3.remove()
            self.red1.remove()
            self.red2.remove()
            self.red3.remove()

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

        self.ball = self.supervisor.getFromDef('BALL')
        self.blue1 = self.supervisor.getFromDef('BLUE_PLAYER1')
        self.blue2 = self.supervisor.getFromDef('BLUE_PLAYER2')
        self.blue3 = self.supervisor.getFromDef('BLUE_PLAYER3')
        self.red1 = self.supervisor.getFromDef('RED_PLAYER1')
        self.red2 = self.supervisor.getFromDef('RED_PLAYER2')
        self.red3 = self.supervisor.getFromDef('RED_PLAYER3')

        self.ball_pos = self.ball.getField('translation')
        self.blue1_pos = self.blue1.getField('translation')
        self.blue1_rot = self.blue1.getField('rotation')
        self.blue2_pos = self.blue2.getField('translation')
        self.blue2_rot = self.blue2.getField('rotation')
        self.blue3_pos = self.blue3.getField('translation')
        self.blue3_rot = self.blue3.getField('rotation')
        self.red1_pos = self.red1.getField('translation')
        self.red1_rot = self.red1.getField('rotation')
        self.red2_pos = self.red2.getField('translation')
        self.red2_rot = self.red2.getField('rotation')
        self.red3_pos = self.red3.getField('translation')
        self.red3_rot = self.red3.getField('rotation')

    def reset(self, seed = None, option = None):
        if seed is not None:
            self._seed(seed=seed)
        self.agents = self.possible_agents
        self._agent_selector.reinit(self.agents)
        self.agent_selection = self._agent_selector.next()
        self.rewards = dict(zip(self.agents, [0 for _ in self.agents]))
        self._cumulative_rewards = {a: 0 for a in self.agents}
        self.terminations = dict(zip(self.agents, [False for _ in self.agents]))
        self.truncations = dict(zip(self.agents, [False for _ in self.agents]))
        self.infos = dict(zip(self.agents, [{} for _ in self.agents]))
        self.reinit()

if __name__ == "__main__":
    env = raw_env()
    env.reset()
    num = 0
    while env.supervisor.step(env.time_step) != 1:
        num += 1
        if num > 300:
            num = 0
            break
    
    for agent in env.agent_iter():
        observation, reward, termination, truncation, info = env.last()

        if termination or truncation:
            action = None
        else:
            if "action_mask" in info:
                mask = info["action_mask"]
            elif isinstance(observation, dict) and "action_mask" in observation:
                mask = observation["action_mask"]
            else:
                mask = None
            action = env.action_space(agent).sample(mask) # this is where you would insert your policy
        env.step(action)
        if env._agent_selector.is_last():
            for i in range(40):
                env.supervisor.step(env.time_step)
    
    #while env.supervisor.step(env.time_step) != 1:
    #    if num % 40 == 0 and num < 40 * 10:
    #        for agent in env.agents:
    #            env.step(0)
    #    if num == 400:
    #        for agent in env.agents:
    #            env.step(6)
    #    if num == 680:
    #        num = 0
    #    num += 1
