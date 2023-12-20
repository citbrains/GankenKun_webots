#!/usr/bin/env python3

import numpy as np
import math
import copy
import random

from controller import Supervisor

from gymnasium.spaces import Box, Discrete, Sequence
from gymnasium.utils import EzPickle, seeding

from pettingzoo import AECEnv
from pettingzoo.utils import wrappers
from pettingzoo.utils.agent_selector import agent_selector
from pettingzoo.utils.conversions import parallel_wrapper_fn

from soccer.player import Player

__all__ = ["env", "parallel_env", "raw_env"]

def env(**kwargs):
    env = raw_env(**kwargs)
    env = wrappers.AssertOutOfBoundsWrapper(env)
    env = wrappers.OrderEnforcingWrapper(env)
    return env

parallel_env = parallel_wrapper_fn(env)

def normalize_angle_rad(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle

class raw_env(AECEnv, EzPickle):
    metadata = {
        "render_modes": ["human", "rgb_array"],
        "name": "soccer_v0",
        "is_parallelizable": True,
    }
    
    supervisor = None
    
    def __init__(self, max_cycles=300, render_mode=None):
        EzPickle.__init__(self, max_cycles=max_cycles, render_mode=render_mode)
        if self.supervisor == None:
            self.supervisor = Supervisor()
        self.time_step = int(self.supervisor.getBasicTimeStep())

        self.frames = 0
        self.render_mode = render_mode
        self._seed()
        self.max_cycles = max_cycles
        self.out_agent = []
        self.agent_name_mapping = {}
        self.agent_dict = {}
        self.kill_list = []
        self.agent_list = []
        self.agents = ["blue1", "blue2", "blue3", "red1", "red2", "red3"]
        #self.agents = ["blue1", "blue2", "blue3"]
        self.dead_agents = []
        for i in range(len(self.agents)):
            self.agent_name_mapping[self.agents[i]] = i
            self.agent_list.append(Player(self.agents[i], self.supervisor))
        obs_space = Box(low=-5, high=5, shape = ([15]), dtype=np.float16)
        #obs_space = Box(low=float('-inf'), high=float('inf'), shape = ([9]), dtype=np.float32)
        self.observation_spaces = dict(zip(self.agents, [obs_space for _ in enumerate(self.agents)]))
        self.action_spaces = dict(zip(self.agents, [Discrete(9) for _ in enumerate(self.agents)]))
        self.actions = ["walk,1,0,0", "walk,-1,0,0", "walk,0,1,0", "walk,0,-1,0", "walk,0,0,1", "walk,0,0,-1", "motion,left_kick", "motion,right_kick", "walk,0,0,0"]
        self.state_space = Box(low=-5, high=5, shape = ([21]), dtype=np.float32)

        self.possible_agents = copy.deepcopy(self.agents)
        self._agent_selector = agent_selector(self.agents)

        self.reinit()

    def __del__(self):
        print("DELETE")

    def observation_space(self, agent):
        return self.observation_spaces[agent]

    def action_space(self, agent):
        return self.action_spaces[agent]

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)

    def observe(self, agent):
        i = self.agent_name_mapping[agent]
        state = self.state()
        ball_x, ball_y = [state[0], state[1]]
        bx, by, bthe = state[i*3+3], state[i*3+4],state[i*3+5]
        s, c = math.sin(bthe), math.cos(bthe)
        blx, bly = ball_x - bx, ball_y - by
        x, y = blx * c + bly * s, - blx * s + bly * c
        if abs(math.degrees(math.atan2(y, x))) > 60:
            obs = [-100, -100]
        else:
            obs = [x, y]
        obs += [bx, by, bthe]
        no_agent = len(self.possible_agents)
        base_index = list(range(no_agent))
        if agent.startswith("red"):
            index = base_index[int(no_agent/2):] + base_index[:int(no_agent/2)]
        else:
            index = base_index
        index.remove(i)
        for j in index:
            rx, ry = state[j*3+3], state[j*3+4]
            lx, ly = rx - bx, ry - by
            x, y = lx * c + ly * s, - lx * s + ly * c
            obs += [x, y]
        if agent.startswith("red"):
            obs[2] = -obs[2]
            obs[3] = -obs[3]
            obs[4] = normalize_angle_rad(obs[4]+math.pi)
        return obs
    
    def state(self):
        ball_x, ball_y, _ = self.ball_pos.getSFVec3f()
        for agent in self.agent_list:
            agent.update()
        player = []
        for i in range(len(self.agent_list)):
            player.append(self.agent_list[i].pos)
        state = [ball_x, ball_y, 0, player[0][0], player[0][1], player[0][2], player[1][0], player[1][1], player[1][2], player[2][0], player[2][1], player[2][2], player[3][0], player[3][1], player[3][2], player[4][0], player[4][1], player[4][2], player[5][0], player[5][1], player[5][2]]
        #state = [ball_x, ball_y, 0, player[0][0], player[0][1], player[0][2], player[1][0], player[1][1], player[1][2],  player[2][0], player[2][1], player[2][2]]
        return state
    
    def step(self, action):
        if self.terminations[self.agent_selection] or self.truncations[self.agent_selection]:
            self._was_dead_step(action)
            return
        self._cumulative_rewards[self.agent_selection] = 0
        agent = self.agent_list[self.agent_name_mapping[self.agent_selection]]
        agent.score = 0

        terminate = False
        truncate = False
        goal = False
        
        #print("frames: "+str(self.frames))

        i = self.agent_name_mapping[self.agent_selection]
        if self.agent_list[i].is_fall:
            while True:
                if self.agents[i].startswith("blue"):
                    x, y = random.uniform(-4.0, -3.0), random.uniform(-2.5, 2.5)
                elif self.agents[i].startswith("red"):
                    x, y = random.uniform(4.0, 3.0), random.uniform(-2.5, 2.5)
                near_robot = False
                for j in range(len(self.agents)):
                    if j == i:
                        continue
                    robot_x, robot_y, _ = self.agent_list[j].pos
                    length = math.sqrt((x-robot_x)**2+(y-robot_y)**2)
                    if length < 1:
                        near_robot = True
                        break
                if near_robot == False:
                    break
            self.init_pos[i][0], self.init_pos[i][1] = x, y
            self.agent_list[i].move(self.init_pos[i])
            self.agent_list[i].is_replace = True
        else:
            message = self.actions[action].encode('utf-8')
            agent.send(message)

        if self._agent_selector.is_last():
            self.frames += 1
            self._clear_rewards()
            for i in range(40):
                self.supervisor.step(self.time_step)
                ball_x, ball_y, _ = self.ball_pos.getSFVec3f()
                ball_vel_x, ball_vel_y = self.ball.getVelocity()[:2]
                for agent in self.agents:
                    x, y, the = self.agent_list[self.agent_name_mapping[agent]].pos
                    length = math.sqrt((x-ball_x)**2+(y-ball_y)**2)
                    self.rewards[agent] += 0.2/length/40
                    if length < 0.3:
                        if agent.startswith("blue"):
                            ball_dx, ball_dy = 4.5 - ball_x, 0 - ball_y
                            ball_len = math.sqrt(ball_dx**2+ball_dy**2)
                            ball_dx, ball_dy = ball_dx / ball_len, ball_dy / ball_len
                            reward = ball_vel_x * ball_dx + ball_vel_y * ball_dy
                            self.rewards[agent] += max(reward, 0) * 10
                        elif agent.startswith("red"):
                            ball_dx, ball_dy = 4.5 - ( -ball_x), 0 - (-ball_y)
                            ball_len = math.sqrt(ball_dx**2+ball_dy**2)
                            ball_dx, ball_dy = ball_dx / ball_len, ball_dy / ball_len
                            reward = (-ball_vel_x) * ball_dx + (-ball_vel_y) * ball_dy
                            self.rewards[agent] += max(reward, 0) * 10
            for agent in self.agents:
                # local rewards
                x, y, the = self.agent_list[self.agent_name_mapping[agent]].pos
                ball_x, ball_y, _ = self.ball_pos.getSFVec3f()
                s, c = math.sin(the), math.cos(the)
                dbx, dby = ball_x - x, ball_y - y
                blx, bly = dbx * c + dby * s, - dbx * s + dby * c

                self.rewards[agent] += -0.01
                #if self.rewards[agent] > 0.1:
                #    print("reward: "+str(agent)+" "+str(self.rewards[agent]))
                if self.agent_list[self.agent_name_mapping[agent]].is_replace:
                    self.rewards[agent] += -10
                    self.agent_list[self.agent_name_mapping[agent]].is_replace = False
                    print("reward(fall): "+str(agent)+" "+str(self.rewards[agent]))
                if abs(math.degrees(math.atan2(bly, blx))) <= 60:
                    self.rewards[agent] += 0.1

                # global rewards
                if ball_x > 4.5 and abs(ball_y) < 1.3:
                    goal = True
                    truncate = True
                    if agent.startswith("blue"):
                        self.rewards[agent] += 1000
                    elif agent.startswith("red"):
                        self.rewards[agent] += -1000
                    print("Team blue Goal, reward: "+str(agent)+" "+str(self.rewards[agent]))
                elif ball_x < -4.5 and abs(ball_y) < 1.3:
                    goal = True
                    truncate = True
                    if agent.startswith("blue"):
                        self.rewards[agent] += -1000
                    elif agent.startswith("red"):
                        self.rewards[agent] += 1000
                    print("Team red Goal, reward: "+str(agent)+" "+str(self.rewards[agent]))

            for agent in self.agents:
                self.total_rewards[agent] += self.rewards[agent]

            if not goal:
                if abs(ball_x) > 4.5 or abs(ball_y) > 3.0:
                    print("The ball out of the field")
                    y = random.uniform(-2.5, 2.5)
                    self.ball.resetPhysics()
                    self.ball_pos.setSFVec3f([0, y, 0])
        
        if self.frames >= self.max_cycles:
            truncate = True
        self.terminations = {a: terminate for a in self.agents}
        self.truncations = {a: truncate for a in self.agents}
        if truncate:
            for agent in self.agents:
                self.infos[agent]["episode"] = {"r": self.total_rewards[agent], "l": self.max_cycles}

        if self._agent_selector.is_last():
            _live_agents = self.agents[:]
            for k in self.kill_list:
                _live_agents.remove(k)
                self.terminations[k] = True
                self.dead_agents.append(k)
            self.kill_list = []
            self._agent_selector.reinit(_live_agents)

        if len(self._agent_selector.agent_order):
            self.agent_selection = self._agent_selector.next()
        
        self._accumulate_rewards()
        self._deads_step_first()
    
    def render():
        pass

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

        y = random.uniform(-2.5, 2.5)
        children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 {y} 0.1 size 1 }}')
        self.ball = self.supervisor.getFromDef('BALL')
        self.ball_pos = self.ball.getField('translation')
        self.init_pos = [[-0.3, 0, 0], [-2, -1, 0], [-2, 1, 0], [1, 0, 3.14], [2, -1, 3.14], [2, 1, 3.14]]
        for i in range(len(self.agent_list)):
            while True:
                if self.agents[i].startswith("blue"):
                    x, y = random.uniform(-4.0, -1.0), random.uniform(-2.5, 2.5)
                elif self.agents[i].startswith("red"):
                    x, y = random.uniform(4.0, 1.0), random.uniform(-2.5, 2.5)
                near_robot = False
                for j in range(len(self.agents)):
                    if j == i:
                        continue
                    length = math.sqrt((x-self.init_pos[j][0])**2+(y-self.init_pos[j][1])**2)
                    if length < 1:
                        near_robot = True
                        break
                if near_robot == False:
                    break
            self.init_pos[i][0], self.init_pos[i][1] = x, y
            self.agent_list[i].reset(self.init_pos[i])
        self.frames = 0

    def reset(self, seed = None, options = None):
        if seed is not None:
            self._seed(seed=seed)
        self.agents = copy.deepcopy(self.possible_agents)
        self._agent_selector.reinit(self.agents)
        self.agent_selection = self._agent_selector.next()
        self.rewards = dict(zip(self.agents, [0 for _ in self.agents]))
        self.total_rewards = dict(zip(self.agents, [0 for _ in self.agents]))
        self._cumulative_rewards = {a: 0 for a in self.agents}
        self.terminations = dict(zip(self.agents, [False for _ in self.agents]))
        self.truncations = dict(zip(self.agents, [False for _ in self.agents]))
        self.infos = dict(zip(self.agents, [{} for _ in self.agents]))
        self.reinit()
