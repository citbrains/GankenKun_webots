#!/usr/bin/env python3

import numpy as np
import math
import copy

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

class raw_env(AECEnv, EzPickle):
    metadata = {
        "render_modes": ["human", "rgb_array"],
        "name": "soccer_v0",
        "is_parallelizable": True,
    }
    
    supervisor = None
    
    def __init__(self, max_cycles=100, render_mode=None):
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
        self.dead_agents = []
        for i in range(len(self.agents)):
            self.agent_name_mapping[self.agents[i]] = i
            self.agent_list.append(Player(self.agents[i], self.supervisor))
        obs_space = Box(low=-5, high=5, shape = ([15]), dtype=np.float16)
        self.observation_spaces = dict(zip(self.agents, [obs_space for _ in enumerate(self.agents)]))
        self.action_spaces = dict(zip(self.agents, [Discrete(6) for _ in enumerate(self.agents)]))
        self.actions = ["walk,1,0,0", "walk,-1,0,0", "walk,0,1,0", "walk,0,-1,0", "walk,0,0,1", "walk,0,0,-1", "motion,left_kick", "motion,right_kick"]
        self.state_space = Box(low=-5, high=5, shape = ([21]), dtype=np.float16)

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
        obs = [x, y]
        obs += [bx, by, bthe]
        no_agent = len(self.possible_agents)
        base_index = list(range(no_agent))
        if i >= no_agent/2:
            index = base_index[int(no_agent/2):] + base_index[:int(no_agent/2)]
        else:
            index = base_index
        index.remove(i)
        for j in index:
            rx, ry = state[j*3+3], state[j*3+4]
            lx, ly = rx - bx, ry - by
            x, y = lx * c + ly * s, - lx * s + ly * c
            obs += [x, y]
        return obs
    
    def state(self):
        ball_x, ball_y, _ = self.ball_pos.getSFVec3f()
        for agent in self.agent_list:
            agent.update()
        player = []
        for i in range(len(self.agent_list)):
            player.append(self.agent_list[i].pos)
        state = [ball_x, ball_y, 0, player[0][0], player[0][1], player[0][2], player[1][0], player[1][1], player[1][2], player[2][0], player[2][1], player[2][2], player[3][0], player[3][1], player[3][2], player[4][0], player[4][1], player[4][2], player[5][0], player[5][1], player[5][2]]
        return state
    
    def step(self, action):
        if self.terminations[self.agent_selection] or self.truncations[self.agent_selection]:
            self._was_dead_step(action)
            return
        self._cumulative_rewards[self.agent_selection] = 0
        agent = self.agent_list[self.agent_name_mapping[self.agent_selection]]
        agent.score = 0
        
        i = self.agent_name_mapping[self.agent_selection]
        if self.agent_list[i].is_fall:
            self.agent_list[i].move(self.init_pos[i])
        else:
            message = self.actions[action].encode('utf-8')
            agent.send(message)

        if self._agent_selector.is_last():
            self.frames += 1
            for i in range(40):
                self.supervisor.step(self.time_step)

        terminate = False
        truncate = self.frames >= self.max_cycles
        self.terminations = {a: terminate for a in self.agents}
        self.truncations = {a: truncate for a in self.agents}
                
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
        
        self._clear_rewards()
        ball_x, ball_y, _ = self.ball_pos.getSFVec3f()
        agent = self.agent_list[self.agent_name_mapping[self.agent_selection]]
        x, y, the = agent.pos
        length = math.sqrt((x-ball_x)**2+(y-ball_y)**2)
        if length < 1.0:
            i = self.agent_name_mapping[self.agent_selection]
            if i < 3:
                self.rewards[self.agent_selection] = max(self.ball.getVelocity()[0], 0)
            else:
                self.rewards[self.agent_selection] = max(-self.ball.getVelocity()[0],0)
            if self.rewards[self.agent_selection] > 0.1:
                print("reward: "+str(self.agent_selection)+" "+str(self.rewards[self.agent_selection]))

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

        children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 0.1 size 1 }}')
        self.ball = self.supervisor.getFromDef('BALL')
        self.ball_pos = self.ball.getField('translation')
        self.init_pos = [[-0.3, 0, 0], [-2, -1, 0], [-2, 1, 0], [1, 0, 3.14], [2, -1, 3.14], [2, 1, 3.14]]
        for i in range(len(self.agent_list)):
            self.agent_list[i].reset(self.init_pos[i])
        self.frames = 0

    def reset(self, seed = None, options = None):
        if seed is not None:
            self._seed(seed=seed)
        self.agents = copy.deepcopy(self.possible_agents)
        self._agent_selector.reinit(self.agents)
        self.agent_selection = self._agent_selector.next()
        self.rewards = dict(zip(self.agents, [0 for _ in self.agents]))
        self._cumulative_rewards = {a: 0 for a in self.agents}
        self.terminations = dict(zip(self.agents, [False for _ in self.agents]))
        self.truncations = dict(zip(self.agents, [False for _ in self.agents]))
        self.infos = dict(zip(self.agents, [{} for _ in self.agents]))
        self.reinit()
