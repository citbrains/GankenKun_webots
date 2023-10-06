#!/usr/bin/env python3

from __future__ import annotations

import numpy as np
import math
import datetime

from gymnasium.spaces import Box, Discrete, Sequence
from gymnasium.utils import EzPickle, seeding

from pettingzoo import AECEnv
from pettingzoo.utils import wrappers
from pettingzoo.utils.agent_selector import agent_selector
from pettingzoo.utils.conversions import parallel_wrapper_fn

import supersuit as ss
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure

import soccer_v0

class SaveModel(BaseCallback):
    def __init__(self, save_path: str, check_freq: int):
        super(SaveModel, self).__init__()
        self.save_path = save_path
        self.check_freq = check_freq
        self.save_num =300*6

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            self.model.save(f"{self.save_path}_step{self.save_num}")
            self.save_num += 300*6
        return True  # continue training

if __name__ == "__main__":
    now = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    log_dir = f'./logs/log_{now}/'
    logger = configure(log_dir, ["stdout", "csv", "tensorboard"])
    env = soccer_v0
    env = env.parallel_env()
    env = ss.black_death_v3(env)
    env.reset()
    env = ss.pettingzoo_env_to_vec_env_v1(env)
    env = ss.concat_vec_envs_v1(env, 1, num_cpus=0, base_class="stable_baselines3")
    model = PPO(MlpPolicy, env, n_steps = 300, batch_size = 300, verbose = 1)
    model.set_logger(logger)
    callback = SaveModel(f'./logs/log_{now}/model', check_freq=300)
    model.learn(total_timesteps=10000000, callback=callback)
    #model.learn(total_timesteps=1000000)

    #for agent in env.agent_iter():
    #    observation, reward, termination, truncation, info = env.last()

    #    if termination or truncation:
    #        action = None
    #    else:
    #        if "action_mask" in info:
    #            mask = info["action_mask"]
    #        elif isinstance(observation, dict) and "action_mask" in observation:
    #            mask = observation["action_mask"]
    #        else:
    #            mask = None
    #        action = env.action_space(agent).sample(mask) # this is where you would insert your policy
    #    env.step(action)
    #    if env._agent_selector.is_last():
    #        for i in range(40):
    #            env.supervisor.step(env.time_step)
    
    #num = 0
    #while True:
    #    if num < 10:
    #        env.step([1,1,1,0,0,0])
    #    elif num < 15:
    #        env.step([2,2,2,3,3,3])
    #    elif num < 20:
    #        env.reset()
    #       #time.sleep(0.96)
    #        num = 0
    #    num += 1
