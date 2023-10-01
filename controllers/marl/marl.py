#!/usr/bin/env python3

from __future__ import annotations

import numpy as np
import math

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

import soccer_v0

class ActionRewardLogger(BaseCallback):
    def __init__(self, check_freq: int):
        super(ActionRewardLogger, self).__init__()
        self.check_freq = check_freq

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            #action, _states = self.model.predict(self.training_env.last_obs, deterministic=True)
            print(self.training_env)
            last_obs = self.training_env.get_attr('last_obs')
            last_rewards = self.training_env.get_attr('last_rewards')
            #print(f"Step: {self.num_timesteps}, Action: {action}, Last Rewards: {last_rewards}")
            print(f"Last Rewards: {last_rewards}, Last Obs: {last_obs}")
        return True  # continue training

if __name__ == "__main__":
    env = soccer_v0
    env = env.parallel_env()
    #env = ss.black_death_v3(env)
    env.reset()
    env = ss.pettingzoo_env_to_vec_env_v1(env)
    env = ss.concat_vec_envs_v1(env, 1, num_cpus=0, base_class="stable_baselines3")
    model = PPO(MlpPolicy, env, n_steps = 100, batch_size = 600)
    #callback = ActionRewardLogger(check_freq=10)
    #model.learn(total_timesteps=10000, callback=callback)
    model.learn(total_timesteps=1000000)
    print("FINISH")

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
