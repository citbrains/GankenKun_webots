from functools import partial
import gym
from gym.spaces import Box
from gym.wrappers import TimeLimit
import numpy as np
#import gfootball.env as football_env
import soccer_v0
#from .encode.obs_encode import FeatureEncoder
#from .encode.rew_encode import Rewarder

from soccer.multiagentenv import MultiAgentEnv


class SoccerEnv(MultiAgentEnv):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.env = soccer_v0.parallel_env(max_cycles=kwargs["env_args"]["episode_length"])
        #self.scenario = kwargs["env_args"]["scenario"]
        self.env.reset()
        self.n_agents = self.env.num_agents
        self.agents = self.env.agents
        #self.reward_type = kwargs["env_args"]["reward"]

        #self.feature_encoder = FeatureEncoder()
        #self.reward_encoder = Rewarder()
        self.action_space = [gym.spaces.Discrete(self.env.action_space(agent).n) for agent in self.agents]

        tmp_obs_dicts, _ = self.env.reset()
        #tmp_obs = [self._encode_obs(obs_dict)[0] for obs_dict in tmp_obs_dicts]
        tmp_obs = np.hstack([np.array(tmp_obs_dicts[k], dtype=np.float32).flatten() for k in sorted(tmp_obs_dicts)])
        #self.observation_space = [Box(low=float("-inf"), high=float("inf"), shape=tmp_obs[n].shape, dtype=np.float32)
        #                          for n in range(self.n_agents)]
        self.observation_space = [self.env.observation_space(agent) for agent in self.agents]
        #self.share_observation_space = self.observation_space.copy()
        self.share_observation_space = [Box(low=float("-inf"), high=float("inf"), shape=tmp_obs.shape, dtype=np.float32) for n in range(self.n_agents)]

        self.pre_obs = None

    def _encode_obs(self, raw_obs):
        #obs = self.feature_encoder.encode(raw_obs.copy())
        obs = raw_obs
        obs_cat = np.hstack(
            [np.array(obs[k], dtype=np.float32).flatten() for k in sorted(obs)]
        )
        return obs_cat
        #ava = obs["avail"]
        #return obs_cat, ava

    def reset(self, **kwargs):
        """ Returns initial observations and states"""
        obs_dicts, _ = self.env.reset()
        self.pre_obs = obs_dicts
        #obs = []
        #ava = []
        obs = [np.array(obs_dicts[k], dtype=np.float32) for k in sorted(obs_dicts)]
        #for obs_dict in obs_dicts:
        #    obs_i, ava_i = self._encode_obs(obs_dict)
        #    obs.append(obs_i)
        #    ava.append(ava_i)
        return obs
        #state = obs.copy()
        #return obs, state, ava

    def step(self, actions):
        #actions_int = [int(a) for a in actions]
        #o, r, d, i = self.env.step(actions_int)
        actions_dict = {}
        for i, agent in enumerate(self.agents):
            actions_dict[agent] = int(actions[i])
        observations, rewards, terminations, truncations, infos = self.env.step(actions_dict)
        #obs = []
        obs = [np.array(observations[k], dtype=np.float32) for k in sorted(observations)]
        #ava = []
        #for obs_dict in observations:
        #    obs_i, ava_i = self._encode_obs(obs_dict)
        #    obs.append(obs_i)
        #    ava.append(ava_i)
        #state = obs.copy()

        #rewards = [[self.reward_encoder.calc_reward(_r, _prev_obs, _obs)]
        #           for _r, _prev_obs, _obs in zip(r, self.pre_obs, o)]
        rewards = [[rewards[k]] for k in sorted(rewards)]

        self.pre_obs = observations

        d = [truncations[k] for k in sorted(truncations)]
        dones = np.ones((self.n_agents), dtype=bool) * d
        #infos = [i for n in range(self.n_agents)]
        #infos = [infos[k] for k in sorted(infos)]
        info_n = []
        for i, agent in enumerate(self.agents):
            info = {'individual_reward': rewrads[agent]}
            info_n.append(info)
        #return obs, state, rewards, dones, infos, ava
        return obs, rewards, dones, info_n

    def render(self, **kwargs):
        # self.env.render(**kwargs)
        pass

    def close(self):
        pass

    def seed(self, args):
        pass

    def get_env_info(self):

        env_info = {"state_shape": self.observation_space[0].shape,
                    "obs_shape": self.observation_space[0].shape,
                    "n_actions": self.action_space[0].n,
                    "n_agents": self.n_agents,
                    "action_spaces": self.action_space
                    }
        return env_info
