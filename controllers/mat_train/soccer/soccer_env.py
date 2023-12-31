import gym
from gym.spaces import Box
import numpy as np
import soccer_v0

from soccer.multiagentenv import MultiAgentEnv


class SoccerEnv(MultiAgentEnv):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.env = soccer_v0.parallel_env(max_cycles=kwargs["env_args"]["episode_length"])
        self.env.reset()
        self.n_agents = self.env.num_agents
        self.agents = self.env.agents

        self.action_space = [gym.spaces.Discrete(self.env.action_space(agent).n) for agent in self.agents]

        tmp_obs_dicts, _ = self.env.reset()
        tmp_obs = np.hstack([np.array(tmp_obs_dicts[k], dtype=np.float32).flatten() for k in sorted(tmp_obs_dicts)])
        self.observation_space = [self.env.observation_space(agent) for agent in self.agents]
        self.share_observation_space = [Box(low=float("-inf"), high=float("inf"), shape=tmp_obs.shape, dtype=np.float32) for n in range(self.n_agents)]

        self.pre_obs = None

    def _encode_obs(self, raw_obs):
        obs = raw_obs
        obs_cat = np.hstack(
            [np.array(obs[k], dtype=np.float32).flatten() for k in sorted(obs)]
        )
        return obs_cat

    def reset(self, **kwargs):
        """ Returns initial observations and states"""
        obs_dicts, _ = self.env.reset()
        self.pre_obs = obs_dicts
        obs = [np.array(obs_dicts[k], dtype=np.float32) for k in sorted(obs_dicts)]
        return obs

    def step(self, actions):
        actions_dict = {}
        for i, agent in enumerate(self.agents):
            actions_dict[agent] = int(actions[i])
        observations, rewards, terminations, truncations, infos = self.env.step(actions_dict)
        obs = [np.array(observations[k], dtype=np.float32) for k in sorted(observations)]
        rewards = [[rewards[k]] for k in sorted(rewards)]

        self.pre_obs = observations

        d = [truncations[k] for k in sorted(truncations)]
        dones = np.ones((self.n_agents), dtype=bool) * d
        info_n = []
        for i, agent in enumerate(self.agents):
            info = {'individual_reward': rewards[i][0]}
            info_n.append(info)
        return obs, rewards, dones, info_n

    def render(self, **kwargs):
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
