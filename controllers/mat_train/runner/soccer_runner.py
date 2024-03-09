import time
import wandb
import numpy as np
import torch
from runner.base_runner import Runner
from pathlib import Path
import os

def _t2n(x):
    return x.detach().cpu().numpy()

class SoccerRunner(Runner):
    """Runner class to perform training, evaluation. and data collection for SMAC. See parent class for details."""
    def __init__(self, config):
        super(SoccerRunner, self).__init__(config)

    def setEpisode(self, episode1, episode2):
        model_dir = Path(os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)) + "/results/test/models")
        self.restore(str(model_dir) + "/transformer_" + str(episode1) + ".pt")
        self.buffer.after_update()
        self.c_restore(str(model_dir) + "/transformer_" + str(episode2) + ".pt")
        self.c_buffer.after_update()

    def run(self):
        self.warmup()

        start = time.time()
        episodes = int(self.num_env_steps) // self.episode_length // self.n_rollout_threads
        save_episode = None
        self.self_play_interval = self.all_args.self_play_interval

        train_episode_rewards = [0 for _ in range(self.n_rollout_threads)]
        done_episodes_rewards = []

        #train_episode_scores = [0 for _ in range(self.n_rollout_threads)]
        #done_episodes_scores = []

        train_individual_rewards = [0 for _ in range(self.num_agents)]
        done_individual_rewards = []

        for episode in range(episodes):
            #if self.use_linear_lr_decay:
            #    self.trainer.policy.lr_decay(episode, episodes)

            count = 0
            num = 0
            for step in range(self.episode_length):
                # Sample actions
                values, actions, action_log_probs, rnn_states, rnn_states_critic = self.collect(step)
                c_values, c_actions, c_action_log_probs, c_rnn_states, c_rnn_states_critic = self.c_collect(step)
                concat_actions = [np.concatenate([actions, c_actions])]

                # Obser reward and next obs
                obs, rewards, dones, infos, available_actions, c_obs, c_rewards, c_dones, c_infos, c_available_actions = self.envs.step(concat_actions)

                dones_env = np.all(dones, axis=1)
                reward_env = np.mean(rewards, axis=1).flatten()
                train_episode_rewards += reward_env

                for agent_id in range(self.num_agents):
                    for info in infos:
                        if 'individual_reward' in info[agent_id].keys():
                            train_individual_rewards[agent_id] += info[agent_id]['individual_reward']


                #score_env = [t_info[0]["score_reward"] for t_info in infos]
                #train_episode_scores += np.array(score_env)
                for t in range(self.n_rollout_threads):
                    if dones_env[t]:
                        done_episodes_rewards.append(train_episode_rewards[t])
                        train_episode_rewards[t] = 0
                        #done_episodes_scores.append(train_episode_scores[t])
                        #train_episode_scores[t] = 0
                        done_individual_rewards.append(train_individual_rewards)
                        train_individual_rewards = [0 for _ in range(self.num_agents)]

                data = obs, rewards, dones, infos, available_actions, \
                       values, actions, action_log_probs, \
                       rnn_states, rnn_states_critic
                c_data = c_obs, c_rewards, c_dones, c_infos, c_available_actions, \
                       c_values, c_actions, c_action_log_probs, \
                       c_rnn_states, c_rnn_states_critic

                # insert data into buffer
                self.insert(data)
                self.c_insert(c_data)

                if np.any(dones) or count >= 300:
                    self.envs.reset()
                    print()
                    count = 0
                    num += 1
                count += 1
                print(str(num)+": "+str(count), end='\r', flush=True)

            # compute return and update network
            self.compute()
            train_infos = self.train()

            # post process
            total_num_steps = (episode + 1) * self.episode_length * self.n_rollout_threads
            # save model
            #if (episode % self.save_interval == 0 or episode == episodes - 1):
            #    save_episode = episode
            #    self.save(save_episode)
            
            # copy team update network
            #if (episode % self.self_play_interval == 0 and episode > self.save_interval):
            #    print("copy team update network")
            #    self.c_restore(str(self.save_dir) + "/transformer_" + str(save_episode) + ".pt")
            #self.c_buffer.after_update()

            # log information
            if episode % self.log_interval == 0:
                end = time.time()
                print("\n Scenario {} Algo {} Exp {} updates {}/{} episodes, total num timesteps {}/{}, FPS {}.\n"
                        .format(self.all_args.scenario_name,
                                self.algorithm_name,
                                self.experiment_name,
                                episode,
                                episodes,
                                total_num_steps,
                                self.num_env_steps,
                                int(total_num_steps / (end - start))))

                self.log_train(train_infos, total_num_steps)

                if len(done_episodes_rewards) > 0:
                    aver_episode_rewards = np.mean(done_episodes_rewards)
                    self.writter.add_scalars("train_episode_rewards", {"aver_rewards": aver_episode_rewards}, total_num_steps)
                    done_episodes_rewards = []

                    #aver_episode_scores = np.mean(done_episodes_scores)
                    #self.writter.add_scalars("train_episode_scores", {"aver_scores": aver_episode_scores}, total_num_steps)
                    #done_episodes_scores = []
                    #print("some episodes done, average rewards: {}, scores: {}"
                    #      .format(aver_episode_rewards, aver_episode_scores))
                    print("some episodes done, average rewards: {}".format(aver_episode_rewards))

                env_infos = {}
                last_individual_rewards = done_individual_rewards[-1]
                done_individual_rewards = []
                for agent_id in range(self.num_agents):
                    agent_k = 'agent%i/individual_rewards' % (agent_id+1)
                    env_infos[agent_k] = [last_individual_rewards[agent_id]]
                self.log_env(env_infos, total_num_steps)


            # eval
            if episode % self.eval_interval == 0 and self.use_eval:
                self.eval(total_num_steps)

    def warmup(self):
        # reset env
        o = self.envs.reset()[0]
        obs, c_obs = o[0], o[1]

        # replay buffer
        if self.use_centralized_V:
            share_obs = obs.reshape(self.n_rollout_threads, -1)
            share_obs = np.expand_dims(share_obs, 1).repeat(self.num_agents, axis=1)
            c_share_obs = c_obs.reshape(self.n_rollout_threads, -1)
            c_share_obs = np.expand_dims(c_share_obs, 1).repeat(self.num_agents, axis=1)
        else:
            share_obs = obs
            c_share_obs = c_obs

        self.buffer.share_obs[0] = share_obs.copy()
        self.buffer.obs[0] = obs.copy()
        self.c_buffer.share_obs[0] = c_share_obs.copy()
        self.c_buffer.obs[0] = c_obs.copy()

    @torch.no_grad()
    def collect(self, step):
        self.trainer.prep_rollout()
        value, action, action_log_prob, rnn_state, rnn_state_critic \
            = self.trainer.policy.get_actions(np.concatenate(self.buffer.share_obs[step]),
                                              np.concatenate(self.buffer.obs[step]),
                                              np.concatenate(self.buffer.rnn_states[step]),
                                              np.concatenate(self.buffer.rnn_states_critic[step]),
                                              np.concatenate(self.buffer.masks[step]),
                                              np.concatenate(self.buffer.available_actions[step]))
 
        # [self.envs, agents, dim]
        values = np.array(np.split(_t2n(value), self.n_rollout_threads))
        actions = np.array(np.split(_t2n(action), self.n_rollout_threads))
        action_log_probs = np.array(np.split(_t2n(action_log_prob), self.n_rollout_threads))
        rnn_states = np.array(np.split(_t2n(rnn_state), self.n_rollout_threads))
        rnn_states_critic = np.array(np.split(_t2n(rnn_state_critic), self.n_rollout_threads))

        return values, actions, action_log_probs, rnn_states, rnn_states_critic

    def c_collect(self, step):
        self.c_trainer.prep_rollout()
        value, action, action_log_prob, rnn_state, rnn_state_critic \
            = self.c_trainer.policy.get_actions(np.concatenate(self.c_buffer.share_obs[step]),
                                              np.concatenate(self.c_buffer.obs[step]),
                                              np.concatenate(self.c_buffer.rnn_states[step]),
                                              np.concatenate(self.c_buffer.rnn_states_critic[step]),
                                              np.concatenate(self.c_buffer.masks[step]),
                                              np.concatenate(self.c_buffer.available_actions[step]))
 
        # [self.envs, agents, dim]
        values = np.array(np.split(_t2n(value), self.n_rollout_threads))
        actions = np.array(np.split(_t2n(action), self.n_rollout_threads))
        action_log_probs = np.array(np.split(_t2n(action_log_prob), self.n_rollout_threads))
        rnn_states = np.array(np.split(_t2n(rnn_state), self.n_rollout_threads))
        rnn_states_critic = np.array(np.split(_t2n(rnn_state_critic), self.n_rollout_threads))

        return values, actions, action_log_probs, rnn_states, rnn_states_critic



    def insert(self, data):
        obs, rewards, dones, infos, available_actions, \
        values, actions, action_log_probs, rnn_states, rnn_states_critic = data

        dones_env = np.all(dones, axis=1)

        rnn_states[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, self.recurrent_N, self.hidden_size), dtype=np.float32)
        rnn_states_critic[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, *self.buffer.rnn_states_critic.shape[3:]), dtype=np.float32)

        masks = np.ones((self.n_rollout_threads, self.num_agents, 1), dtype=np.float32)
        masks[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, 1), dtype=np.float32)

        active_masks = np.ones((self.n_rollout_threads, self.num_agents, 1), dtype=np.float32)
        active_masks[dones == True] = np.zeros(((dones == True).sum(), 1), dtype=np.float32)
        active_masks[dones_env == True] = np.ones(((dones_env == True).sum(), self.num_agents, 1), dtype=np.float32)

        # bad_masks = np.array([[[0.0] if info[agent_id]['bad_transition'] else [1.0] for agent_id in range(self.num_agents)] for info in infos])

        if self.use_centralized_V:
            share_obs = obs.reshape(self.n_rollout_threads, -1)
            share_obs = np.expand_dims(share_obs, 1).repeat(self.num_agents, axis=1)
        else:
            share_obs = obs

        self.buffer.insert(share_obs, obs, rnn_states, rnn_states_critic,
                           actions, action_log_probs, values, rewards, masks, None, active_masks,
                           available_actions)

    def c_insert(self, data):
        obs, rewards, dones, infos, available_actions, \
        values, actions, action_log_probs, rnn_states, rnn_states_critic = data

        dones_env = np.all(dones, axis=1)

        rnn_states[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, self.recurrent_N, self.hidden_size), dtype=np.float32)
        rnn_states_critic[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, *self.c_buffer.rnn_states_critic.shape[3:]), dtype=np.float32)

        masks = np.ones((self.n_rollout_threads, self.num_agents, 1), dtype=np.float32)
        masks[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, 1), dtype=np.float32)

        active_masks = np.ones((self.n_rollout_threads, self.num_agents, 1), dtype=np.float32)
        active_masks[dones == True] = np.zeros(((dones == True).sum(), 1), dtype=np.float32)
        active_masks[dones_env == True] = np.ones(((dones_env == True).sum(), self.num_agents, 1), dtype=np.float32)

        # bad_masks = np.array([[[0.0] if info[agent_id]['bad_transition'] else [1.0] for agent_id in range(self.num_agents)] for info in infos])

        if self.use_centralized_V:
            share_obs = obs.reshape(self.n_rollout_threads, -1)
            share_obs = np.expand_dims(share_obs, 1).repeat(self.num_agents, axis=1)
        else:
            share_obs = obs

        self.c_buffer.insert(share_obs, obs, rnn_states, rnn_states_critic,
                           actions, action_log_probs, values, rewards, masks, None, active_masks,
                           available_actions)

    def log_train(self, train_infos, total_num_steps):
        train_infos["average_step_rewards"] = np.mean(self.buffer.rewards)
        print("average_step_rewards is {}.".format(train_infos["average_step_rewards"]))
        for k, v in train_infos.items():
            if self.use_wandb:
                wandb.log({k: v}, step=total_num_steps)
            else:
                self.writter.add_scalars(k, {k: v}, total_num_steps)

    @torch.no_grad()
    def eval(self, total_num_steps):
        eval_episode = 0
        eval_episode_rewards = []
        one_episode_rewards = [0 for _ in range(self.all_args.eval_episodes)]
        eval_episode_scores = []
        one_episode_scores = [0 for _ in range(self.all_args.eval_episodes)]

        eval_obs, eval_share_obs, ava = self.eval_envs.reset()
        eval_rnn_states = np.zeros((self.all_args.eval_episodes, self.num_agents, self.recurrent_N,
                                    self.hidden_size), dtype=np.float32)
        eval_masks = np.ones((self.all_args.eval_episodes, self.num_agents, 1), dtype=np.float32)

        while True:
            self.trainer.prep_rollout()
            eval_actions, eval_rnn_states = \
                self.trainer.policy.act(np.concatenate(eval_share_obs),
                                        np.concatenate(eval_obs),
                                        np.concatenate(eval_rnn_states),
                                        np.concatenate(eval_masks),
                                        np.concatenate(ava),
                                        deterministic=True)
            eval_actions = np.array(np.split(_t2n(eval_actions), self.all_args.eval_episodes))
            eval_rnn_states = np.array(np.split(_t2n(eval_rnn_states), self.all_args.eval_episodes))

            # Obser reward and next obs
            eval_obs, eval_share_obs, eval_rewards, eval_dones, eval_infos, ava = self.eval_envs.step(eval_actions)
            eval_rewards = np.mean(eval_rewards, axis=1).flatten()
            one_episode_rewards += eval_rewards

            eval_scores = [t_info[0]["score_reward"] for t_info in eval_infos]
            one_episode_scores += np.array(eval_scores)

            eval_dones_env = np.all(eval_dones, axis=1)
            eval_rnn_states[eval_dones_env == True] = np.zeros(((eval_dones_env == True).sum(), self.num_agents,
                                                                self.recurrent_N, self.hidden_size), dtype=np.float32)
            eval_masks = np.ones((self.all_args.eval_episodes, self.num_agents, 1), dtype=np.float32)
            eval_masks[eval_dones_env == True] = np.zeros(((eval_dones_env == True).sum(), self.num_agents, 1),
                                                          dtype=np.float32)

            for eval_i in range(self.all_args.eval_episodes):
                if eval_dones_env[eval_i]:
                    eval_episode += 1
                    eval_episode_rewards.append(one_episode_rewards[eval_i])
                    one_episode_rewards[eval_i] = 0

                    eval_episode_scores.append(one_episode_scores[eval_i])
                    one_episode_scores[eval_i] = 0

            if eval_episode >= self.all_args.eval_episodes:
                key_average = '/eval_average_episode_rewards'
                key_max = '/eval_max_episode_rewards'
                key_scores = '/eval_average_episode_scores'
                eval_env_infos = {key_average: eval_episode_rewards,
                                  key_max: [np.max(eval_episode_rewards)],
                                  key_scores: eval_episode_scores}
                self.log_env(eval_env_infos, total_num_steps)

                print("eval average episode rewards: {}, scores: {}."
                      .format(np.mean(eval_episode_rewards), np.mean(eval_episode_scores)))
                break
