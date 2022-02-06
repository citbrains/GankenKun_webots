#!/usr/bin/env python3

import sys
from controller import Supervisor
import random

try:
    import gym
    import numpy as np
    from stable_baselines3 import PPO
    from stable_baselines.common.env_checker import check_env
except ImportError:
    sys.exit(
        'Please make sure you have all dependencies installed. '
        'Run: "pip3 install numpy gym stable_baselines"'
    )

#from field import Field

sys.path.append('./GankenKun')
from kinematics import *
from foot_step_planner import *
from preview_control import *
from walking import *
from scipy.spatial.transform import Rotation as R

motorNames = [
  "head_yaw_joint",                        # ID1
  "left_shoulder_pitch_joint [shoulder]",  # ID2
  "left_shoulder_roll_joint",              # ID3
  "left_elbow_pitch_joint",                # ID4
  "right_shoulder_pitch_joint [shoulder]", # ID5
  "right_shoulder_roll_joint",             # ID6
  "right_elbow_pitch_joint",               # ID7
  "left_waist_yaw_joint",                  # ID8
  "left_waist_roll_joint [hip]",           # ID9
  "left_waist_pitch_joint",                # ID10
  "left_knee_pitch_joint",                 # ID11
  "left_ankle_pitch_joint",                # ID12
  "left_ankle_roll_joint",                 # ID13
  "right_waist_yaw_joint",                 # ID14
  "right_waist_roll_joint [hip]",          # ID15
  "right_waist_pitch_joint",               # ID16
  "right_knee_pitch_joint",                # ID17
  "right_ankle_pitch_joint",               # ID18
  "right_ankle_roll_joint"                 # ID19
]

class OpenAIGymEnvironment(Supervisor, gym.Env):
    def __init__(self, max_episode_steps=1000):
        super().__init__()

        self.time_step = 0        
        self.goal_pos = [4.5, 0]
        self.goal_rightpole = self.goal_pos[1] + 1.3
        self.goal_leftpole = self.goal_pos[1] - 1.3
        self.x_threshold = 5.0
        self.y_threshold = 3.5
        self.ball_x_threshold = 4.5
        self.ball_y_threshold = 3.0
        self.direction_deg_threshold = 90
        self.dist_threshold = 0.55
        self.obs_dist_threshold = 0.3
        self.obs_direction_deg_threshold = 45
        self.field_x = 9.0
        self.field_y = 6.0

        high = np.array([self.x_threshold, self.y_threshold, 1.0, 1.0, self.field_x, self.field_y, self.field_x, self.field_y], dtype=np.float32)

        self.action_space = gym.spaces.Discrete(6)
        self.observation_space = gym.spaces.Box(-high, high, dtype=np.float32)
        self.state = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32)
        self.spec = gym.envs.registration.EnvSpec(id='GankenKunObstacleEnv-v0', max_episode_steps=max_episode_steps)

        # Environment specific
        self.__timestep = int(self.getBasicTimeStep())
        self.__motors = []
        
        joint_angles = [0]*len(motorNames)
        left_foot  = [-0.02, +0.054, 0.02]
        right_foot = [-0.02, -0.054, 0.02]
        self.pc = preview_control(self.__timestep/1000, 1.0, 0.27)
        self.walk = walking(self.__timestep/1000, motorNames, left_foot, right_foot, joint_angles, self.pc)
        self.actions_list = [[ self.walk.max_stride_x, 0.0, 0.0],
                             [-self.walk.max_stride_x, 0.0, 0.0],
                             [0.0,  self.walk.max_stride_y, 0.0],
                             [0.0, -self.walk.max_stride_y, 0.0],
                             [0.0, 0.0,  self.walk.max_stride_th],
                             [0.0, 0.0, -self.walk.max_stride_th],]
        # Tools
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.__timestep)

    def wait_keyboard(self):
        while self.keyboard.getKey() != ord('Y'):
            super().step(self.__timestep)

    def reset(self):
        # Reset the simulation
        x_goal, y_goal, th_goal = 0.0, 0.0, 0.0
        self.foot_step = self.walk.setGoalPos([x_goal, y_goal, th_goal])
        super().step(self.__timestep)
        joint_angles = [0]*len(motorNames)
        left_foot  = [-0.02, +0.054, 0.02]
        right_foot = [-0.02, -0.054, 0.02]
        self.pc = preview_control(self.__timestep/1000, 1.0, 0.27)
        self.walk = walking(self.__timestep/1000, motorNames, left_foot, right_foot, joint_angles, self.pc)

        self.simulationResetPhysics()
        self.simulationReset()
        super().step(self.__timestep)
        self.getSelf().getField('translation').setSFVec3f([-0.3, random.uniform(-1.0, 1.0), 0.450])

        # Motors
        self.__motors = []
        for name in motorNames:
            self.__motors.append(self.getDevice(name))

        # Open AI Gym generic
        return np.array(self.state, dtype=np.float32)

    def step(self, action):
        # Execute the action
        action = self.actions_list[action]
        x_goal = self.foot_step[0][1] + action[0]
        y_goal = self.foot_step[0][2] - self.foot_step[0][5] + action[1]
        th_goal = self.foot_step[0][3] + action[2]
        
        self.foot_step = self.walk.setGoalPos([x_goal, y_goal, th_goal])
        while super().step(self.__timestep) != -1:
            joint_angles,lf,rf,xp,n = self.walk.getNextPos()
            if n == 0:
                if self.foot_step[0][4] == 'left':
                    break
                else:
                    self.foot_step = self.walk.setGoalPos()
            for i in range(len(motorNames)):
                self.__motors[i].setPosition(joint_angles[i])

        # Observation
        x, y, _ = self.getSelf().getPosition()
        q = self.getSelf().getField('rotation').getSFRotation()
        r  = R.from_quat([q[0], q[1], q[2], q[3]])
        yaw, _, _ = r.as_euler('zyx', degrees=True)
        ball_x, ball_y, _ = self.getFromDef('BALL').getPosition()
        ball_x_lc = (ball_x - x) * math.cos(-yaw) - (ball_y - y) * math.sin(-yaw)
        ball_y_lc = (ball_x - x) * math.sin(-yaw) + (ball_y - y) * math.cos(-yaw)

        self.state = np.array([x, y, math.sin(yaw), math.cos(yaw), ball_x_lc, ball_y_lc, 2, 0], dtype=np.float32)

        # Done
        ball_distance = math.sqrt(ball_x_lc**2 + ball_y_lc**2)
        ball_direction_deg = math.degrees(math.atan2(ball_y_lc, ball_x_lc))
        goal_x, goal_y = self.goal_pos
        ball_goal_distance = math.sqrt((goal_x - ball_x)**2 + (goal_y - ball_y)**2)
        obs_distance = 10
        ball_obs_distance = 10
        done = bool(
                abs(x) > self.x_threshold
                or abs(y) > self.y_threshold
                or abs(ball_x) > self.ball_x_threshold
                or abs(ball_y) > self.ball_y_threshold
                or ball_distance > self.dist_threshold
                or abs(ball_direction_deg) > self.direction_deg_threshold
                or obs_distance < self.obs_dist_threshold
                or ball_obs_distance < self.obs_dist_threshold
        )

        # Reward
        reward = 0
        if not done:
#            reward += math.floor(-10.0 * ball_goal_distance)/10
            dis_ball_goal = round(ball_goal_distance, 1)
            dis_ball_obs =round(ball_obs_distance, 1)
            ball_obs_threshold = 0.5 
            ball_goal_threshold = 4.5
            
            if 0 <= dis_ball_goal <= ball_goal_threshold:
                ball_goal_reward = 1 - 2 * (dis_ball_goal / ball_goal_threshold)
            elif ball_goal_threshold < dis_ball_goal:
                ball_goal_reward = -1
            
            if 0 <= dis_ball_obs <= ball_obs_threshold:
                ball_obs_reward = 2 * (dis_ball_obs / ball_obs_threshold) - 1
            elif ball_obs_threshold < dis_ball_obs:
                ball_obs_reward = 1
            
            w1 = 0.8
            w2 = 1 - w1
            reward = round(w1*ball_goal_reward + w2*ball_obs_reward, 2) 

        return self.state.astype(np.float32), reward, done, {}


if __name__ == '__main__':
    env = OpenAIGymEnvironment()
    check_env(env)

    # Train
    model = PPO('MlpPolicy', env, n_steps=2048, verbose=1)
    model.learn(total_timesteps=1e5)

    # Replay
    print('Training is finished, press `Y` for replay...')
    env.wait_keyboard()

    obs = env.reset()
    for _ in range(100000):
        action, _states = model.predict(obs)
        obs, reward, done, info = env.step(action)
        print(obs, reward, done, info)
        if done:
            obs = env.reset()

