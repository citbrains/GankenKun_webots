#!/usr/bin/env python3

import sys
from controller import Supervisor
import random

import gym
import numpy as np
from stable_baselines import PPO2
from stable_baselines.common.vec_env import DummyVecEnv

sys.path.append('./GankenKun')
from kinematics import *
from foot_step_planner import *
from preview_control import *
from walking import *

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

        obs_x_list = [random.uniform(1.0, 2.0), random.uniform(2.5, 3.5), random.uniform(2.5, 3.5)]
        obs_y_list = [random.uniform(-0.25, 0.25), random.uniform(-1.8, -0.25), random.uniform(0.25, 1.8)]
        self.getSelf().getField('translation').setSFVec3f([0.0, 0.0, 0.450])
        self.getFromDef('BALL').getField('translation').setSFVec3f([0.2, 0.0, 0.1])
        self.getFromDef('ENEMY1').getField('translation').setSFVec3f([obs_x_list[0], obs_y_list[0],0.450])
        self.getFromDef('ENEMY2').getField('translation').setSFVec3f([obs_x_list[1], obs_y_list[1],0.450])
        self.getFromDef('ENEMY3').getField('translation').setSFVec3f([obs_x_list[2], obs_y_list[2],0.450])

        # Motors
        self.__motors = []
        for name in motorNames:
            self.__motors.append(self.getDevice(name))

        # Open AI Gym generic
        return np.array(self.state, dtype=np.float32)

    def step(self, action_no):
        # Execute the action
        action = self.actions_list[action_no]
        x_goal = self.foot_step[0][1] + action[0]
        y_goal = self.foot_step[0][2] - self.foot_step[0][5] + action[1]
        th_goal = self.foot_step[0][3] - action[2]
        
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
        yaw = q[3] if q[2] > 0 else -q[3]
        ball_x, ball_y, _ = self.getFromDef('BALL').getPosition()
        ball_x_lc = (ball_x - x) * math.cos(-yaw) - (ball_y - y) * math.sin(-yaw)
        ball_y_lc = (ball_x - x) * math.sin(-yaw) + (ball_y - y) * math.cos(-yaw)
        obs1_x, obs1_y, _ = self.getFromDef('ENEMY1').getPosition()
        obs2_x, obs2_y, _ = self.getFromDef('ENEMY2').getPosition()
        obs3_x, obs3_y, _ = self.getFromDef('ENEMY3').getPosition()
        obs_x_list = [obs1_x, obs2_x, obs3_x]
        obs_y_list = [obs1_y, obs2_y, obs3_y]
        obs_x_gl_list = []
        obs_y_gl_list = []
        obs_x_lc_list = []
        obs_y_lc_list = []
        obs_dist_list = []
        for i in range(len(obs_x_list)):
            obs_x, obs_y = obs_x_list[i], obs_y_list[i]
            obs_x_lc = (obs_x - x) * math.cos(-yaw) - (obs_y - y) * math.sin(-yaw)
            obs_y_lc = (obs_x - x) * math.sin(-yaw) + (obs_y - y) * math.cos(-yaw)
            if obs_x_lc < 0:
                continue
            obs_x_gl_list.append(obs_x)
            obs_y_gl_list.append(obs_y)
            obs_x_lc_list.append(obs_x_lc)
            obs_y_lc_list.append(obs_y_lc)
            obs_dist = math.sqrt(obs_x_lc**2 + obs_y_lc**2)
            obs_dist_list.append(obs_dist)
        if obs_dist_list:
            num = np.argmin(obs_dist_list)
            obs_x_gl, obs_y_gl = obs_x_gl_list[num], obs_y_gl_list[num]
            self.old_obs_x_gl, self.old_obs_y_gl = obs_x_gl, obs_y_gl
            obs_x_lc, obs_y_lc = obs_x_lc_list[num], obs_y_lc_list[num]
            self.observation_obs_time = 0
        else:
            obs_x, obs_y = self.old_obs_x_gl, self.old_obs_y_gl
            obs_x_lc = (obs_x - x) * math.cos(-yaw) - (obs_y - y) * math.sin(-yaw)
            obs_y_lc = (obs_x - x) * math.sin(-yaw) + (obs_y - y) * math.cos(-yaw)
        #print('\r%dstep [obs_x_lc, obs_y_lc] = %f, %f'% (self.time_step, obs_x_lc, obs_y_lc), end='')
        self.old_obs_x_lc, self.old_obs_y_lc = obs_x_lc, obs_y_lc

        self.state = (x, y, math.sin(yaw), math.cos(yaw), ball_x_lc, ball_y_lc, obs_x_lc, obs_y_lc)
        print(str(action)+": %f, %f, %f, %f, %f, %f, %f"%(x, y, yaw, ball_x_lc, ball_y_lc, obs_x_lc, obs_y_lc))

        ball_distance = math.sqrt(ball_x_lc**2 + ball_y_lc**2)
        ball_direction_deg = math.degrees(math.atan2(ball_y_lc, ball_x_lc))
        goal_x, goal_y = self.goal_pos
        ball_goal_distance = math.sqrt((goal_x - ball_x)**2 + (goal_y - ball_y)**2)
        obs_distance = math.sqrt(obs_x_lc**2 + obs_y_lc**2)
        dist_ball_obs_list = []
        for i in range(len(obs_x_list)):
            dist_ball_obs = math.sqrt((obs_x_list[i] - ball_x)**2 + (obs_y_list[i] - ball_y)**2)
            dist_ball_obs_list.append(dist_ball_obs)
        num = np.argmin(dist_ball_obs_list)
        ball_obs_distance = dist_ball_obs_list[num]

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
        else:
            #print()
            if ball_x > goal_x and self.goal_leftpole < ball_y < self.goal_rightpole:
                self.num_successes += 1
                print('GOAL!!')
                print("%d step" %self.time_step)
                print("successe_times : ", self.num_successes)
                reward = 1500
            elif ball_distance > self.dist_threshold or abs(ball_direction_deg) > self.direction_deg_threshold:
                print('ball lost...')
                reward = math.floor(-100.0 * ball_goal_distance)
            else:
                print('else')
                reward = -500
        
        return np.array(self.state, dtype=np.float32), reward, done, {}

if __name__ == '__main__':
    env = OpenAIGymEnvironment()
    env = DummyVecEnv([lambda: env])
    model = PPO2.load('sample')

    for num in range(100):
        print("--------------------------")
        print("{}episode...".format(num+1))
        state = env.reset()
        sleep(1)
        for i in range(200):
            action, _ = model.predict(state)
            state, rewards, done, info = env.step(action)
            if done:
                break

    env.close()

