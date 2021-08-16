#!/usr/bin/env python3

import sys
from controller import Robot
sys.path.append('./GankenKun')
from kinematics import *
from foot_step_planner import *
from preview_control import *
from walking import *
from random import random 

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

if __name__ == '__main__':
  robot = Robot()
  timestep = int(robot.getBasicTimeStep())

  motor = [None]*len(motorNames)
  for i in range(len(motorNames)):
    motor[i] = robot.getDevice(motorNames[i])

  joint_angles = [0]*len(motorNames)

  left_foot  = [-0.02, +0.054, 0.02]
  right_foot = [-0.02, -0.054, 0.02]

  pc = preview_control(timestep/1000, 1.0, 0.27)
  walk = walking(timestep/1000, motorNames, left_foot, right_foot, joint_angles, pc)

  #goal position (x, y) theta
#  foot_step = walk.setGoalPos([0.0*1.5, 0.0*1.3, 3.1*1.6])
  foot_step = walk.setGoalPos([0.4, 0.0, 0.5])
  step = 0
  while robot.step(timestep) != -1:
    joint_angles,lf,rf,xp,n = walk.getNextPos()
    step = 0
    if n == 0:
      if (len(foot_step) <= 5):
        x_goal, y_goal, th = (random()-0.5)*2, random()-0.5, (random()-0.5)*4
        print("Goal: ("+str(x_goal)+", "+str(y_goal)+", "+str(th)+")")
        foot_step = walk.setGoalPos([x_goal, y_goal, th])
      else:
        foot_step = walk.setGoalPos()
      #if you want new goal, please send position
    for i in range(len(motorNames)):
      motor[i].setPosition(joint_angles[i])
    pass

