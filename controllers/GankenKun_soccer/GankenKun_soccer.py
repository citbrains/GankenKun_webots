#!/usr/bin/env python3

import sys
from controller import Robot
sys.path.append('./GankenKun')
from kinematics import *
from foot_step_planner import *
from preview_control import *
from walking import *
from play_motion import *

motorNames = [
  "right_ankle_roll_joint",                # ID1
  "right_ankle_pitch_joint",               # ID2
  "right_knee_pitch_joint",                # ID3
  "right_waist_pitch_joint",               # ID4
  "right_waist_roll_joint [hip]",          # ID5
  "right_waist_yaw_joint",                 # ID6
  "right_shoulder_pitch_joint [shoulder]", # ID7
  "right_shoulder_roll_joint",             # ID8
  "right_elbow_pitch_joint",               # ID9
  "left_ankle_roll_joint",                 # ID10
  "left_ankle_pitch_joint",                # ID11
  "left_knee_pitch_joint",                 # ID12
  "left_waist_pitch_joint",                # ID13
  "left_waist_roll_joint [hip]",           # ID14
  "left_waist_yaw_joint",                  # ID15
  "left_shoulder_pitch_joint [shoulder]",  # ID16
  "left_shoulder_roll_joint",              # ID17
  "left_elbow_pitch_joint",                # ID18
  "head_yaw_joint"                         # ID19
]

if __name__ == '__main__':
  robot = Robot()
  timestep = int(robot.getBasicTimeStep())
  receiver = robot.getDevice("receiver")
  receiver.enable(timestep)

  motor = [None]*len(motorNames)
  for i in range(len(motorNames)):
    motor[i] = robot.getDevice(motorNames[i])

  joint_angles = [0]*len(motorNames)

  left_foot  = [-0.02, +0.054, 0.02]
  right_foot = [-0.02, -0.054, 0.02]

  pc = preview_control(timestep/1000, 1.0, 0.27)
  walk = walking(timestep/1000, motorNames, left_foot, right_foot, joint_angles, pc)
  foot_step = walk.setGoalPos([0.0, 0.0, 0.0])
  pm = play_motion(timestep/1000, motorNames)

  next_mode = ""
  mode = "walk"
  new_walk_command = False
  prev_foot_step_num = 0
  is_first = False
  while robot.step(timestep) != -1:
    if receiver.getQueueLength() > 0:
      while receiver.getQueueLength() > 0:
        received_message = receiver.getData().decode('utf-8')
        receiver.nextPacket()
      message_parts = received_message.split(',')
      next_mode = message_parts[0]
      if message_parts[0] == "walk":
        new_walk_command = True

    if mode == "":
      mode = next_mode
      next_mode = ""
      is_first = True
    else:
      is_first = False

    if mode == "walk":
      joint_angles,lf,rf,xp,n = walk.getNextPos()
      if n == 0:
        if new_walk_command:
          x_goal = foot_step[0][1] + float(message_parts[1])
          y_goal = foot_step[0][2] - foot_step[0][5] + float(message_parts[2])
          th_goal = foot_step[0][3] - float(message_parts[3])
          foot_step = walk.setGoalPos([x_goal, y_goal, th_goal])
          new_walk_command = False
        else:
          foot_step = walk.setGoalPos()
          if prev_foot_step_num == 3 and len(foot_step) == 3:
            mode = ""
        prev_foot_step_num = len(foot_step)

    elif mode == "motion":
      if is_first:
        pm.setMotionFile(f"./{message_parts[1]}.csv")
      joint_angles = pm.getNextPos()
      if joint_angles == None:
        mode = ""
        continue

    if joint_angles != None:
      for i in range(len(motorNames)):
        motor[i].setPosition(joint_angles[i])
