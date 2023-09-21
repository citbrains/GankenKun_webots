#!/usr/bin/env python3

import sys
from controller import Robot
sys.path.append('./GankenKun')
from kinematics import *
from foot_step_planner import *
from preview_control import *
from walking import *
import csv
import math

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
  "head_yaw_joint"                        # ID19
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

  file_name = "./right_kick.csv"
  csv_file = open(file_name, "r")
  f = csv.reader(csv_file, delimiter=",", lineterminator="\r\n")
  motion = [row for row in f]
  t = 0.0
  tm = 0.0
  index = -1
  angles = [0.0] * len(motor)
  delta_angle = [0.0] * len(motor)
  direction = [-1, -1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1]

  #while True:
  #  t = 0.0   # elapsed simulation time
  #  tm = 0.0
  #  index = -1
  #  angles = [0.0] * len(motor)
  #  delta_angles = [0.0] * len(motor)

  #  while robot.step(timestep) != -1:
  #      if t >= tm:
  #          index += 1
  #          if index >= len(motion):
  #              break
  #          delta_angles = [(float(motion[index][i+1]) - angles[i])/(float(motion[index][0])*0.008) for i in range(len(motor))]
  #          tm += float(motion[index][0]) * 0.008
  #      for i in range(len(motor)):
  #          angles[i] += delta_angles[i] * 0.008
  #      [m.setPosition(math.radians(direction[i] * float(angles[i]))) for m, i in zip(motor, range(len(motor)))]
  #      t += timestep / 1000.0
  #  break

  #while robot.step(timestep) != -1:
  #  if t >= tm:
  #    index += 1
  #    print(index)
  #    if index >= 14:
  #      break
  #    delta_angles = [math.radians((float(motion[index][i+1]) - angles[i])/(float(motion[index][0])*0.008))*direction[i] for i in range(len(motor))]
  #    tm += float(motion[index][0]) * 0.008
  #  for i in range(len(motor)):
  #    joint_angles[i] += delta_angles[i] * 0.008
  #  t += timestep / 1000.0

  while robot.step(timestep) != -1:
    joint_angles,lf,rf,xp,n = walk.getNextPos()
    if n == 0:
      if receiver.getQueueLength() > 0:
        received_message = ""
        while receiver.getQueueLength() > 0:
          received_message = receiver.getData().decode('utf-8')
          receiver.nextPacket()
        message_parts = received_message.split(',')
        if message_parts[0] == "walk":
          x_goal = foot_step[0][1] + float(message_parts[1])
          y_goal = foot_step[0][2] - foot_step[0][5] + float(message_parts[2])
          th_goal = foot_step[0][3] - float(message_parts[3])
        foot_step = walk.setGoalPos([x_goal, y_goal, th_goal])
      else:
        foot_step = walk.setGoalPos()
    for i in range(len(motorNames)):
      motor[i].setPosition(joint_angles[i])
