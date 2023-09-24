#!/usr/bin/env python3

from controller import Robot
import csv
import math

class play_motion():
  def __init__(self, dt, motorNames):
    self.direction = [-1, -1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1]
    self.motorNames = motorNames
    self.dt = dt

  def setMotionFile(self, motion_file):
    with open(motion_file, "r") as csv_file:
      f = csv.reader(csv_file, delimiter=",", lineterminator="\r\n")
      self.motion = [row for row in f]

    self.t = 0.0
    self.tm = 0.0
    self.index = -1      
    self.joint_angles = [0]*len(motorNames)
    self.delta_angles = [0]*len(motorNames)

  def getNextPos(self):
    if self.t >= self.tm:
      self.index += 1
      if self.index >= len(self.motion):
        return None
      period = float(self.motion[self.index][0]) * self.dt
      self.tm += period
      next_angle_rad = [math.radians(self.direction[i]*(float(self.motion[self.index][i+1]))) for i in range(len(self.joint_angles))]
      self.delta_angles = [(next_angle_rad[i] - self.joint_angles[i])/period for i in range(len(self.joint_angles))]
    for i in range(len(self.joint_angles)):
      self.joint_angles[i] += self.delta_angles[i] * self.dt
    self.t += self.dt
    return self.joint_angles
    
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

  motor = [None]*len(motorNames)
  for i in range(len(motorNames)):
    motor[i] = robot.getDevice(motorNames[i])

  pm = play_motion(timestep/1000, motorNames)
  while True:
    pm.setMotionFile("./right_kick.csv")
    while robot.step(timestep) != -1:
      joint_angles = pm.getNextPos()
      if joint_angles != None:
        for i in range(len(motorNames)):
          motor[i].setPosition(joint_angles[i])
