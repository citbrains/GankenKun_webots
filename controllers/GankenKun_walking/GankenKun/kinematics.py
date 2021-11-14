#!/usr/bin/env python3
# 
# solving kinematics for the GankenKun

import math

class kinematics():
  L1  = 0.118
  L12 = 0.023
  L2  = 0.118
  L3  = 0.043
  OFFSET_W   = 0.044
  OFFSET_X   = -0.0275
  def __init__(self, motorNames):
    self.motorNames = motorNames

  def solve_ik(self, left_foot, right_foot, current_angles):
    joint_angles = current_angles.copy()
    l_x, l_y, l_z, l_roll, l_pitch, l_yaw = left_foot
    l_x -= self.OFFSET_X
    l_y -= self.OFFSET_W
    l_z = self.L1 + self.L12 + self.L2 + self.L3 - l_z
    l_x2 =  l_x * math.cos(l_yaw) + l_y * math.sin(l_yaw)
    l_y2 = -l_x * math.sin(l_yaw) + l_y * math.cos(l_yaw)
    l_z2 =  l_z - self.L3
    waist_roll = math.atan2(l_y2, l_z2)
    l2 = l_y2**2 + l_z2**2
    l_z3 = math.sqrt(max(l2 - l_x2**2, 0.0)) - self.L12
    pitch = math.atan2(l_x2, l_z3)
    l = math.sqrt(l_x2**2 + l_z3**2)
    knee_disp = math.acos(min(max(l/(2.0*self.L1),-1.0),1.0))
    waist_pitch = - pitch - knee_disp
    knee_pitch  = - pitch + knee_disp
    joint_angles[self.motorNames.index('left_waist_yaw_joint'       )] = -l_yaw
    joint_angles[self.motorNames.index('left_waist_roll_joint [hip]')] = waist_roll
    joint_angles[self.motorNames.index('left_waist_pitch_joint'     )] = -waist_pitch
    joint_angles[self.motorNames.index('left_knee_pitch_joint'      )] = -knee_pitch
    joint_angles[self.motorNames.index('left_ankle_pitch_joint'     )] = l_pitch
    joint_angles[self.motorNames.index('left_ankle_roll_joint'      )] = l_roll - waist_roll

    r_x, r_y, r_z, r_roll, r_pitch, r_yaw = right_foot
    r_x -= self.OFFSET_X
    r_y += self.OFFSET_W
    r_z = self.L1 + self.L12 + self.L2 + self.L3 - r_z
    r_x2 =  r_x * math.cos(r_yaw) + r_y * math.sin(r_yaw)
    r_y2 = -r_x * math.sin(r_yaw) + r_y * math.cos(r_yaw)
    r_z2 =  r_z - self.L3
    waist_roll = math.atan2(r_y2, r_z2)
    r2 = r_y2**2 + r_z2**2
    r_z3 = math.sqrt(max(r2 - r_x2**2, 0.0)) - self.L12
    pitch = math.atan2(r_x2, r_z3)
    l = math.sqrt(r_x2**2 + r_z3**2)
    knee_disp = math.acos(min(max(l/(2.0*self.L1),-1.0),1.0))
    waist_pitch = - pitch - knee_disp
    knee_pitch  = - pitch + knee_disp
    joint_angles[self.motorNames.index('right_waist_yaw_joint'       )] = -r_yaw
    joint_angles[self.motorNames.index('right_waist_roll_joint [hip]')] = waist_roll
    joint_angles[self.motorNames.index('right_waist_pitch_joint'     )] = -waist_pitch
    joint_angles[self.motorNames.index('right_knee_pitch_joint'      )] = -knee_pitch
    joint_angles[self.motorNames.index('right_ankle_pitch_joint'     )] = r_pitch
    joint_angles[self.motorNames.index('right_ankle_roll_joint'      )] = r_roll - waist_roll

    return joint_angles

if __name__ == '__main__':
  TIME_STEP = 0.001
  physicsClient = p.connect(p.GUI)
  p.setGravity(0, 0, -9.8)
  p.setTimeStep(TIME_STEP)

  planeId = p.loadURDF("../URDF/plane.urdf", [0, 0, 0])
  RobotId = p.loadURDF("../URDF/gankenkun.urdf", [0, 0, 0])
  kine = kinematics(RobotId)

  index = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
  for id in range(p.getNumJoints(RobotId)):
    index[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = id

  left_foot_pos0,  left_foot_ori0  = p.getLinkState(RobotId, index['left_foot_link' ])[:2]
  right_foot_pos0, right_foot_ori0 = p.getLinkState(RobotId, index['right_foot_link'])[:2]

  index_dof = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
  for id in range(p.getNumJoints(RobotId)):
    index_dof[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = p.getJointInfo(RobotId, id)[3] - 7

  joint_angles = []
  for id in range(p.getNumJoints(RobotId)):
    if p.getJointInfo(RobotId, id)[3] > -1:
      joint_angles += [0,]

  height = 0.0
  velocity = 0.1
  while p.isConnected():
    height += velocity * TIME_STEP
    body_pos, body_ori = p.getLinkState(RobotId, index['body_link'])[:2]
    tar_left_foot_pos  = [left_foot_pos0[0] , left_foot_pos0[1] , height, 0.0, 0.0, 0.0]
    tar_right_foot_pos = [right_foot_pos0[0], right_foot_pos0[1], height, 0.0, 0.0, 0.0]
    joint_angles = kine.solve_ik(tar_left_foot_pos, tar_right_foot_pos, joint_angles)

    for id in range(p.getNumJoints(RobotId)):
      qIndex = p.getJointInfo(RobotId, id)[3]
      if qIndex > -1:
        p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-7])

    if height <= 0.0 or height >= 0.1:
      velocity *= -1.0

    p.stepSimulation()
#    sleep(TIME_STEP)

