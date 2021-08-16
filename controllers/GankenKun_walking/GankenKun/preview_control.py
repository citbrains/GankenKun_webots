#!/usr/bin/env python3
# 
# preview control

import math
import numpy as np
import control
import control.matlab
import csv

class preview_control():
  def __init__(self, dt, period, z, Q = 1.0e+8, H = 1.0):
    self.dt = dt
    self.period = period
    G = 9.8
    A = np.matrix([
      [0.0, 1.0, 0.0],
      [0.0, 0.0, 1.0],
      [0.0, 0.0, 0.0]])
    B = np.matrix([[0.0], [0.0], [1.0]])
    C = np.matrix([[1.0, 0.0, -z/G]])
    D = 0
    sys = control.matlab.ss(A, B, C, D)
    sys_d = control.c2d(sys, dt)
    self.A_d, self.B_d, self.C_d, D_d = control.matlab.ssdata(sys_d)
    E_d = np.matrix([[dt], [1.0], [0.0]])
    Zero = np.matrix([[0.0], [0.0], [0.0]])
    Phai = np.block([[1.0, -self.C_d * self.A_d], [Zero, self.A_d]])
    G = np.block([[-self.C_d*self.B_d], [self.B_d]])
    GR = np.block([[1.0], [Zero]])
    Gd = np.block([[-self.C_d*E_d], [E_d]])
    Qm = np.zeros((4,4))
    Qm[0][0] = Q
    P = control.dare(Phai, G, Qm, H)[0]
    self.F = -np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*P*Phai
    xi = (np.eye(4)-G*np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*P)*Phai;
    self.f = []
    self.xp, self.yp = np.matrix([[0.0],[0.0],[0.0]]), np.matrix([[0.0],[0.0],[0.0]])
    self.ux, self.uy = 0.0, 0.0
    for i in range(0,round(period/dt)):
      self.f += [-np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*np.linalg.matrix_power(xi.transpose(),i-1)*P*GR]

  def set_param(self, t, current_x, current_y, foot_plan, pre_reset = False):
    x, y = current_x.copy(), current_y.copy()
    if pre_reset == True:
      self.xp, self.yp = x.copy(), y.copy()
      self.ux, self.uy = 0.0, 0.0
    COG_X = []
    for i in range(round((foot_plan[1][0] - t)/self.dt)):
      px, py = self.C_d * x, self.C_d * y
      ex, ey = foot_plan[0][1] - px, foot_plan[0][2] - py
      X, Y = np.block([[ex], [x - self.xp]]), np.block([[ey], [y - self.yp]])
      self.xp, self.yp = x.copy(), y.copy()
      dux, duy = self.F * X, self.F * Y
      index = 1
      for j in range(1,round(self.period/self.dt)-1):
        if round((i+j)+t/self.dt) >= round(foot_plan[index][0]/self.dt):
          dux += self.f[j] * (foot_plan[index][1]-foot_plan[index-1][1])
          duy += self.f[j] * (foot_plan[index][2]-foot_plan[index-1][2])
          index += 1
      self.ux, self.uy = self.ux + dux, self.uy + duy
      x, y = self.A_d * x + self.B_d * self.ux, self.A_d * y + self.B_d * self.uy
      COG_X += [np.block([x[0][0], y[0][0], px[0][0], py[0][0]])]
    return COG_X, x, y

if __name__ == '__main__':
  pc = preview_control(0.01, 1.0, 0.27)
  foot_step = [[0, 0, 0], [0.34, 0, 0.06+0.00], [0.68, 0.05, -0.06+0.02], [1.02, 0.10, 0.06+0.04], [1.36, 0.15, -0.06+0.06], [1.7, 0.20, 0.06+0.08], [2.04, 0.25, 0.0+0.10], [2.72, 0.25, 0.0+0.10], [100, 0.25, 0.0+0.10]]
  x, y = np.matrix([[0.0],[0.0],[0.0]]), np.matrix([[0.0],[0.0],[0.0]])
  with open('result.csv', mode='w') as f:
    f.write('')
  t = 0
  while True:
    if len(foot_step) <= 2:
      break
    cog, x, y = pc.set_param(t, x, y, foot_step)
    with open('result.csv', mode='a') as f:
      writer = csv.writer(f)
      for i in cog:
        writer.writerow(i.tolist()[0])
      f.write('\n')
    del foot_step[0]
    t = foot_step[0][0]
#  print(pc.set_param([0,0], [0,0], [0,0], foot_step))
