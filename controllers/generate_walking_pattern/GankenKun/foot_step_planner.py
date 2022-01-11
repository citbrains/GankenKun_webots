#!/usr/bin/env python3
# 
# foot step planner

import math

class foot_step_planner():
  def __init__(self, max_stride_x, max_stride_y, max_stride_th, period, width):
    self.max_stride_x = max_stride_x
    self.max_stride_y = max_stride_y
    self.max_stride_th = max_stride_th
    self.period = period
    self.width = width

  def calculate(self, goal_x, goal_y, goal_th, current_x, current_y, current_th, next_support_leg, status):
    # calculate the number of foot step
    time = 0.0
    goal_distance = math.sqrt((goal_x-current_x)**2 + (goal_y-current_y)**2)
    step_x  = abs(goal_x  - current_x )/self.max_stride_x
    step_y  = abs(goal_y  - current_y )/self.max_stride_y
    step_th = abs(goal_th - current_th)/self.max_stride_th
    max_step = max(step_x, step_y, step_th)
    stride_x  = (goal_x  - current_x )/max_step
    stride_y  = (goal_y  - current_y )/max_step
    stride_th = (goal_th - current_th)/max_step

    # first step
    foot_step = []
    if status == 'start':
      foot_step += [[0.0, current_x, current_y, current_th, 'both']]
      time += self.period * 2.0
    if next_support_leg == 'right':
      foot_step += [[time, current_x, -self.width+current_y, current_th, next_support_leg]]
      next_support_leg = 'left'
    elif next_support_leg == 'left':
      foot_step += [[time, current_x,  self.width+current_y, current_th, next_support_leg]]
      next_support_leg = 'right'

    # walking
    counter = 0
    while True:
      counter += 1
      diff_x  = abs(goal_x  - current_x )
      diff_y  = abs(goal_y  - current_y )
      diff_th = abs(goal_th - current_th)
      if diff_x <= self.max_stride_x and diff_y <= self.max_stride_y and diff_th <= self.max_stride_th:
        break
      time += self.period
      next_x  = current_x  + stride_x
      next_y  = current_y  + stride_y
      next_th = current_th + stride_th
      if next_support_leg == 'right':
        foot_step += [[time, next_x, next_y-self.width, next_th, next_support_leg]]
        next_support_leg = 'left'
      elif next_support_leg == 'left':
        foot_step += [[time, next_x, next_y+self.width, next_th, next_support_leg]]
        next_support_leg = 'right'
      current_x, current_y, current_th = next_x, next_y, next_th

    # last step
    next_x, next_y, next_th = goal_x, goal_y, goal_th
    if not status == 'stop':
      time += self.period
      if next_support_leg == 'right':
        foot_step += [[time, next_x, next_y-self.width, next_th, next_support_leg]]
      elif next_support_leg == 'left':
        foot_step += [[time, next_x, next_y+self.width, next_th, next_support_leg]]
      time += self.period
      next_support_leg = 'both'
      foot_step += [[time, next_x, next_y, next_th, next_support_leg]]
      time += 2.0
      foot_step += [[time, next_x, next_y, next_th, next_support_leg]]
      time += 100.0
      foot_step += [[time, next_x, next_y, next_th, next_support_leg]]

    return foot_step

if __name__ == '__main__':
  planner = foot_step_planner(0.06, 0.04, 0.1, 0.34, 0.044)
  foot_step = planner.calculate(1.0, 0.0, 0.5, 0.5, 0.0, 0.1, 'right', 'start')
  for i in foot_step:
    print(i)
