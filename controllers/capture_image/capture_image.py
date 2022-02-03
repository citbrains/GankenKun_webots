"""Sample Webots controller for the visual tracking benchmark."""

from controller import Robot
import os
import sys

fileName = 'camera.jpg'
if len(sys.argv) > 1:
    fileName = sys.argv[1]

deviceImagePath = os.getcwd()
robot = Robot()
timestep = int(robot.getBasicTimeStep())
camera = robot.getDevice('camera_sensor')
camera.enable(timestep)

while robot.step(timestep) != -1:
    camera.saveImage(deviceImagePath + '/images/' + fileName, 80)

