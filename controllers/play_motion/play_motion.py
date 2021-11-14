from controller import Robot, Motor
import csv
import math
import sys

file_name = "./motion.csv"
if len(sys.argv) > 1:
    file_name = sys.argv[1]
csv_file = open(file_name, "r")
f = csv.reader(csv_file, delimiter=",", lineterminator="\r\n")
data = [row for row in f]

TIME_STEP = 8

motorNames = [
  "right_ankle_roll_joint", # ID19
  "right_ankle_pitch_joint", # ID18
  "right_knee_pitch_joint", # ID17
  "right_waist_pitch_joint", # ID16
  "right_waist_roll_joint [hip]", # ID15
  "right_waist_yaw_joint", # ID14
  "right_shoulder_pitch_joint [shoulder]", # ID5
  "right_shoulder_roll_joint", # ID6
  "right_elbow_pitch_joint", # ID7
  "left_ankle_roll_joint", # ID13
  "left_ankle_pitch_joint", # ID12
  "left_knee_pitch_joint", # ID11
  "left_waist_pitch_joint", # ID10
  "left_waist_roll_joint [hip]", # ID9
  "left_waist_yaw_joint", # ID8
  "left_shoulder_pitch_joint [shoulder]", # ID2
  "left_shoulder_roll_joint", # ID3
  "left_elbow_pitch_joint", # ID4
  "head_yaw_joint" # ID1
]
direction = [-1, -1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1]

robot = Robot()

motors = [robot.getDevice(motorName) for motorName in motorNames]

while True:
    t = 0.0   # elapsed simulation time
    tm = 0.0
    index = -1
    angles = [0.0] * len(motors)
    delta_angles = [0.0] * len(motors)

    while robot.step(TIME_STEP) != -1:
        if t >= tm:
            index += 1
            if index >= len(data):
                break
            delta_angles = [(float(data[index][i+1]) - angles[i])/(float(data[index][0])*0.008) for i in range(len(motors))]
            tm += float(data[index][0]) * 0.008
        for i in range(len(motors)):
            angles[i] += delta_angles[i] * 0.008
        [motor.setPosition(math.radians(direction[i] * float(angles[i]))) for motor, i in zip(motors, range(len(motors)))]
        t += TIME_STEP / 1000.0
    break
