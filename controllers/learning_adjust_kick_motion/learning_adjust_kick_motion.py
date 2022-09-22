from email import header
from controller import Supervisor
import math
import csv
import math
import sys
import numpy as np


motor_names = [
    "right_ankle_roll_joint",                       # ID1
    "right_ankle_pitch_joint",                      # ID2
    "right_knee_pitch_joint",                       # ID3
    "right_waist_pitch_joint",                      # ID4
    "right_waist_roll_joint [hip]",                 # ID5
    "right_waist_yaw_joint",                        # ID6
    "right_shoulder_pitch_joint [shoulder]",        # ID7
    "right_shoulder_roll_joint",                    # ID8
    "right_elbow_pitch_joint",                      # ID9
    "left_ankle_roll_joint",                        # ID10
    "left_ankle_pitch_joint",                       # ID11
    "left_knee_pitch_joint",                        # ID12
    "left_waist_pitch_joint",                       # ID13
    "left_waist_roll_joint [hip]",                  # ID14
    "left_waist_yaw_joint",                         # ID15
    "left_shoulder_pitch_joint [shoulder]",         # ID16
    "left_shoulder_roll_joint",                     # ID17
    "left_elbow_pitch_joint",                       # ID18
    "head_yaw_joint"                                # ID19
]

motor_sensor_names = [
    "right_ankle_roll_joint_sensor",                # ID1
    "right_ankle_pitch_joint_sensor",               # ID2
    "right_knee_pitch_joint_sensor",                # ID3
    "right_waist_pitch_joint_sensor",               # ID4
    "right_waist_roll_joint [hip]_sensor",          # ID5
    "right_waist_yaw_joint_sensor",                 # ID6
    "right_shoulder_pitch_joint [shoulder]_sensor", # ID7
    "right_shoulder_roll_joint_sensor",             # ID8
    "right_elbow_pitch_joint_sensor",               # ID9
    "left_ankle_roll_joint_sensor",                 # ID10
    "left_ankle_pitch_joint_sensor",                # ID11
    "left_knee_pitch_joint_sensor",                 # ID12
    "left_waist_pitch_joint_sensor",                # ID13
    "left_waist_roll_joint [hip]_sensor",           # ID14
    "left_waist_yaw_joint_sensor",                  # ID15
    "left_shoulder_pitch_joint [shoulder]_sensor",  # ID16
    "left_shoulder_roll_joint_sensor",              # ID17
    "left_elbow_pitch_joint_sensor",                # ID18
    "head_yaw_joint_sensor"                         # ID19
]

direction = [-1, -1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1]

def read_file():
    # file_name = "./kick_motion.csv""  # 正常のキック動作を再生する場合は有効化する
    file_name = "./falldown_kick.csv"   # キック後に転倒する動作を再生する場合は有効化する
    if len(sys.argv) > 1:
        file_name = sys.argv[1]
    csv_file = open(file_name, "r")
    f = csv.reader(csv_file, delimiter=",", lineterminator="\r\n")
    data = [row for row in f]
    return data
    

def create_file():
    header = ['acc x', 'acc y', 'acc z', 'gyro x', 'gyro y', 'gyro z']
    with open('sensor_data.csv', 'w', newline='') as df:
        writer = csv.writer(df)
        writer.writerow(header)
        
def update_file(acc_data, gyro_data):
    listdata = [acc_data[0], acc_data[1], acc_data[2], gyro_data[0], gyro_data[1], gyro_data[2]]
    with open('sensor_data.csv', 'a', newline='') as df:
        writer = csv.writer(df)
        writer.writerow(listdata)

def main():
    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())
    data = read_file()
    motors = [supervisor.getDevice(i) for i in motor_names] # センサーと異なり.enableはいらない
    motor_sensors = [supervisor.getDevice(i) for i in motor_sensor_names]
    accelerometer = supervisor.getDevice('accelerometer')
    gyro = supervisor.getDevice('gyro')
    create_file()

    for i in range (len(motor_sensor_names)):
        motor_sensors[i].enable(time_step)

    accelerometer.enable(time_step)
    gyro.enable(time_step)

    while True:
        t = 0.0   # elapsed simulation time
        tm = 0.0
        index = -1
        angles = [0.0] * len(motors)
        delta_angles = [0.0] * len(motors)

        while supervisor.step(time_step) != -1:
            acc_data = accelerometer.getValues()
            gyro_data = gyro.getValues()
            update_file(acc_data, gyro_data)
            print("acc_data = {}".format(acc_data))
            print("gyro_data = {}".format(gyro_data))
            if t >= tm:
                index += 1
                if index >= len(data):
                    break
                try:
                    delta_angles = [(float(data[index][i+1]) - angles[i])/(float(data[index][0])*0.01) for i in range(len(motors))]
                except ZeroDivisionError:
                    break
                tm += float(data[index][0]) * 0.01
            for i in range(len(motors)):
                angles[i] += delta_angles[i] * 0.01
                # motor_sensor_data = math.degrees(motor_sensors[i].getValue())
                # print("ID = {0} {1} : {2} deg". format(i+1, motor_names[i], angles[i]*direction[i]))   
                # print("sensor ID = {0} {1} : {2} deg". format(i+1, motor_sensor_names[i], motor_sensor_data))   
            [motor.setPosition(math.radians(direction[i] * float(angles[i]))) for motor, i in zip(motors, range(len(motors)))]        
            t += time_step / 1000.0
        break
        
if __name__ == "__main__":
    main()