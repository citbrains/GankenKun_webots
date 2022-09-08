from controller import Supervisor
import numpy as np

supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

accelerometer = supervisor.getDevice('accelerometer')
gyro = supervisor.getDevice('gyro')
accelerometer.enable(time_step)
gyro.enable(time_step)

while supervisor.step(time_step) != -1:
    acc_data = accelerometer.getValues()
    gyro_data = gyro.getValues()
    print("acc_data = {}".format(acc_data))
    print("gyro_data = {}".format(gyro_data))
    