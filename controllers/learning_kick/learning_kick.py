"""
このコードは、転倒するキック動作に対して、修正する方策を学習するためのものになります。
右キックのモーションのみに対応し、初期姿勢のランダム化を行うことで複数の転倒パターンを学習します
"""
import warnings
warnings.filterwarnings('ignore')
from controller import Supervisor, Node
import math
import csv
import sys
import os
import numpy as np
import gym
from stable_baselines import SAC
from stable_baselines.sac.policies import MlpPolicy

log_dir = './logs/'
os.makedirs(log_dir, exist_ok=True)

MOTOR_NAMES = [
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

MOTOR_SENSOR_NAMES = [
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

DIRECTION = [-1, -1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1]
SENSOR_DATA_MAX = 65535
SENSOR_DATA_MIN = 0
SC_MAX = 1
SC_MIN = -1
# learning_parameter
LEARNING_TARGET_JOINT = [1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0 ]
LEARNING_TARGET_JOINT_NUM = 6
LEARNING_TARGET_MOTION_FRAME_NUM = 8
LEARNING_ACTION_ANGLE_RANGE = 30

class OpenAIGymEnvironment(Supervisor, gym.Env):
    # max_episode_steps = エピソードの最大長
    def __init__(self, max_episode_steps=1000) :
        super().__init__()
        
        action_space_high = np.array(
            [
                #修正する角度量[deg]
                # right_ankle_roll,
                # left_ankle_roll,
                # right_ankle_pitch, 
                # left_ankle_pitch, 
                # right_waist_roll, 
                # left_waist_roll
                10, 
                10, 
                10, 
                10, 
                10,                
                10, 
            ], dtype=np.float32)
        
        observation_space_high = np.array(
            [
                #加速度 x, y, x
                1.0,
                1.0,
                1.0,
                #ジャイロ x, y, z
                1.0,
                1.0,
                1.0,
                #エージェントの姿勢 x, y, z, deg
                1.0,
                1.0,
                1.0,
                3.0,
                #各関節の目標関節角度 
                # right_ankle_roll,
                # left_ankle_roll,
                # right_ankle_pitch, 
                # left_ankle_pitch, 
                # right_waist_roll, 
                # left_waist_roll
                180.0,
                180.0,
                180.0,
                180.0,
                180.0,
                180.0,
                #各関節の現在の関節角度 ＊対象は上と同じ
                180.0,
                180.0,
                180.0,
                180.0,
                180.0,
                180.0,
            ], dtype=np.float32
        )
        
        # OpenAIGym_environmental_preference 
        self.action_space = gym.spaces.Box(-action_space_high, action_space_high, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-observation_space_high, observation_space_high, dtype=np.float32)
        self.state = np.array([
                                0, 0, 0, # 加速度 x, y, x
                                0, 0, 0, # ジャイロ x, y, z
                                0, 0, 0, 0, # エージェントの姿勢 x, y, z, deg
                                0, 0, 0, 0, 0, 0,  # 各関節の目標関節角度 
                                0, 0, 0, 0, 0, 0   # 各関節の現在の関節角度 
                                ], dtype=np.float32)
        self.spec = gym.envs.registration.EnvSpec(id='GankenKun_motion_fix_Env-v0', max_episode_steps=max_episode_steps)
        # Webots_environmental_preference
        self.__timestep = int(self.getBasicTimeStep()) #シミュレータ内部の仮想時間 10mm sec = 0.01sec
        self.__motors = []
        self.__motor_angle_sensors = []
        
                
    def reset(self):
        self.simulationResetPhysics()
        self.simulationReset()
        self.player = self.getFromDef('PLAYER')
        self.player_rotation = self.getFromDef('PLAYER').getField('rotation')
        
        #motor reset
        self.__motors = []
        for name in MOTOR_NAMES:
            self.__motors.append(self.getDevice(name))
        #motor sensor reset
        self.__motor_angle_sensors = []
        for name in MOTOR_SENSOR_NAMES:
            self.__motor_angle_sensors.append(self.getDevice(name)) 
        for name in range (len(MOTOR_SENSOR_NAMES)):
            self.__motor_angle_sensors[name].enable(self.__timestep)
        #accelerometer reset
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(self.__timestep)
        #gyro sensor reset
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(self.__timestep)
        # motion parameter reset
        self.t = 0.0
        self.tm = 0.0
        self.motion_frame_num = 0
        self.motion_total_time = 0.0
        self.angles = [np.random.uniform(low=-5.0, high=5.0) for _ in range(len(MOTOR_NAMES))] # ロボットの初期姿勢(各関節軸の角度)をランダムにする
        self.delta_angles = [0.0] * len(MOTOR_NAMES)     #モーション再生中の補間角度
        self.data = OpenAIGymEnvironment.read_motion_file(self)        
        # learning_parameter
        self.reward = 0
        
        return np.array(self.state, dtype=np.float32) # return 初期状態

    def read_motion_file(self):
        # file_name = "./leg_up_motion.csv"  #正常のキック動作を再生する場合は有効化する
        file_name = "./falldown_kick.csv"   # キック後に転倒する動作を再生する場合は有効化する
        if len(sys.argv) > 1:
            file_name = sys.argv[1]
        csv_file = open(file_name, "r")
        f = csv.reader(csv_file, delimiter=",", lineterminator="\r\n")
        data = [row for row in f]
        for i in range(len(data)):
            self.motion_total_time += float(data[i][0]) / 100.0 # motionの総再生時間
        
        return data
    
    def Normalization(sensor_data):
        return (sensor_data - SENSOR_DATA_MIN) / (SENSOR_DATA_MAX - SENSOR_DATA_MIN) * (SC_MAX - SC_MIN) + SC_MIN
    
    def step(self, action):
        motion_frame_data = self.data[self.motion_frame_num][0] #motion fileのdata(フレーム間の時間)の中身を1フレームずつに分解
        motor_target_angle_data = self.data[self.motion_frame_num][1:20]#motion fileのdata(フレーム間の各関節角度)の中身を1フレームずつに分解
        action_space_target_num = 0 #action_spaceの
        for i in range(len(MOTOR_NAMES)): # action_spaceの対象の修正角度の追加
            if LEARNING_TARGET_JOINT[i] == 1:
                motor_target_angle_data[i] = float(motor_target_angle_data[i]) + action[action_space_target_num]
                action_space_target_num += 1
                
        self.tm += float(self.data[self.motion_frame_num][0]) * 0.01 # tm = モーションファイルの書くフレームごとの時間 ÷ webotsのtime step[sec]
        motor_angle_sensor_data = [] # motor内部のセンサにより取得した現在の角度情報
        observe_angle_data = [] # motorの目標関節角度
        frame_end_flag = False
        
        while True:
            if frame_end_flag == True:
                break
            
            while super().step(self.__timestep) != -1:
                
                if self.t >= self.tm:
                    frame_end_flag = True #1フレーム終了のフラグ
                    self.motion_frame_num += 1
                    # Observation
                    acc_data = self.accelerometer.getValues()    #取得情報は対象のx, y, zの加速度 単位は[]
                    acc_x, acc_y, acc_z= list(map(OpenAIGymEnvironment.Normalization, acc_data)) # 加速度を正規化
                    gyro_data = self.gyro.getValues()                    #取得情報は対象のx, y, zのジャイロ 単位は[] 
                    gyro_x, gyro_y, gyro_z = list(map(OpenAIGymEnvironment.Normalization, gyro_data)) # ジャイロを正規化
                    rot_x, rot_y, rot_z, rot_deg = self.player_rotation.getSFRotation()     #取得情報は対象の姿勢 x, y, z, deg 軸角度表現で表せる  単位は[] 
                    for i in range(len(MOTOR_NAMES)):
                        if LEARNING_TARGET_JOINT[i] == 1:
                            motor_angle_sensor_data.append(math.degrees(self.__motor_angle_sensors[i].getValue() * DIRECTION[i] ))
                            observe_angle_data.append(motor_target_angle_data[i])
                    self.state = [acc_x, acc_y, acc_z, gyro_x,gyro_y, gyro_z, rot_x, rot_y, rot_z, rot_deg]
                    self.state += observe_angle_data
                    self.state += motor_angle_sensor_data

                    if abs(rot_deg) >= 1.0: #各軸度表現の角度を利用して転倒判定を行う
                        self.reward -= 1
                    else:
                        self.reward += 1
                    # # done 
                    done = bool(self.t >= self.motion_total_time)

                    break
                
                else:
                    self.t += self.__timestep / 1000.0
                    try:
                        for i in range(len(MOTOR_NAMES)):
                            self.delta_angles[i] = (float(motor_target_angle_data[i]) - self.angles[i]) / (float(motion_frame_data) * 0.01)
                    except ZeroDivisionError:
                        print("ZeroDivisionError")
                        print("The frame of the motion file contains 0.")
                        break
                    for i in range(len(MOTOR_NAMES)):
                        self.angles[i] += self.delta_angles[i] * 0.01
                    [motor.setPosition(math.radians(DIRECTION[i] * float(self.angles[i]))) for motor, i in zip(self.__motors, range(len(MOTOR_NAMES)))]
                    
            info = {} #使用しない
            return np.array(self.state, dtype=np.float32), self.reward, done, info  # return  1step後の状態，即時報酬，正常終了したかどうかの判定，情報
                    

def main():
    env = OpenAIGymEnvironment()
    print("environment comp")
    model = SAC(MlpPolicy, env, verbose=1, tensorboard_log=log_dir)
    print("model comp")
    model.learn(total_timesteps=1000000, log_interval=10)
    print("learn comp")
    model.save("sac_motion_fix_model")
    print("model save")
    state = env.reset()
            
    while True:
        action, _ = model.predict(state)
        state, reward, done, info = env.step(action)

        if done:
            print("state = {}".format(state))
            print("reward = {}".format(reward))
            print("done = {}".format(done))
            print("info = {}".format(info))
            state = env.reset()
            break
    env.close()
if __name__ == "__main__":
    main()