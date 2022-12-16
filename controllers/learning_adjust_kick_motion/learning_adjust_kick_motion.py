import warnings
warnings.filterwarnings('ignore')
from controller import Supervisor, Node
import time
import math
import csv
import sys
import numpy as np
import gym
from stable_baselines import SAC
from stable_baselines.sac.policies import MlpPolicy

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
                30, 
                30, 
                30, 
                30, 
                30,                
                30, 
                30, 
                30, 
                30, 
                30,
                30, 
                30, 
                30, 
                30, 
                30, 
                30, 
                30, 
                30, 
                30, 
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
                1.0,
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
        self.action_spaces = gym.spaces.Box(-action_space_high, action_space_high, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-observation_space_high, observation_space_high, dtype=np.float32)
        self.state = np.array([
                                0, 0, 0, # 加速度 x, y, x
                                0, 0, 0, # ジャイロ x, y, z
                                0, 0, 0, 0, # エージェントの姿勢 x, y, z, deg
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  # 各関節の目標関節角度 
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  # 各関節の現在の関節角度 
                                ], dtype=np.float32)
        self.reward_range = (0, 1)
        self.spec = gym.envs.registration.EnvSpec(id='GankenKun_motion_fix_Env-v0', max_episode_steps=max_episode_steps)
        
        # Webots_environmental_preference
        self.__timestep = int(self.getBasicTimeStep()) #シミュレータ内部の仮想時間 10mm sec = 0.01sec
        self.__motors = []
        self.__motor_angle_sensors = []
        self.__robot_body_solids = []        
        
                
    def reset(self):
        self.simulationResetPhysics()
        self.simulationReset()
        self.player = self.getFromDef('PLAYER')
        self.player_rotation = self.getFromDef('PLAYER').getField('rotation')
        OpenAIGymEnvironment.append_solid(self.player, self.__robot_body_solids )
        
        self.__target_joint_angle_data = [0.0] * len(MOTOR_SENSOR_NAMES)
        self.__motor_angle_sensors_data = [0.0] * len(MOTOR_SENSOR_NAMES)
        
        #motor 
        self.__motors = []
        for name in MOTOR_NAMES:
            self.__motors.append(self.getDevice(name))
        
        #motor sensor
        self.__motor_angle_sensors = []
        for name in MOTOR_SENSOR_NAMES:
            self.__motor_angle_sensors.append(self.getDevice(name)) 
        
        for name in range (len(MOTOR_SENSOR_NAMES)):
            self.__motor_angle_sensors[name].enable(self.__timestep)
        #accelerometer
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(self.__timestep)
        
        #gyro
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(self.__timestep)
                    
        # motion parameter
        self.t = 0.0
        self.tm = 0.0
        self.motion_total_time = 0.0
        self.angles = [0.0] * len(MOTOR_NAMES)                  #モータの目標角度
        self.delta_angles = [0.0] * len(MOTOR_NAMES)     #モーション再生中の補間角度
        
        return np.array(self.state, dtype=np.float32) # return 初期状態
        
    
    def append_solid(solid, solids):  # we list only the hands and feet
        if solid.getField('name'):
            solids.append(solid)
        children = solid.getProtoField('children') if solid.isProto() else solid.getField('children')
        for i in range(children.getCount()):
            child = children.getMFNode(i)
            if child.getType() in [Node.ROBOT, Node.SOLID, Node.GROUP, Node.TRANSFORM, Node.ACCELEROMETER, Node.CAMERA, Node.GYRO, Node.TOUCH_SENSOR]:
                OpenAIGymEnvironment.append_solid(child, solids)
                continue
            if child.getType() in [Node.HINGE_JOINT, Node.HINGE_2_JOINT, Node.SLIDER_JOINT, Node.BALL_JOINT]:
                endPoint = child.getProtoField('endPoint') if child.isProto() else child.getField('endPoint')
                solid = endPoint.getSFNode()
                if solid.getType() == Node.NO_NODE or solid.getType() == Node.SOLID_REFERENCE:
                    continue
                OpenAIGymEnvironment.append_solid(solid, solids)  # active tag is reset after a joint

    def read_motion_file(self):
        file_name = "./leg_up_motion.csv"  #正常のキック動作を再生する場合は有効化する
        # file_name = "./falldown_kick.csv"   # キック後に転倒する動作を再生する場合は有効化する
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
    
    def step(self, data, ):
        self.tm += float(data[0]) * 0.01 # tm = モーションファイルの書くフレームごとの時間 ÷ webotsのtime step[sec]
        motor_angle_sensor_data = [0]*len(MOTOR_SENSOR_NAMES)
        motor_target_angle_data = data[1:20]
        frame_end_flag = False
        
        while True:
            if frame_end_flag == True:
                break
            
            while super().step(self.__timestep) != -1:
                if self.t >= self.tm:
                    frame_end_flag = True #1フレーム終了のフラグ
                    
                    # Observation
                    acc_data = self.accelerometer.getValues()    #取得情報は対象のx, y, zの加速度 単位は[]
                    acc_x, acc_y, acc_z= list(map(OpenAIGymEnvironment.Normalization, acc_data))
                    gyro_data = self.gyro.getValues()                    #取得情報は対象のx, y, zのジャイロ 単位は[] 
                    gyro_x, gyro_y, gyro_z = list(map(OpenAIGymEnvironment.Normalization, gyro_data))
                    rot_x, rot_y, rot_z, rot_deg = self.player_rotation.getSFRotation()     #取得情報は対象の姿勢 x, y, z, deg 軸角度表現で表せる  単位は[] 
                    for i in range(len(MOTOR_NAMES)):
                        motor_angle_sensor_data[i] = (math.degrees(self.__motor_angle_sensors[i].getValue() * DIRECTION[i] ))
                    self.state = [acc_x, acc_y, acc_z, gyro_x,gyro_y, gyro_z, rot_x, rot_y, rot_z, rot_deg]
                    self.state += motor_target_angle_data
                    self.state += motor_angle_sensor_data

                    if abs(rot_deg) >= 1.0: #各軸度表現の角度を利用して転倒判定を行う
                            self.falldown_flag = True

                    # # done 
                    if self.t >= self.motion_total_time:
                        done = True
                    else:
                        done = False
                        reward = 0
                        
                    if done:
                        if self.falldown_flag == True:
                            reward = 1
                        else:
                            reward = 0
                        
                    
                    
                    break
                else:
                    self.t += self.__timestep / 1000.0
                    try:
                        for i in range(len(MOTOR_NAMES)):
                            self.delta_angles[i] = (float(data[i+1]) - self.angles[i]) / (float(data[0])*0.01)
                    except ZeroDivisionError:
                        print("ZeroDivisionError")
                        print("The frame of the motion file contains 0.")
                        break
                    for i in range(len(MOTOR_NAMES)):
                        self.angles[i] += self.delta_angles[i] * 0.01
                    [motor.setPosition(math.radians(DIRECTION[i] * float(self.angles[i]))) for motor, i in zip(self.__motors, range(len(MOTOR_NAMES)))]
                    
                    
                    info = {} #使用しない
                    
                    return self.state, reward, done, info  # return  1step後の状態，即時報酬，正常終了したかどうかの判定，情報
                    

def main():
    env = OpenAIGymEnvironment()
    model = SAC(MlpPolicy, env, verbose=1)
    model.learn(total_timesteps=50000, log_interval=10)
    
    for episode in range(2):
        print("------------------")
        print("{}episode...".format(episode))
        state = env.reset()
        data = env.read_motion_file()
        
        for i in range(len(data)):
            action, _ = model.predict(state)
            # action = env.action_space.sample()
            state, rewards, done, info = env.step(action)
            if done:
                break
    env.close()
    
if __name__ == "__main__":
    main()