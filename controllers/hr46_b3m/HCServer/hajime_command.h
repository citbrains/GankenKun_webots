/*-------------------------------------------------
	Hajiem Robot no. 18. Command utility

	2007/11/24 Shouhei FUJITA (fujita@brains.co.jp)

-------------------------------------------------*/

#ifndef  _HAJIME_COMMAND_H__
#define  _HAJIME_COMMAND_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef COMMAND_PARAM_4
#define ACTION_WALK      'F'
#define ACTION_BACK      'B'
#define ACTION_TURN      'S'
#define ACTION_SIDESTEP  'D'
#endif
#define ACTION_ALL       'A'
#define ACTION_WALK_NEW  'A'
#define ACTION_STAND     'T'
#define ACTION_MOTION    'M'

#define ACTION_HEAD_PAN  'H'
#define ACTION_HEAD_TILT 'h'
#define ACTION_HEAD_PANTILT 'N'

#define ACTION_CANCEL     'C'

#define MOTOR_TORQUE      'P'
#define GYRO_FEEDBACK     'J'
#define GYRO_CALIBRARION  'j'
#define STATE_READ        'R'
#define STATE_READ_VOL    'r'
#define ACCURATE_ONE_STEP 'O'
#define ACCURATE_WALK     'U'
#define VARIABLE_MOTION   'V'
#define STATE_READ_QUATERNION 'Q'

// Parameters for Command

// Parameter limit
#define PARAMETER_MAX    26
#define PARAMETER_MIN    -26
#define HEAD_PAN_MAX     135
#define HEAD_PAN_MIN     -135
#define HEAD_TILT_MAX    50
#define HEAD_TILT_MIN    -30


// Motor Power control
#define MOTOR_TORQUE_OFF 0
#define MOTOR_TORQUE_ON  1

// Gyro Feedback control
#define GYRO_FEEDBACK_AUTO   0 // Gyro feedback control ON, Auto
#define GYRO_FEEDBACK_MANUAL 1 // Gyro feedback control ON, Manual
#define GYRO_FEEDBACK_OFF    2 // Gyro feedback control OFF

// Current state
#define STATE_IDLE             0 // Motion Active
#define STATE_ACTIVE           1 // Waitting command
#define STATE_POSTURE_STAND    0 // Standstill
#define STATE_POSTURE_DOWN     1 // Face Down
#define STATE_POSTURE_UP       2 // Face Up
#define STATE_POSTURE_SIDE_DOWN 3// Side Down
#define STATE_POSTURE_SIDE_UP  4// Side Up
#define STATE_CPU_VOLTAGE_OK   0 // CPU Board power supply OK
#define STATE_CPU_VOLTAGE_NG   1 // CPU Board power supply NG
#define STATE_MOTOR_VOLTAGE_OK 0 // Motor power supply OK
#define STATE_MOTOR_VOLTAGE_NG 1 // Motor power supply NG


// RobotStatusdefine 
#ifdef COMMAND_PARAM_4
#define STATE_RESPONSE_LEN		5
#else
#define STATE_RESPONSE_LEN		6
#endif

#ifdef COMMAND_PARAM_4
#define STATE_STATUS_LEN		77
#else
#define STATE_STATUS_LEN		76
#endif

#ifdef COMMAND_PARAM_4
#define STATE_STATUS_VOLT_LEN	8
#else
#define STATE_STATUS_VOLT_LEN	(11+16*4)
#endif

#define STATE_TEMPARATURE_LEN	54
#define STATE_MOTOR_NUM			16
#define STATE_MOTION_NUM        20
#define STATE_ODOMETRY_NUM		3

#define RESPONSE_READ_ERR_NOERROR  0
#define RESPONSE_READ_ERR_NODATA  -1
#define RESPONSE_READ_ERR_TIMEOUT -2

#define TEMPERATURE_ARRAY_LEN 24

typedef struct RobotStatus_t{
	int   type;
	short Active;
	char  Command;
	short Posture;
	short Temperature;
	short servo1temp;
	short MotorVoltage;
	short Odometry[3];
	float gyroMotion; //rad
	short Servo[STATE_MOTOR_NUM];
	float servo_rad[STATE_MOTOR_NUM];
	float quaternion[4];
} RobotStatus;

#define SERVO_1  0
#define SERVO_2  1
#define SERVO_3  2
#define SERVO_4  3
#define SERVO_5  4
#define SERVO_6  5
#define SERVO_7  6
#define SERVO_8  7
#define SERVO_9  8
#define SERVO_10 9
#define SERVO_11 10
#define SERVO_12 11
#define SERVO_13 12
#define SERVO_14 13
#define SERVO_HEAD_Y 14
#define SERVO_HEAD_P 15

int WalkCommandOLD      (char *command, int step_num, int step_angle, int step_width, int step_time );
int WalkCommand         (char *command, int step_num, int step_angle, int step_width, int step_time, int side_step_num );
int TurnCommand         (char *command, int step_num, int turn_angle, int step_time);
int SideStepCommand     (char *command, int step_num, int steo_angle, int step_width, int step_time);
int StandCommand        (char *command);
int MotionCommand       (char *command, int motion_num, int loop_count);
int HeadPanCommand      (char *command, int pan_angle, int time);
int HeadTiltCommand     (char *command, int tilt_angle, int time);
void HeadPantTiltCommand(char *command,int pan,int tilt,int time);
int CancelCommand       (char *command);
int AccurateOneStep     (char *command, int x, int y, int th);          //x[cm], y[cm], th[2deg]
int AccurateWalk        (char *command, int num, int x, int y, int th);
int VariableMotionCommand(char *command, int motion_num, int shift);

int MotorTorqueCommand  (char *command, int ctrl);
int GyroFeedbackCommand (char *command, int ctrl);
int GyroCalibrationCommand (char *command);
int StatusReadCommand   (char *command);
int StatusReadQuaternionCommand(char *command);
int StatusReadVolCommand(char *command);
int ReadTemperatureCommand(char *command);

int ParseRobotStatusResponse (RobotStatus *status, const char *res, int len);
int ParseRobotStatusQuaternionResponse(RobotStatus *status, const char *res, int len);
int ParseRobotStatusVoltResponse (RobotStatus *status, const char *res, int len);
int ParseTemperatureResponse(int temp[TEMPERATURE_ARRAY_LEN], const char *res, int len);

#ifdef __cplusplus
};
#endif

#endif //_HAJIME_COMMAND_H__
