/*-------------------------------------------------
	Hajiem Robot no. 18. Command Utility

	2007/11/24 Shouhei FUJITA (fujita@brains.co.jp)

-------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "hajime_command.h"

//disable Visual Studio warning for sprintf
#pragma warning(disable:4996)

#define PARAM_TABLE_OFFSET  26
#define MAX_PARAM           26
#define MIN_PARAM          -26

#define COMMAND_MAGIC                  '*'

#define COMMAND_TYPE_LENGTH            2
#define COMMAND_TYPE_READ_MEM          "10" // TODO
#define COMMAND_TYPE_WRITE_MEM         "20" // TODO
#define COMMAND_TYPE_ACTION            "30"
#define COMMAND_TYPE_MOTION_SEND       "40" // TODO
#define COMMAND_TYPE_MOTION_EXE        "41" // TODO
#define COMMAND_TYPE_MOTION_EXE_EEP    "42" // TODO
#define COMMAND_TYPE_MOTION_SAVE_EEP   "43" // TODO
#define COMMAND_TYPR_SERVO_POWER_CTRL  "44" // TODO
#define COMMAND_TYPR_SERVO_TORQUE_CTRL "45" // TODO
#define COMMAND_TYPR_SERVO_DATA_READ   "46" // TODO
#define COMMAND_TYPE_PARAM_SAVE_EEP    "50" // TODO
#define COMMAND_TYPE_READ_LOG          "60" // TODO

#define ACTION_DATA_LENGTH              5
#define STATUS_LENGTH                   81

int MakeCommand(char *command, char *type, char *string);
static int MakeActionCommand(char *command, char action, int *data, int data_len);
int hex2i(char hex);

// Table for data -> command to send
char ParamTable[ MAX_PARAM - MIN_PARAM + 1] =
{
	'z','y','x','w','v','u','t','s','r','q','p','o','n','m','l','k','j','i','h','g','f','e','d','c','b','a', // -26 - -1
	'0',                                                                                                     // 0
	'1','2','3','4','5','6','7','8','9','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'  // 1 - 26
};

// Table for recive command -> data
char DataTable[0x100] =
{
	  0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,
	  0,  1,  2,  3,  4,  5,  6,  7,    8,  9,  0,  0,  0,  0,  0,  0,
	  0,  1,  2,  3,  4,  5,  6,  7,    8,  9, 10, 11, 12, 13, 14, 15,
	 16, 17, 18, 19, 20, 21, 22, 23,   24, 25, 26,  0,  0,  0,  0,  0,
	  0, -1, -2, -3, -4, -5, -6, -7,   -8, -9,-10,-11,-12,-13,-14,-15,
	-16,-17,-18,-19,-20,-21,-22,-23,  -24,-25,-26,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0
};

/*
MakeCommand---MakeActionCommand---WalkCommandOLD
            |                   |-WalkCommand
            |                   |-TurnCommand
            |                   |-SideStepCommand
            |                   |-StandCommand
            |                   |-MotionCommand
            |                   |-HeadPanCommand
            |                   |-HeadTiltCommand
            |                   |-CancelCommand
            |                   |-MotorTorqueCommand
            |                   |-GyroFeedbackCommand
            |                   |-StatusReadCommand
            |
            |---------------------MemReadCommand           // TODO
            |---------------------MemWriteCommand          // TODO
            |
            |-MakeMotionCommand---MotionSendCommand        // TODO
            |                   |-MotionExecCommand        // TODO
            |                   |-MotionExecEEPCommand     // TODO
            |                   |-MotionSaveEEPCommand     // TODO
            |                   |-MotionServoPowerCommand  // TODO
            |                   |-MotionServoTorqueCommand // TODO
            |                   |-MotionReadServoCommand   // TODO
            |
*/



int
MakeCommand(char *command, char *type, char *string)
{
	int len = strlen(string) + COMMAND_TYPE_LENGTH + 1;

	if( len > 0xff ){
		return -1;
	}

	sprintf(command, "%c%02x%s%s", COMMAND_MAGIC, len, type,  string);
//                   |  |  | |- Data
//                   |  |  |- Command Type
//                   |  |- Length
//                   |- Magic charactor

	return (strlen(command) +1);
}

#define MAX_CMD_SIZE 512 //maybe enough
static int
MakeActionCommand(char *command, char action, int *data, int data_len)
{
	int i;
	char tmp[MAX_CMD_SIZE];

	if (data_len >= MAX_CMD_SIZE) {
		/* if this assertion failed, increase MAX_CMD_SIZE */
		assert(0);
	}

	// data range check
	for( i=0; i<data_len; i++ ){
		if( (MIN_PARAM > data[i]) || (MAX_PARAM < data[i]) ){
			return -1;
		}
	}
	tmp[0]     = action;
	for( i=0; i< data_len; i++ ){
		tmp[i+1] = ParamTable[ (int)(data[i]+PARAM_TABLE_OFFSET) ];
	}
	tmp[i+1]   = '\0';
	
	return MakeCommand(command, COMMAND_TYPE_ACTION, tmp);

}

int
AllCommand(char *command, int step_num, int step_angle, int step_width, int step_time, int side_step_width )
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = abs(step_num);
	data[i++] = step_angle;
	if(step_num!=0){
		data[i++] = step_width * (step_num / abs(step_num));
	}else{
		data[i++] = step_width;
	}
	data[i++] = step_time;
	data[i++] = side_step_width;

	return MakeActionCommand(command, ACTION_ALL, data, i);
}

int
WalkCommand(char *command, int step_num, int step_angle, int step_width, int step_time, int side_step_num )
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = abs(step_num);
	data[i++] = step_angle;
  if(step_num!=0){
    data[i++] = step_width * (step_num / abs(step_num));
  }else{
    data[i++] = step_width;
  }
	data[i++] = step_time;
#ifndef COMMAND_PARAM_4
	data[i++] = side_step_num;
#endif

  return MakeActionCommand(command, ACTION_WALK_NEW, data, i);
}

#ifdef COMMAND_PARAM_4
int
WalkCommandOLD(char *command, int step_num, int step_angle, int step_width, int step_time )
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = abs(step_num);
	data[i++] = step_angle;
	data[i++] = step_width;
	data[i++] = step_time;

	if( step_num >= 0 ){ // Walk forward
		return MakeActionCommand(command, ACTION_WALK, data, i);
	}else{ // Walk behind
		return MakeActionCommand(command, ACTION_BACK, data, i);
	}
	return -1;
}
#endif

#ifdef COMMAND_PARAM_4
int
TurnCommand(char *command, int step_num, int turn_angle, int step_time)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = abs(step_num);
	data[i++] = turn_angle;
	data[i++] = 0;
	data[i++] = step_time;

	return MakeActionCommand(command, ACTION_TURN, data, i);
}
#else /* 5 parameters */
int
TurnCommand(char *command, int step_num, int turn_angle, int step_time)
{
	return AllCommand(command, step_num, -turn_angle, 0, step_time, 0 );
}
#endif

#ifdef COMMAND_PARAM_4
int
SideStepCommand(char *command, int step_num, int step_angle, int step_width, int step_time)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = step_num;
	data[i++] = step_angle;
	data[i++] = step_width;
	data[i++] = step_time;

	return MakeActionCommand(command, ACTION_SIDESTEP, data, i);
}
#else /* 5 parameters */
int
SideStepCommand(char *command, int step_num, int step_angle, int step_width, int step_time)
{
	if (step_width > 0) step_angle *= -1;
	return AllCommand(command, step_num, step_angle, 0, step_time, -step_width );
}
#endif

int
StandCommand(char *command)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
#ifndef COMMAND_PARAM_4
	data[i++] = 0;
#endif

	return MakeActionCommand(command, ACTION_STAND, data, i);
}

int
MotionCommand(char *command, int motion_num, int loop_count)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = (motion_num % 1000) / 100;
	data[i++] = (motion_num % 100)  / 10;
	data[i++] = (motion_num % 10);
	data[i++] = loop_count;
#ifndef COMMAND_PARAM_4
	data[i++] = 0;
#endif

	return MakeActionCommand(command, ACTION_MOTION, data, i);
}

int
GyroCalibrationCommand(char *command)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = 1;
	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
#ifndef COMMAND_PARAM_4
	data[i++] = 0;
#endif

	return MakeActionCommand(command, GYRO_CALIBRARION, data, i);
}

int
HeadPanCommand(char *command, int pan_angle, int time)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;
	if (pan_angle > 135) pan_angle = 135;
	if (pan_angle < -135) pan_angle = -135;
	data[i++] = -(pan_angle % 1000) / 100;
	data[i++] = -(pan_angle % 100)  / 10;
	data[i++] = -(pan_angle % 10);
	data[i++] = time;
#ifndef COMMAND_PARAM_4
	data[i++] = 0;
#endif

	return MakeActionCommand(command, ACTION_HEAD_PAN, data, i);
}

int
HeadTiltCommand(char *command, int tilt_angle, int time)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = (tilt_angle % 1000) / 100;
	data[i++] = (tilt_angle % 100)  / 10;
	data[i++] = (tilt_angle % 10);
	data[i++] = time;
#ifndef COMMAND_PARAM_4
	data[i++] = 0;
#endif

	return MakeActionCommand(command, ACTION_HEAD_TILT, data, i);
}

int
CancelCommand(char *command)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
#ifndef COMMAND_PARAM_4
	data[i++] = 0;
#endif

	return MakeActionCommand(command, ACTION_CANCEL, data, i);
}

int AccurateOneStep(char *command, int x, int y, int th){ //x[cm], y[cm], th[2deg]
    int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = x;
	data[i++] = y;
    data[i++] = th;
    data[i++] = 0;
    data[i++] = 0;

    return MakeActionCommand(command, ACCURATE_ONE_STEP, data, i);
}

int AccurateWalk(char *command,int num, int x,  int y, int th){
    int data[ACTION_DATA_LENGTH];
    int i=0;

    data[i++] = num;
    data[i++] = x;
    data[i++] = y;
    data[i++] = th;
    data[i++] = 0;

    return MakeActionCommand(command, ACCURATE_WALK, data, i);
}

int VariableMotionCommand(char *command, int motion_num, int shift){
    int data[ACTION_DATA_LENGTH];
    int i=0;

    data[i++] = (motion_num % 1000) / 100;
	data[i++] = (motion_num % 100)  / 10;
	data[i++] = (motion_num % 10);
    data[i++] = shift;

    return MakeActionCommand(command, VARIABLE_MOTION, data, i);
}

int
MotorTorqueCommand(char *command, int ctrl)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = ctrl;
	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
#ifndef COMMAND_PARAM_4
	data[i++] = 0;
#endif

	return MakeActionCommand(command, MOTOR_TORQUE, data, i);
}

int
GyroFeedbackCommand(char *command, int ctrl)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = ctrl;
	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
#ifndef COMMAND_PARAM_4
	data[i++] = 0;
#endif

	return MakeActionCommand(command, GYRO_FEEDBACK, data, i);

}

int
StatusReadCommand(char *command)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
#ifndef COMMAND_PARAM_4
	data[i++] = 0;
#endif

	return MakeActionCommand(command, STATE_READ, data, i);
}

int
StatusReadQuaternionCommand(char *command)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
#ifndef COMMAND_PARAM_4
	data[i++] = 0;
#endif

	return MakeActionCommand(command, STATE_READ_QUATERNION, data, i);
}

int
StatusReadVolCommand(char *command)
{
	int data[ACTION_DATA_LENGTH];
	int i=0;

	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0;
#ifndef COMMAND_PARAM_4
	data[i++] = 0;
#endif

	return MakeActionCommand(command, STATE_READ_VOL, data, i);
}

int
ParseRobotStatusResponse(RobotStatus *status, const char *res, int len)
{
	int data;
	int i=0, j;

	// Need response string start with 'R'
	if( (*res++) != STATE_READ ){
		return RESPONSE_READ_ERR_NODATA;
	}

	if( len < STATE_RESPONSE_LEN ){
		return RESPONSE_READ_ERR_NODATA;
	}

	status -> Active        = DataTable[ (int)(*res++) ];
	status -> Command       =                 (*res++);
	status -> Posture       = DataTable[ (int)(*res++) ];
	data                    =                 (*res++);
/*	switch(data){
		case('0'):
			status -> CPUVoltage   = STATE_CPU_VOLTAGE_OK;
			status -> MotorVoltage = STATE_MOTOR_VOLTAGE_OK;
			break;

		case('1'):
			status -> CPUVoltage   = STATE_CPU_VOLTAGE_NG;
			status -> MotorVoltage = STATE_MOTOR_VOLTAGE_OK;
			break;

		case('2'):
			status -> CPUVoltage   = STATE_CPU_VOLTAGE_OK;
			status -> MotorVoltage = STATE_MOTOR_VOLTAGE_NG;
			break;

		case('3'):
		default:
			status -> CPUVoltage   = STATE_CPU_VOLTAGE_NG;
			status -> MotorVoltage = STATE_MOTOR_VOLTAGE_NG;
			break;
	}*/

	for(i=0; i<STATE_ODOMETRY_NUM; i++){
		for(j=0; j<4; j++){
			status -> Odometry[i]  = ((status -> Odometry[i]) <<4) + (hex2i(*res++));
		}
	}
	for(i=0; i<STATE_MOTOR_NUM; i++){
		for(j=0; j<4; j++){
			status -> Servo[i]     = ((status -> Servo[i]) <<4) + (hex2i(*res++));
			status -> servo_rad[i] = (float) (status -> Servo[i] * M_PI / 180.0);
		}
	}
	status->gyroMotion = (float)(status->Servo[13] / 100.0f * M_PI / 180.0f);
	return RESPONSE_READ_ERR_NOERROR;
}

int
ParseRobotStatusQuaternionResponse(RobotStatus *status, const char *res, int len)
{
	int i, j;
	int data;

	// Need response string start with 'Q'
	if( (*res++) != STATE_READ_QUATERNION ){
		return RESPONSE_READ_ERR_NODATA;
	}

	if( len < STATE_RESPONSE_LEN ){
		return RESPONSE_READ_ERR_NODATA;
	}

	status -> Active        = DataTable[ (int)(*res++) ];
	status -> Command       =                 (*res++);
	status -> Posture       = DataTable[ (int)(*res++) ];
	data                    =                 (*res++);

	for(i = 0; i < STATE_ODOMETRY_NUM; i++){
		data = 0;
		for(j = 0; j < 4; j ++){
			data = (data << 4) + hex2i(*res++);
		}
		status -> Odometry[i] = (short)data;
	}
	for(i = 0; i < STATE_MOTOR_NUM; i++){
		data = 0;
		for(j = 0; j < 4; j++){
			data = (data << 4) + hex2i(*res++);
		}
		status -> Servo[i] = (short)data;
		status -> servo_rad[i] = (float) (status -> Servo[i] * M_PI / 180.0f);
	}
	status->gyroMotion = (float)(status->Servo[13] / 100.0f * M_PI / 180.0f);

	for(i = 0; i < 4; i++){
		data = 0;
		for(j = 0; j < 4; j ++){
			data = (data << 4) + hex2i(*res++);
		}
		status->quaternion[i] = (float)((short)data) / 0x8000;
	}

	return RESPONSE_READ_ERR_NOERROR;
}

int
ParseRobotStatusVoltResponse(RobotStatus *status, const char *res, int len)
{
	int data;
	int i=0;

	// Need response string start with 'R'
	if( (res[0]) != STATE_READ_VOL ){
		return RESPONSE_READ_ERR_NODATA;
	}

	if( len < STATE_RESPONSE_LEN ){
		return RESPONSE_READ_ERR_NODATA;
	}

	status -> Active        = DataTable[ (int)(res[1]) ];
	status -> Command       =                 (res[2]);
	status -> Posture       = DataTable[ (int)(res[3]) ];
	data                    =                 (res[4]); // robot version (Ex. 18)
	
//	status -> CPUVoltage	= hex2i(*res++)*16*16*16 + hex2i(*res++)*16*16 + hex2i(*res++)*16 + hex2i(*res++);
	//res += 4;
	status -> MotorVoltage	= hex2i(res[9])*16*16*16 + hex2i(res[10])*16*16 + hex2i(res[11])*16+ hex2i(res[12]);
	//status -> servo1temp    = hex2i(res[13])*16*16*16 + hex2i(res[14])*16*16 + hex2i(res[15])*16+ hex2i(res[16]);
/*	for(i=0; i<STATE_MOTOR_NUM; i++){
		for(j=0; j<4; j++){
			status -> Servo[i]     = ((status -> Servo[i]) <<4) + (hex2i(*res++));
		}
	}*/
	return RESPONSE_READ_ERR_NOERROR;
}

int
ParseTemperatureResponse(int temp[TEMPERATURE_ARRAY_LEN], const char *res, int len)
{
	int i=0, j;
	static char hex[16] = "0123456789abcdef";

	if( len < STATE_TEMPARATURE_LEN ) {
		return -1;
	}

	for(j = 0; j < TEMPERATURE_ARRAY_LEN; j++){
		temp[j] = (strchr(hex,res[j*2+5])-hex) * 16 + (strchr(hex,res[j*2+6])-hex);
	}
	return 0;
}

int
hex2i(char hex)
{
	if(       (hex>='a')&&(hex<='f') ){
		return hex - 'a' + 0x0a;
	}else if( (hex>='A')&&(hex<='F') ) {
		return hex - 'A' + 0x0a;
	}else if( (hex>='0')&&(hex<='9') ){
		return hex - '0';
	}
	return 0;
}

int ReadTemperatureCommand(char *command)
{
	strcpy(command,"*0235");

	return strlen(command) + 1;
}

void HeadPantTiltCommand(char *command,int pan,int tilt,int time){
	int i=0;
	int data[5];

	if(pan < -135) pan  = -135;
	if(pan > 135 ) pan  =  135;
	if(tilt < -90) tilt =  -90;
	if(tilt > 90 ) tilt =   90;

	data[i++] = -(pan / 10);
	data[i++] = -(pan % 10);
	data[i++] = (tilt / 10);
	data[i++] = (tilt % 10);
	data[i++] = time;

	MakeActionCommand(command,ACTION_HEAD_PANTILT, data,i);
	
}

