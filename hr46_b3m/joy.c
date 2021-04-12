/*----------------------------------------------------------*/
/*	command interface										*/
/*															*/
/*															*/
/*	file name	:	joy.c									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.8.22								*/
/*----------------------------------------------------------*/
#define		_JOY_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	<stdio.h>
#include    <stdlib.h>
#include	<string.h>
#include    <math.h>
#include	"var.h"
#include	"func.h"
#include	"joy.h"
#include	"calc_mv.h"
#include	"sq_straight.h"
#include	"sq_ready.h"
#include	"sq_walk.h"
#include	"sq_motion.h"
#include	"serv.h"
#include	"acc.h"
#include	"sq_start.h"
#include	"motion.h"
#include	"gyro.h"
#include	"servo_rs.h"
#include	"b3m.h"
#include 	"mvtbl.h"

char 			sfmt[256];
char 			rfmt[256];
float			ad_volt[8];
float			old_yaw = 0.0f;

/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/

unsigned short	count_joy;						// �R�}���h�̐�
tp_xv_comm		xv_comm;
tp_xv_comm_bin	xv_comm_bin;
tp_joy_status	joy_status;
tp_xv_joy		xv_joy;
char			xv_comm_response[8];
int				is_walk_change;					// ���s��ύX�������ǂ����̃t���O

#define MAX_ONE_STEP_X 200   //[mm]
#define MAX_ONE_STEP_Y 200   //[mm]
#define MAX_ONE_STEP_Z 80    //[mm]
#define MAX_ONE_STEP_TH 50   //[deg]

void set_xv_comm(tp_xv_comm *xv,
				 unsigned char	cmd,
				 unsigned char	para1,
				 unsigned char	para2,
				 unsigned char	para3,
				 unsigned char	para4,
				 unsigned char	para5
				 )
{
	xv->cmd   = cmd  ;
	xv->para1 = para1;
	xv->para2 = para2;
	xv->para3 = para3;
	xv->para4 = para4;
	xv->para5 = para5;
}

//x[mm], y[mm], z[mm], th[deg]
void accurate_walk_command(short steps, float x, float y, float z, float th, float time){
    accurate_one_step_mode = 1;
	if( !sq_flag.walk ) xv_mv.count = 0;

    xv_joy.walk_num = steps != 0 ? limit(steps, 100, 2) + (short)xv_mv.count : 10000;

    sq_flag.walk		=	ON;
    is_walk_change		=	1;
    
    xv_mv_walk.accurate_step_x = limit(x, MAX_ONE_STEP_X, -MAX_ONE_STEP_X);
    xv_mv_walk.accurate_step_y = -limit(y, MAX_ONE_STEP_Y, -MAX_ONE_STEP_Y);
    xv_mv_walk.accurate_step_z = limit(z, MAX_ONE_STEP_Z, 0);
    xv_mv_walk.accurate_step_th = -limit(th, MAX_ONE_STEP_TH, -MAX_ONE_STEP_TH);
    xv_mv_walk.accurate_step_time = time;
    xv_joy.walk_time = time;
    
    if( fabs(xv_joy.walk_time - xv_mv_walk.time_old) > EPS_DATA ){
        xv_joy.walk_zmp = zmp_fun( xv_joy.walk_time, xp_mv_walk.y_balance);
		xv_joy.walk_time_dutyfactor = limit( 0.30f / xv_joy.walk_time, 1.0f, 0.05f );
		xv_mv_walk.time_old = xv_joy.walk_time;
    }
}

void convert_bin(tp_xv_comm_bin *xv_bin, tp_xv_comm *xv)
{
	xv_bin->para1 = ascii2bin( xv->para1 );
	xv_bin->para2 = ascii2bin( xv->para2 );
	xv_bin->para3 = ascii2bin( xv->para3 );
	xv_bin->para4 = ascii2bin( xv->para4 );
	xv_bin->para5 = ascii2bin( xv->para5 );
}

int scif1_tx_fun();

/*--------------------------------------*/
/*	initialize							*/
/*--------------------------------------*/
void	joy_init( void )
{
	count_joy			=	0;
	memset(&xv_comm, '0', sizeof(xv_comm));
	memset(&xv_comm_bin, 0, sizeof(xv_comm_bin));
	memset(xv_comm_response, '0', 8);
	xv_comm_response[0]	=	EOF_CODE;			/*	response data buffer	*/
	memset(&joy_status, '0', sizeof(joy_status));

	xv_joy.walk_num					=	0;
	xv_joy.walk_time				=	0.f;
	xv_joy.walk_x_percent			=	0.f;
	xv_joy.walk_y_percent			=	0.f;
	xv_joy.walk_theta_percent		=	0.f;
	xv_joy.walk_zmp					=	xv_mv_walk.zmp;
	xv_joy.walk_time_dutyfactor		=	xv_mv_walk.time_dutyfactor;
	xv_joy.walk_step_len_offset		= 	0.f;

	is_walk_change					=	0;		// ���s��ύX�������ǂ����̃t���O
}


/*--------------------------------------*/
/*	helper function to write an int as hexadecimal number to a string	*/
/*--------------------------------------*/
/** 16hex to string	***/
unsigned char hexTable[17] = "0123456789abcdef";
void writeHexNumber(unsigned char * buffer, int nr, int len)
{
  int digit;
  int i;
  buffer += len;
  for(i = 0; i < len ; i++){
	buffer--;
	digit = nr & 0x0f;
	*buffer = hexTable[digit];
	nr = nr >> 4;
  }
}

float max180(float ang)
{
	while(ang >  180.0) ang -= 360.0f;
	while(ang < -180.0) ang += 360.0f;
	return ang;
}

/*--------------------------------------*/
/*	joy									*/
/*--------------------------------------*/
void	joy( void )
{
	unsigned char * tmpCharPtr;
	int i;

	static	tp_xv_comm		_xv_comm;
	static 	tp_xv_comm_bin	_xv_comm_bin;
	int	 	w;
	float	w1, w2;
	short	x_s[SERV_NUM];

	/***	copy data	***/
	memcpy(&_xv_comm, &xv_comm, sizeof(_xv_comm));
	memcpy(&_xv_comm_bin, &xv_comm_bin, sizeof(_xv_comm_bin));

	/***	clear data	***/
	memset(&xv_comm, '0', sizeof(xv_comm));
	memset(&xv_comm_bin, 0, sizeof(xv_comm_bin));
	
	if( _xv_comm.cmd != '0' )						/*	command receive check	*/
	{
	 	xv_comm_response[0]	=	_xv_comm.cmd;
		xv_comm_response[1]	=	_xv_comm.para1;
		xv_comm_response[2]	=	_xv_comm.para2;
		xv_comm_response[3]	=	_xv_comm.para3;
		xv_comm_response[4]	=	_xv_comm.para4;
		xv_comm_response[5]	=	_xv_comm.para5;
	}

	switch( _xv_comm.cmd ){
		case	'P':		/*	servo torque on/off	*/
			switch( _xv_comm_bin.para1 ) {
			  case	0:		// �T�[�{OFF
				printf("SERVO OFF !!\n");
				flag_servo_off		=	ON;
				flag_ukemi			=	OFF;
				flag_motion_accept	=	ON;
				break;
			  case	1:		// �T�[�{ON
				printf("SERVO ON !!\n");
				sq_flag.start		=	ON;
				break;
			  case	2:
				flag_ukemi			=	ON;
				break;
			  case	3:
				flag_ukemi			=	OFF;
				flag_motion_accept	=	ON;
				sq_flag.straight	=	ON;
				sq_flag.start		=	ON;
				break;
			  case	4:
				flag_ukemi			=	OFF;
				flag_motion_accept	=	ON;
				break;
			  default:
				break;
			}
			break;

		/*
		 * return data format
		 * 0:'R', 1:status, 2:'0', 3:fall, 4:'0', 5-8:odometry_x, 9-12:odometry_y, 13-16:odometry_the,
		 * 17-20:servo0 , 21-24:servo1 , 25-28:servo2 , 29-32:servo3 , 33-36:servo4 , 37-40:servo5 , 
		 * 41-44:servo6 , 45-48:servo7 , 49-52:servo8 , 53-56:servo9 , 57-60:servo10, 61-64:servo11, 
		 * 65-68:servo12, 69-72:gyro   , 73-76:pan    , 77-80:tilt 
		 */
		case	'R':		/*	status of robot	*/
			joy_status.cmd		=	'R';

			/*	robot is moving	*/
			if     ( flag_moving == STATE_WALKING )	joy_status.para1	=	'1';	/*	walking					*/
			else if( flag_moving == STATE_MOTION )	joy_status.para1	=	'2';	/*	acting special action	*/
			else if( flag_moving == STATE_MOVING )	joy_status.para1	=	'3';	/*	moving					*/
			else									joy_status.para1	=	'0';	/*	stop					*/

			/*	fall down flag	*/
			joy_status.para2	=	'0';
			if( flag_gyro.fall == 1 )					joy_status.para3	=	'1';	/*	front	*/
			else if( flag_gyro.fall == 2 )				joy_status.para3	=	'2';	/*	back	*/
			else if( flag_gyro.fall == 3 )				joy_status.para3	=	'3';	/*	right	*/
			else if( flag_gyro.fall == 4 )				joy_status.para3	=	'4';	/*	left	*/
			else 										joy_status.para3	=	'0';	/*	not fallen	*/

			joy_status.para4	=	'0';

			xv_comm_response[0]	=	EOF_CODE;
			/*	legs and waist joint angle	*/
			for( w = 0; w < 6; w++ )
			{
				x_s[w]	=	(short)(xv_sv[w  ].d / 100);
			}
			for( w = 6; w < 12; w++ )
			{
				x_s[w]	=	(short)(xv_sv[w+3].d / 100);
			}
			x_s[13]	=	0;
			/*	head joint angle	*/
			x_s[HEAD_YAW]	=	(short)(xv_sv[HEAD_YAW  ].d / 100);
			x_s[HEAD_PITCH]	=	(short)(xv_sv[HEAD_PITCH].d / 100);
			/*	odometry	*/
			x_s[14]	=	(short)(xv_odometry.moveX);
			x_s[15]	=	(short)(xv_odometry.moveY);
			x_s[16]	=	(short)(xv_odometry.rotZ);

			/*	write joy-status to output	*/
			sfmt[0] = joy_status.cmd;
			sfmt[1] = joy_status.para1;
			sfmt[2] = joy_status.para2;
			sfmt[3] = joy_status.para3;
			sfmt[4] = joy_status.para4;
/*			sfmt[5] = joy_status.para5;

			tmpCharPtr = sfmt + 6;
*/
			tmpCharPtr = sfmt + 5;

			/*	write odometry to output	*/
			writeHexNumber(tmpCharPtr, (int)xv_odometry.moveX, 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr, (int)xv_odometry.moveY, 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr, (int)xv_odometry.rotZ * 100, 4);
			tmpCharPtr += 4;

			for(i = 0; i < 13; i++){
			writeHexNumber(tmpCharPtr, x_s[i], 4);
			tmpCharPtr += 4;
			}

			/* 	write gyro yaw as 14th servo	*/
			writeHexNumber(tmpCharPtr, (int)(max180(xv_gyro.gyro_yaw2 - old_yaw) * 100), 4);
			tmpCharPtr += 4;

			writeHexNumber(tmpCharPtr,x_s[HEAD_YAW], 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,x_s[HEAD_PITCH], 4);
			tmpCharPtr += 4;

			*tmpCharPtr = EOF_CODE3;

			/*	 writing outputdata to serial port	*/

			scif1_tx_fun();

		
			/* 	reset values	*/		
			xv_odometry.moveX			=	0.f;
			xv_odometry.moveY			=	0.f;
			xv_odometry.rotZ			=	0.f;

			/* reset gyro_yaw	*/
//			old_yaw = xv_gyro.gyro_yaw2;
			break;

		/*
		 * return data format
		 * 0:'r', 1:status, 2:'0', 3:fall, 4:'0', 5-8:servo3 voltage, 9-12:servo3 voltage, 13-16:servo0 temparature,
		 * 17-20:gyro, 21-24:gyro, 25-28:gyro , 29-32:gyro
		 * 33-36:servo0 current, 37-40:servo1 current, 41-44:servo2 current, 45-48:servo3 current, 49-52:servo4 current
		 * 53-56:servo5 current, 57-60:servo6 current, 61-64:servo7 current, 65-68:servo8 current,
		 * 69-72:gyro, 73-76:pan, 77-80:tilt
		 */
		case	'r':		/*	status of robot	(read voltage) */
			joy_status.cmd		=	'r';

			/*	robot is moving	*/
			if     ( flag_moving == STATE_WALKING )	joy_status.para1	=	'1';	/*	walking					*/
			else if( flag_moving == STATE_MOTION )	joy_status.para1	=	'2';	/*	acting special action	*/
			else if( flag_moving == STATE_MOVING )	joy_status.para1	=	'3';	/*	moving					*/
			else									joy_status.para1	=	'0';	/*	stop					*/

			/*	fall down flag	*/
			joy_status.para2	=	'0';
			if( flag_acc.fall == 1 )					joy_status.para3	=	'1';	/*	front	*/
			else if( flag_acc.fall == 2 )				joy_status.para3	=	'2';	/*	back	*/
			else if( flag_acc.fall == 3 )				joy_status.para3	=	'3';	/*	right	*/
			else if( flag_acc.fall == 4 )				joy_status.para3	=	'4';	/*	left	*/
			else 										joy_status.para3	=	'0';	/*	not fallen	*/

			joy_status.para4	=	'0';

			xv_comm_response[0]	=	EOF_CODE;
			/*	legs and waist joint angle	*/
			for( w = 0; w < 6; w++ )
			{
				x_s[w]	=	(short)(xv_sv[w  ].d / 100);
			}
			for( w = 6; w < 12; w++ )
			{
				x_s[w]	=	(short)(xv_sv[w+3].d / 100);
			}
			x_s[13]	=	0;
			/*	head joint angle	*/
			x_s[HEAD_YAW]	=	(short)(xv_sv[HEAD_YAW  ].d / 100);
			x_s[HEAD_PITCH]	=	(short)(xv_sv[HEAD_PITCH].d / 100);
			/*	odometry	*/
			x_s[14]		=	(short)(xv_odometry.moveX);
			x_s[15]		=	(short)(xv_odometry.moveY);
			x_s[16]		=	(short)(xv_odometry.rotZ);

			/*	write joy-status to output	*/
			sfmt[0] = joy_status.cmd;
			sfmt[1] = joy_status.para1;
			sfmt[2] = joy_status.para2;
			sfmt[3] = joy_status.para3;
			sfmt[4] = joy_status.para4;
			
			tmpCharPtr = sfmt + 5;

			// write battery voltage * 100
			if(mode_motion == MOTION_NONE){
				struct ServoStatus s;
				ad_volt[3] = B3MGetServoStatus(18, &s) == 0 ? s.voltage * 100 : 0;
				printf("voltage 1: %f\n", s.voltage);
				if(5<s.voltage && s.voltage<11.1){
					system("aplay sound/lowvoltage.wav &");
				}
			}else{
				ad_volt[3] = 9999;
			}
			writeHexNumber(tmpCharPtr,(int)(ad_volt[3]), 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,(int)(ad_volt[3]), 4);
			tmpCharPtr += 4;

			writeHexNumber(tmpCharPtr,(int)(xv_pv.temp[0]), 4);	// servo no.1 temperature
			tmpCharPtr += 4;

		// begin 2008.5.25
			writeHexNumber(tmpCharPtr,(int)(xv_gyro.gyro_data3*100), 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,(int)(xv_gyro.gyro_data3_d*100), 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,(int)(xv_gyro.gyro_data3_flt*100), 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,(int)(xv_gyro.gyro_data3_flt2), 4);
			tmpCharPtr += 4;
			for(i = 0; i < 9; i++){
				writeHexNumber(tmpCharPtr, (int)(xv_pv.current[i]), 4);
				tmpCharPtr += 4;
			}

		// end 2008.5.25
			/* 	write gyro yaw as 14th servo	*/
			writeHexNumber(tmpCharPtr, (int)(max180(xv_gyro.gyro_yaw2 - old_yaw) *100), 4);
			tmpCharPtr += 4;
			
			writeHexNumber(tmpCharPtr,x_s[HEAD_YAW], 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,x_s[HEAD_PITCH], 4);
			tmpCharPtr += 4;
			
			*tmpCharPtr = EOF_CODE3;

			/*	 writing outputdata to serial port	*/
			scif1_tx_fun();

			/* 	reset values	*/		
			xv_odometry.moveX			=	0.f;
			xv_odometry.moveY			=	0.f;
			xv_odometry.rotZ			=	0.f;

			/* reset gyro_yaw	*/
//			old_yaw = xv_gyro.gyro_yaw2;
			break;
		
		/*
		 * return data format
		 * 0:'L', 1:status, 2:'0', 3:fall, 4:'0', 5-8:voltage(HC5), 9-12:voltage(HC5), 13-16:servo0 temparature,
		 * 17-20:servo0 current, 21-24:servo1 current, 25-28:servo2 current , 29-32:servo3 current ,
		 * 33-36:servo4 current, 37-40:servo5 current, 41-44:servo6 current , 45-48:servo7 current ,
		 * 49-52:servo8 current, 53-56:servo9 current, 57-60:servo10 current, 61-64:servo11 current, 65-68:servo12 current,
		 * 69-72:gyro, 73-76:pan, 77-80:tilt
		 */
		case	'L':		/*	status of robot	for servo current */
            printf("Call Read Servo Current [Obsolete Function]");
            exit(1);
#if 0
			joy_status.cmd		=	'L';

			/*	robot is moving	*/
			if     ( flag_moving == STATE_WALKING )	joy_status.para1	=	'1';	/*	walking					*/
			else if( flag_moving == STATE_MOTION )	joy_status.para1	=	'2';	/*	acting special action	*/
			else if( flag_moving == STATE_MOVING )	joy_status.para1	=	'3';	/*	moving					*/
			else									joy_status.para1	=	'0';	/*	stop					*/

			/*	fall down flag	*/
			joy_status.para2	=	'0';
			if( flag_acc.fall == 1 )					joy_status.para3	=	'1';	/*	front	*/
			else if( flag_acc.fall == 2 )				joy_status.para3	=	'2';	/*	back	*/
			else if( flag_acc.fall == 3 )				joy_status.para3	=	'3';	/*	right	*/
			else if( flag_acc.fall == 4 )				joy_status.para3	=	'4';	/*	left	*/
			else 										joy_status.para3	=	'0';	/*	not fallen	*/

			joy_status.para4	=	'0';

			xv_comm_response[0]	=	EOF_CODE;
			/*	legs and waist joint angle	*/
			for( w = 0; w < 6; w++ )
			{
				x_s[w]	=	(short)(xv_sv[w  ].d / 100);
			}
			for( w = 6; w < 12; w++ )
			{
				x_s[w]	=	(short)(xv_sv[w+3].d / 100);
			}
			x_s[13]	=	0;
			/*	head joint angle	*/
			x_s[HEAD_YAW]	=	(short)(xv_sv[HEAD_YAW  ].d / 100);
			x_s[HEAD_PITCH]	=	(short)(xv_sv[HEAD_PITCH].d / 100);
			/*	odometry	*/
			x_s[14]		=	(short)(xv_odometry.moveX);
			x_s[15]		=	(short)(xv_odometry.moveY);
			x_s[16]		=	(short)(xv_odometry.rotZ);

			/*	write joy-status to output	*/
			sfmt[0] = joy_status.cmd;
			sfmt[1] = joy_status.para1;
			sfmt[2] = joy_status.para2;
			sfmt[3] = joy_status.para3;
			sfmt[4] = joy_status.para4;
			
			tmpCharPtr = sfmt + 5;
			
			// write battery voltage * 100
			writeHexNumber(tmpCharPtr,(int)(ad_volt[3]*610), 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,(int)(ad_volt[3]*610), 4);
			tmpCharPtr += 4;

			//read_1st_servo_temp();
			writeHexNumber(tmpCharPtr,(int)(xv_pv.temp[0]), 4);	// servo no.1 temperature
			tmpCharPtr += 4;
		
			read_servo_rs_all(SERVO_B3M_ADDR_PESENT_CURRENT, &xv_servo_rs.present_current[0], 2);
			servocurrent_to_current();
			for(i = 0; i < 13; i++){
				writeHexNumber(tmpCharPtr, (int)(xv_pv.current[i]), 4);
				tmpCharPtr += 4;
			}

			/* 	write gyro yaw as 14th servo	*/
			writeHexNumber(tmpCharPtr, (int)(max180(xv_gyro.gyro_yaw2 - old_yaw)*100), 4);
			tmpCharPtr += 4;
			
			writeHexNumber(tmpCharPtr,x_s[HEAD_YAW], 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,x_s[HEAD_PITCH], 4);
			tmpCharPtr += 4;
			
			*tmpCharPtr = EOF_CODE3;

			/*	 writing outputdata to serial port	*/
			scif1_tx_fun();

			/* 	reset values	*/		
			xv_odometry.moveX			=	0.f;
			xv_odometry.moveY			=	0.f;
			xv_odometry.rotZ			=	0.f;

			/* reset gyro_yaw	*/
//			old_yaw = xv_gyro.gyro_yaw2;
#endif
			break;
		
		/*
		 * return data format
		 * 0:'R', 1:status, 2:'0', 3:fall, 4:'0', 5-8:odometry_x, 9-12:odometry_y, 13-16:odometry_the,
		 * 17-20:servo0 , 21-24:servo1 , 25-28:servo2 , 29-32:servo3 , 33-36:servo4 , 37-40:servo5 , 
		 * 41-44:servo6 , 45-48:servo7 , 49-52:servo8 , 53-56:servo9 , 57-60:servo10, 61-64:servo11, 
		 * 65-68:servo12, 69-72:gyro   , 73-76:pan    , 77-80:tilt   ,
		 * 81-84:quaternion_w, 85-88:quaternion_x, 89-92:quaternion_y, 93-96:quaternion_z
		 */
		case	'Q':		/*	Quaternion�̎擾 */
			joy_status.cmd		=	'Q';

			/*	robot is moving	*/
			if     ( flag_moving == STATE_WALKING )	joy_status.para1	=	'1';	/*	walking					*/
			else if( flag_moving == STATE_MOTION )	joy_status.para1	=	'2';	/*	acting special action	*/
			else if( flag_moving == STATE_MOVING )	joy_status.para1	=	'3';	/*	moving					*/
			else									joy_status.para1	=	'0';	/*	stop					*/

			/*	fall down flag	*/
			joy_status.para2	=	'0';
			if( flag_gyro.fall == 1 )					joy_status.para3	=	'1';	/*	front	*/
			else if( flag_gyro.fall == 2 )				joy_status.para3	=	'2';	/*	back	*/
			else if( flag_gyro.fall == 3 )				joy_status.para3	=	'3';	/*	right	*/
			else if( flag_gyro.fall == 4 )				joy_status.para3	=	'4';	/*	left	*/
			else 										joy_status.para3	=	'0';	/*	not fallen	*/

			joy_status.para4	=	'0';

			xv_comm_response[0]	=	EOF_CODE;
			/*	legs and waist joint angle	*/
			for( w = 0; w < 6; w++ )
			{
				x_s[w]	=	(short)(xv_sv[w  ].d / 100);
			}
			for( w = 6; w < 12; w++ )
			{
				x_s[w]	=	(short)(xv_sv[w+3].d / 100);
			}
			x_s[13]	=	0;
			/*	head joint angle	*/
			x_s[HEAD_YAW]	=	(short)(xv_sv[HEAD_YAW  ].d / 100);
			x_s[HEAD_PITCH]	=	(short)(xv_sv[HEAD_PITCH].d / 100);
			/*	odometry	*/
			x_s[14]	=	(short)(xv_odometry.moveX);
			x_s[15]	=	(short)(xv_odometry.moveY);
			x_s[16]	=	(short)(xv_odometry.rotZ);

			/*	write joy-status to output	*/
			sfmt[0] = joy_status.cmd;
			sfmt[1] = joy_status.para1;
			sfmt[2] = joy_status.para2;
			sfmt[3] = joy_status.para3;
			sfmt[4] = joy_status.para4;

			tmpCharPtr = sfmt + 5;

			/*	write odometry to output	*/
			writeHexNumber(tmpCharPtr, (int)xv_odometry.moveX, 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr, (int)xv_odometry.moveY, 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr, (int)xv_odometry.rotZ * 100, 4);
			tmpCharPtr += 4;

			for(i = 0; i < 13; i++){
				writeHexNumber(tmpCharPtr, x_s[i], 4);
				tmpCharPtr += 4;
			}

			/* 	write gyro yaw as 14th servo	*/
			writeHexNumber(tmpCharPtr, (int)(max180(xv_gyro.gyro_yaw2 - old_yaw) * 100), 4);
			tmpCharPtr += 4;

			writeHexNumber(tmpCharPtr,x_s[HEAD_YAW], 4);
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,x_s[HEAD_PITCH], 4);
			tmpCharPtr += 4;

			writeHexNumber(tmpCharPtr,(int)(xv_gyro.quaternion[0]*32767), 4);	// w
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,(int)(xv_gyro.quaternion[1]*32767), 4);	// x
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,(int)(xv_gyro.quaternion[2]*32767), 4);	// y
			tmpCharPtr += 4;
			writeHexNumber(tmpCharPtr,(int)(xv_gyro.quaternion[3]*32767), 4);	// z
			tmpCharPtr += 4;
			
			*tmpCharPtr = EOF_CODE3;

			/*	 writing outputdata to serial port	*/
			scif1_tx_fun();

			/* 	reset values	*/		
			xv_odometry.moveX			=	0.f;
			xv_odometry.moveY			=	0.f;
			xv_odometry.rotZ			=	0.f;

			/* reset gyro_yaw	*/
			old_yaw = xv_gyro.gyro_yaw2;

			break;

//##############################################################################
		case	'S':
			xv_joy.walk_num	= 0;
			sq_flag.ready	= ON;
			break;

		case	'A':		/*	walk all direction	*/
			accurate_one_step_mode = 0;
			/***	number of steps	***/
			if( !sq_flag.walk ) xv_mv.count = 0;	// ���s���Ă��Ȃ����ɂ́C�����J�E���^��0�ɂ���D

			if( _xv_comm_bin.para1 != 0 ) {
				xv_joy.walk_num	=	(short)(limit( _xv_comm_bin.para1, 100, 2 ) + xv_mv.count);		/*	last steps	[num/bit]	*/
			} else {								/* 0 means non stop	*/
				xv_joy.walk_num	=	10000;			/*	[num/bit]	*/
			}

			sq_flag.walk		=	ON;
			is_walk_change		=	1;				// ���s��ύX�������ǂ����̃t���O

			/***	hip yaw angle		***/		/*	[deg/bit]	*/
			w1	=	limit( _xv_comm_bin.para2, xp_mv_walk.theta, -xp_mv_walk.theta );
			xv_joy.walk_theta_percent	=	-w1 / xp_mv_walk.theta;		/*	- : +Y=LEFT turn, -Y=RIGHT turn		*/

			/***	stride X			***/		/*	[5mm/bit]	*/
			w1	=	xp_mv_walk.x_fwd_swg - xp_mv_walk.x_fwd_spt;
			w2	=	limit( xv_joy.walk_step_len_offset + ((float)_xv_comm_bin.para3*5.f), w1, -w1 );
			xv_joy.walk_x_percent	=	w2 / w1;

			/***	time of one step	***/
			if ( _xv_comm_bin.para4 > 0 ){			/*	offset + 0.10sec  [0.02sec/bit]	*/
				xv_joy.walk_time	=	((float)_xv_comm_bin.para4) / 50.f + 0.10f;
			} else {								/*	<= 0 means default time	*/
				xv_joy.walk_time	=	xp_mv_walk.time;
			}

			if( fabs(xv_joy.walk_time - xv_mv_walk.time_old) > EPS_DATA ){	/*	time changed	*/
				xv_joy.walk_zmp				=	zmp_fun( xv_joy.walk_time, xp_mv_walk.y_balance);
				xv_joy.walk_time_dutyfactor	=	limit( 0.30f / xv_joy.walk_time, 1.0f, 0.05f );
				// �V�r�̎��� 0.30[s]
				xv_mv_walk.time_old	=	xv_joy.walk_time;
			}

			/***	stride Y			***/	/*	[2.5mm/bit]	*/
			if( _xv_comm_bin.para5 == 0 )
			{
				xv_joy.walk_y_percent	=	0.f;
			}
			else
			{
				w1	=	xp_mv_walk.y_swg - xp_mv_walk.y_spt;
				w2	=	limit( (float)_xv_comm_bin.para5*2.5f, w1, -w1 );
				xv_joy.walk_y_percent	=	-w2 / w1;				/*	- : +Y=LEFT side, -Y=RIGHT side		*/
			}

			break;

		case	'H':		/*	head yaw	*/
			w	=	_xv_comm_bin.para1 * 100 + _xv_comm_bin.para2 * 10 + _xv_comm_bin.para3;

			if( abs(w) < 150.f && _xv_comm_bin.para4 > 0 )
			{
				xv_data_d[HEAD_YAW].time	=	(float)_xv_comm_bin.para4 / 10.f;
// check
				{								// ��̊p�x�̌v�Z�l�Ǝ��ۂ̒l�����킷����
					float min_period		= (float)fabs(w  - xv_mvdata_d[HEAD_YAW].out_old) / 60.0f * 0.22f;
					if (min_period < 0.1f) min_period = 0.1f;
					if (xv_data_d[HEAD_YAW].time < min_period) xv_data_d[HEAD_YAW].time = min_period;
				}
				xv_data_d[HEAD_YAW].pos		=	(float)w;
			}

			if( _xv_comm_bin.para1 == 0 && _xv_comm_bin.para2 == 0 && _xv_comm_bin.para3 == 0 && _xv_comm_bin.para4 == 0 )
			{
				flag_face_control	=	OFF;
			}
			else
			{
				flag_face_control	=	ON;
			}
			break;

		case	'h':		/*	head pitch	*/
			w	=	_xv_comm_bin.para1 * 100 + _xv_comm_bin.para2 * 10 + _xv_comm_bin.para3;

			if( abs(w) < 150.f && _xv_comm_bin.para4 > 0 )
			{
				xv_data_d[HEAD_PITCH].time	=	(float)_xv_comm_bin.para4 / 10.f;
				xv_data_d[HEAD_PITCH].pos	=	(float)w;
			}

			if( _xv_comm_bin.para1 == 0 && _xv_comm_bin.para2 == 0 && _xv_comm_bin.para3 == 0 && _xv_comm_bin.para4 == 0 )
			{
				flag_face_control	=	OFF;
			}
			else
			{
				flag_face_control	=	ON;
			}
			break;
		
		case	'N':		/*	neck control	*/
			if( _xv_comm_bin.para5 > 0) {
				xv_data_d[HEAD_YAW].time	=	(float)_xv_comm_bin.para5 / 10.f;
// check
				{								// ��̊p�x�̌v�Z�l�Ǝ��ۂ̒l�����킷����
					float w					= (float)(_xv_comm_bin.para1 * 10 + _xv_comm_bin.para2);
					float min_period		= (float)fabs(w  - xv_mvdata_d[HEAD_YAW].out_old) / 60.0f * 0.22f;
					if (min_period < 0.1f) min_period = 0.1f;
					if (xv_data_d[HEAD_YAW].time < min_period) xv_data_d[HEAD_YAW].time = min_period;
				}
				xv_data_d[HEAD_PITCH].time	=	(float)_xv_comm_bin.para5 / 10.f;
			} else {
				xv_data_d[HEAD_YAW].time	=	0.1f;
				xv_data_d[HEAD_PITCH].time	=	0.1f;
			}
			xv_data_d[HEAD_YAW  ].pos	=	(float)(_xv_comm_bin.para1 * 10 + _xv_comm_bin.para2);
			xv_data_d[HEAD_PITCH].pos	=	(float)(_xv_comm_bin.para3 * 10 + _xv_comm_bin.para4);
			break;
		
		case	'M':		/*	motion (special action)	*/
			if(((flag_motion_accept == ON) || (flag_acc.fall == 0)) && (flag_servo_on)){
				if( _xv_comm_bin.para1 >= 0 &&
					_xv_comm_bin.para2 >= 0 &&
					_xv_comm_bin.para3 >= 0 )
				{
					w	=	_xv_comm_bin.para1 * 100 + _xv_comm_bin.para2 * 10 + _xv_comm_bin.para3;
	
					if( w >= 1 && w <= 100 )
					{
						flag_motion_select	=	w;
						sq_flag.motion		=	ON;
					}
				}
	
				/*	number of repeat	*/
				if( _xv_comm_bin.para4 > 0 )
				{
					flag_motion_repeat	=	ON;
					xv_motion_repeat_num		=	limit_h( _xv_comm_bin.para4, 100 );
				}
				else
				{
					flag_motion_repeat	=	OFF;
				}
				break;
			}
			break;

		case	'C':		/*	cancel	*/
			printf("CANCEL !!\n");
			if( _xv_comm_bin.para1 == 0 )
			{
				if(mode_motion != MOTION_MOTION) // only stop if current motion is not special action
				{
					xv_mv.count				=	10001; // max. step number is 10000 (set by '0' as desired step number
					reset_flag(sq_flag);		// �S�Ă̏�ԃt���O���N���A
					xv_joy.walk_x_percent		=	0.f;
					xv_joy.walk_y_percent		=	0.f;
					xv_joy.walk_theta_percent	=	0.f;

//					flag_sq_motion_cancel	=	ON;
				}
				else /*	end of repeat motion	*/
				{
					count_motion_repeat	=	1000;
				}
			}
			else if( _xv_comm_bin.para1 == 1 )
			{
				/*	end of repeat motion	*/
				count_motion_repeat	=	1000;
			}
			break;

		case	'T':		/*	standing upright	*/
			sq_flag.straight	=	ON;
			break;

		case	'J':		/*	gyro on/off	*/
			switch( _xv_comm_bin.para1 ) {
			  case	0:
				flag_gyro.vib_auto	=	ON;
				flag_gyro.vib_manu	=	OFF;
				break;
			  case	1:
				flag_gyro.vib_auto	=	OFF;
				flag_gyro.vib_manu	=	ON;
				break;
			  case	2:
				flag_gyro.vib_auto	=	OFF;
				flag_gyro.vib_manu	=	OFF;
				break;
			  case	3:
				flag_auto_gyro_offset	=	ON;
				break;
			  case	4:
				flag_auto_gyro_offset	=	OFF;
				break;
			  default:
				break;
			}
			break;
		
		case	'j':	/*	gyro reset	*/
			if( _xv_comm_bin.para1 == 1 ){
				flag_gyro.zero = ON;
				gyro_fun();
			}
			break;
			
		case	'w':	/* change ready.z3	*/
			switch( _xv_comm_bin.para1 ) {
				case 0:
					xp_mv_ready.z3	= xp_mv_walk.h_cog;
					break;
				case 1:
					xp_mv_ready.z3	-= 10.f;
					break;
				case 2:
					xp_mv_ready.z3	-= 20.f;
					break;
				default:
					break;
			}
			sq_flag.straight	=	ON;
			break;
            
        case    'O':
			xv_mv.count = 0;
			accurate_walk_command(2, _xv_comm_bin.para1 * 10, _xv_comm_bin.para2 * 10, xp_mv_walk.accurate_step_z, _xv_comm_bin.para3 * 2, xv_joy.walk_time);
			break;
        case    'U':
            accurate_walk_command(_xv_comm_bin.para1, _xv_comm_bin.para2 * 10, _xv_comm_bin.para3 * 10, xp_mv_walk.z, _xv_comm_bin.para4 * 2, xv_joy.walk_time);
            break;
        case     'V':
            if(((flag_motion_accept == ON) || (flag_acc.fall == 0)) && (flag_servo_on)){
				if( _xv_comm_bin.para1 >= 0 &&
					_xv_comm_bin.para2 >= 0 &&
					_xv_comm_bin.para3 >= 0 )
				{
					w	=	_xv_comm_bin.para1 * 100 + _xv_comm_bin.para2 * 10 + _xv_comm_bin.para3;
	                
					if( w >= 1 && w <= 100 )
					{
						flag_motion_select	=	w;
						sq_flag.motion		=	ON;
                        flag_variable_motion = ON;
					}
				}
	            
                flag_motion_repeat      =   OFF;
                xv_motion_repeat_num    =   1;
                variable_amount = _xv_comm_bin.para4;

				break;
			}
            break;

		default:
			break;
	}

	if( xv_comm_response[0] != EOF_CODE && xv_comm_response[0] != '0' )
	{
		sfmt[0]		=	xv_comm_response[0];
		sfmt[1]		=	xv_comm_response[1];
		sfmt[2]		=	xv_comm_response[2];
		sfmt[3]		=	xv_comm_response[3];
		sfmt[4]		=	xv_comm_response[4];
		sfmt[5]		=	xv_comm_response[5];
		sfmt[6]		=	EOF_CODE3;

		xv_comm_response[0]	=	EOF_CODE;

		scif1_tx_fun();
	}

}


/*--------------------------------------*/
/*	read joy stick data from rx format	*/
/*--------------------------------------*/
void	joy_read( void )
{
	sscanf( (char *)&rfmt[5], "%c%c%c%c%c%c", &xv_comm.cmd, &xv_comm.para1, &xv_comm.para2, &xv_comm.para3, &xv_comm.para4, &xv_comm.para5 );

	/*	character to binary data	*/
	xv_comm_bin.para1	=	ascii2bin( xv_comm.para1 );
	xv_comm_bin.para2	=	ascii2bin( xv_comm.para2 );
	xv_comm_bin.para3	=	ascii2bin( xv_comm.para3 );
	xv_comm_bin.para4	=	ascii2bin( xv_comm.para4 );
	xv_comm_bin.para5	=	ascii2bin( xv_comm.para5 );
	++count_joy;
}

/*--------------------------------------*/
/*	ascii2bin							*/
/*--------------------------------------*/
/***	character to binary data	***/
short	ascii2bin( unsigned char c )
{
	short	c1;

	c1	=	c;

	if( c >= '0' && c<= '9' )
		return	(short)( c1 - '0' );
	else if ( c >= 'A' && c<= 'Z' )
		return	(short)( c1 - 'A' + 1 );
	else if ( c >= 'a' && c<= 'z' )
		return	(short)( -( c1 - 'a' + 1 ) );

	return	0;
}


/*--------------------------------------*/
/*	copy_joy_parameter				 	*/
/*--------------------------------------*/
void 	copy_joy_parameter( void )
{
	xv_mv_walk.num				=	xv_joy.walk_num;				/*	number of steps		*/
	xv_mv_walk.time				=	xv_joy.walk_time;				/*	walk period / 2		*/
	xv_mv_walk.zmp				=	xv_joy.walk_zmp;				/* 	Yzmp [mm]			*/
	xv_mv_walk.time_dutyfactor	=	xv_joy.walk_time_dutyfactor;	/*	duty factor			*/
	xv_mv_walk.x_percent		=	xv_joy.walk_x_percent;			/* 	foot x (0-1)		*/
	xv_mv_walk.y_percent		=	xv_joy.walk_y_percent;			/* 	foot y (0-1)		*/
	xv_mv_walk.theta_percent	=	xv_joy.walk_theta_percent;		/* 	foot theta (0-1)	*/
}
