/*----------------------------------------------------------*/
/*	control													*/
/*															*/
/*															*/
/*	file name	:	cntr.c									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#define		_CNTR_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	"var.h"
#include	"func.h"
#include	"acc.h"
#include	"gyro.h"
#include	"joy.h"
#include	"motion.h"
#include	"calc_mv.h"
#include	"kine.h"
#include	"calc_deg.h"
#include	"serv.h"
#include 	"servo_rs.h"
#include	"b3m.h"
#include	"sq_walk.h"
#include	"mvtbl.h"
#include	"sq_straight.h"
#include	"sq_motion.h"


/*--------------------------------------*/
/*	control								*/
/*--------------------------------------*/
void	cntr( void )
{
	joy();					/*	command receive						*/
	motion();				/*	motion sequense						*/
	calc_mv();				/*	calculate trajectory tables			*/
	calc_z();				/*	calculate leg length by hip roll 	*/
	kine();					/*	calculate inv. kinetics				*/
	acc_fun();				/*	acceleration sensor					*/
	gyro_fun();				/*	gyro sensor							*/
	calc_deg();				/*	calculate joint angle by inv. kinetics data	*/
	serv();					/*	servo motor control					*/
/*	if (flag_servo_off){
		xv_mv.count				=	10001; // max. step number is 10000 (set by '0' as desired step number
		sq_flag.straight		=	ON;
		sq_flag.walk			=
		sq_flag.motion			=	OFF;
		flag_sq_motion_cancel	=	ON;
		flag_face_control		=	OFF;
	}*/

	servo_rs_fun();			/*	output to servo motors				*/
}
