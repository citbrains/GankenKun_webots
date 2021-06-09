/*----------------------------------------------------------*/
/*	acceleration sensor										*/
/*															*/
/*															*/
/*	file name	:	acc.c									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.5.30								*/
/*----------------------------------------------------------*/

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include    <math.h>
#include	"var.h"
#include	"func.h"
#include	"acc.h"
#include	"gyro.h"
#include	"servo_rs.h"
#include	"b3m.h"
#include	"calc_mv.h"
#include	"sq_straight.h"
#include	"sq_start.h"
#include	"motion.h"

/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
tp_xp_acc		xp_acc;
tp_xv_acc		xv_acc;
tp_flag_acc		flag_acc;
short	flag_restart;
short	flag_ukemi_finished;
short	flag_ukemi_start;
short	flag_servo_restart;
short	flag_restart_work;
short	flag_ukemi_finished_work;
short	flag_motion_accept;
short	flag_motion_accept_work;
float	touchdown_gain;

/*--------------------------------------*/
/*	acc									*/
/*--------------------------------------*/
void	acc_fun( void )
{
	/***	low pass filter	***/
	xv_acc.acc_data1_d	=	diff( xv_acc.acc_data1, xp_acc.t1, xp_acc.t2, &xv_acc.acc_data1_flt );
	xv_acc.acc_data2_d	=	diff( xv_acc.acc_data2, xp_acc.t1, xp_acc.t2, &xv_acc.acc_data2_flt );
	xv_acc.acc_data3_d	=	diff( xv_acc.acc_data3, xp_acc.t1, xp_acc.t2, &xv_acc.acc_data3_flt );
	
	/***	calculate angle from acc	***/
	xv_acc.acc_pitch	=	rad2deg( asin(limit(xv_acc.acc_data1_flt, 1.0f, -1.0f)));
	xv_acc.acc_roll		=	rad2deg( asin(limit(xv_acc.acc_data2_flt, 1.0f, -1.0f)));
	xv_acc.acc_pitch_d	=	rad2deg( asin(limit(xv_acc.acc_data1_d, 1.0f, -1.0f)));
	xv_acc.acc_roll_d	=	rad2deg( asin(limit(xv_acc.acc_data2_d, 1.0f, -1.0f)));

	/***	check fall down	for servo off		***/
	xv_acc.acc_pitch2	=	rad2deg( asin(limit(xv_acc.acc_data1, 1.0f, -1.0f)));
	xv_acc.acc_roll2	=	rad2deg( asin(limit(xv_acc.acc_data2, 1.0f, -1.0f)));

#ifdef	ACC_SENSOR
	/***	check fall down	for auto standing up		***/
	TIMER( xv_acc.acc_pitch >= xp_acc.fall_fwd, flag_acc.fall_fwd_on, 
			xp_acc.fall_check_time, xv_acc.fall_fwd_work );

	TIMER( xv_acc.acc_pitch <= xp_acc.fall_bwd, flag_acc.fall_bwd_on, 
			xp_acc.fall_check_time, xv_acc.fall_bwd_work );

	TIMER( xv_acc.acc_roll >= xp_acc.fall_right, flag_acc.fall_right_on, 
			xp_acc.fall_check_time, xv_acc.fall_right_work );

	TIMER( xv_acc.acc_roll <= xp_acc.fall_left, flag_acc.fall_left_on, 
			xp_acc.fall_check_time, xv_acc.fall_left_work );

	/***	auto standing up after fall down		***/
	if( flag_acc.fall_fwd_on == ON && flag_acc.fall_bwd_on == OFF )
	{
		flag_acc.standup_select	=	STANDUP_FWD;
	}
	else if( flag_acc.fall_bwd_on == ON && flag_acc.fall_fwd_on == OFF )
	{
		flag_acc.standup_select	=	STANDUP_BWD;
	}
	else if( flag_acc.fall_right_on == ON && flag_acc.fall_left_on == OFF )
	{
		flag_acc.standup_select	=	STANDUP_RIGHT;
	}
	else if( flag_acc.fall_left_on == ON && flag_acc.fall_right_on == OFF )
	{
		flag_acc.standup_select	=	STANDUP_LEFT;
	}
	else
	{
		flag_acc.standup_select	=	OFF;
	}

	/***	check fall down	***/
	if( xv_acc.acc_roll >= xp_acc.fall_roll || xv_acc.acc_roll <= -xp_acc.fall_roll)
	{
	     if( xv_acc.acc_pitch >= 0 )	flag_acc.fall		=	3;
	     else flag_acc.fall		=	4;
	}
	else if(( xv_acc.acc_pitch >= xp_acc.fall_pitch ) ||
			(( xv_acc.acc_pitch >= xp_acc.fall_pitch_oblique) && (fabs(xv_acc.acc_roll) >= xp_acc.fall_roll_oblique)))
		flag_acc.fall		=	1;

	else if(( xv_acc.acc_pitch <= -xp_acc.fall_pitch ) || 
			(( xv_acc.acc_pitch <= -xp_acc.fall_pitch_oblique) && (fabs(xv_acc.acc_roll) >= xp_acc.fall_roll_oblique)))
		flag_acc.fall		=	2;	
	else if((fabs(xv_acc.acc_pitch) < 0.5 * xp_acc.fall_pitch) &&
				(fabs(xv_acc.acc_roll) < 0.5 * xp_acc.fall_roll)) {
			flag_acc.fall	=	0;
			flag_motion_accept	=	ON;
			//if( flag_ukemi_start !=	ON)	flag_motion_accept	=	ON;
		}

	
//	begin 2010.5.9
	
	TIMER(flag_ukemi_start, flag_restart, 1000, flag_restart_work);
	if(flag_restart == ON) {
		flag_restart		=	OFF;
		flag_ukemi_start	=	OFF;
		flag_servo_restart	=	OFF;
		flag_restart_work	=	0;
		flag_motion_accept_work = 0;
		flag_motion_accept	=	ON;
	} else if(flag_ukemi == 1) {
		TIMER(flag_ukemi_start, flag_ukemi_finished, 100, flag_ukemi_finished_work);
		TIMER(flag_ukemi_start, flag_motion_accept, 300, flag_motion_accept_work);
		if((flag_ukemi_finished == ON) && (flag_servo_restart == OFF)) {
			flag_ukemi_finished	=	OFF;
			flag_servo_restart	=	ON;
			sq_flag.start		=	ON;
			xv_mv.count					=	0;
			flag_ukemi_finished_work	=	0;
		} else if(flag_acc.fall_fwd_on || flag_acc.fall_bwd_on || flag_acc.fall_right_on || flag_acc.fall_left_on) {
			if(flag_ukemi_start == OFF){
				flag_ukemi_start	= ON;
				flag_servo_off		= ON;
				flag_motion_accept	= OFF;
			}
		}

	}
//	end 2010.5.9

#endif
}


/*--------------------------------------*/
/*	acc_init							*/
/*--------------------------------------*/
void	acc_init( void )
{
//	acc sensor = ADXL300
	xp_acc.acc_k1				=	3.1f;		/*	3.03 [G/V] = 0.33V/G	*/
	xp_acc.acc_k2				=	-3.1f;		/*	[G/V]					*/
	xp_acc.acc_k3				=	3.1f;		/*	[G/V]					*/
	xp_acc.ad_volt_offset1		=	-1.65f;		/*	-1.65[V]	*/
	xp_acc.ad_volt_offset2		=	-1.65f;		/*	-1.65[V]	*/
	xp_acc.ad_volt_offset3		=	-1.65f;		/*	-1.65[V]	*/
	xp_acc.t1					=	100.f;		/*	[msec]		*/
	xp_acc.t2					=	300.f;		/*	diff T2=3*T1[msec]	*/

	xp_acc.fall_fwd				=	40.f;			/*	[deg]	*/
	xp_acc.fall_bwd				=	-40.f;			/*	[deg]	*/
	xp_acc.fall_right			=	40.f;			/*	[deg]	*/
	xp_acc.fall_left			=	-40.f;			/*	[deg]	*/
	xp_acc.fall_check_time		=	1.f * RTC_TICK;	/*	1 [sec]	*/
	xp_acc.fall_pitch			=	40.f;			/*	[deg]	*/
	xp_acc.fall_roll			=	40.f;			/*	[deg]	*/
	xp_acc.fall_pitch_oblique	=	30.f;			/*	[deg]	*/
	xp_acc.fall_roll_oblique	=	40.f;			/*	[deg]	*/

	xv_acc.acc_data1			=
	xv_acc.acc_data2			=
	xv_acc.acc_data3			=
	xv_acc.acc_data1_d			=
	xv_acc.acc_data2_d			=
	xv_acc.acc_data3_d			=
	xv_acc.acc_data1_flt		=
	xv_acc.acc_data2_flt		=
	xv_acc.acc_data3_flt		=
	xv_acc.acc_pitch			=
	xv_acc.acc_roll				=
	xv_acc.acc_pitch_d			=
	xv_acc.acc_roll_d			=	0.;
	xv_acc.fall_fwd_work		=
	xv_acc.fall_bwd_work		=
	xv_acc.fall_right_work		=
	xv_acc.fall_left_work		=	0;
	flag_restart_work			=	0;
	flag_ukemi_finished_work	=	0;
	flag_motion_accept_work		=	0;

	flag_acc.zero				=
	flag_acc.fall_fwd_on		=
	flag_acc.fall_bwd_on		=
	flag_acc.fall_right_on		=
	flag_acc.fall_left_on		=
	flag_acc.fall				=
	flag_acc.standup_select		=	OFF;
	flag_restart				=	OFF;
	flag_ukemi_finished			=	OFF;
	flag_ukemi_start			=	OFF;
	flag_servo_restart			=	OFF;
	flag_motion_accept			=	ON;
	
}
