/*----------------------------------------------------------*/
/*	straight sequence										*/
/*															*/
/*															*/
/*	file name	:	sq_straight.c							*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#define		_SQ_STRAIGHT_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	"var.h"
#include	"sq_straight.h"
#include	"calc_mv.h"
#include	"serv.h"
#include	"gyro.h"
#include	"sq_walk.h"
#include	"kine.h"
#include	"motion.h"
#include	"servo_rs.h"
#include	"b3m.h"

short flag_md_straight_end;				// mode‚ÌI—¹ƒtƒ‰ƒO
//short mode_sq_straight;

void sq_straight_init()
{
	flag_md_straight_end 	= 	OFF;

	xv_mv_straight_time	=	xp_mv_straight.time;	/*	slow	*/

	if( !flag_servo_on  && mode_motion != MOTION_START )
	{
		xv_mv_straight_time	=	0.2f;				/*	fast	*/
	}

	if( check_sw_ref_d() == JOINT_ANGLE )
	{
		trk_kine();
	}

	/* status */
	mode_sq_straight	=	SQ_STRAIGHT;
}



/*--------------------------------------*/
/*	sq_straight_mode					*/
/*--------------------------------------*/
int	sq_straight()
{
	static float	mode_sq_time;
	
	flag_gyro.vib			=	ON;			//	normal gain
	switch( mode_sq_straight )
	{
		case	SQ_STRAIGHT:		/*	straight position	*/
			/* action */
			xv_data_x_r.time		=
			xv_data_y_r.time		=
			xv_data_y_r2.time		=
			xv_data_z_r.time		=
			xv_data_x_l.time		=
			xv_data_y_l.time		=
			xv_data_y_l2.time		=
			xv_data_z_l.time		=	xv_mv_straight_time;

			xv_data_x_r.pos			=	0.f;
			xv_data_y_r.pos			=	0.f;
			xv_data_y_r2.pos		=	0.f;
			xv_data_z_r.pos			=	xp_mv_straight.z3;
			xv_data_x_l.pos			=	0.f;
			xv_data_y_l.pos			=	0.f;
			xv_data_y_l2.pos		=	0.f;
			xv_data_z_l.pos			=	xp_mv_straight.z3;

			/* status */
			mode_sq_straight	=	SQ_STRAIGHT2;

			break;

		case	SQ_STRAIGHT2:		/*	straight position	*/
			/* action */
			xv_data_pitch.time		=
			xv_data_roll2.time		=	xv_mv_straight_time;

			xv_data_pitch.pos		=
			xv_data_roll2.pos		=	0.f;

			/* status */
			mode_sq_straight	=	SQ_STRAIGHT3;

			break;

		case	SQ_STRAIGHT3:		/*	straight position	*/
			/* action */
			xv_data_d[ 0].time		=
			xv_data_d[ 1].time		=
			xv_data_d[ 2].time		=
			xv_data_d[ 3].time		=
			xv_data_d[ 4].time		=
			xv_data_d[ 5].time		=
			xv_data_d[ 6].time		=
			xv_data_d[ 7].time		=	xv_mv_straight_time;

			xv_data_d[ 0].pos		=
			xv_data_d[ 1].pos		=
			xv_data_d[ 2].pos		=
			xv_data_d[ 3].pos		=
			xv_data_d[ 4].pos		=
			xv_data_d[ 5].pos		=
			xv_data_d[ 6].pos		=
			xv_data_d[ 7].pos		=	0.f;

			/* status */
			mode_sq_straight	=	SQ_STRAIGHT4;

			break;

		case	SQ_STRAIGHT4:		/*	straight position	*/
			/* action */
			xv_data_d[ 8].time		=
			xv_data_d[ 9].time		=
			xv_data_d[10].time		=
			xv_data_d[11].time		=
			xv_data_d[12].time		=
			xv_data_d[13].time		=	xv_mv_straight_time;

			xv_data_d[ 8].pos		=
			xv_data_d[ 9].pos		=
			xv_data_d[10].pos		=
			xv_data_d[11].pos		=
			xv_data_d[12].pos		=
			xv_data_d[13].pos		=	0.f;

			xv_data_d[ARM_PITCH_L].time		=
			xv_data_d[ARM_PITCH_R].time		=
			xv_data_d[ARM_ROLL_L].time		=
			xv_data_d[ARM_ROLL_R].time		=	xv_mv_straight_time;
			xv_data_d[ARM_PITCH_L].pos		=
			xv_data_d[ARM_PITCH_R].pos		=	xp_mv_straight.arm_sh_pitch;
			xv_data_d[ARM_ROLL_L].pos		=	-xp_mv_straight.arm_sh_roll;
			xv_data_d[ARM_ROLL_R].pos		=	xp_mv_straight.arm_sh_roll;

			/* status */
			mode_sq_straight	=	SQ_STRAIGHT5;

			break;

		case	SQ_STRAIGHT5:		/*	straight position	*/
			/* action */
			if( !flag_face_control )
			{
				xv_data_d[HEAD_YAW].time		=
				xv_data_d[HEAD_PITCH].time		=	xv_mv_straight_time;
				xv_data_d[HEAD_YAW].pos			=
				xv_data_d[HEAD_PITCH].pos		=	0.f;
			}

			xv_data_d[ELBOW_PITCH_L].time		=
			xv_data_d[SPARE17].time		=
			xv_data_d[ELBOW_PITCH_R].time		=
			xv_data_d[SPARE21].time		=	xv_mv_straight_time;

			xv_data_d[ELBOW_PITCH_L].pos		=	xp_mv_straight.arm_el_pitch;
			xv_data_d[SPARE17].pos		=	xp_mv_straight.arm_el_yaw;
			xv_data_d[ELBOW_PITCH_R].pos		=	xp_mv_straight.arm_el_pitch;
			xv_data_d[SPARE21].pos		=	xp_mv_straight.arm_el_yaw;

			/* status */
			mode_sq_straight	=	SQ_STRAIGHT_END;
			mode_sq_time	=	0.f;

			break;

		case	SQ_STRAIGHT_END:	/*	end		*/
			if( mode_sq_time >= (xv_mv_straight_time - EPS_TIME) )
			{
				flag_gyro.vib	=	ON2;		//	low gain
				/* action */
				set_sw_ref_d(FOOT_XYZ);			//	foot xyz position control

//				xv_gyro.gyro_roll		=
//				xv_gyro.gyro_pitch		=
//				xv_gyro.gyro_yaw		=
//				xv_gyro.gyro_yaw2		=	0.f;

				/* status */
				sq_flag.straight	=	OFF;
				flag_md_straight_end 	= 	ON;
//				mode_sq_straight	=	SQ_STRAIGHT_INIT;
				mode_sq_time	=	0.f;
			}

			break;

		default:
			break;
	}

	mode_sq_time	+=	RTC_TIME_SEC;

	return flag_md_straight_end;
}
