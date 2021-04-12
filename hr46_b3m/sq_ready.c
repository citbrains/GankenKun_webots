/*----------------------------------------------------------*/
/*	ready sequence											*/
/*															*/
/*															*/
/*	file name	:	sq_ready.c								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#define		_SQ_READY_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	"var.h"
#include	"sq_ready.h"
#include	"calc_mv.h"
#include  	"serv.h"
#include	"sq_walk.h"
#include	"kine.h"
#include	"motion.h"
#include	"gyro.h"

short flag_md_ready_end;
//short mode_sq_ready;

void sq_ready_init(int slow_mode)
{
	flag_md_ready_end 		= 	OFF;
	flag_walk.upleg 		= 	OFF;

	if( slow_mode )
	{
		xv_mv_ready_time	=	xp_mv_ready.time;	/*	slow	*/
		flag_ready_gyro		=	ON;
	}
	else
	{
		xv_mv_ready_time	=	0.04f;				/*	fast	*/
		flag_ready_gyro		=	ON3;
	}

	if( check_sw_ref_d() == JOINT_ANGLE )
	{
		trk_kine();
		set_sw_ref_d(FOOT_XYZ);		//	foot xyz position control
	}

	/* status */
	mode_sq_ready	=	SQ_READY;
}



/*--------------------------------------*/
/*	sq_ready_mode						*/
/*--------------------------------------*/
int sq_ready()
{
	static float	mode_sq_time;

	flag_gyro.vib			=	ON;			//	normal gain

	switch( mode_sq_ready )
	{
		case	SQ_READY:			/*	ready position	*/
			/* action */
			xv_data_x_r.time		=
			xv_data_y_r.time		=
			xv_data_y_r2.time		=
			xv_data_z_r.time		=
			xv_data_x_l.time		=
			xv_data_y_l.time		=
			xv_data_y_l2.time		=
			xv_data_z_l.time		=	xv_mv_ready_time;

			xv_data_x_r.pos			=	0.f;
			xv_data_y_r.pos			=	 xp_mv_walk.y_wide;
			xv_data_y_r2.pos		=	0.f;
			xv_data_z_r.pos			=	xp_mv_ready.z3;
			xv_data_x_l.pos			=	0.f;
			xv_data_y_l.pos			=	-xp_mv_walk.y_wide;
			xv_data_y_l2.pos		=	0.f;
			xv_data_z_l.pos			=	xp_mv_ready.z3;

			/* status */
			mode_sq_ready	=	SQ_READY2;

			break;

		case	SQ_READY2:			/*	ready position	*/
			/* action */
			xv_data_pitch.time		=
			xv_data_roll2.time		=   xv_mv_ready_time;

			xv_data_roll2.pos		=   0.0;

			xv_data_pitch.pos		=	xp_mv_ready.pitch;

			/* status */
			mode_sq_ready	=	SQ_READY3;

			break;

		case	SQ_READY3:			/*	ready position	*/
			/* action */
			xv_data_d[LEG_YAW_L].time		=
			xv_data_d[LEG_YAW_R].time		=
			xv_data_d[12].time				=
			xv_data_d[13].time				=	xv_mv_ready_time;

			xv_data_d[LEG_YAW_L].pos		=
			xv_data_d[LEG_YAW_R].pos		=
			xv_data_d[12].pos				=
			xv_data_d[13].pos				=	0.f;

			xv_data_d[ELBOW_PITCH_L].time =
			xv_data_d[ELBOW_PITCH_R].time =
			xv_data_d[ARM_PITCH_L].time		=
			xv_data_d[ARM_PITCH_R].time		=	xv_mv_ready_time;
			xv_data_d[ELBOW_PITCH_L].pos	= xp_mv_ready.arm_el_pitch;
			xv_data_d[ELBOW_PITCH_R].pos	= xp_mv_ready.arm_el_pitch;
			xv_data_d[ARM_PITCH_L].pos		=	
			xv_data_d[ARM_PITCH_R].pos		=	xp_mv_ready.arm_sh_pitch;

			/* status */
			mode_sq_ready	=	SQ_READY4;

			break;

		case	SQ_READY4:			/*	ready position	*/
			/* action */
			if( !flag_face_control )
			{
				xv_data_d[HEAD_YAW].time		=
				xv_data_d[HEAD_PITCH].time		=	xv_mv_ready_time;
				xv_data_d[HEAD_YAW].pos			=
				xv_data_d[HEAD_PITCH].pos		=	0.f;
			}

			xv_data_d[ARM_ROLL_L].time		=
			xv_data_d[ARM_ROLL_R].time		=	xv_mv_ready_time;

			xv_data_d[ARM_ROLL_L].pos		=	-xp_mv_ready.arm_sh_roll;
			xv_data_d[ARM_ROLL_R].pos		=	xp_mv_ready.arm_sh_roll;

			/* status */
			mode_sq_ready	=	SQ_READY_END;	mode_sq_time	=	0.f;

			break;

		case	SQ_READY_END:		/*	end		*/
			if( mode_sq_time >= (xv_mv_ready_time - EPS_TIME) )
			{
				flag_gyro.vib	=	ON2;		//	low gain
				sq_flag.ready	=	OFF;
				flag_md_ready_end 	= 	ON;
				mode_sq_ready	=	SQ_READY_INIT;
				mode_sq_time	=	0.f;

				flag_ready_gyro		=	OFF;

				set_sw_ref_d(FOOT_XYZ);		//	foot xyz position control
			}

			break;

		default:
			break;
	}

	mode_sq_time	+=	RTC_TIME_SEC;
	return flag_md_ready_end;
}
