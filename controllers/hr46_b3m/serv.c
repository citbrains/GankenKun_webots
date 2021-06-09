/*----------------------------------------------------------*/
/*	servo control											*/
/*															*/
/*															*/
/*	file name	:	serv.c									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.7								*/
/*----------------------------------------------------------*/
#define		_SERV_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	"var.h"
#include	"func.h"
#include	"serv.h"
#include	"kine.h"
#include	"motion.h"
#include	"calc_mv.h"
#include	"servo_rs.h"
#include	"b3m.h"
#include	"sq_walk.h"
#include	"sq_straight.h"
#include	"sq_ready.h"



/*--------------------------------------*/
/*	serv								*/
/*--------------------------------------*/
void 	serv( void )
{
	short	i;
	long	offset, hlim, llim;
	float	w;

//	if((!(mode_motion == MOTION_STRAIGHT && mode_sq_straight == SQ_STRAIGHT_END))
//		&& (!(mode_motion == MOTION_READY && mode_sq_ready == SQ_READY_END)))
	if( 1 )
	{
		xv_ref.d[LEG_ROLL_R]	=	sw2( sw.ref_d, xv_kine[0].hip_r, xv_ref.d_ref[LEG_ROLL_R] );		//	right 	hip (roll)
		xv_ref.d[LEG_PITCH_R]	=	sw2( sw.ref_d, xv_kine[0].leg, xv_ref.d_ref[LEG_PITCH_R] );		//	right	leg
		xv_ref.d[KNEE_R2]		=	sw2( sw.ref_d, xv_kine[0].knee, xv_ref.d_ref[KNEE_R2] );				//	right	knee
		xv_ref.d[KNEE_R1]		=	sw2( sw.ref_d, xv_kine[0].foot_p, xv_ref.d_ref[KNEE_R1] );				//	right	foot (pitch)
		xv_ref.d[FOOT_ROLL_R]	=	sw2( sw.ref_d, xv_kine[0].foot_r, xv_ref.d_ref[FOOT_ROLL_R] );		//	right	foot (roll)

		xv_ref.d[LEG_ROLL_L]	=	sw2( sw.ref_d, xv_kine[1].hip_r, xv_ref.d_ref[LEG_ROLL_L] );		//	left 	hip (roll)
		xv_ref.d[LEG_PITCH_L]	=	sw2( sw.ref_d, xv_kine[1].leg, xv_ref.d_ref[LEG_PITCH_L] );		//	left	leg
		xv_ref.d[KNEE_L2]		=	sw2( sw.ref_d, xv_kine[1].knee, xv_ref.d_ref[KNEE_L2] );				//	left	knee
		xv_ref.d[KNEE_L1]		=	sw2( sw.ref_d, xv_kine[1].foot_p, xv_ref.d_ref[KNEE_L1] );					//	left	foot (pitch)
		xv_ref.d[FOOT_ROLL_L]	=	sw2( sw.ref_d, xv_kine[1].foot_r, xv_ref.d_ref[FOOT_ROLL_L] );		//	left	foot (roll)
	}
	else		// 直立とレディの終了後に呼び出される
	{
		/*	added dlimit prevent from angle gap after special action	*/
		w	=	sw2( sw.ref_d, xv_kine[0].hip_r, xv_ref.d_ref[LEG_ROLL_R] );		//	right 	hip (roll)
		xv_ref.d[LEG_ROLL_R]	=	dlimit( w, xp_ref_d_lim, xv_ref.d[LEG_ROLL_R] );

		w	=	sw2( sw.ref_d, xv_kine[0].leg, xv_ref.d_ref[LEG_PITCH_R] );		//	right	leg
		xv_ref.d[LEG_PITCH_R]	=	dlimit( w, xp_ref_d_lim, xv_ref.d[LEG_PITCH_R] );

		w		=	sw2( sw.ref_d, xv_kine[0].knee, xv_ref.d_ref[KNEE_R2] );			//	right	knee
		xv_ref.d[KNEE_R2]	=	dlimit( w, xp_ref_d_lim, xv_ref.d[KNEE_R2] );

		w	=	sw2( sw.ref_d, xv_kine[0].foot_p, xv_ref.d_ref[KNEE_R1] );				//	right	foot (pitch)
		xv_ref.d[KNEE_R1]	=	dlimit( w, xp_ref_d_lim, xv_ref.d[KNEE_R1] );

		w	=	sw2( sw.ref_d, xv_kine[0].foot_r, xv_ref.d_ref[FOOT_ROLL_R] );		//	right	foot (roll)
		xv_ref.d[FOOT_ROLL_R]	=	dlimit( w, xp_ref_d_lim, xv_ref.d[FOOT_ROLL_R] );

		w	=	sw2( sw.ref_d, xv_kine[1].hip_r, xv_ref.d_ref[LEG_ROLL_L] );		//	left 	hip (roll)
		xv_ref.d[LEG_ROLL_L]	=	dlimit( w, xp_ref_d_lim, xv_ref.d[LEG_ROLL_L] );

		w	=	sw2( sw.ref_d, xv_kine[1].leg, xv_ref.d_ref[LEG_PITCH_L] );		//	left	leg
		xv_ref.d[LEG_PITCH_L]	=	dlimit( w, xp_ref_d_lim, xv_ref.d[LEG_PITCH_L] );

		w		=	sw2( sw.ref_d, xv_kine[1].knee, xv_ref.d_ref[KNEE_L2] );			//	left	knee
		xv_ref.d[KNEE_L2]	=	dlimit( w, xp_ref_d_lim, xv_ref.d[KNEE_L2] );

		w	=	sw2( sw.ref_d, xv_kine[1].foot_p, xv_ref.d_ref[KNEE_L1] );				//	left	foot (pitch)
		xv_ref.d[KNEE_L1]	=	dlimit( w, xp_ref_d_lim, xv_ref.d[KNEE_L1] );

		w	=	sw2( sw.ref_d, xv_kine[1].foot_r, xv_ref.d_ref[FOOT_ROLL_L] );		//	left	foot (roll)
		xv_ref.d[FOOT_ROLL_L]	=	dlimit( w, xp_ref_d_lim, xv_ref.d[FOOT_ROLL_L] );
	}

	xv_ref.d[LEG_YAW_R]		=	( xv_ref.d_ref[LEG_YAW_R] + xv_posture.yaw );	//	right	hip (yaw)
	xv_ref.d[LEG_YAW_L]		=	( xv_ref.d_ref[LEG_YAW_L] + xv_posture.yaw );	//	left	hip (yaw)

	xv_ref.d[SPARE12]		=	xv_ref.d_ref[SPARE12];			//	body (pitch)
	xv_ref.d[SPARE13]		=	xv_ref.d_ref[SPARE13];			//	body (yaw)

	xv_ref.d[ARM_PITCH_L]	=	xv_ref.d_ref[ARM_PITCH_L];		//	left	shoulder (pitch)
	xv_ref.d[ARM_ROLL_L]	=	xv_ref.d_ref[ARM_ROLL_L];		//	left	shoulder (roll)
	xv_ref.d[ELBOW_PITCH_L]		=	xv_ref.d_ref[ELBOW_PITCH_L];			//	left	elbow (yaw)
	xv_ref.d[SPARE17]		=	xv_ref.d_ref[SPARE17];			//	left	elbow (pitch)

	xv_ref.d[ARM_PITCH_R]	=	xv_ref.d_ref[ARM_PITCH_R];		//	right	shoulder (pitch)
	xv_ref.d[ARM_ROLL_R]	=	xv_ref.d_ref[ARM_ROLL_R];		//	right	shoulder (roll)
	xv_ref.d[ELBOW_PITCH_R]		=	xv_ref.d_ref[ELBOW_PITCH_R];			//	right	elbow (yaw)
	xv_ref.d[SPARE21]		=	xv_ref.d_ref[SPARE21];			//	right   elbow (pitch)

	xv_ref.d[HEAD_YAW]		=	xv_ref.d_ref[HEAD_YAW];			//	head (yaw)(pan)
	xv_ref.d[HEAD_PITCH]	=	xv_ref.d_ref[HEAD_PITCH];		//	head (pitch)(tilt)
	xv_ref.d[SPARE24]		=	xv_ref.d_ref[SPARE24];	//	spare
	xv_ref.d[SPARE25]		=	xv_ref.d_ref[SPARE25];	//	spare
	xv_ref.d[SPARE26]		=	xv_ref.d_ref[SPARE26];	//	spare
	xv_ref.d[SPARE27]		=	xv_ref.d_ref[SPARE27];	//	spare
	xv_ref.d[SPARE28]		=	xv_ref.d_ref[SPARE28];	//	spare


	for( i=0; i<SERV_NUM; i++ )
	{
		xv_sv[i].d	=	(long)(xv_ref.d[i] * 100);

		offset	=	xp_sv[i].deg_offset * 100;

		xv_sv[i].deg_sw_out	=	xv_sv[i].d;

 		xv_sv[i].deg_sv		=	xv_sv[i].deg_sw_out * xp_sv[i].deg_sign + offset;

		hlim	=	xp_sv[i].deg_lim_h * 100;
		llim	=	xp_sv[i].deg_lim_l * 100;
		xv_sv[i].deg_lim	=	limit( xv_sv[i].deg_sv, hlim, llim );

		xv_sv[i].pls		=	((xv_sv[i].deg_lim + xp_sv[i].deg_lim_offset*100) * xp_sv[i].deg2pls) / 10000;
		xv_sv[i].pls_out	=	limit( xv_sv[i].pls, 1500, -1500 );		/*	RS405CB limit +150[deg] ... -150[deg]	*/
		xv_servo_rs.goal_position[i]	=	(unsigned short)(xv_sv[i].deg_lim);
	}
	xv_servo_rs.goal_position[LEG_ROLL_R] *= REDUCTION_RATION;
	xv_servo_rs.goal_position[LEG_ROLL_L] *= REDUCTION_RATION;
}


/*--------------------------------------*/
/*	set_sw_ref_d						*/
/*--------------------------------------*/
void	set_sw_ref_d( int n )
{
	int		i;

	/* 1: foot xyz position control	*/
	/* 2: joint angle control	*/

	if( n == JOINT_ANGLE )	// joint angle control
	{
		for( i = 0; i < SERV_NUM; i++ )
		{
			if ( FOOT_ROLL_R <= i && LEG_YAW_R >= i ||
				 FOOT_ROLL_L <= i && LEG_YAW_L >= i ) {
				
			}
			//	tracking
			xv_ref.d_ref[i]			=	xv_ref.d[i];
			xv_mvdata_d[i].amp		=	0.f;
			xv_mvdata_d[i].start	=
			xv_mvdata_d[i].out_old	=
			xv_mvdata_d[i].pos_old	=	xv_ref.d[i];

			xv_data_d[i].time		=	0.1f;
			xv_data_d[i].pos		=	xv_ref.d[i];

		}
		sw.ref_d					=	JOINT_ANGLE;
	}
	else					// foot xyz position control
	{
		for( i = 0; i < SERV_NUM; i++ )
		{
			if( i != ARM_PITCH_R && i != ARM_ROLL_R && i != ELBOW_PITCH_R &&
				i != ARM_PITCH_L && i != ARM_ROLL_L && i != ELBOW_PITCH_L &&
				i != HEAD_YAW	&& 	i != HEAD_PITCH )
				xv_data_d[i].pos	=	0.f;
		}
		sw.ref_d					=	FOOT_XYZ;
	}
}


/*--------------------------------------*/
/*	check_sw_ref_d						*/
/*--------------------------------------*/
short	check_sw_ref_d( void )
{
	return		sw.ref_d;
}
