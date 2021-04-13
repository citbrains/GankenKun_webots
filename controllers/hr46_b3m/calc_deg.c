/*----------------------------------------------------------*/
/*	caluculation degrees after kine							*/
/*															*/
/*															*/
/*	file name	:	calc_deg.c								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.5.30								*/
/*----------------------------------------------------------*/
#define		_CALC_DEG_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include    <math.h>
#include	"var.h"
#include	"func.h"
#include	"calc_mv.h"
#include	"kine.h"
#include	"acc.h"
#include	"gyro.h"
#include	"serv.h"
#include	"sq_walk.h"


/*--------------------------------------*/
/*	calc_deg							*/
/*--------------------------------------*/
void	calc_deg( void )
{
	/*** calculate foot roll	***/
	/* foot is paralell to the ground	*/
	if( flag_walk.upleg == RIGHT )										/*	right leg is up	*/
	{
		xv_kine[0].foot_r	=	- xv_kine[0].hip_r;						/*	right foot roll */
		xv_kine[1].foot_r	=	- xv_kine[1].hip_r;						/*	left foot roll 	*/
		xv_kine[1].hip_r -= 2;
	}
	else if( flag_walk.upleg  == LEFT )									/*	right leg is up	*/
	{
		xv_kine[0].foot_r	=	- xv_kine[0].hip_r;						/*	right foot roll */
		xv_kine[1].foot_r	=	- xv_kine[1].hip_r;						/*	left foot roll 	*/
		xv_kine[0].hip_r += 2;
	}
	else																/*	both legs touches to the ground	*/
	{
		xv_kine[0].foot_r	=	- xv_kine[0].hip_r;						/*	right foot roll */
		xv_kine[1].foot_r	=	- xv_kine[1].hip_r;						/*	left foot roll 	*/
	}

	/***	calculate posture pitch ***/
	xv_kine[0].leg		+=	xv_posture.pitch;							/*	right leg pitch	*/
	xv_kine[1].leg	 	+=	xv_posture.pitch;							/*	left leg pitch	*/

	/***	calculate posture roll ***/
	xv_kine[0].hip_r	+=	xv_posture.roll2;							/*	right hip roll	*/
	xv_kine[1].hip_r	+=	xv_posture.roll2;							/*	lef hip roll	*/

	/***	calculate gyro feedback control ***/
	gyro_cntr_fun();
	//PostureControl();

	/***	foot touch control	***/
	if( flag_walk.upleg == RIGHT )										/*	right leg is up		*/
	{
		if( flag_walk.y_on == RIGHT )									/*	walk right					*/
			xv_kine[0].foot_r		+=	xp_mv_walk.foot_cntl_r;			/*	right foot right edge up	*/
		else if( flag_walk.y_on == LEFT )								/*	walk left					*/
			xv_kine[0].foot_r		-=	xp_mv_walk.foot_cntl_r;			/*	right foot left edge up		*/
	}
	else if( flag_walk.upleg == LEFT )									/*	left leg is up		*/
	{
		if( flag_walk.y_on == RIGHT )									/*	walk right					*/
			xv_kine[1].foot_r		+=	xp_mv_walk.foot_cntl_r;			/*	left foot right edge up		*/
		else if( flag_walk.y_on == LEFT )								/*	walk left					*/
			xv_kine[1].foot_r		-=	xp_mv_walk.foot_cntl_r;			/*	left foot left edge up		*/
	}
}


/*--------------------------------------*/
/*	calc_z								*/
/*--------------------------------------*/
/***	leg length re-calculation by hip roll	***/
void	calc_z( void )
{
	xv_mv_walk.sidestep_roll_z	=	WIDTH / 2.0f * (float)tan(deg2rad(xv_posture.roll2));
	if( xv_posture.roll2 > EPS)
	{
		xv_kine[0].z	-=	xv_mv_walk.sidestep_roll_z;
	}
	else if( xv_posture.roll2 < -EPS)
	{
		xv_kine[1].z	+=	xv_mv_walk.sidestep_roll_z;
	}
}
