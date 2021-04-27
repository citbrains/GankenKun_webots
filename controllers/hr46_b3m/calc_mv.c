/*----------------------------------------------------------*/
/*	calculation trajectry									*/
/*															*/
/*															*/
/*	file name	:	calc_mv.c								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#define		_CALC_MV_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	<stdio.h>
#include    <math.h>
#include	"motion.h"
#include	"var.h"
#include	"func.h"
#include	"mvtbl.h"
#include	"calc_mv.h"
#include	"kine.h"
#include	"serv.h"
#include	"sq_straight.h"
#include	"sq_walk.h"
#include	"motion.h"


/*--------------------------------------*/
/*	calc_mv initialize					*/
/*--------------------------------------*/
void	calc_mv_init( void )
{
	int	i;


	for( i = 0; i < SERV_NUM; i++ )
	{
		xv_mvdata_d[i].t		=	0.f;
		xv_mvdata_d[i].mv_tbl	=	MV_TBL_PI;
		xv_mvdata_d[i].dt		=	MV_TBL_PI;
		xv_mvdata_d[i].start	=	0.f;
		xv_mvdata_d[i].amp		=	0.f;
		xv_mvdata_d[i].out_old	=	0.f;
		xv_mvdata_d[i].pos_old	=	0.f;
	}
	for( i = 0; i < MVDATA_NUM; i++ )
	{
		xv_mvdata[i].t			=	0.f;
		xv_mvdata[i].mv_tbl		=	MV_TBL_PI;
		xv_mvdata[i].dt			=	MV_TBL_PI;
		xv_mvdata[i].start		=	0.f;
		xv_mvdata[i].amp		=	0.f;
		xv_mvdata[i].out_old	=	0.f;
		xv_mvdata[i].pos_old	=	0.f;
	}

	for( i = 0; i < SERV_NUM; i++ )
	{
		xv_data_d[i].time			=	0.1f;
		xv_data_d[i].pos			=	0.f;
		xv_data_d[i].mv_tbl_select	=	0;
	}

	xv_data_x_r.time			=	0.1f;
	xv_data_x_r.pos				=	0.f;
	xv_data_x_r.mv_tbl_select	=	0;
	xv_data_y_r.time			=	0.1f;
	xv_data_y_r.pos				=	0.f;
	xv_data_y_r.mv_tbl_select	=	0;
	xv_data_y_r2.time			=	0.1f;
	xv_data_y_r2.pos			=	0.f;
	xv_data_y_r2.mv_tbl_select 	=	0;
	xv_data_z_r.time			=	0.1f;
	xv_data_z_r.pos				=	xp_mv_straight.z3;
	xv_data_z_r.mv_tbl_select	=	0;
	xv_data_x_l.time			=	0.1f;
	xv_data_x_l.pos				=	0.f;
	xv_data_x_l.mv_tbl_select	=	0;
	xv_data_y_l.time			=	0.1f;
	xv_data_y_l.pos				=	0.f;
	xv_data_y_l.mv_tbl_select	=	0;
	xv_data_y_l2.time			=	0.1f;
	xv_data_y_l2.pos			=	0.f;
	xv_data_y_l2.mv_tbl_select 	=	0;
	xv_data_z_l.time			=	0.1f;
	xv_data_z_l.pos				=	xp_mv_straight.z3;
	xv_data_z_l.mv_tbl_select	=	0;
	xv_data_pitch.time			=	0.1f;
	xv_data_pitch.pos			=	0.f;
	xv_data_pitch.mv_tbl_select	=	0;
	xv_data_roll2.time			=	0.1f;
	xv_data_roll2.pos			=	0.f;
	xv_data_roll2.mv_tbl_select	=	0;

	xv_mvdata[ 2].start			=
	xv_mvdata[ 2].out_old		=
	xv_mvdata[ 2].pos_old		=	xp_mv_straight.z3;
	xv_mvdata[ 5].start			=
	xv_mvdata[ 5].out_old		=
	xv_mvdata[ 5].pos_old		=	xp_mv_straight.z3;

	xv_odometry.moveX			=	0.f;
	xv_odometry.moveY			=	0.f;
	xv_odometry.rotZ			=	0.f;
	xv_odometry.x[0]			=
	xv_odometry.x[1]			=
	xv_odometry.x_old[0]		=
	xv_odometry.x_old[1]		=
	xv_odometry.y[0]			=
	xv_odometry.y[1]			=
	xv_odometry.y_old[0]		=
	xv_odometry.y_old[1]		=
	xv_odometry.theta[0]		=
	xv_odometry.theta[1]		=
	xv_odometry.theta_old[0]	=
	xv_odometry.theta_old[1]	=	0.f;
}


/*--------------------------------------*/
/*	calc_mv								*/
/*--------------------------------------*/
/***	calculate trajectory tables		***/
void 	calc_mv( void )
{
	int		i;
	float	_xv_kine0y, _xv_kine1y, _xv_kine0y2, _xv_kine1y2;
	float	_tmp_x, _tmp_y, _sinx, _cosx;

	/***	calculate trajectory tables for joint angle	***/
	for( i = 0; i < SERV_NUM; i++ )
	{
		xv_ref.d_ref[i]	=	calc_mvdata( &xv_mvdata_d[i], &xv_data_d[i] );
	}

	if(mode_motion != MOTION_MOTION )
	{
		/***	calculate trajectory tables for inv. kinetics XY pooint	***/
		xv_kine[0].x	=	calc_mvdata( &xv_mvdata[ 0], &xv_data_x_r	);		/*	right foot x			*/
		_xv_kine0y		=	calc_mvdata( &xv_mvdata[ 1], &xv_data_y_r	);		/*	right foot y(zmp)		*/
		xv_kine[0].z	=	calc_mvdata( &xv_mvdata[ 2], &xv_data_z_r	);		/*	right foot z			*/
		xv_kine[1].x	=	calc_mvdata( &xv_mvdata[ 3], &xv_data_x_l	);		/*	left foot x				*/
		_xv_kine1y		=	calc_mvdata( &xv_mvdata[ 4], &xv_data_y_l	);		/*	left foot y(zmp)		*/
		xv_kine[1].z	=	calc_mvdata( &xv_mvdata[ 5], &xv_data_z_l	);		/*	left foot z				*/
		_xv_kine0y2		=	calc_mvdata( &xv_mvdata[ 9], &xv_data_y_r2	);		/*	right foot y(side step)	*/
		_xv_kine1y2		=	calc_mvdata( &xv_mvdata[10], &xv_data_y_l2	);		/*	left foot y(side step)	*/
		xv_kine[0].y	=	_xv_kine0y + _xv_kine0y2;							/*	right foot y total		*/
		xv_kine[1].y	=	_xv_kine1y + _xv_kine1y2;							/*	left foot y total		*/

		/***	calculate trajectory tables for pitch and roll posture	***/
		xv_posture.pitch	=	calc_mvdata( &xv_mvdata[ 6], &xv_data_pitch	);	/*	leg pitch			*/		// TODO: 今は足首のピッチになっていない．修正必要
//		xv_posture.pitch	=	xv_data_pitch.pos;								/*	leg pitch			*/		// TODO: 今は足首のピッチになっていない．修正必要
		xv_posture.roll2	=	calc_mvdata( &xv_mvdata[11], &xv_data_roll2	);	/*	roll				*/
	}

	/***	calculate odometry	***/
	xv_odometry.x[0]		=	xv_kine[0].x;
	xv_odometry.x[1]		=	xv_kine[1].x;
	xv_odometry.y[0]		=	xv_kine[0].y;
	xv_odometry.y[1]		=	xv_kine[1].y;
	xv_odometry.theta[0]	=	xv_ref.d[LEG_YAW_R];
	xv_odometry.theta[1]	=	xv_ref.d[LEG_YAW_L];

	if( flag_walk.upleg == LEFT )
		xv_odometry.rotZ			-=	(xv_odometry.theta[0] - xv_odometry.theta_old[0]); 	/*	tracks of right leg	*/
	else 
		xv_odometry.rotZ			-=	(xv_odometry.theta[1] - xv_odometry.theta_old[1]); 	/*	tracks of left leg	*/

	if( flag_walk.upleg == LEFT )
	{
		/*	tracks of right leg	*/
		_tmp_x	=	(xv_odometry.x[0] - xv_odometry.x_old[0])*odometry_correct_para_x;
//		_tmp_y	=	(xv_odometry.y[0] - xv_odometry.y_old[0])*odometry_correct_para_y;	//オドメトリ補正値　08.03.20	
		_tmp_y	=	((xv_odometry.y[0] - xv_odometry.y[1]) - (xv_odometry.y_old[0] - xv_odometry.y_old[1]))*odometry_correct_para_y;	//横方向の振動抑制　14.06.07
	}
	else
	{
		/*	tracks of left leg	*/
		_tmp_x	=	(xv_odometry.x[1] - xv_odometry.x_old[1])*odometry_correct_para_x;
//		_tmp_y	=	(xv_odometry.y[1] - xv_odometry.y_old[1])*odometry_correct_para_y;	//オドメトリ補正値　08.03.20	
		_tmp_y	=	((xv_odometry.y[1] - xv_odometry.y[0]) - (xv_odometry.y_old[1] - xv_odometry.y_old[0]))*odometry_correct_para_y;	//横方向の振動抑制　14.06.07
	}

	_sinx	=	(float)sin( deg2rad( xv_odometry.rotZ ) );
	_cosx	=	(float)cos( deg2rad( xv_odometry.rotZ ) );

	xv_odometry.moveX			-=	(_tmp_x * _cosx) - (_tmp_y * _sinx);
	xv_odometry.moveY			-=	(_tmp_x * _sinx) - (_tmp_y * _cosx);

	xv_odometry.x_old[0]		=	xv_odometry.x[0];
	xv_odometry.x_old[1]		=	xv_odometry.x[1];
	xv_odometry.y_old[0]		=	xv_odometry.y[0];
	xv_odometry.y_old[1]		=	xv_odometry.y[1];
	xv_odometry.theta_old[0]	=	xv_odometry.theta[0];
	xv_odometry.theta_old[1]	=	xv_odometry.theta[1];
}


/*--------------------------------------*/
/*	calc mvdata							*/
/*--------------------------------------*/
/***	calculate trajectory table	***/
float	calc_mvdata( tp_xv_mvdata *x, tp_xv_data *a )
{
	float	w1;

	w1	=	a->pos - x->pos_old;
	
	if( fabs( w1 ) > EPS )	//	sv data changed
	{
		/*	set mv_data		*/
		x->t			=	0.f;
		x->mv_tbl		=	MV_TBL_PI;
		x->dt			=	MV_TBL_PI / a->time / RTC_TICK;
		x->start		=	x->out_old;
		x->amp			=	a->pos - x->out_old;
	}

	x->t		=	dlimit( x->mv_tbl, x->dt, x->t );

	x->out_old	=	x->amp * mv_tbl[a->mv_tbl_select][(int)(x->t+0.5f)] + x->start;
	x->pos_old 	=	a->pos;

	return	x->out_old;
}

/*--------------------------------------*/
/*	chg_mvtbl							*/
/*--------------------------------------*/
/***	reset trajectory data	***/
void	chg_mvtbl( tp_xv_mvdata *x, tp_xv_data *a )
{
	x->t			=	0.f;
	x->mv_tbl		=	MV_TBL_PI;
	x->dt			=	MV_TBL_PI;
	x->amp			=	0.f;
	x->start		=
	x->pos_old		=	x->out_old;
}
