/*----------------------------------------------------------*/
/*	kinetics caluculation									*/
/*															*/
/*															*/
/*	file name	:	kine.c									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.5.30								*/
/*----------------------------------------------------------*/
#define		_KINE_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	<math.h>
#include	"func.h"
#include	"var.h"
#include	"kine.h"
#include	"serv.h"
#include	"calc_mv.h"
#include	"motion.h"


/*--------------------------------------*/
/*	kine								*/
/*--------------------------------------*/
void 	kine( void )
{
	/***	right leg : xv_kine[0]	***/
	xv_kine[0].yaw	=	xv_ref.d_ref[11];			/*	right hip yaw angle		*/
 	kine_fun( &xv_kine[0].x, &xv_kine[0].hip_r );	/*	calculate right leg		*/

	/***	left leg  : xv_kine[1]	***/
	xv_kine[1].yaw	=	xv_ref.d_ref[5];			/*	left hip yaw angle		*/
 	kine_fun( &xv_kine[1].x, &xv_kine[1].hip_r );	/*	calculate left leg		*/
}


/*------------------------------------------------------*/
/*	calclulation kinematics								*/
/*		input											*/
/*			u[0]	=	X								*/
/*			u[1]	=	Y								*/
/*			u[2]	=	Z								*/
/*			u[3]	=	yaw								*/
/*		output											*/
/*			y[0]	=	hip(roll)						*/
/*			y[1]	=	hip(pitch)						*/
/*			y[2]	=	knee2(pitch)					*/
/*			y[3]	=	knee1(pitch)					*/
/*------------------------------------------------------*/
void 	kine_fun( float *u, float *y )
{
	static tp_xv_k		xv_k;

	float	_sinx;
	float	_cosx;
	float	w1, w2;

	_sinx	=	sinf( deg2rad( *(u+3) ) );
	_cosx	=	cosf( deg2rad( *(u+3) ) );

	xv_k.x[3]	=	 (*u) * _cosx + (*(u+1)) * _sinx;
	xv_k.y[3]	=	-(*u) * _sinx + (*(u+1)) * _cosx;
	xv_k.z[3]	=	 (*(u+2)) - L3;
	xv_k.d[0]	=	asinf( xv_k.y[3] / xv_k.z[3] );
	w1	=	xv_k.z[3] * xv_k.z[3] + xv_k.y[3] * xv_k.y[3];
	w2	=	w1 - xv_k.x[3] * xv_k.x[3];
	if( w2 < 0.f )	w2	=	0.f;
	xv_k.z[3]	=	sqrtf( w2 );
	cal_inv_kine( (tp_xv_k *)&xv_k.x );

	(*y)		=	rad2deg( xv_k.d[0] );
	(*(y+1))	=	rad2deg( xv_k.d[1] );
	(*(y+2))	=	rad2deg( xv_k.d[2] );
	(*(y+3))	=	rad2deg( xv_k.d[3] );
}


/*--------------------------------------*/
/*	calclulation inverted kinematics	*/
/*		input	x3, z3					*/
/*		output	d1, d2, d3				*/
/*--------------------------------------*/
/*!
 * @param[out] d[1] ŒÒ‚ÌŠp“xi‘«æ‚ª•½s‚Å‚È‚­‚È‚éj
 * @param[out] d[2] ‘«Žñ‚ÌŠp“xi•G‚©‚ç‰º‚ÌŠp“xj
 * @param[out] d[3] ŒÒ‚ÌŠp“x
 */
void	cal_inv_kine( tp_xv_k *a )
{
	float	zz3;
	float	w1;
	float	w2;
	float	w3;
	float	w4;
	float	w5;

	zz3	=	a->z[3] - L01 - L12 - L23;		/*	parallel link leg	*/

	w1	=	atan2f( a->x[3], zz3 );			// ƒsƒbƒ`‚ÌŠp“x

	w2	=	a->x[3] * a->x[3] + zz3 * zz3;	// ŒÒ‚©‚ç‘«Žñ‚Ü‚Å‚Ì‹——£‚Ì‚Qæ
	w3	=	acosf(limit(sqrtf(w2) / (2. * L1), 1.0, -1.0)); // •G‚ÌŠp“x/2
	w4	=	w1 + w3;
	w5	=	atan2f( a->x[3] - L1 * sin(w4), zz3 - L1 * cos(w4) );		/*	parallel link leg	*/

	a->d[1]	=	0.f;					/*	hip(pitch)		*/
	a->d[2]	=	w4;						/*	knee2(pitch)	*/
	a->d[3]	=	w1 - w3;		/*	knee1(pitch)	*/
//	a->d[3]	=	limit_h( w5, w4 );		/*	knee1(pitch)	*/
}


/*------------------------------------------------------*/
/*	calclulation kinematics								*/
/*	input d												*/
/*		xv_kine[0].hip_r								*/
/*		xv_kine[0].leg									*/
/*		xv_kine[0].knee									*/
/*		xv_kine[0].foot_p								*/
/*		xv_kine[0].foot_r								*/
/*	output x											*/
/*		xv_kine[0].x									*/
/*		xv_kine[0].y									*/
/*		xv_kine[0].z									*/
/*		xv_kine[0].yaw(input)							*/
/*------------------------------------------------------*/
void 	fwd_kine_fun( float *d, float *x )
{
	static tp_xv_k		xv_k;

	float	_sinx;
	float	_cosx;
	float	x3, y3;

	xv_k.d[0]	=	deg2rad( *d );
	xv_k.d[1]	=	deg2rad( *(d+1) );
	xv_k.d[2]	=	deg2rad( *(d+2) );
	xv_k.d[3]	=	deg2rad( *(d+3) );
	xv_k.d[4]	=	deg2rad( *(d+4) );

	cal_fwd_kine( (tp_xv_k *)&xv_k.x );

	x3		=	xv_k.x[3];
	y3		=	xv_k.y[3];

	_sinx	=	sinf( deg2rad( *(x+3) ) );
	_cosx	=	cosf( deg2rad( *(x+3) ) );

	xv_k.x[3]	=	x3 * _cosx - y3 * _sinx;
	xv_k.y[3]	=	x3 * _sinx + y3 * _cosx;

	(*x)		=	xv_k.x[3];
	(*(x+1))	=	xv_k.y[3];
	(*(x+2))	=	xv_k.z[3];
}


/*--------------------------------------*/
/*	calclulation kinematics				*/
/*		input	d1, d2, d3				*/
/*		output	x0, z0, x1, z1, x2, z2 	*/
/*				x3, z3, x4, z4		 	*/
/*--------------------------------------*/
void	cal_fwd_kine( tp_xv_k *a )
{
	float	w1;
	float	w2;
	float	w3;

	/*	parallel link leg	*/
	w3		=	-a->d[3];		/*	foot(pitch)		*/
	a->d[3]	-=	a->d[2];		/*	knee1(pitch)	*/

	/*	parallel link leg	*/
	a->d[1]	=	a->d[2];		/*	hip(pitch)		*/
	a->d[2]	=	a->d[3];		/*	knee2(pitch)	*/
	a->d[3]	=	w3;				/*	knee1(pitch)	*/

	w1		=	a->d[1] + a->d[2] ;
	w2		=	w1 + a->d[3] ;

	a->x[0]	= 	a->z[0]	=	a->y[0]	=	0;
	a->x[1]	=	a->x[0] + L1 * sinf( a->d[1] );
	a->z[1]	=	a->z[0] + L01 + L1 * cosf( a->d[1] );			//	 parallel link leg
	a->x[2]	=	a->x[1] + L2 * sinf( w1 );
	a->z[2]	=	a->z[1] + L12 + L2 * cosf( w1 );					//	 parallel link leg
	a->x[3]	= 	a->x[4] =	a->x[2] + L3 * sinf( w2 );
	a->z[3]	= 	a->z[4]	=	a->z[2] + L23 + L3 * cosf( w2 );		//	 parallel link leg

	a->y[1]	=	a->z[1] * sinf( a->d[0] );
	a->y[2]	=	a->z[2] * sinf( a->d[0] );
	a->y[3]	=	(a->z[2] + L3) * sinf( a->d[0] );
}


/*--------------------------------------*/
/*	tracking from degree to xyz			*/
/*--------------------------------------*/
void	trk_kine( void )
{
	xv_kine[0].hip_r	=	xv_ref.d[LEG_ROLL_R];
	xv_kine[0].leg		=	0.f;
	xv_kine[0].knee		=	xv_ref.d[KNEE_R2];
	xv_kine[0].foot_p	=	xv_ref.d[KNEE_R1];
	xv_kine[0].foot_r	=	xv_ref.d[FOOT_ROLL_R];
	xv_kine[0].yaw		=	xv_ref.d[LEG_YAW_R];

	fwd_kine_fun( &xv_kine[0].hip_r, &xv_kine[0].x );

	xv_kine[1].hip_r	=	xv_ref.d[LEG_ROLL_L];
	xv_kine[1].leg		=	0.f;
	xv_kine[1].knee		=	xv_ref.d[KNEE_L2];
	xv_kine[1].foot_p	=	xv_ref.d[KNEE_L1];
	xv_kine[1].foot_r	=	xv_ref.d[FOOT_ROLL_L];
	xv_kine[1].yaw		=	xv_ref.d[LEG_YAW_L];

	fwd_kine_fun( &xv_kine[1].hip_r, &xv_kine[1].x );

	xv_data_x_r.time		=
	xv_data_y_r.time		=
	xv_data_y_r2.time		=
	xv_data_z_r.time		=
	xv_data_x_l.time		=
	xv_data_y_l.time		=
	xv_data_y_l2.time		=
	xv_data_z_l.time		=	EPS_TIME;

	xv_data_x_r.pos			=	xv_kine[0].x;
	xv_data_y_r.pos			=	0.f;
	xv_data_y_r2.pos		=	xv_kine[0].y;
	xv_data_z_r.pos			=	xv_kine[0].z;
	xv_data_x_l.pos			=	xv_kine[1].x;
	xv_data_y_l.pos			=	0.f;
	xv_data_y_l2.pos		=	xv_kine[1].y;
	xv_data_z_l.pos			=	xv_kine[1].z;
}
