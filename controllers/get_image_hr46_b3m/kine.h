/*----------------------------------------------------------*/
/*	kinetics caluculation									*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	kine.h									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#ifndef 	_KINE_H_
#define 	_KINE_H_

#ifdef 		_KINE_C_
#define 	extern 
#endif 		/* _KINE_C_ */


/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
typedef struct st_xv_k
{
	float	x[5];					/*	[mm]	*/
	float	y[5];					/*	[mm]	*/
	float	z[5];					/*	[mm]	*/
	float	d[5];					/*	[deg]	*/
}	tp_xv_k;

typedef struct st_xv_kine			/*	inv. kinetics data	*/
{
	float	x;						/*	foot x [mm]			*/
	float	y;						/*	foot y [mm]			*/
	float	z;						/*	foot z [mm]			*/
	float	yaw;					/*	hip yaw [deg]		*/
	float	hip_r;					/*	hip roll [deg]		*/
	float	leg;					/*	hip pitch [deg]		*/
	float	knee;					/*	knee pitch [deg]	*/
	float	foot_p;					/*	foot pitch [deg]	*/
	float	foot_r;					/*	foot roll [deg]		*/
}	tp_xv_kine;


typedef struct st_xv_posture
{
	float	pitch;					/*	[deg]	*/
	float	roll;					/*	[deg]	*/
	float	yaw;					/*	[deg]	*/
	float	roll2;					/*	[deg]	*/
}	tp_xv_posture;

/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern tp_xv_kine		xv_kine[2];
extern tp_xv_posture	xv_posture;

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern void 	kine( void );
extern void 	kine_fun( float *, float * );
extern void		cal_inv_kine( tp_xv_k * );
extern void 	fwd_kine_fun( float *, float * );
extern void		cal_fwd_kine( tp_xv_k * );
extern void		trk_kine( void );


#ifdef 		_KINE_C_
#undef 		extern
#endif 		/* _KINE_C_ */

#endif 		/* _KINE_H_ */
