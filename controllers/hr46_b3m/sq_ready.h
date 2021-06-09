/*----------------------------------------------------------*/
/*	ready sequence											*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	sq_ready.h								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#ifndef 	_SQ_READY_H_
#define 	_SQ_READY_H_

#ifdef 		_SQ_READY_C_
#define 	extern 
#endif 		/* _SQ_READY_C_ */


/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
enum {
	SQ_READY_INIT 		= 	0,
	SQ_READY 			= 	1,
	SQ_READY2 			= 	2,
	SQ_READY3 			= 	3,
	SQ_READY4 			= 	4,
	SQ_READY_END 		= 	5
};

typedef struct st_xp_mv_ready
{
	float	time;				/* move time [sec]			*/
	float	z3;					/* hight of hip joint [mm]	*/
	float	arm_sh_pitch;		/* shoulder pitch [deg]		*/
	float	arm_sh_roll;		/* shoulder roll [deg]		*/
	float	arm_el_yaw;			/* elbow yaw [deg]			*/
	float	arm_el_pitch;		/* elbow pitch [deg]		*/
	float	pitch;				/* pitch [deg]				*/
}	tp_xp_mv_ready;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern tp_xp_mv_ready	xp_mv_ready;
//extern short		flag_sq_ready;
//extern short		flag_md_ready_end;
extern short		mode_sq_ready;
extern short		flag_ready_gyro;
extern float		xv_mv_ready_time;


/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
void sq_ready_init(int slow_mode);
int sq_ready();

#ifdef 		_SQ_READY_C_
#undef 		extern
#endif 		/* _SQ_READY_C_ */

#endif 		/* _SQ_READY_H_ */
