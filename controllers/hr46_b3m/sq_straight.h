/*----------------------------------------------------------*/
/*	straight sequence										*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	sq_straight.h							*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#ifndef 	_SQ_STRAIGHT_H_
#define 	_SQ_STRAIGHT_H_

#ifdef 		_SQ_STRAIGHT_C_
#define 	extern 
#endif 		/* _SQ_STRAIGHT_C_ */


/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
enum {
//	SQ_STRAIGHT_INIT	= 	0,
	SQ_STRAIGHT	 		= 	1,
	SQ_STRAIGHT2	 	= 	2,
	SQ_STRAIGHT3	 	= 	3,
	SQ_STRAIGHT4	 	= 	4,
	SQ_STRAIGHT5	 	= 	5,
	SQ_STRAIGHT_END 	= 	6
};

typedef struct st_xp_mv_straight
{
	float	time;				/* move time [sec]			*/
	float	z3;					/* hight of hip joint [mm]	*/
	float	arm_sh_pitch;		/* shoulder pitch [deg]		*/
	float	arm_sh_roll;		/* shoulder roll [deg]		*/
	float	arm_el_yaw;			/* elbow yaw [deg]			*/
	float	arm_el_pitch;		/* elbow pitch [deg]		*/
}	tp_xp_mv_straight;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern tp_xp_mv_straight	xp_mv_straight;
//extern short		flag_md_straight_end;
extern short		mode_sq_straight;
extern float		xv_mv_straight_time;


/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern void sq_straight_init();
extern int sq_straight();

#ifdef 		_SQ_STRAIGHT_C_
#undef 		extern
#endif 		/* _SQ_STRAIGHT_C_ */

#endif 		/* _SQ_STRAIGHT_H_ */
