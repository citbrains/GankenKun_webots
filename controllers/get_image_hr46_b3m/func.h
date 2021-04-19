/*----------------------------------------------------------*/
/*	control functions										*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	func.h									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2002.3.23								*/
/*----------------------------------------------------------*/
#ifndef 	_FUNC_H_
#define 	_FUNC_H_


/*--------------------------------------*/
/*	definitions							*/
/*--------------------------------------*/
#define		rad2deg(x)		(float)((x)*57.2958f)
#define		deg2rad(x)		((x)*0.0174533f)
#define		sw2(s,x1,x2)	(((s)==(1))?(x1):(x2))
#define		limit(x,h,l)	(((x)>(h))?(h):(((x)<(l))?(l):(x)))
#define		limit_h(x,h)	(((x)>(h))?(h):(x))
#define		limit_l(x,l)	(((x)<(l))?(l):(x))
// xの値を(y-d)から(y+d)の間に制限する．
#define		dlimit(x,d,y)	(((x)>((y)+(d)))?((y)+(d)):(((x)<((y)-(d)))?((y)-(d)):(x)))
#define		filterf(in,out,t)	(((in)*RTC_TIME+(out)*(t))/((t)+RTC_TIME))
#define		integrator_f(x,y,lim)	(((x+y)>(lim))?(lim):(((x+y)<(-lim))?(-lim):(x+y)))
								/* t : time constant [ms] */
#define		deadband(x,d)		(((x)>(d))?(x):(((x)<-(d))?(x):(0)))

/*	output	=	HOLD( set, reset, output )	*/
#define		HOLD( a, b, c )		( ((c) || (a)) && (!(b)) )
#define		TIMER( in, out, t, w )	( (in) ? (((++(w))>(t))?((out)=1,(w)=(t)):0) : ((out)=(w)=0) )

#ifdef 		_FUNC_C_
#define 	extern 
#endif 		/* _FUNC_C_ */

/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/

typedef		struct st_xp_pid
{
	float	kp;
	float	ki;
	float	i_lim;
	float	d_lim;
	short	sw;
}	tp_xp_pid;


typedef		struct st_xv_pid
{
	float	sv;
	float	pv;
	float	err;
	float	pv_s;
	float	k_out;
	float	mv;
	float	pv_old;
}	tp_xv_pid;


typedef	struct	st_xp_dlim_wait
{
	float	dlim;					// 変化の最大値(1/s)
	float	wait_time;				// 方向を変えるときの待ち時間(s)
}	tp_xp_dlim_wait;


typedef	struct	st_xv_dlim_wait
{
	short	sign_old;				// 以前どちらに動いていたか
	float	in;						// 入力（最終的な目標値）
	float	out;					// 出力（ゆるやかに変更していく値）
	float	dout;					// 出力の変化
	float	time_work;				// 方向を切り替えてからの経過時間(s)
}	tp_xv_dlim_wait;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern float	diff( float, float, float, float * );
extern int		pulse1( short, short * );
extern void		pid_fun( tp_xp_pid *, tp_xv_pid * );
extern void		pid_fun_init( tp_xp_pid *, tp_xv_pid * );
extern float	dlim_wait_fun( tp_xp_dlim_wait *, tp_xv_dlim_wait * );
extern void		dlim_wait_fun_init( tp_xp_dlim_wait *, tp_xv_dlim_wait * );


#ifdef 		_FUNC_C_
#undef 		extern
#endif 		/* _FUNC_C_ */

#endif 		/* _FUNC_H_ */
