/*----------------------------------------------------------*/
/*	control functions										*/
/*															*/
/*															*/
/*	file name	:	func.c									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.5.20								*/
/*----------------------------------------------------------*/
#define		_FUNC_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	"var.h"
#include	"func.h"
#include	<math.h>

#define	min(x,y)	((x)<(y))?(x):(y)
#define	max(x,y)	((x)>(y))?(x):(y)

/*--------------------------------------*/
/*	differential						*/
/*--------------------------------------*/
float	diff( float x, float t1, float t2, float *work )
{
	float	flt_out, out;

	flt_out	=	filterf( x, *work, t1 );
	out		=	( flt_out - *work ) * t2 / RTC_TIME;
	*work	=	flt_out;

	return	out;
}


/*--------------------------------------*/
/*	pulse1								*/
/*--------------------------------------*/
int		pulse1( short in, short *old )
{
	short		out;

	if( in && !*old )
	{
		out =	1;
	}
	else
	{
		out	=	0;
	}

	*old	=	in;

	return	out;
}


/*--------------------------------------*/
/*	pid_fun								*/
/*--------------------------------------*/
void	pid_fun( tp_xp_pid *xp, tp_xv_pid *xv )
{
	xv->err		=	xv->sv - xv->pv;
	xv->pv_s	=	xv->pv - xv->pv_old;
	xv->k_out	=	xv->err * xp->ki - xv->pv_s * xp->kp;

	if( xp->sw == 1 )
	{
		xv->mv	=	integrator_f( xv->k_out / RTC_TICK, xv->mv, xp->i_lim );
	}
	else if( xp->sw == 0 )
	{
		xv->mv	=	dlimit( 0.f, xp->d_lim / RTC_TICK, xv->mv);
	}

	xv->pv_old	=	xv->pv;
}


/*--------------------------------------*/
/*	pid_fun_init						*/
/*--------------------------------------*/
void	pid_fun_init( tp_xp_pid *xp, tp_xv_pid *xv )
{
	xv->sv			=	0.f;
	xv->pv			=	0.f;
	xv->err			=	0.f;
	xv->pv_s		=	0.f;
	xv->k_out		=	0.f;
	xv->mv			=	0.f;
	xv->pv_old		=	0.f;
}


/*--------------------------------------*/
/*	dlim_wait_fun						*/
/*--------------------------------------*/
// ‚Ü‚¾ŽÀ‘•‚ªŠ®—¹‚µ‚Ä‚¢‚È‚³‚»‚¤
float	dlim_wait_fun( tp_xp_dlim_wait *xp, tp_xv_dlim_wait *xv )
{
	float y2 = dlimit( xv->in, xp->dlim / RTC_TICK, xv->out );
	xv->dout = (y2 - xv->out) * RTC_TICK;

	/*	ZERO	*/
	if((y2 >= -EPS_DATA)&&(y2 <= EPS_DATA))
	{
		xv->sign_old	=	 0;
		xv->time_work 	+=	RTC_TIME_SEC;
	}
	/*	PLUS	*/
	else if((xv->sign_old > 0)&&(y2 > EPS_DATA))
	{
		xv->sign_old	=	 1;
		xv->time_work	=	0.f;
	}
	/*	MINUS	*/
	else if((xv->sign_old < 0)&&(y2 < -EPS_DATA))
	{
		xv->sign_old	=	-1;
		xv->time_work	=	0.f;
	}
	/*	PLUS -> wait_time -> MINUS	*/
	else if((xv->sign_old >= 0)&&(y2 < -EPS_DATA)&&(xv->time_work >= xp->wait_time))
	{
		xv->sign_old	=	-1;
	}
	/*	MINUS -> wait_time -> PLUS	*/
	else if((xv->sign_old <= 0)&&(y2 >  EPS_DATA)&&(xv->time_work >= xp->wait_time))
	{
		xv->sign_old	=	 1;
	}
	else
	{
		y2		=	0.f;
		xv->time_work 	+=	RTC_TIME_SEC;
	}

	xv->out		=	y2;

	return		xv->out;
}


/*--------------------------------------*/
/*	dlim_wait_fun_init					*/
/*--------------------------------------*/
void	dlim_wait_fun_init( tp_xp_dlim_wait *xp, tp_xv_dlim_wait *xv )
{
	xv->sign_old	=	0;
	xv->in			=
	xv->out			=
	xv->dout		=
	xv->time_work	=	0.f;
}
