/*----------------------------------------------------------*/
/*	servo control											*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	serv.h									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#ifndef 	_SERV_H_
#define 	_SERV_H_

#ifdef 		_SERV_C_
#define 	extern 
#endif 		/* _SERV_C_ */

/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
typedef struct st_xv_sv
{
	long	d;					/*	[0.01deg]	*/
	long	deg_sw_out;			/*	[0.01deg]	*/
	long	deg_sv;				/*	[0.01deg]	*/
	long	deg_lim;			/*	[0.01deg]	*/
	long	pls;				/*	[bit]		*/
	long	pls_out;			/*	[bit]		*/
}	tp_xv_sv;

typedef struct st_xp_sv
{
	long	deg_sign;			/*	[+/-]	*/
	long	deg_offset;			/*	[deg]	*/
	long	deg_lim_h;			/*	[deg]	*/
	long	deg_lim_l;			/*	[deg]	*/
	long	deg_lim_offset;		/*	[0.01deg]		*/
	long	deg2pls;			/*	[bit/(0.01deg)]	*/
}	tp_xp_sv;

typedef struct st_xv_ref		
{
	float	d[SERV_NUM];		/*	[deg]		*/
	float	d_ref[SERV_NUM];	/*	[deg]		*/
}	tp_xv_ref;

typedef struct st_sw
{
	short	ref_d;
}	tp_sw;

typedef struct st_xv_pv
{
	short	temp[SERV_NUM];		/*	[deg]		*/
	float	deg[SERV_NUM];		/*	[deg]		*/
	short	current[SERV_NUM];	/*	[mA]		*/
}	tp_xv_pv;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern 	tp_xv_sv	xv_sv[SERV_NUM];
extern 	tp_xp_sv	xp_sv[SERV_NUM];
extern 	tp_xv_ref	xv_ref;
extern	tp_sw		sw;
extern 	tp_xv_pv	xv_pv;
extern	float		xp_ref_d_lim;

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern void 	serv( void );
extern void 	set_sw_ref_d( int );
extern short	check_sw_ref_d( void );


#ifdef 		_SERV_C_
#undef 		extern
#endif 		/* _SERV_C_ */

#endif 		/* _SERV_H_ */
