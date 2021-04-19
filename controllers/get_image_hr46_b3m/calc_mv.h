/*----------------------------------------------------------*/
/*	calculation trajectry									*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	calc_mv.h								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#ifndef 	_CALC_MV_H_
#define 	_CALC_MV_H_

#ifdef 		_CALC_MV_C_
#define 	extern 
#endif 		/* _CALC_MV_C_ */


/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
#define		MVDATA_NUM				(12)				/*	number of trajectory tables		*/
#define		EPS						(0.001)				/*	epsilon : nearly equals zero	*/


typedef struct st_xv_mv
{
	long	count;				/* number of walk steps	[count] 		*/
}	tp_xv_mv;

typedef struct st_xv_mvdata		/*	trajectory calculatoin data	*/
{
	float		t;				/* [bit] 		*/
	float		mv_tbl;			/* [bit] 		*/
	float		dt;				/* [bit] 		*/
	float		start;			/* [deg]or[mm] 	*/
	float		amp;			/* [deg]or[mm]	*/
	float		out_old;		/* [deg]or[mm]	*/
	float		pos_old;		/* [deg]or[mm]	*/
}	tp_xv_mvdata;

typedef struct st_xv_data		/*	trajectory input data from motion sequense	*/
{
	float		time;			/* [mm]or[deg]	*/
	float		pos;			/* [sec] 		*/
	long		mv_tbl_select;	/* [bit] 		*/
}	tp_xv_data;

typedef struct st_xv_odometry	/*	odometry data	*/
{
	float		moveX;			/*	X [mm] 		*/
	float		moveY;			/*	Y [mm] 		*/
	float		rotZ;			/*	THETA [deg] */
	float		x[2];			/*	foot point x			*/
	float		x_old[2];		/*	foot point x old data	*/
	float		y[2];			/*	foot point y			*/
	float		y_old[2];		/*	foot point y old data	*/
	float		theta[2];		/*	leg yaw theta			*/
	float		theta_old[2];	/*	leg yaw theta old data	*/
}	tp_xv_odometry;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern tp_xv_mv			xv_mv;
extern tp_xv_mvdata		xv_mvdata_d[SERV_NUM];
extern tp_xv_mvdata		xv_mvdata[MVDATA_NUM];
extern tp_xv_data		xv_data_d[SERV_NUM];
extern tp_xv_data		xv_data_x_r;
extern tp_xv_data		xv_data_y_r;
extern tp_xv_data		xv_data_y_r2;
extern tp_xv_data		xv_data_z_r;
extern tp_xv_data		xv_data_x_l;
extern tp_xv_data		xv_data_y_l;
extern tp_xv_data		xv_data_y_l2;
extern tp_xv_data		xv_data_z_l;
extern tp_xv_data		xv_data_pitch;
extern tp_xv_data		xv_data_roll2;
extern tp_xv_odometry	xv_odometry;
extern float		odometry_correct_para_x;
extern float		odometry_correct_para_y;


/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern void 	calc_mv_init( void );
extern void 	calc_mv( void );
extern float	calc_mvdata( tp_xv_mvdata *, tp_xv_data * );
extern void		chg_mvtbl( tp_xv_mvdata *, tp_xv_data * );


#ifdef 		_CALC_MV_C_
#undef 		extern
#endif 		/* _CALC_MV_C_ */

#endif 		/* _CALC_MV_H_ */
