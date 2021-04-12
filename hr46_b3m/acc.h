/*----------------------------------------------------------*/
/*	acceleration sensor										*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	acc.h									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.5.30								*/
/*----------------------------------------------------------*/
#ifndef 	_ACC_H_
#define 	_ACC_H_

/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
#define		ACC_SENSOR			/*	use acc sensor	*/

enum {
	STANDUP_FWD			=	1,
	STANDUP_BWD			=	2,
	STANDUP_RIGHT		=	3,
	STANDUP_LEFT		=	4
};


typedef struct st_xp_acc
{
	float	acc_k1;					/*	x[G/bit]		*/
	float	acc_k2;					/*	y[G/bit]		*/
	float	acc_k3;					/*	z[G/bit]		*/
	float	ad_volt_offset1;		/*	x[G]			*/
	float	ad_volt_offset2;		/*	y[G]			*/
	float	ad_volt_offset3;		/*	z[G]			*/
	float	t1;						/*	filter time constant [msec]	*/
	float	t2;						/*	filter time constant [msec]	*/
	float	fall_fwd;				/*	fall forward check [deg]	*/
	float	fall_bwd;				/*	fall backward check [deg]	*/
	float	fall_right;				/*	fall right check [deg]		*/
	float	fall_left;				/*	fall left check [deg]		*/
	float	fall_check_time;		/*	fall check time [bit]		*/
	float	fall_pitch;				/*	fall pitch [deg]			*/
	float	fall_roll;				/*	fall roll [deg]				*/
	float	fall_pitch_oblique;		/*	fall pitch [deg]			*/
	float	fall_roll_oblique;		/*	fall roll [deg]				*/
}	tp_xp_acc;

typedef struct st_xv_acc
{
	float	acc_data1;				/*	x[G]			*/
	float	acc_data2;				/*	y[G]			*/
	float	acc_data3;				/*	z[G]			*/
	float	acc_data1_d;			/*	x[G/sec]		*/
	float	acc_data2_d;			/*	y[G/sec]		*/
	float	acc_data3_d;			/*	z[G/sec]		*/
	float	acc_data1_flt;			/*	filer x[G]		*/
	float	acc_data2_flt;			/*	filter y[G]		*/
	float	acc_data3_flt;			/*	filter z[G]		*/
	float	acc_pitch;				/*	pitch angle [deg]			*/
	float	acc_roll;				/*	roll angle [deg]			*/
	float	acc_pitch_d;			/*	pitch angle velocity [deg/s]*/
	float	acc_roll_d;				/*	roll angle velocity [deg/s]	*/
	float	acc_pitch2;				/*	pitch angle [deg]			*/
	float	acc_roll2;				/*	roll angle [deg]			*/
	short	fall_fwd_work;			/*	[bit]			*/
	short	fall_bwd_work;			/*	[bit]			*/
	short	fall_right_work;		/*	[bit]			*/
	short	fall_left_work;			/*	[bit]			*/
}	tp_xv_acc;

typedef struct st_flag_acc
{
	short	zero;					/*	offset clear	*/
	short	fall_fwd_on;			/*	fall forward	*/
	short	fall_bwd_on;			/*	fall backward	*/
	short	fall_right_on;			/*	fall right		*/
	short	fall_left_on;			/*	fall left		*/
	short	fall;					/*	fall			*/
	short	standup_select;			/*	standup selection	*/
}	tp_flag_acc;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern	tp_xp_acc		xp_acc;
extern	tp_xv_acc		xv_acc;
extern	tp_flag_acc		flag_acc;
extern	short	flag_restart;
extern	short	flag_ukemi_finished;
extern	short	flag_ukemi_start;
extern	short	flag_servo_restart;
extern	short	flag_restart_work;
extern	short	flag_ukemi_finished_work;
extern	short	flag_motion_accept;
extern	short	flag_motion_accept_work;
extern	float	touchdown_gain;

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern void 	acc_init( void );
extern void 	acc_fun( void );

#endif 		/* _ACC_H_ */
