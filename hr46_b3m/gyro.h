/*----------------------------------------------------------*/
/*	gyro sensor												*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	gyro.h									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.5.30								*/
/*----------------------------------------------------------*/
#ifndef 	_GYRO_H_
#define 	_GYRO_H_

#ifdef 		_GYRO_C_
#define 	extern 
#endif 		/* _GYRO_C_ */


/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
typedef struct st_xp_gyro
{
	float	kp1_foot;				/*	gain (roll)		*/
	float	kp2_foot;				/*	gain (pitch)	*/
	float	kp1_hip;				/*	gain (roll)		*/
	float	kp2_hip;				/*	gain (pitch)	*/
	float	kp1_arm;				/*	gain (roll)		*/
	float	kp2_arm;				/*	gain (pitch)	*/
	float	kp2_waist;				/*	gain (pitch)	*/
	float	kp3_waist;				/*	gain (yaw)		*/
	float	gyro_k1;				/*	[(deg/sec)/V]	*/
	float	gyro_k2;				/*	[(deg/sec)/V]	*/
	float	gyro_k3;				/*	[(deg/sec)/V]	*/
	float	ad_volt_offset1;		/*	[deg/sec]		*/
	float	ad_volt_offset2;		/*	[deg/sec]		*/
	float	ad_volt_offset3;		/*	[deg/sec]		*/
	float	t1;						/*	[msec]			*/
	float	t2;						/*	[msec]			*/
	float	gyro_data3_flt2_t1;		/*	[msec]			*/
	float	yaw_cntl_gain;			/*	[-]				*/
	float	yaw_cntl_dead;			/*	[deg]			*/
	float	yaw_cntl_theta;			/*	[deg]			*/
	float	gyro_omega;				/*	[1/rad]			*/
	float	fall_roll_deg1;			/*	fall check [deg]	2010.2.12	*/
	float	fall_pitch_deg1;		/*	fall check [deg]	2010.2.12	*/
}	tp_xp_gyro;

typedef struct st_xv_gyro
{
	float	gyro_data1;				/*	[deg/sec]		*/
	float	gyro_data2;				/*	[deg/sec]		*/
	float	gyro_data3;				/*	[deg/sec]		*/
	float	gyro_data1_d;			/*	[deg/sec/sec]	*/
	float	gyro_data2_d;			/*	[deg/sec/sec]	*/
	float	gyro_data3_d;			/*	[deg/sec/sec]	*/
	float	gyro_data1_flt;			/*	[deg/sec]		*/
	float	gyro_data2_flt;			/*	[deg/sec]		*/
	float	gyro_data3_flt;			/*	[deg/sec]		*/
	float	gyro_data3_flt2;		/*	[deg/sec]		*/
	float	gyro_roll;				/*	roll [deg]		*/
	float	gyro_pitch;				/*	pitch [deg]		*/
	float	gyro_yaw;				/*	yaw [deg]		*/
	float	gyro_yaw2;				/*	yaw [deg]		*/
	float	gyro_roll2;				/*	roll [deg]		*/
	float	gyro_pitch2;			/*	pitch [deg]		*/
	float	deg_foot_roll;			/*	[deg]			*/
	float	deg_foot_pitch;			/*	[deg]			*/
	float	deg_hip_roll;			/*	[deg]			*/
	float	deg_hip_pitch;			/*	[deg]			*/
	float	deg_arm_roll;			/*	[deg]			*/
	float	deg_arm_pitch;			/*	[deg]			*/
	float	deg_waist_pitch;		/*	[deg]			*/
	float	deg_waist_yaw;			/*	[deg]			*/
	float	yaw_cntl_ref;			/*	reference [deg]	*/
	float	yaw_cntl_fb;			/*	feedback [deg]	*/
	float	quaternion[4];			/*  quaternion (w, x, y, z) */
}	tp_xv_gyro;

typedef struct st_flag_gyro
{
	short	vib;					/*	vibraion feedback	*/
	short	vib_auto;				/*	auto/manual			*/
	short	vib_manu;				/*	manual setting		*/
	short	zero;					/*	offset clear		*/
	short	yaw_cntl;				/*	yaw control			*/
	short	fall_on;				/*	fall status			*/
	short	fall_cntl;				/*	fall servo off 		*/
	short	fall;					/*	fall				*/
}	tp_flag_gyro;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern	tp_xp_gyro		xp_gyro;
extern	tp_xv_gyro		xv_gyro;
extern	tp_flag_gyro	flag_gyro;
extern	short	flag_auto_gyro_offset;

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern void 	gyro_fun( void );
extern void 	gyro_init( void );
extern void 	gyro_cntr_fun( void );
extern void 	gyro_yaw_cntr_fun( void );
extern void		gyro_offset_tune( void );
extern void PostureControl(void);

#ifdef 		_GYRO_C_
#undef 		extern
#endif 		/* _GYRO_C_ */

#endif 		/* _GYRO_H_ */
