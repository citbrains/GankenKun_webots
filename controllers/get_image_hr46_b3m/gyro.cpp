/*----------------------------------------------------------*/
/*	gyro sensor												*/
/*															*/
/*															*/
/*	file name	:	gyro.c									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.5.30								*/
/*----------------------------------------------------------*/
#define		_GYRO_C_
#define		DEADBAND			10.0
#define		FORGETTING_FACTOR	0.9
#define		FORGETTING_FACTOR_YAW	0.7
#define		ACC_DEADBAND_LIMIT	0.0001

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/

#include <eigen3/Eigen/Dense>

extern "C" {
#include	<complex.h>
#include    <math.h>
#include 	"var.h"
#include	"func.h"
#include	"sq_walk.h"
#include	"gyro.h"
#include	"kine.h"
#include	"serv.h"
#include	"motion.h"
#include	"sq_ready.h"
#include	"sq_motion.h"
#include	"acc.h"
#include	"servo_rs.h"
#include	"joy.h"
}

/*--------------------------------------*/
/*	gyro								*/
/*--------------------------------------*/
void	gyro_fun( void )
{
	static 	short	_w_pulse1	=	0;
	float	_w_gyro_roll2, _w_gyro_pitch2;
	
	static float count = 0.0f;
	const float wakeup_time = 0.5;

	/***	low pass filter	***/
	xv_gyro.gyro_data1_d	=	diff( xv_gyro.gyro_data1, xp_gyro.t1, xp_gyro.t2, &xv_gyro.gyro_data1_flt );
	xv_gyro.gyro_data2_d	=	diff( xv_gyro.gyro_data2, xp_gyro.t1 * 5, xp_gyro.t2, &xv_gyro.gyro_data2_flt );
	xv_gyro.gyro_data3_d	=	diff( xv_gyro.gyro_data3, xp_gyro.t1, xp_gyro.t2, &xv_gyro.gyro_data3_flt );
	xv_gyro.gyro_data3_flt2	=	filterf( xv_gyro.gyro_data3_flt, xv_gyro.gyro_data3_flt2, xp_gyro.gyro_data3_flt2_t1 );

	/***	calculate angle from gyro sensor	***/
//	xv_gyro.gyro_roll		=	integrator_f( xv_gyro.gyro_data1_flt/RTC_TICK, xv_gyro.gyro_roll, 180.f );
//	xv_gyro.gyro_pitch		=	integrator_f( xv_gyro.gyro_data2_flt/RTC_TICK, xv_gyro.gyro_pitch, 180.f );
//	xv_gyro.gyro_yaw		=	integrator_f( xv_gyro.gyro_data3_flt/RTC_TICK, xv_gyro.gyro_yaw, 180.f );
//	xv_gyro.gyro_yaw2		=	integrator_f( xv_gyro.gyro_data3_flt/RTC_TICK, xv_gyro.gyro_yaw2, 180.f ); // 1800->180 130430

	/***	check fall down	for servo off		***/
//	xv_gyro.gyro_roll2	=	(1.0f - xp_gyro.gyro_omega * RTC_TIME_SEC ) * xv_gyro.gyro_roll2 + xp_gyro.gyro_omega * RTC_TIME_SEC * xv_acc.acc_roll2 
//							- RTC_TIME_SEC * xv_gyro.gyro_data1;
//	xv_gyro.gyro_pitch2	=	(1.0f - xp_gyro.gyro_omega * RTC_TIME_SEC ) * xv_gyro.gyro_pitch2 + xp_gyro.gyro_omega * RTC_TIME_SEC * xv_acc.acc_pitch2 
//							- RTC_TIME_SEC * xv_gyro.gyro_data2;

	_w_gyro_roll2	=	(float)fabs( xv_gyro.gyro_roll2  );
	_w_gyro_pitch2	=	(float)fabs( xv_gyro.gyro_pitch2 );

	if( _w_gyro_roll2 > xp_gyro.fall_roll_deg1 || _w_gyro_pitch2 > xp_gyro.fall_pitch_deg1 )
	{
		flag_gyro.fall_on		=	ON;
	}
	else
	{
		flag_gyro.fall_on		=	OFF;
	}

	if( pulse1( (flag_gyro.fall_cntl && flag_gyro.fall_on), &_w_pulse1 ) )
	{
		flag_servo_off	=	ON;
	}

	/***	check fall down	***/
	if( xv_gyro.gyro_roll2 >= xp_gyro.fall_roll_deg1 ){
		count += RTC_TIME_SEC;
		if (count > wakeup_time){
			flag_gyro.fall		=	STANDUP_LEFT;
		}
	} else if( xv_gyro.gyro_roll2 <= -xp_gyro.fall_roll_deg1 ){
		count += RTC_TIME_SEC;
		if (count > wakeup_time){
			flag_gyro.fall		=	STANDUP_RIGHT;
		}
	} else if( xv_gyro.gyro_pitch2 >= xp_gyro.fall_pitch_deg1 ){
		count += RTC_TIME_SEC;
		if (count > wakeup_time){
			flag_gyro.fall		=	STANDUP_BWD;
		}
	} else if( xv_gyro.gyro_pitch2 <= -xp_gyro.fall_pitch_deg1 ){
		count += RTC_TIME_SEC;
		if (count > wakeup_time){
			flag_gyro.fall		=	STANDUP_FWD;
		}
	} else if((fabs(xv_gyro.gyro_roll2 ) < 0.5 * xp_gyro.fall_roll_deg1 ) &&
			(fabs(xv_gyro.gyro_pitch2) < 0.5 * xp_gyro.fall_pitch_deg1)){
		flag_gyro.fall	=	0;
		flag_motion_accept	=	ON;
		count = 0.0f;
	}
}


/*--------------------------------------*/
/*	gyro_init							*/
/*--------------------------------------*/
void 	gyro_init( void )
{
	xp_gyro.kp1_foot			=	0.f;
	xp_gyro.kp2_foot			=	-0.f;
	xp_gyro.kp1_hip				=	0.f;
	xp_gyro.kp2_hip				=	0.f;
	xp_gyro.kp1_arm				=	0.1f;
	xp_gyro.kp2_arm				=	0.1f;
	xp_gyro.kp2_waist			=	0.f;
	xp_gyro.kp3_waist			=	0.f;
	xp_gyro.yaw_cntl_gain		=	1.0f;
	xp_gyro.yaw_cntl_dead		=	2.f;
	xp_gyro.yaw_cntl_theta		=	5.f;

	//	IDG500
	xp_gyro.gyro_k1				=	1000.f;		/*	? [deg/sec]			*/
	xp_gyro.gyro_k2				=	1000.f;		/*	? [deg/sec]			*/
	xp_gyro.ad_volt_offset1		=	-1.350f;	/*	-1.35[V]			*/
	xp_gyro.ad_volt_offset2		=	-1.350f;	/*	-1.35[V]			*/

	//	CRS03-04
	xp_gyro.gyro_k3				=	100.f;		/*	[deg/sec]			*/
	xp_gyro.ad_volt_offset3		=	-2.5f;		/*	-2.5[V]				*/

	xp_gyro.t1					=	5.f;		/*	[msec]				*/
	xp_gyro.t2					=	15.f;		/*	diff T2=3*T1[msec]	*/
	xp_gyro.gyro_data3_flt2_t1 	=	1000.f;		/*	[msec]				*/

	xp_gyro.gyro_omega			=	0.3f;		/*	1/T : time constant = 3[s]	*/
	xp_gyro.fall_roll_deg1		=	50.f;		/*	fall check [deg]	2010.2.12	*/
	xp_gyro.fall_pitch_deg1		=	55.f;		/*	fall check [deg]	2010.2.12	*/

	xv_gyro.gyro_data1			=
	xv_gyro.gyro_data2			=
	xv_gyro.gyro_data3			=
	xv_gyro.gyro_data1_d		=
	xv_gyro.gyro_data2_d		=
	xv_gyro.gyro_data3_d		=
	xv_gyro.gyro_data1_flt		=
	xv_gyro.gyro_data2_flt		=
	xv_gyro.gyro_data3_flt		=
	xv_gyro.gyro_data3_flt2 	=
	xv_gyro.gyro_roll			=
	xv_gyro.gyro_pitch			=
	xv_gyro.gyro_yaw			=
	xv_gyro.gyro_yaw2			=
	xv_gyro.deg_foot_roll		=
	xv_gyro.deg_foot_pitch		=
	xv_gyro.deg_hip_roll		=
	xv_gyro.deg_hip_pitch		=
	xv_gyro.deg_arm_roll		=
	xv_gyro.deg_arm_pitch		=
	xv_gyro.deg_waist_pitch		=
	xv_gyro.deg_waist_yaw		=
	xv_gyro.yaw_cntl_ref		=
	xv_gyro.yaw_cntl_fb			=	0.f;

	xv_gyro.gyro_roll2			=
	xv_gyro.gyro_pitch2			=	0.f;

	flag_gyro.vib				=	OFF;
	flag_gyro.vib_auto			=	ON;
	flag_gyro.vib_manu			=	OFF;
	flag_gyro.zero				=	OFF;
	flag_gyro.yaw_cntl			=	OFF;
	flag_gyro.fall_on			=	OFF;
	flag_gyro.fall_cntl			=	OFF;
	flag_auto_gyro_offset		=	ON;

	xv_gyro.quaternion[0]		=	1.0f;
	xv_gyro.quaternion[1]		=	0.0f;
	xv_gyro.quaternion[2]		=	0.0f;
	xv_gyro.quaternion[3]		=	0.0f;
}


/*--------------------------------------*/
/*	gyro_cntr_fun						*/
/*--------------------------------------*/
void 	gyro_cntr_fun( void )
{
	float	w, w2, w3;

	float gain = 0.001;

	//TODO 宣言文をヘッダにする
	constexpr float degree_to_radian = M_PI / 180;

	constexpr float absolute_pitch_degree_limit = 40;
	constexpr float absolute_roll_degree_limit = 30;
	constexpr float absolute_yaw_degree_limit = 20;

	constexpr float maximum_degree_per_seconds = 0.01188; //単位時間に稼働する限界角度
	constexpr float flame_per_seconds = 0.01;

	constexpr float fall_down_radian = 30.0 * degree_to_radian;

	const Eigen::Vector3f chien_hrones_reswick_zero(0.95, 2.4, 0.4); //CHR法

	const float gradient = gain / maximum_degree_per_seconds / flame_per_seconds;
	const float distance = flame_per_seconds;

	static Eigen::Vector3f rotate_velocity_old;

	Eigen::Vector3f rotate_acceleration = Eigen::Vector3f::Zero();
	Eigen::Vector3f rotate_velocity = Eigen::Vector3f::Zero();
	Eigen::Vector3f rotate_displacement = Eigen::Vector3f::Zero();

	Eigen::Vector3f pid_gain = Eigen::Vector3f::Zero();
	Eigen::Vector3f pid_degree = Eigen::Vector3f::Zero();
	Eigen::Vector3f pid_pitch = Eigen::Vector3f::Zero();
	Eigen::Vector2f pid_roll = Eigen::Vector2f::Zero();

	float pid_roll_hip_right = 0.0,
		  pid_roll_hip_left = 0.0,
		  pid_roll_foot_right = 0.0,
		  pid_roll_foot_left = 0.0;

	rotate_velocity << xv_gyro.gyro_data1_flt, xv_gyro.gyro_data2_flt, xv_gyro.gyro_data3_flt;

	rotate_displacement << xv_gyro.gyro_roll, xv_gyro.gyro_pitch, xv_gyro.gyro_yaw;

	rotate_displacement *= degree_to_radian;

	rotate_acceleration = rotate_velocity - rotate_velocity_old;

	rotate_velocity_old = rotate_velocity;

	if(-fall_down_radian  < rotate_displacement.x() && rotate_displacement.x() < fall_down_radian) {
		pid_gain = distance * chien_hrones_reswick_zero;
		pid_gain = pid_gain.asDiagonal() * Eigen::Vector3f(1/(gradient * pow(distance, 2)), 1, 1);

		pid_degree = pid_gain(2) * rotate_acceleration;
		pid_degree += pid_gain(1) * rotate_velocity;
		pid_degree += pid_gain(0) * rotate_displacement;

		pid_pitch = Eigen::Vector3f(0.2, 0.3, 0.5); //倒立振子モデルへの変換
		pid_pitch = pid_degree.y() * pid_pitch; 

		pid_roll = Eigen::Vector2f(0.3333, 1 - 0.3333); //IMUのY軸距離変換
		pid_roll = (pid_degree.x() - (pid_gain(0) * rotate_displacement.x())) * pid_roll;
		pid_roll = pid_roll - (pid_gain(0) * rotate_displacement.x() * Eigen::Vector2f(1,1));

		pid_roll_hip_right = limit(pid_roll(0), absolute_roll_degree_limit, -absolute_roll_degree_limit);
		pid_roll_hip_left = limit(pid_roll(1), absolute_roll_degree_limit, -absolute_roll_degree_limit);

		pid_roll_foot_right = limit(pid_roll(0), absolute_roll_degree_limit, -absolute_roll_degree_limit);
		pid_roll_foot_left = limit(pid_roll(1), absolute_roll_degree_limit, -absolute_roll_degree_limit);
	}
	else if(rotate_displacement.x() >= fall_down_radian) {
		pid_roll_foot_left = limit(1, absolute_roll_degree_limit, -absolute_roll_degree_limit);
	}
	else if(-fall_down_radian >= rotate_displacement.x()) {
		pid_roll_foot_right = limit(1, absolute_roll_degree_limit, -absolute_roll_degree_limit);
	}

	xv_gyro.deg_foot_pitch	=	limit( pid_pitch(0), absolute_pitch_degree_limit, -absolute_pitch_degree_limit);

	xv_gyro.deg_hip_pitch	=	limit( pid_pitch(1), absolute_pitch_degree_limit, -absolute_pitch_degree_limit);

	xv_gyro.deg_waist_pitch	=	limit( pid_pitch(2), absolute_pitch_degree_limit, -absolute_pitch_degree_limit);

	xv_gyro.deg_arm_pitch	=	limit( pid_degree.y(), absolute_pitch_degree_limit, -absolute_pitch_degree_limit);

	xv_gyro.deg_arm_roll	=	limit( pid_degree.x(), absolute_roll_degree_limit, -absolute_roll_degree_limit);

	w	=	xp_gyro.kp3_waist * xv_gyro.gyro_data3_flt;
	xv_gyro.deg_waist_yaw	=	limit( w, absolute_yaw_degree_limit, -absolute_yaw_degree_limit);

	if(flag_gyro.vib_auto == ON && flag_gyro.vib == ON)
	{
		double len = sqrt(xv_joy.walk_x_percent * xv_joy.walk_x_percent + xv_joy.walk_y_percent * xv_joy.walk_y_percent);
		double ratio = 1.0;
		if (len != 0) ratio = fabs(xv_joy.walk_x_percent / len);
		const float right_foot_roll_degree = pid_roll_foot_right * ratio;
		const float left_foot_roll_degree = pid_roll_foot_left * ratio;
		const float right_hip_roll_degree = pid_roll_hip_right * ratio;
		const float left_hip_roll_degree = pid_roll_hip_left * ratio;

		xv_kine[0].foot_r   -=  right_foot_roll_degree;
		xv_kine[1].foot_r   -=  left_foot_roll_degree;
		xv_ref.d_ref[FOOT_ROLL_R]   -=  right_foot_roll_degree;
		xv_ref.d_ref[FOOT_ROLL_L]   -=  left_foot_roll_degree;

		xv_kine[0].hip_r    -=  right_hip_roll_degree;
		xv_kine[1].hip_r    -=  left_hip_roll_degree;
		xv_ref.d_ref[LEG_ROLL_R]    -=  right_hip_roll_degree;
		xv_ref.d_ref[LEG_ROLL_L]    -=  left_hip_roll_degree;

		/***	knee feedback	***/
		xv_kine[0].knee		-=	xv_gyro.deg_hip_pitch;
		xv_kine[1].knee		-=	xv_gyro.deg_hip_pitch;
		xv_ref.d_ref[KNEE_R2]	-=	xv_gyro.deg_hip_pitch;
		xv_ref.d_ref[KNEE_L2]	-=	xv_gyro.deg_hip_pitch;

		w	=	xv_kine[0].foot_p - xv_gyro.deg_foot_pitch;
		w2	=	limit( w, xv_kine[0].knee, -90.0f );
		xv_kine[0].foot_p		=	w2;
		xv_ref.d_ref[KNEE_R1]	-=	xv_gyro.deg_foot_pitch;

		w	=	xv_kine[1].foot_p - xv_gyro.deg_foot_pitch;
		w2	=	limit( w, xv_kine[1].knee, -90.0f );
		xv_kine[1].foot_p		=	w2;
		xv_ref.d_ref[KNEE_L1]	-=	xv_gyro.deg_foot_pitch;

		if( xv_gyro.deg_arm_roll < 0.0f )
		{
			xv_ref.d_ref[ARM_ROLL_R]	-=	xv_gyro.deg_arm_roll;
		}
		else if( xv_gyro.deg_arm_roll > 0.0f )
		{
			xv_ref.d_ref[ARM_ROLL_L]	-=	xv_gyro.deg_arm_roll;
		}

		xv_ref.d_ref[ARM_PITCH_R]	-=	xv_gyro.deg_arm_pitch;
		xv_ref.d_ref[ARM_PITCH_L]	-=	xv_gyro.deg_arm_pitch;

		xv_kine[0].leg			-=	xv_gyro.deg_waist_pitch;
		xv_kine[1].leg			-=	xv_gyro.deg_waist_pitch;
	}
	else if(flag_gyro.vib_auto == ON && flag_gyro.vib == ON2)
	{
		const float right_foot_roll_degree = pid_roll_foot_right * 0.5;
		const float left_foot_roll_degree = pid_roll_foot_left * 0.5;
		const float right_hip_roll_degree = pid_roll_hip_right * 0.5;
		const float left_hip_roll_degree = pid_roll_hip_left * 0.5;

		xv_kine[0].foot_r   -=  right_foot_roll_degree;
		xv_kine[1].foot_r   -=  left_foot_roll_degree;
		xv_ref.d_ref[FOOT_ROLL_R]   -=  right_foot_roll_degree;
		xv_ref.d_ref[FOOT_ROLL_L]   -=  left_foot_roll_degree;

		xv_kine[0].hip_r    -=  right_hip_roll_degree;
		xv_kine[1].hip_r    -=  left_hip_roll_degree;
		xv_ref.d_ref[LEG_ROLL_R]    -=  right_hip_roll_degree;
		xv_ref.d_ref[LEG_ROLL_L]    -=  left_hip_roll_degree;

		/***	knee feedback	***/
		w	=	xv_gyro.deg_hip_pitch/2.f;
		xv_kine[0].knee		-=	w;
		xv_kine[1].knee		-=	w;
		xv_ref.d_ref[KNEE_R2]	-=	w;
		xv_ref.d_ref[KNEE_L2]	-=	w;

		w3	=	xv_gyro.deg_foot_pitch/2.f;
		w	=	xv_kine[0].foot_p - w3;
		w2	=	limit( w, xv_kine[0].knee, -90.0f );
		xv_kine[0].foot_p		=	w2;
		xv_ref.d_ref[KNEE_R1]	-=	w3;

		w	=	xv_kine[1].foot_p - w3;
		w2	=	limit( w, xv_kine[1].knee, -90.0f );
		xv_kine[1].foot_p		=	w2;
		xv_ref.d_ref[KNEE_L1]	-=	w3;

		w	=	xv_gyro.deg_arm_roll/2.f;
		if( w < 0.f )
		{
			xv_ref.d_ref[ARM_ROLL_R]	-=	w;
		}
		else if( w > 0.f )
		{
			xv_ref.d_ref[ARM_ROLL_L]	-=	w;
		}

		xv_kine[0].leg			-=	xv_gyro.deg_waist_pitch;
		xv_kine[1].leg			-=	xv_gyro.deg_waist_pitch;
	}
	else if(flag_gyro.vib_auto == ON && flag_gyro.vib == ON3)
	{
		const float right_foot_roll_degree = pid_roll_foot_right * 1.3f;
		const float left_foot_roll_degree = pid_roll_foot_left * 1.3f;
		const float right_hip_roll_degree = pid_roll_hip_right * 1.3f;
		const float left_hip_roll_degree = pid_roll_hip_left * 1.3f;

		xv_kine[0].foot_r   -=  right_foot_roll_degree;
		xv_kine[1].foot_r   -=  left_foot_roll_degree;
		xv_ref.d_ref[FOOT_ROLL_R]   -=  right_foot_roll_degree;
		xv_ref.d_ref[FOOT_ROLL_L]   -=  left_foot_roll_degree;

		xv_kine[0].hip_r    -=  right_hip_roll_degree;
		xv_kine[1].hip_r    -=  left_hip_roll_degree;
		xv_ref.d_ref[LEG_ROLL_R]    -=  right_hip_roll_degree;
		xv_ref.d_ref[LEG_ROLL_L]    -=  left_hip_roll_degree;

		/***	knee feedback	***/
		w	=	xv_gyro.deg_hip_pitch*1.3f;
		xv_kine[0].knee		-=	w;
		xv_kine[1].knee		-=	w;
		xv_ref.d_ref[KNEE_R2]	-=	w;
		xv_ref.d_ref[KNEE_L2]	-=	w;

		w3	=	xv_gyro.deg_foot_pitch*1.3f;
		w	=	xv_kine[0].foot_p - w3;
		w2	=	limit( w, xv_kine[0].knee, -90.0f );
		xv_kine[0].foot_p		=	w2;
		xv_ref.d_ref[KNEE_R1]	-=	w3;

		w	=	xv_kine[1].foot_p - w3;
		w2	=	limit( w, xv_kine[1].knee, -90.0f );
		xv_kine[1].foot_p		=	w2;
		xv_ref.d_ref[KNEE_L1]	-=	w3;

		w	=	xv_gyro.deg_arm_roll*2.f;
		if( w < 0.f )
		{
			xv_ref.d_ref[ARM_ROLL_R]	-=	w;
		}
		else if( w > 0.f )
		{
			xv_ref.d_ref[ARM_ROLL_L]	-=	w;
		}

		xv_kine[0].leg			-=	xv_gyro.deg_waist_pitch;
		xv_kine[1].leg			-=	xv_gyro.deg_waist_pitch;
	}
}


/*--------------------------------------*/
/*	gyro_yaw_cntr_fun					*/
/*--------------------------------------*/
void 	gyro_yaw_cntr_fun( void )
{
	float	_err, _dead;

	if( flag_gyro.yaw_cntl )																		/*	gyro yaw control on	*/
	{
		_err	=	xv_gyro.yaw_cntl_ref - xv_gyro.gyro_yaw;
		_dead	=	xp_gyro.yaw_cntl_gain * deadband( _err , xp_gyro.yaw_cntl_dead );				/*	P control	*/
		xv_gyro.yaw_cntl_fb	=	limit( _dead, xp_gyro.yaw_cntl_theta, -xp_gyro.yaw_cntl_theta );	/*	limit		*/

		if( xv_gyro.yaw_cntl_fb > EPS_DATA )
		{
			xv_mv_walk.theta_percent_dlim	=	xv_gyro.yaw_cntl_fb / xp_mv_walk.theta;
		}
		else if( xv_gyro.yaw_cntl_fb < - EPS_DATA )
		{
			xv_mv_walk.theta_percent_dlim	=	xv_gyro.yaw_cntl_fb / xp_mv_walk.theta;
		}
		else
		{
			xv_mv_walk.theta_percent_dlim	=	0.f;
		}
	}
}

void PostureControl(void)
{
	float w,w1,w2;
	float hip_pitch_fd;
	float knee_pitch_fd;
	float foot_pitch_fd;
	float gain=0.85;
	const float thre_control = 3.0;

	//Hip Picth Control
	w = gain * xv_gyro.gyro_pitch;
	hip_pitch_fd = limit(w,25,-25);
	xv_kine[0].leg	-= hip_pitch_fd;
	xv_kine[1].leg	-= hip_pitch_fd;
	xv_ref.d_ref[LEG_PITCH_L]	-= hip_pitch_fd;
	xv_ref.d_ref[LEG_PITCH_R]	-= hip_pitch_fd;

	if(xv_gyro.gyro_pitch < -thre_control){
		w1 = gain * xv_gyro.gyro_pitch;
		knee_pitch_fd = limit(w1,18,-18);
		xv_kine[0].knee -= knee_pitch_fd;
		xv_kine[1].knee -= knee_pitch_fd;
		xv_ref.d_ref[LEG_PITCH_L] -= knee_pitch_fd;
		xv_ref.d_ref[LEG_PITCH_R] -= knee_pitch_fd;
	}else if(thre_control < xv_gyro.gyro_pitch){
		w2 = gain * xv_gyro.gyro_pitch;
		foot_pitch_fd = limit(w2,18,-18);
		xv_kine[0].foot_p -= foot_pitch_fd;
		xv_kine[1].foot_p -= foot_pitch_fd;
		xv_ref.d_ref[LEG_PITCH_L] -= foot_pitch_fd;
		xv_ref.d_ref[LEG_PITCH_R] -= foot_pitch_fd;
	}
}

void posture_pid_controller(tp_xv_gyro gyro) {
}

