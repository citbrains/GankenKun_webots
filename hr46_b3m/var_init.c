/*----------------------------------------------------------*/
/*	global variables initialization							*/
/*															*/
/*															*/
/*	file name	:	var_init.c								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*															*/
/*	memo		:	南方，土橋が実験用に数値を変更			*/
/*	date		:	2011.12.29								*/
/*----------------------------------------------------------*/

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/

#include	<string.h>
#include	<math.h>
#include	"var.h"
#include	"func.h"
#include	"servo_rs.h"
#include	"b3m.h"
#include	"motion.h"
#include	"calc_mv.h"
#include	"mvtbl.h"
#include	"kine.h"
#include	"calc_deg.h"
#include	"serv.h"
#include	"sq_start.h"
#include	"sq_straight.h"
#include	"sq_ready.h"
#include	"sq_walk.h"
#include	"acc.h"
#include	"gyro.h"
#include	"sq_motion.h"
#include	"joy.h"

/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
volatile unsigned long 	count_time_l 	;	//	rtc time
short 	flag_face_control		;


/*--------------------------------------*/
/*	variables initialize				*/
/*--------------------------------------*/
void	var_init( void )
{
	short	i;

	count_time_l	=	0;					// メインループカウンタの初期化
	mv_tbl_zmp_sel		=	MV_TBL_ZMP2;	// テーブルZMP2を使用する
	mode_init();							// モードの初期化

	/*	flag initialize	*/
	flag_moving				=	STATE_STOP;	// 動きの状態のフラグ
	flag_face_control		=	OFF;		// 頭の動きは停止
//	sq_flag.straight		=	ON;			// 立ち上がるシーケンスオン

	/*	servo reference angle select switch	*/
	sw.ref_d	=	FOOT_XYZ;				/* foot xyz position control (inv. kinematics) 	*/

 	acc_init();								// 加速度センサの初期化
 	gyro_init();							// ジャイロの初期化

	/*	inv. kinematics initial data = straight leg length	*/
	xv_kine[0].z	=	Z3_LIMIT_H;			// 腰の高さ
	xv_kine[1].z	=	Z3_LIMIT_H;			// 腰の高さ

	for( i=0; i<1; i++ )					// 足先位置の初期化
	{
		tp_xv_kine *kine = &xv_kine[i];
		kine->x			= 0.0f;				// x座標(mm)
		kine->y			= 0.0f;				// y座標(mm)
		kine->z			= 0.0f;				// z座標(mm)
		kine->yaw		= 0.0f;				// 股のヨー軸(deg)
		kine->hip_r		= 0.0f;				// 股のロール軸(deg)
		kine->leg		= 0.0f;				// 股のピッチ軸(deg)
		kine->knee		= 0.0f;				// 膝のピッチ軸(deg)
		kine->foot_p	= 0.0f;				// 足のピッチ軸(deg)
		kine->foot_r	= 0.0f;				// 足のロール軸(deg)
	}

	xv_posture.pitch	=	0.f;			// 上半身の姿勢の初期化
	xv_posture.yaw		=	0.f;
	xv_posture.roll2	=	0.f;

	// 直立姿勢
	xp_mv_straight.time			=	1.0f;					// 直立するための時間　　　（定数）
	xp_mv_straight.z3			=	Z3_LIMIT_H;				// 腰の高さ　　　　　　　　（定数）
	xp_mv_straight.arm_sh_pitch	=	0.f;					// 腕のピッチ軸の角度（度）（定数）
	xp_mv_straight.arm_sh_roll	=	0.f;					// 腕のロール軸の角度（度）（定数）
	xp_mv_straight.arm_el_yaw	=	0.f;					// 肘のピッチ軸の角度（度）（定数）
	xp_mv_straight.arm_el_pitch	=	0.f;					// 肘のロール軸の角度（度）（定数）
	xv_mv_straight_time			=	1.0f;	// 直立するための時間

	// レディ姿勢
	xp_mv_ready.time			=	1.5f;					// レディ姿勢になるための時間
	xp_mv_ready.z3				=	Z3_LIMIT_H * 0.95f;		// 腰の高さ：5%腰を下げる
	xp_mv_ready.arm_sh_pitch	=	0.f;					// 腕のピッチ軸の角度（度）
	xp_mv_ready.arm_sh_roll		=	2.f;					// 腕のロール軸の角度（度）
	xp_mv_ready.arm_el_yaw		=	0.f;					// 肘のピッチ軸の角度（度）
	xp_mv_ready.arm_el_pitch	=	0.f;					// 肘のロール軸の角度（度）
	xp_mv_ready.pitch			=	0.f;					// 腰のピッチ軸の角度（度）
	flag_ready_gyro				=	OFF;					// readyでジャイロを切る
	xv_mv_ready_time			=	xp_mv_ready.time;		// レディ姿勢になるための時間
	
	touchdown_gain				=	0.0f;

	//オドメトリ補正値
	odometry_correct_para_x		=	1.0f;					// オドメトリの係数
	odometry_correct_para_y		=	1.0f;					// オドメトリの係数

	xv_mv.count					=	0;						// 歩行する歩数

	for( i=0; i<SERV_NUM; i++ )
	{
		xv_pv.temp[i]			=	0;						// モータの温度(deg)
		xv_pv.deg[i]			=	0.0f;					// モータの角度(deg)
		xv_pv.current[i]		=	0;						// モータの電流(mA)
	}

	sq_walk_init();											// 歩行の初期化
	servo_rs_init();										// サーボの初期化

	sq_motion_init();										// eepromから読み込むモーションの初期化
	joy_init();												// コマンド関連の初期化

	xp_ref_d_lim	=	360.f/RTC_TICK;						// dlimit for leg joint = 360 [deg/s]
															// ハードウェアの速度制限はもっと厳しい
	// 脚の上下を楕円の円弧にする．
	for(i = 0; i <= MV_TBL_PI; i ++) mv_tbl[MV_TBL_Z_UP][i] =       sin(M_PI / 2.0 * i / MV_TBL_PI);
	for(i = 0; i <= MV_TBL_PI; i ++) mv_tbl[MV_TBL_Z_DW][i] = 1.0 - cos(M_PI / 2.0 * i / MV_TBL_PI);
	for(i = 0; i <= MV_TBL_PI; i ++) mv_tbl[MV_TBL_X_UP][i] = 1.0 - cos(M_PI / 2.0 * i / MV_TBL_PI);
	for(i = 0; i <= MV_TBL_PI; i ++) mv_tbl[MV_TBL_X_DW][i] =       sin(M_PI / 2.0 * i / MV_TBL_PI);
}


/*--------------------------------------*/
/*	variables initialize				*/
/*--------------------------------------*/
void	mode_init( void )
{
	mode_motion				=	MOTION_START		;			// 現在の動きのモード
	flag_servo_on			=	0;
//	mode_sq_ready			=	SQ_READY_INIT	;
	mode_sq_walk			=	SQ_WALK_INIT	;
//	mode_sq_motion			=	SQ_MODE2_INIT	;

	memset(&sq_flag, 0, sizeof(sq_flag));					// シーケンスを実行するかのフラグ

//	flag_md_ready_end		=
//	flag_md_walk_end		=	ON				;
//	flag_md_motion_end		=	ON				;
}


/*--------------------------------------*/
/*	walk_init							*/
/*--------------------------------------*/
void	sq_walk_init( void )
{
	flag_walk.upleg				=	OFF;
	flag_walk.upleg_last		=	OFF;
	flag_walk.y					=	STRAIGHT;
	flag_walk.turn				=	STRAIGHT;
	flag_walk.y_on				=	STRAIGHT;
	flag_walk.turn_on			=	STRAIGHT;

	//	high speed walk
	xp_mv_walk.num					=	8;
	xp_mv_walk.h_cog				=	300.0f	;
	//	hight of center of gravity	xp_mv_ready.z3 + 0mm from CAD
	xp_mv_walk.time					=	0.23f	;	// 歩行の周期
	xp_mv_walk.x_fwd_swg			=	55.0f	;
	xp_mv_walk.x_fwd_spt			=	-55.0f	;
	xp_mv_walk.x_bwd_swg			=	-55.0f	;
	xp_mv_walk.x_bwd_spt			=	55.0f	;
	xp_mv_walk.y_swg				=	30.0f	;
	xp_mv_walk.y_spt				=	-40.0f	;
	xp_mv_walk.theta				=	18.0f	;
	xp_mv_walk.z					=	25.0f	;
	xp_mv_walk.y_balance			=	110.0f	;
	xp_mv_walk.hip_roll				=	2.0f	; // 歩くときにヒップをロールさせる量
	xp_mv_walk.x_fwd_pitch			=	2.0f	;
	xp_mv_walk.x_bwd_pitch			=	-2.0f	; // 前進するときに前に倒す角度
	xp_mv_walk.x_fwd_acc_pitch		=	2.0f	;
	xp_mv_walk.x_bwd_acc_pitch		=	-2.0f	; // 前に加速するときに前に倒す角度
	xp_mv_walk.arm_sh_pitch			=	15.0f	;
	xp_mv_walk.start_zmp_k1			=	1.2f	;
	xp_mv_walk.start_time_k1		=	0.8f	;
//	xp_mv_walk.start_time_k2		=	0.5f	;
	xp_mv_walk.foot_cntl_p			=	0.0		;
	xp_mv_walk.foot_cntl_r			=	1.0		;	
	xp_mv_walk.sidestep_time_k		=	0.05f	;
	xp_mv_walk.sidestep_roll		=	1.0f	;
	xp_mv_walk.y_wide				=	1.0f	;
	xp_mv_walk.time_dutyfactor		=	1.0f	;
    xp_mv_walk.accurate_x_percent_dlim  = 1.0f  ;
    xp_mv_walk.accurate_y_percent_dlim  = 1.0f  ;
    xp_mv_walk.accurate_th_percent_dlim = 0.6667f;
    xp_mv_walk.accurate_step_z          = 55.f;
    xp_mv_walk.accurate_step_time       = 0.32f;

	//	4 is even
	//	walk begins right foot first and fast walk begins left foot first,
	//	because of streight walk.
	xv_mv_walk.num					=	xp_mv_walk.num;
	xv_mv_walk.time					=	xp_mv_walk.time;
	xv_mv_walk.time_old				=	0.f;
	xv_mv_walk.x_swg				=	0.f;
	xv_mv_walk.x_spt				=	0.f;
	xv_mv_walk.y_swg				=	0.f;
	xv_mv_walk.y_spt				=	0.f;
	xv_mv_walk.theta				=	0.f;
	xv_mv_walk.z					=	xp_mv_walk.z;
	xv_mv_walk.pitch				=	0.f;
	xv_mv_walk.arm_sh_pitch			=	0.f;
	xv_mv_walk.zmp					=	0.f;
	xv_mv_walk.x_percent			=	0.f;
	xv_mv_walk.y_percent			=	0.f;
	xv_mv_walk.theta_percent		=	0.f;
	xv_mv_walk.sidestep_time_k_r	=	0.f;
	xv_mv_walk.sidestep_time_k_l	=	0.f;
	xv_mv_walk.sidestep_roll		=	0.f;
	xv_mv_walk.sidestep_roll_z		=	0.f;
	xv_mv_walk.x_percent_dlim		=	0.f;
	xv_mv_walk.y_percent_dlim		=	0.f;
	xv_mv_walk.theta_percent_dlim	=	0.f;
	xv_mv_walk.pitch_percent_dlim	=	0.f;
	xv_mv_walk.time_dutyfactor		=	xp_mv_walk.time_dutyfactor;
    xv_mv_walk.accurate_step_x      =   0.f;
    xv_mv_walk.accurate_step_y      =   0.f;
    xv_mv_walk.accurate_step_z      =   xp_mv_walk.accurate_step_z;
    xv_mv_walk.accurate_step_th     =   0.f;
    xv_mv_walk.accurate_step_time   =   xp_mv_walk.accurate_step_time;

	xp_dlim_wait_x.dlim				=	1.0;
	xp_dlim_wait_y.dlim				=	1.0;
	xp_dlim_wait_theta.dlim			=	1.0f;			/*	[1/sec]		*/
	xp_dlim_wait_pitch.dlim			=	1.0f;			/*	[1/sec]		*/
	xp_dlim_wait_x.wait_time		=
	xp_dlim_wait_y.wait_time		=
	xp_dlim_wait_theta.wait_time	=	0.05f;			/*	[sec]		*/
	xp_dlim_wait_pitch.wait_time	=	0.0f;			/*	[sec]		*/	// ピッチの切り替えには待ち時間を作らない
	
	dlim_wait_fun_init( (tp_xp_dlim_wait *) &xp_dlim_wait_x, (tp_xv_dlim_wait *) &xv_dlim_wait_x );
	dlim_wait_fun_init( (tp_xp_dlim_wait *) &xp_dlim_wait_y, (tp_xv_dlim_wait *) &xv_dlim_wait_y );
	dlim_wait_fun_init( (tp_xp_dlim_wait *) &xp_dlim_wait_theta, (tp_xv_dlim_wait *) &xv_dlim_wait_theta );
	dlim_wait_fun_init( (tp_xp_dlim_wait *) &xp_dlim_wait_pitch, (tp_xv_dlim_wait *) &xv_dlim_wait_pitch );
}


/*--------------------------------------*/
/*	servo_rs_init						*/
/*--------------------------------------*/
void	servo_rs_init( void )
{
	static const int is_b3m1170[] = {0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    short	i;
	servo_rs_id					=	1;
//	servo_rs_address			=	SERVO_B3M_ADDR_TORQUE_ENABLE;
	servo_rs_len				=	1;
	servo_rs_sdata				=	1;
	servo_rs_rdata				=	0;
	flag_write_servo_rs_scif2	=	0;
	flag_write_servo_rs_scif3	=	0;
	flag_read_servo_rs_scif2	=	0;
	flag_read_servo_rs_scif3	=	0;
	flag_write_servo_rs_all		=	OFF;
	flag_read_servo_rs_all		=	OFF;
	flag_servo_rs_test			=	OFF;
	flag_servo_on				=	OFF;
	flag_servo_off				=	OFF;
	flag_servo_output_wait		=	0;
	servo_on_totaltime			=	0.f;
	flag_servo_output_wait		=	0;
	flag_read_servo_position	=	0;
	flag_ukemi					=	0;	//2010.4.29

	for( i = 0; i < SERV_NUM; i++ )
	{
		xp_servo_rs.normal_mode[i]          =	B3M_OPTIONS_RUN_NORMAL;
		xp_servo_rs.free_mode[i]			=	B3M_OPTIONS_RUN_FREE;
		xp_servo_rs.control_mode[i]         =	B3M_OPTIONS_CONTROL_POSITION;
		xp_servo_rs.run_or_control[i]		=	xp_servo_rs.free_mode[i-1] + xp_servo_rs.control_mode[i-1];
		xp_servo_rs.trajectory_normal[i]	=	B3M_OPTIONS_TRAJECTORY_NORMAL;
		xp_servo_rs.trajectory_even[i]      =	B3M_OPTIONS_TRAJECTORY_1;
		xp_servo_rs.goal_time_slow[i]		=	SERVO_B3M_DATA_GOAL_TIME_SLOW;
    	xp_servo_rs.gain_preset[i]          =	B3M_CONTROL_GAIN_PRESET_DEF;
        
        xp_servo_rs.deadband_width[i]               =   15;
        xp_servo_rs.control_kp0[i]                  =   is_b3m1170[i] ? 42000 :  35000 ;
        xp_servo_rs.control_kd0[i]                  =   is_b3m1170[i] ?   400 :    500 ;
        xp_servo_rs.control_ki0[i]                  =   is_b3m1170[i] ?  1000 :   1800 ;
        xp_servo_rs.control_static_friction0[i]     =   is_b3m1170[i] ?     0 :      0 ;
        xp_servo_rs.control_dynamic_friction0[i]    =   is_b3m1170[i] ?     0 :      0 ;
        xp_servo_rs.control_kp1[i]                  =   is_b3m1170[i] ?     0 :   5500 ;
        xp_servo_rs.control_kd1[i]                  =   is_b3m1170[i] ?     0 :    100 ;
        xp_servo_rs.control_ki1[i]                  =   is_b3m1170[i] ?     0 :    580 ;
        xp_servo_rs.control_static_friction1[i]     =   is_b3m1170[i] ?     0 :    100 ;
        xp_servo_rs.control_dynamic_friction1[i]    =   is_b3m1170[i] ?     0 :     50 ;
        xp_servo_rs.control_kp2[i]                  =   is_b3m1170[i] ? 42000 : 100000 ;
        xp_servo_rs.control_kd2[i]                  =   is_b3m1170[i] ?   750 :    500 ;
        xp_servo_rs.control_ki2[i]                  =   is_b3m1170[i] ?  6000 :  10000 ;
        xp_servo_rs.control_static_friction2[i]     =   is_b3m1170[i] ?     0 :    100 ;
        xp_servo_rs.control_dynamic_friction2[i]    =   is_b3m1170[i] ?    50 :     50 ;

		xv_servo_rs.goal_position[i]		=
		xv_servo_rs.present_position[i]     =
		xv_servo_rs.present_temp[i]         =
		xv_servo_rs.present_current[i]      =	0;
	}
	xp_servo_rs.gain_preset[ARM_PITCH_R]    =	B3M_CONTROL_GAIN_PRESET_LOW;
	xp_servo_rs.gain_preset[ARM_ROLL_R]     =	B3M_CONTROL_GAIN_PRESET_LOW;
	xp_servo_rs.gain_preset[ARM_PITCH_L]    =	B3M_CONTROL_GAIN_PRESET_LOW;
	xp_servo_rs.gain_preset[ARM_ROLL_L]     =	B3M_CONTROL_GAIN_PRESET_LOW;
	xp_servo_rs.gain_preset[LEG_ROLL_R]     =	B3M_CONTROL_GAIN_PRESET_HIGH;
	xp_servo_rs.gain_preset[LEG_ROLL_L]     =	B3M_CONTROL_GAIN_PRESET_HIGH;
	xp_servo_rs.gain_preset[LEG_PITCH_R]	=	B3M_CONTROL_GAIN_PRESET_HIGH;
	xp_servo_rs.gain_preset[LEG_PITCH_L]	=	B3M_CONTROL_GAIN_PRESET_HIGH;
	xp_servo_rs.gain_preset[FOOT_ROLL_R]	=	B3M_CONTROL_GAIN_PRESET_HIGH;
	xp_servo_rs.gain_preset[FOOT_ROLL_L]	=	B3M_CONTROL_GAIN_PRESET_HIGH;
}
