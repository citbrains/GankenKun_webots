/*----------------------------------------------------------*/
/*	start sequence											*/
/*															*/
/*	file name	:	sq_start.c								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2004.10.26								*/
/*----------------------------------------------------------*/

#include "sq_start.h"
#include "servo_rs.h"
#include "b3m.h"
#include "motion.h"

// short flag_sq_start;					// sq_startを実行するかのフラグ（順番待ちあり）
short flag_md_start_end;				// modeの終了フラグ
short mode_sq_start, mode_sq_start_prev;

// 起動シーケンスの初期化
void sq_start_init()
{
//	sq_flag.start       =   ON;
	flag_md_start_end	= 	OFF;		// modeの終了フラグ
	mode_sq_start_prev	=   -1;
	mode_sq_start		=	0;
	servo_period		=	0;
}

int	servo_offset[SERV_NUM];	// オフセット保存用

/*--------------------------------------*/
/*	sq_start_mode						*/
/*--------------------------------------*/
// 起動時のシーケンス
int sq_start()
{
	static float mode_sq_time;

	if (mode_sq_start != mode_sq_start_prev)
	{
		switch( mode_sq_start )
		{
			case 0:
				/***	servo_reset		***/
				B3MAllReset( B3M_RESET_AFTER_TIME );
				mode_sq_time	=	0.06f;
				break;

			case 1:
				/***	servo_free_mode	***/
				write_servo_rs_all( B3M_SERVO_SERVO_MODE            , &xp_servo_rs.free_mode[0], 1 );
				mode_sq_time	=	0.02f;
				break;

			case 2:
				write_servo_rs_all( B3M_SYSTEM_DEADBAND_WIDTH       , &xp_servo_rs.deadband_width[0], 2 );
				mode_sq_time	=	0.02f;
				break;

			case 3:
				/***	servo_control_mode	***/
				write_servo_rs_all( B3M_SERVO_SERVO_MODE, &xp_servo_rs.run_or_control[0], 1 );
				mode_sq_time	=	0.02f;
				break;

			case 4:
				/***	servo_trajectory_even	***/
				write_servo_rs_all( B3M_SERVO_RUN_MODE, &xp_servo_rs.trajectory_even[0], 1 );
				mode_sq_time	=	0.02f;
				break;

			case 5:
				/***	write_servo_normal_mode	***/
				write_servo_rs_all( B3M_SERVO_SERVO_MODE, &xp_servo_rs.normal_mode[0], 1 );
				mode_sq_time	=	0.02f;
				break;

			case 6:
				write_servo_rs_all( B3M_CONTROL_KP0                 , &xp_servo_rs.control_kp0[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 7:
				write_servo_rs_all( B3M_CONTROL_KD0                 , &xp_servo_rs.control_kd0[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 8:
				write_servo_rs_all( B3M_CONTROL_KI0                 , &xp_servo_rs.control_ki0[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 9:
				write_servo_rs_all( B3M_CONTROL_STATIC_FRICTION0    , &xp_servo_rs.control_static_friction0[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 10:
				write_servo_rs_all( B3M_CONTROL_DYNAMIC_FRICTION0   , &xp_servo_rs.control_dynamic_friction0[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 11:
				write_servo_rs_all( B3M_CONTROL_KP1                 , &xp_servo_rs.control_kp1[0], 2 );
				mode_sq_time	=	0.02f;
				break;

			case 12:
				write_servo_rs_all( B3M_CONTROL_KD1                 , &xp_servo_rs.control_kd1[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 13:
				write_servo_rs_all( B3M_CONTROL_KI1                 , &xp_servo_rs.control_ki1[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 14:
				write_servo_rs_all( B3M_CONTROL_STATIC_FRICTION1    , &xp_servo_rs.control_static_friction1[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 15:
				write_servo_rs_all( B3M_CONTROL_DYNAMIC_FRICTION1   , &xp_servo_rs.control_dynamic_friction1[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 16:
				write_servo_rs_all( B3M_CONTROL_KP2                 , &xp_servo_rs.control_kp2[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 17:
				write_servo_rs_all( B3M_CONTROL_KD2                 , &xp_servo_rs.control_kd2[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 18:
				write_servo_rs_all( B3M_CONTROL_KI2                 , &xp_servo_rs.control_ki2[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 19:
				write_servo_rs_all( B3M_CONTROL_STATIC_FRICTION2    , &xp_servo_rs.control_static_friction2[0], 2 );
				mode_sq_time	=	0.02f;
				break;
                
			case 20:
				write_servo_rs_all( B3M_CONTROL_DYNAMIC_FRICTION2   , &xp_servo_rs.control_dynamic_friction2[0], 2 );
				mode_sq_time	=	0.02f;
				break;

			case 21:
				/***	write_servo_gain_change	***/
				write_servo_rs_all( B3M_CONTROL_GAIN_PRESETNO, &xp_servo_rs.gain_preset[0], 1 );
				mode_sq_time	=	0.02f;
				break;

			case 22:
				/***	write_servo_position	***/
				{
					int i;
					unsigned short position[SERV_NUM];
					for(i = 0; i < SERV_NUM; i ++){
						position[i] = xv_servo_rs.goal_position[i] + servo_offset[i];
					}
					Write_All_B3M_Position_or_Time( (unsigned short *)&position[0], &xp_servo_rs.goal_time_slow[0], 2 );
				}
				mode_sq_time	=	3.0f;
				break;

			case 23:
				/***	servo_trajectory_normal	***/
				write_servo_rs_all( B3M_SERVO_RUN_MODE, &xp_servo_rs.trajectory_normal[0], 1 );
				mode_sq_time	=	0.02f;
				break;

			case 24:
				sq_flag.start		=	OFF;
				flag_md_start_end	= 	ON;
				break;

			default:
				break;
		}
	}
	mode_sq_start_prev = mode_sq_start;

	if ((!flag_md_start_end)&&(mode_sq_time < EPS_TIME)){
		mode_sq_start ++;
	} else {
		mode_sq_time -= RTC_TIME_SEC;
	}

	return flag_md_start_end;		// 終了かどうかのフラグ
}
