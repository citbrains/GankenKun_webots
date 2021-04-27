/*----------------------------------------------------------*/
/*	motion play - special action							*/
/*															*/
/*															*/
/*	file name	:	sq_motion.c								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.8.19								*/
/*----------------------------------------------------------*/
#define		_SQ_MODE2_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	"var.h"
#include	"sq_motion.h"
#include	"calc_mv.h"
#include	"serv.h"
#include	"sq_straight.h"
#include	"motion.h"
#include	"gyro.h"
#include    <stdio.h>

static float	xv_ref_d_org[SERV_NUM];
short flag_md_motion_end;

/*--------------------------------------*/
/*	motion							*/
/*--------------------------------------*/
int sq_motion()
{
	static float	mode_sq_time	=	0;
	short	i, j, n;
	short	_flag_w;
	static	short	_flag_motion_select;

	if( mode_sq_motion == SQ_MODE2_INIT )
	{
		_flag_motion_select	=	flag_motion_select;
	}
	n = _flag_motion_select;


	switch( mode_sq_motion )
	{
		case	SQ_MODE2_INIT:			/* init */
			/* action */
			count_motion_repeat	=	0;
			flag_md_motion_end 	= 	OFF;
			flag_md_motion_hold	=	OFF;
			flag_sq_motion_cancel		=	OFF;
			flag_motion_gyro		=	OFF;
			flag_gyro.vib_auto	=	ON;	// Gyro feedback off

			{
				for( i = 0; i < SERV_NUM; i++ )		xv_ref_d_org[i]	=	xv_ref.d[i];
			}

			if( n >= MODE2_LAMP1 && n <= MODE2_LAMP2 )
			{
				for( i = 0; i < SERV_NUM; i++ )
				{
					xv_data_d[i].mv_tbl_select	=	MV_TBL_LAMP;
				}
			}

			/* action */
				sq_motion_mtn	=	0;

				xv_motion_n_last		=	n;		/*	store last played motion number	*/

				/* status */
				if( n >= 0 && n < MODE2_MOTION_NUM )	/*	size check	*/
				{
				if( xp_mv_motionbuf[n][sq_motion_mtn].time > 0 )
				{
					mode_sq_motion	=	SQ_MODE2_1A;	mode_sq_time	=	0.f;

					set_sw_ref_d(JOINT_ANGLE);			//	joint angle control
				}
				else
				{
					mode_sq_motion	=	SQ_MODE2_END;
				}
				}
				else
				{
					mode_sq_motion	=	SQ_MODE2_END;
				}
			break;

		case	SQ_MODE2_1A:			/* 1 */
			/*** send motion data divided by cycle time	***/

            for(i=0; i <= 28; i++){
                if(i != HEAD_YAW && i != HEAD_PITCH){
                    if(flag_variable_motion == ON){
                        xv_data_d[i].time = xp_mv_motionbuf[n][sq_motion_mtn].time / 100.f;
                        xv_data_d[i].pos  = xp_mv_motionbuf[n][sq_motion_mtn].d[i] + xp_mv_motionbuf_var[n][sq_motion_mtn].d[i] * variable_amount;
                        //printf("var_d[%d] = %d", i, xp_mv_motionbuf_var[n][sq_motion_mtn].d[i]);
                    }else{
                        xv_data_d[i].time = xp_mv_motionbuf[n][sq_motion_mtn].time / 100.f;
                        xv_data_d[i].pos  = xp_mv_motionbuf[n][sq_motion_mtn].d[i];
                    }
                }else{
                    if(!flag_face_control){
                        if(flag_variable_motion == ON){
                            xv_data_d[i].time = xp_mv_motionbuf[n][sq_motion_mtn].time / 100.f;
                            xv_data_d[i].pos  = xp_mv_motionbuf[n][sq_motion_mtn].d[i] + xp_mv_motionbuf_var[n][sq_motion_mtn].d[i] * variable_amount;
                        }else{
                            xv_data_d[i].time = xp_mv_motionbuf[n][sq_motion_mtn].time / 100.f;
                            xv_data_d[i].pos  = xp_mv_motionbuf[n][sq_motion_mtn].d[i];
                        }
                    }
                }
            }

			if( (xp_mv_motionbuf[n][sq_motion_mtn].cntr[2] & 0xF0) == MODE2_GYRO1 )
			{
				flag_motion_gyro		=	ON;
			}
			else if( (xp_mv_motionbuf[n][sq_motion_mtn].cntr[2] & 0xF0) == MODE2_GYRO2 )
			{
				flag_motion_gyro		=	ON3;
			}
			else
			{
				flag_motion_gyro		=	OFF;
			}

			/* status */
			mode_sq_motion	=	SQ_MODE2_WAIT;	mode_sq_time	=	0.f;

			break;

		case	SQ_MODE2_WAIT:			/* wait */
			/* action */
			_flag_w	=	0;

			/* status */
			/*	time passed		*/
			if( mode_sq_time >= (xp_mv_motionbuf[n][sq_motion_mtn].time/100.f - EPS_TIME) )
			{
				if( flag_sq_motion_cancel )
				{
					_flag_w	=	1;
				}
				else if( (xp_mv_motionbuf[n][sq_motion_mtn].cntr[2] & 0x0F) >= MODE2_HOLD && sq_motion_mtn < MODE2_MOTION_SIZE )
				{
					_flag_w	=	2;
					flag_md_motion_hold	=	ON;
				}
				else if( (!flag_motion_repeat && xp_mv_motionbuf[n][sq_motion_mtn].cntr[0] == 0) ||
						 ( flag_motion_repeat && xp_mv_motionbuf[n][sq_motion_mtn].cntr[1] == 0) )
				{
					_flag_w	=	3;
				}
				else if( (!flag_motion_repeat && sq_motion_mtn == xp_mv_motionbuf[n][sq_motion_mtn].cntr[0]-1 ) || 
						(  flag_motion_repeat && sq_motion_mtn == xp_mv_motionbuf[n][sq_motion_mtn].cntr[1]-1 ) ) 
				{
					if( sq_motion_mtn >= 0 && sq_motion_mtn <= MODE2_MOTION_SIZE-1 )
					{
						++sq_motion_mtn;
					}
					else
					{
						_flag_w	=	4;
					}
				}
				else if( !flag_motion_repeat || (xp_mv_motionbuf[n][sq_motion_mtn].cntr[0] == xp_mv_motionbuf[n][sq_motion_mtn].cntr[1]) )
				{
					if( xp_mv_motionbuf[n][sq_motion_mtn].cntr[0] >= 1 && xp_mv_motionbuf[n][sq_motion_mtn].cntr[0] <= MODE2_MOTION_SIZE )
					{
						sq_motion_mtn	=	xp_mv_motionbuf[n][sq_motion_mtn].cntr[0] - 1;
					}
					else
					{
						_flag_w	=	5;
					}
				}
				else if( flag_motion_repeat && count_motion_repeat < xv_motion_repeat_num )
				{
					if( xp_mv_motionbuf[n][sq_motion_mtn].cntr[1] >= 1 && xp_mv_motionbuf[n][sq_motion_mtn].cntr[1] <= MODE2_MOTION_SIZE )
					{
						++count_motion_repeat;
						sq_motion_mtn	=	xp_mv_motionbuf[n][sq_motion_mtn].cntr[1] - 1;
					}
					else
					{
						_flag_w	=	6;
					}
				}
				else
				{
					if( sq_motion_mtn >= 1 && sq_motion_mtn <= MODE2_MOTION_SIZE )
					{
						++sq_motion_mtn;
					}
					else
					{
						_flag_w	=	7;
					}
				}

				if( !_flag_w )
				{
					if( xp_mv_motionbuf[n][sq_motion_mtn].time > 0 && sq_motion_mtn < MODE2_MOTION_SIZE )
					{
						mode_sq_motion	=	SQ_MODE2_1A;	mode_sq_time	=	0.f;
					}
					else
					{
						mode_sq_motion	=	SQ_MODE2_END;
					}
				}
				else
				{
					mode_sq_motion	=	SQ_MODE2_END;
				}
			}

			/*	cancel now	2009.11.23	*/
			if( flag_sq_motion_cancel )
			{
				mode_sq_motion	=	SQ_MODE2_END;
			}

			break;

		case	SQ_MODE2_END:			/* end	 */
			flag_gyro.vib_auto	=	ON;
			flag_gyro.vib		=	ON;
            flag_variable_motion = OFF;

			/* action */
			if( n >= MODE2_LAMP1 && n <= MODE2_LAMP2 )
			{
				for( i = 0; i < SERV_NUM; i++ )
				{
					xv_data_d[i].mv_tbl_select	=	MV_TBL_SIN;
				}
			}

			/*	store last played motion data	*/
			for( j = 0; j < MODE2_MOTION_SIZE; j ++ )
			{
				xp_mv_motion_last[j].time		=	xp_mv_motionbuf[xv_motion_n_last][j].time;

				xp_mv_motion_last[j].cntr[0]	=	xp_mv_motionbuf[xv_motion_n_last][j].cntr[0];
				xp_mv_motion_last[j].cntr[1]	=	xp_mv_motionbuf[xv_motion_n_last][j].cntr[1];
				xp_mv_motion_last[j].cntr[2]	=	xp_mv_motionbuf[xv_motion_n_last][j].cntr[2];

				for( i = 0; i < SERV_NUM; i ++ )
				{
					xp_mv_motion_last[j].d[i]	=	xp_mv_motionbuf[xv_motion_n_last][j].d[i];
				}
			}

			/* status */
			if( flag_sq_motion_cancel )
			{
				sq_flag.motion	=	OFF;
				flag_md_motion_end	= 	ON;
				mode_sq_motion	=	SQ_MODE2_INIT;	mode_sq_time	=	0.f;
			}
			else if( (xp_mv_motionbuf[n][sq_motion_mtn].cntr[2] & 0x0F) == MODE2_CONTINUE )
			{
				flag_motion_select	=	n + 1;
				sq_flag.motion		=	ON;
				mode_sq_motion	=	SQ_MODE2_INIT;	mode_sq_time	=	0.f;
			}
			else if( (xp_mv_motionbuf[n][sq_motion_mtn].cntr[2] & 0x0F) == MODE2_CONTINUE_SW )
			{
				flag_motion_select	=	n + 1;
				sq_flag.motion		=	OFF;
				flag_md_motion_end	= 	ON;
				mode_sq_motion	=	SQ_MODE2_INIT;	mode_sq_time	=	0.f;
			}
			else
			{
				sq_flag.motion	=	OFF;
				flag_md_motion_end	= 	ON;
				mode_sq_motion	=	SQ_MODE2_INIT;	mode_sq_time	=	0.f;
			}

			break;

		default:
			break;
	}

	mode_sq_time	+=	RTC_TIME_SEC;
	return flag_md_motion_end;
}


/*--------------------------------------*/
/*	sq_motion_init						*/
/*--------------------------------------*/
void sq_motion_init()
{
	short	i, j, k;

	sq_flag.motion			=	OFF;
	flag_md_motion_end		=	ON;
	mode_sq_motion			=	SQ_MODE2_INIT;

	flag_motion_select			=	0;
	flag_md_motion_hold			=	OFF;
	flag_sq_motion_cancel		=	OFF;
	flag_motion_repeat			=	OFF;
	sq_motion_mtn				=	0;
	count_motion_repeat			=	0;
	xv_motion_repeat_num			=	1000;
	flag_motion_preload			=	OFF;
	flag_motion_preload_end		=	OFF;
	xv_motion_n_last				=	0;
	flag_motion_gyro				=	OFF;

	for( k = 0; k < MODE2_MOTION_NUM; k ++ )
	{
		for( j = 0; j < MODE2_MOTION_SIZE; j ++ )
		{
			xp_mv_motionbuf[k][j].time		=	0;

			xp_mv_motionbuf[k][j].cntr[0]	=	0;
			xp_mv_motionbuf[k][j].cntr[1]	=	0;
			xp_mv_motionbuf[k][j].cntr[2]	=	0;

			for( i = 0; i < SERV_NUM; i ++ )
			{
				xp_mv_motionbuf[k][j].d[i]	=	0;
			}
		}
	}
}
