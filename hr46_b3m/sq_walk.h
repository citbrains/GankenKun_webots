/*----------------------------------------------------------*/
/*	high speed walk sequence								*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	sq_walk.h								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#ifndef 	_SQ_WALK_H_
#define 	_SQ_WALK_H_

#include	"func.h"

#ifdef 		_SQ_WALK_C_
#define 	extern 
#endif 		/* _SQ_WALK_C_ */


/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
enum {							//											サイドステップが無いときの時間
	SQ_WALK_INIT	,			//  1)初期化								t: 0
	SQ_WALK_MV_L	,			//  2)左に重心を移す						t: 0 - period/4*k2 (0にリセット)
	SQ_WALK_UP_R0	,			//  3)一歩目の右足を上げるまでの待ちの時間	t: period/4*k2
	SQ_WALK_UP_R	,			//  4)右足を上げるまでの動作				t: 0 - period/2*(1-duty)
	SQ_WALK_UP_R_1	,			//  5)右足を上げる動作						t: period/2*(1-duty)
	SQ_WALK_DW_R	,			//  7)右足を下げる動作						t: period/2*(1-duty/2) - period/2
	SQ_WALK_MV_R2	,			//  8)右に重心を移す						t: 0

	SQ_WALK_MV_R	,			//    右に重心を移す
	SQ_WALK_UP_L0	,			//    一歩目の左足を上げるまでの待ちの時間
	SQ_WALK_UP_L	,			//  9)左足を上げるまでの動作				t: 0 - period/2*(1-duty)
	SQ_WALK_UP_L_1	,			// 10)左足を上げる動作						t: period/2*(1-duty)
	SQ_WALK_DW_L	,			// 12)左足を下げる動作						t: period/2*(1-duty/2) - period/2
	SQ_WALK_MV_L2	,			// 13)左に重心を移す → 4)　最後の一歩 → 14)

	SQ_WALK_UP_R2	,			// 14)右足を上げる動作（最後の一歩）
	SQ_WALK_UP_R2_1	,			// 15)右足を上げる動作（最後の一歩）
	SQ_WALK_UP_R2_2	,			// 16)右足を上げる動作（最後の一歩）
	SQ_WALK_DW_R2	,			// 17)右足を下げる動作（最後の一歩）

	SQ_WALK_UP_L2	,			//    左足を上げる動作（最後の一歩）
	SQ_WALK_UP_L2_1	,			//    左足を上げる動作（最後の一歩）
	SQ_WALK_UP_L2_2	,			//    左足を上げる動作（最後の一歩）
	SQ_WALK_DW_L2	,			//    左足を下げる動作（最後の一歩）

	SQ_WALK_READY	,			//    レディの状態に戻る
	SQ_WALK_END		,			//    終了処理
};


typedef struct st_xp_mv_walk
{
	long	num;				/* number of steps for test [-]			*/
	float	h_cog;				/* height of center of gravity [mm]		*/
	float	time;				/* step time [sec] 						*/
//	float	next_walk;			/* timing for next step [1] 			*/
	float	x_fwd_swg;			/* forward stride of swing leg [mm]		*/
	float	x_fwd_spt;			/* forward stride of support leg [mm]	*/
	float	x_bwd_swg;			/* backward stride of swing leg [mm]	*/
	float	x_bwd_spt;			/* backward stride of support leg [mm]	*/
	float	y_swg;				/* sidestep stride of swing leg [mm]	*/
	float	y_spt;				/* sidestep stride of support leg [mm]	*/
	float	theta;				/* hip yaw angle of each leg for turn [deg]	*/
	float	z;					/* hight of foot up [mm] 				*/
	float	y_balance;			/* right and left balance point [mm] 	*/	// 股間の距離/2
	float	hip_roll;			/* hip roll when leg up [deg] 			*/
	float	x_fwd_pitch;		/* pitch for forward [deg] 				*/
	float	x_bwd_pitch;		/* pitch for backward [deg] 			*/
	float	x_fwd_acc_pitch;	/* pitch for forward under acceleration [deg]	*/
	float	x_bwd_acc_pitch;	/* pitch for backward under acceleration [deg]	*/
	float	arm_sh_pitch;		/* shoulder pitch for walk [deg] 		*/
    float   arm_el_pitch;		/* elbow pitch for walk [dag]			*/
	float	start_time_k1;		/* first step time to balance [1] 		*/
	float	start_zmp_k1;		/* first step zmp y to balance [1] 		*/
	float	start_time_k2;		/* first step check time to balance [1]	*/
	float	foot_cntl_p;		/* pitch of foot control [deg] 			*/
	float	foot_cntl_r;		/* roll of foot control [deg] 			*/
	float	sidestep_time_k;	/* sidestep time offset [0] 			*/
	float	sidestep_roll;		/* sidestep roll [deg] 					*/
	float	y_wide;				/* y position of walk [mm] 				*/
	float	time_dutyfactor;	/* duty factor of walk [1] 				*/
    float   accurate_x_percent_dlim;
    float   accurate_y_percent_dlim;
    float   accurate_th_percent_dlim;
    float   accurate_step_z;
    float   accurate_step_time;
}	tp_xp_mv_walk;


typedef struct st_xv_mv_walk
{
	long	num;				/* 歩数							[-]		*/
	float	time;				/* 歩行の周期/2					[sec]	*/
	float	time_old;			/* 歩行周期を最後に変更した時間	[sec]	*/
	float	x_swg;				/* 遊脚の前後の歩幅				[mm]	*/
	float	x_spt;				/* 支持脚の前後の歩幅			[mm]	*/
	float	y_swg;				/* 遊脚の左右の歩幅				[mm]	*/
	float	y_spt;				/* 支持脚の左右の歩幅			[mm]	*/
	float	theta;				/* ターン毎の股のヨー軸の角度	[deg]	*/
	float	z;					/* 足を上げる高さ				[mm]	*/
	float	pitch;				/* 前にかがむ角度				[deg]	*/
	float	arm_sh_pitch;		/* 肩のピッチ角度				[deg]	*/
    float   arm_el_pitch;		/* 肘のピッチ角度				[deg]	*/
	float	zmp;				/* ZMP規範で足をふる幅?			[mm]	*/
	float	x_percent;			/* x方向の歩幅の最終的な割合(0-1)[-]	*/
	float	y_percent;			/* y方向の歩幅の最終的な割合(0-1)[-]	*/
	float	theta_percent;		/* 股ヨー軸角の最終的な割合(0-1)[-]		*/
	float	sidestep_time_k_r;	/* 右足移動の時間の割合(0-1)	[bit]	*/
	float	sidestep_time_k_l;	/* 左足移動の時間の割合(0-1)	[bit]	*/
	float	sidestep_roll;		/* 横歩き時のロール軸回転角度	[deg]	*/
	float	sidestep_roll_z;	/* 横歩き時の股軸の高さの変化	[mm]	*/
	float	x_percent_dlim;		/* x方向の歩幅の現在の割合(0-1)	[-] 	*/
	float	y_percent_dlim;		/* y方向の歩幅の現在の割合(0-1)	[-] 	*/
	float	theta_percent_dlim;	/* 股ヨー軸角の現在の割合(0-1)	[-] 	*/
	float	pitch_percent_dlim;	/* ピッチ軸の現在の割合(0-1)	[-]		*/
	float	time_dutyfactor;	/* 遊脚である時間の比率(0-1)	[-]		*/
    float   accurate_step_x;    /* 一歩で前進する距離           [mm]    */
    float   accurate_step_y;
    float   accurate_step_z;
    float   accurate_step_th;
    float   accurate_step_time;
}	tp_xv_mv_walk;


typedef struct st_flag_walk
{
	short	upleg;				/*	up right leg or left leg		*/
	short	upleg_last;			/*	the last up lag					*/
	short	y;					/*	walk sidestep					*/
	short	turn;				/*	turn right or left				*/
	short	y_on;				/*	walk sidestep now				*/
	short	turn_on;			/*	turn right or left now			*/
}	tp_flag_walk;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern 	tp_xp_mv_walk	xp_mv_walk 			;	// 歩行に関する定数
extern 	tp_xv_mv_walk	xv_mv_walk 			;	// 歩行に関する変数
extern 	tp_flag_walk	flag_walk 			;	// 歩行のフラグ
extern 	short			mode_sq_walk	 	;	// 歩行のモードの番号
extern	tp_xp_dlim_wait	xp_dlim_wait_x		;	// x方向の歩幅の割合の定数
extern	tp_xv_dlim_wait	xv_dlim_wait_x		;	// x方向の歩幅の割合の変数
extern	tp_xp_dlim_wait	xp_dlim_wait_y		;	// y方向の歩幅の割合の定数
extern	tp_xv_dlim_wait	xv_dlim_wait_y		;	// y方向の歩幅の割合の変数
extern	tp_xp_dlim_wait	xp_dlim_wait_theta	;	// 股ヨー軸角の割合の定数
extern	tp_xv_dlim_wait	xv_dlim_wait_theta	;	// 股ヨー軸角の割合の変数
extern	tp_xp_dlim_wait	xp_dlim_wait_pitch	;	// ピッチ方向の傾きの定数
extern	tp_xv_dlim_wait	xv_dlim_wait_pitch	;	// ピッチ方向の傾きの変数
extern  char            accurate_one_step_mode;

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern int sq_walk( void );				// 歩行（1:終了，0:歩行中）20ms毎に呼び出し
extern void sq_walk_init( void );		// 歩行の初期化

#ifdef 		_SQ_WALK_C_
#undef 		extern
#endif 		/* _SQ_WALK_C_ */

#endif 		/* _SQ_WALK_H_ */
