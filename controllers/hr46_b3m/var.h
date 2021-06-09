/*----------------------------------------------------------*/
/*	global variables declaration							*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	var.h									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#ifndef 	_VAR_H_
#define 	_VAR_H_

/*--------------------------------------*/
/*	definitions							*/
/*--------------------------------------*/
#define 	RTC_TIME	(10)			/* real time clock [msec] 			*/
#define		RTC_TICK	(100)			/* 1000[msec]/10(RTC_TIME)[msec] 	*/
#define 	RTC_TIME_SEC	(0.010f)	/* real time clock [sec] 			*/

#define 	EPS_TIME	(0.001f)		/* epsilon time [sec]	 			*/
#define 	EPS_DATA	(0.001f)		/* epsilon				 			*/

#define 	SERV_NUM	(29)			/* number of servos	(never change)		*/
#define		ACCELITE_SERV_NUM	19

#define		PI		(3.14159f)
#define		PI2		(6.28318f)
#define		PIW2	(1.57080f)
/*	G	[mm/s^2]	*/
#define		GRAVITY	(9800.f)

/***	set mechanical data	***/
#define		WIDTH	( 88.0f)
#define		L01		(  0.0f)
#define		L1		(118.0f)
#define		L12		( 23.0f)
#define		L2		(118.0f)
#define		L23		(  0.0f)
#define		L3		( 43.0f)			/*	43+0(sole)mm	*/

#define		Z3_LIMIT_H	((float)(L01+L1+L12+L2+L23+L3))	// 腰の高さの最大値
#define		Z3_LIMIT_L	(120.f)			// 腰の高さの最小値

#define		REDUCTION_RATION	(2.0f)

/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
enum
{
	OFF 	= 	0,
	ON		=	1,
	ON2		=	2,
	ON3		=	3
};

enum {
	STOP 		= 	0,
	FWD 		= 	1,
	BWD 		= 	2
};

enum {
	STRAIGHT	=	0,
	RIGHT 		= 	1,
	LEFT		=	2
};

enum {
	UP 			= 	1,
	DOWN		=	2
};

enum {
	FOOT_XYZ		=	1,	// 足をデカルト座標系で制御
	JOINT_ANGLE		=	2	// 関節座標系で制御
};

// 歩行のテーブル
enum {
	MV_TBL_SIN		=	0,
	MV_TBL_LAMP 	= 	1,
	MV_TBL_ZMP		=	2,
	MV_TBL_Z_UP		=	3,
	MV_TBL_Z_DW		=	4,
	MV_TBL_ZMP2		=	5,
	MV_TBL_COS		=	6,
	MV_TBL_X_UP		=	7,
	MV_TBL_X_DW		=	8,
};

enum {
	FOOT_ROLL_R 	= 	0,
	LEG_PITCH_R 	= 	1,
	KNEE_R1		 	= 	2,
	KNEE_R2 		= 	3,
	LEG_ROLL_R	 	= 	4,
	LEG_YAW_R 		= 	5,
	ARM_PITCH_R		= 	6,
	ARM_ROLL_R		= 	7,
	ELBOW_PITCH_R	= 	8,
	FOOT_ROLL_L 	= 	9,
	LEG_PITCH_L 	= 	10,
	KNEE_L1 		= 	11,
	KNEE_L2 		= 	12,
	LEG_ROLL_L	 	= 	13,
	LEG_YAW_L 		= 	14,
	ARM_PITCH_L		= 	15,
	ARM_ROLL_L		= 	16,
	ELBOW_PITCH_L	= 	17,
	HEAD_YAW		= 	18,
	HEAD_PITCH		= 	19,
	SPARE12			= 	20,
	SPARE13			= 	21,
	SPARE17			= 	22,
	SPARE21			= 	23,
	SPARE24			= 	24,
	SPARE25			= 	25,
	SPARE26			= 	26,
	SPARE27			= 	27,
	SPARE28			= 	28
};


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern	volatile unsigned long 	count_time_l	;	// rtc time
extern	short 	flag_face_control				;	// 頭を動かしている時のフラグ

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern 	void	var_init( void );
extern 	void	mode_init( void );

#endif 		/* _VAR_H_ */
