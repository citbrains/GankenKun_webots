/*----------------------------------------------------------*/
/*	motion play - special action							*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	sq_motion.h								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.8.19								*/
/*----------------------------------------------------------*/
#ifndef 	_SQ_MODE2_H_
#define 	_SQ_MODE2_H_

#ifdef 		_SQ_MODE2_C_
#define 	extern 
#endif 		/* _SQ_MODE2_C_ */


/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
#define			MODE2_MOTION_SIZE		(16)
#define			MODE2_MOTION_NUM		(101)
///#define			MODE2_MOTION_NUM		(10)		//	DEBUG
#define			MODE2_LAMP1				(100)
#define			MODE2_LAMP2				(101)
#define			MODE2_HOLD				(0x0B)			/*	old 251	*/
#define			MODE2_CONTINUE			(0x0F)			/*	old 255	*/
#define			MODE2_CONTINUE_SW		(0x0E)			/*	old 254	*/
#define			MODE2_GYRO1				(0x10)
#define			MODE2_GYRO2				(0x20)

enum {
	SQ_MODE2_INIT 		= 	0,
	SQ_MODE2_INIT_2		= 	1,
	SQ_MODE2_1A			= 	2,
	SQ_MODE2_1B			= 	3,
	SQ_MODE2_1C			= 	4,
	SQ_MODE2_1D			= 	5,
	SQ_MODE2_WAIT		= 	6,
	SQ_MODE2_END		= 	7
};

enum {
	MODE2_TEST			=	0,
	MODE2_1				=	1,
	MODE2_2				=	2,
	MODE2_3				=	3,
	MODE2_4				=	4,
	MODE2_5				=	5,
	MODE2_6				=	6,
	MODE2_7				=	7,
	MODE2_8				=	8,
	MODE2_9				=	9,
	MODE2_10			=	10,
	MODE2_11			=	11,
	MODE2_12			=	12,
	MODE2_13			=	13,
	MODE2_14			=	14,
	MODE2_15			=	15,
	MODE2_16			=	16,
	MODE2_17			=	17,
	MODE2_18			=	18,
	MODE2_19			=	19,
	MODE2_20			=	20,
	MODE2_21			=	21,
	MODE2_22			=	22,
	MODE2_23			=	23,
	MODE2_24			=	24,
	MODE2_25			=	25,
	MODE2_26			=	26,
	MODE2_27			=	27,
	MODE2_28			=	28,
	MODE2_29			=	29,
	MODE2_30			=	30,
	MODE2_31			=	31,
	MODE2_32			=	32,
	MODE2_33			=	33,
	MODE2_34			=	34,
	MODE2_35			=	35,
	MODE2_36			=	36,
	MODE2_37			=	37,
	MODE2_38			=	38,
	MODE2_39			=	39,
	MODE2_40			=	40,
	MODE2_41			=	41,
	MODE2_42			=	42,
	MODE2_43			=	43,
	MODE2_44			=	44,
	MODE2_45			=	45,
	MODE2_46			=	46,
	MODE2_47			=	47,
	MODE2_48			=	48,
	MODE2_49			=	49,
	MODE2_50			=	50,
	MODE2_51			=	51,
	MODE2_52			=	52,
	MODE2_53			=	53,
	MODE2_54			=	54,
	MODE2_55			=	55,
	MODE2_56			=	56,
	MODE2_57			=	57,
	MODE2_58			=	58,
	MODE2_59			=	59,
	MODE2_60			=	60,
	MODE2_61			=	61,
	MODE2_62			=	62,
	MODE2_63			=	63,
	MODE2_64			=	64,
	MODE2_65			=	65,
	MODE2_66			=	66,
	MODE2_67			=	67,
	MODE2_68			=	68,
	MODE2_69			=	69,
	MODE2_70			=	70,
	MODE2_71			=	71,
	MODE2_72			=	72,
	MODE2_73			=	73,
	MODE2_74			=	74,
	MODE2_75			=	75,
	MODE2_76			=	76,
	MODE2_77			=	77,
	MODE2_78			=	78,
	MODE2_79			=	79,
	MODE2_80			=	80,
	MODE2_81			=	81,
	MODE2_82			=	82,
	MODE2_83			=	83,
	MODE2_84			=	84,
	MODE2_85			=	85,
	MODE2_86			=	86,
	MODE2_87			=	87,
	MODE2_88			=	88,
	MODE2_89			=	89,
	MODE2_90			=	90,
	MODE2_91			=	91,
	MODE2_92			=	92,
	MODE2_93			=	93,
	MODE2_94			=	94,
	MODE2_95			=	95,
	MODE2_96			=	96,
	MODE2_97			=	97,
	MODE2_98			=	98,
	MODE2_99			=	99,
	MODE2_100			=	100
};


typedef struct st_xp_mv_motion
{
	short	time;				/* [0.01sec]		*/
	short	d[SERV_NUM];		/* [deg]			*/
	short	cntr[3];			/* [-]				*/
}	tp_xp_mv_motion;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern tp_xp_mv_motion	xp_mv_motionbuf[MODE2_MOTION_NUM][MODE2_MOTION_SIZE];
extern tp_xp_mv_motion  xp_mv_motionbuf_var[MODE2_MOTION_NUM][MODE2_MOTION_SIZE];
extern tp_xp_mv_motion	xp_mv_motion_last[MODE2_MOTION_SIZE];
extern tp_xp_mv_motion  xp_mv_motion_last_var[MODE2_MOTION_SIZE];
//extern short		flag_md_motion_end;
extern short		mode_sq_motion;
extern short        flag_variable_motion;
extern short        variable_amount;
extern short		flag_motion_select;
extern short		flag_md_motion_hold;
extern short		flag_sq_motion_cancel;
extern short		flag_motion_repeat;
extern short		sq_motion_mtn;
extern short		count_motion_repeat;
extern short		xv_motion_repeat_num;
extern short		flag_motion_preload;
extern short		flag_motion_preload_end;
extern short		xv_motion_n_last;
extern short		flag_motion_gyro;

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern int sq_motion();
extern void sq_motion_init();

#ifdef 		_SQ_MODE2_C_
#undef 		extern
#endif 		/* _SQ_MODE2_C_ */

#endif 		/* _SQ_MODE2_H_ */
