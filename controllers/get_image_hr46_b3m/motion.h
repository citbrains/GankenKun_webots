/*----------------------------------------------------------*/
/*	motion sequence											*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	motion.h								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#ifndef 	_MOTION_H_
#define 	_MOTION_H_

#ifdef 		_MOTION_C_
#define 	extern 
#endif 		/* _MOTION_C_ */

/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
// 状態
enum {
	MOTION_NONE,		// 何もしない状態
	MOTION_START,		// サーボオン
	MOTION_STRAIGHT,	// 直立姿勢
	MOTION_READY,		// レディ姿勢
	MOTION_WALK,		// 歩行
	MOTION_MOTION,		// モーション再生
};

enum{
	STATE_STOP,
	STATE_WALKING,
	STATE_MOTION,	
	STATE_MOVING		// stand, ready
};

/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern short			mode_motion;
extern short			flag_moving;		// 0:STOP,1:WALKING,2:MOTION,3:MOVING?

// シーケンスのフラグ（上から順番に実行される）
extern struct sq_flag_T{
	int start	;	// サーボONのシーケンス
	int straight;	// 直立のシーケンス
	int ready	;	// レディ状態へのシーケンス
	int walk	;	// 歩行のシーケンス
	int motion	;	// モーションのシーケンス
} sq_flag;

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
void motion();
void reset_flag();

#ifdef 		_MOTION_C_
#undef 		extern
#endif 		/* _MOTION_C_ */

#endif 		/* _MOTION_H_ */
