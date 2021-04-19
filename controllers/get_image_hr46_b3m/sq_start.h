/*----------------------------------------------------------*/
/*	start sequence											*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	sq_start.h								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2004.11.1								*/
/*----------------------------------------------------------*/
#ifndef 	_SQ_START_H_
#define 	_SQ_START_H_

/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
// extern short		flag_sq_start;			// サーボONのシーケンスを行なっているかのフラグ

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
void sq_start_init(); 
int sq_start();
int servo_period;

#endif 		/* _SQ_START_H_ */
