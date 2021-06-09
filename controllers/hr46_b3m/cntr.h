/*----------------------------------------------------------*/
/*	control													*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	cntr.h									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#ifndef 	_CNTR_H_
#define 	_CNTR_H_

#ifdef 		_CNTR_C_
#define 	extern 
#endif 		/* _CNTR_C_ */

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern void 	cntr( void );

#ifdef 		_CNTR_C_
#undef 		extern
#endif 		/* _CNTR_C_ */

#endif 		/* _CNTR_H_ */
