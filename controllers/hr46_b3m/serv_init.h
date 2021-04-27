/*----------------------------------------------------------*/
/*	servo motor data initilaize								*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	serv_init.h								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.6.3								*/
/*----------------------------------------------------------*/
#ifndef 	_SERV_INIT_H_
#define 	_SERV_INIT_H_

#ifdef 		_SERV_INIT_C_
#define 	extern 
#endif 		/* _SERV_INIT_C_ */

extern void 	serv_init( void );


#ifdef 		_SERV_INIT_C_
#undef 		extern
#endif 		/* _SERV_INIT_C_ */

#endif 		/* _SERV_INIT_H_ */
