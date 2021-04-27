/*----------------------------------------------------------*/
/*	normalized trajectory tables							*/
/*															*/
/*															*/
/*	file name	:	mvtbl.c									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.6.3								*/
/*----------------------------------------------------------*/
#define		_MVTBL_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	<stdio.h>
#include	<math.h>
#include	"var.h"
#include	"mvtbl.h"
#include	"sq_walk.h"

/*--------------------------------------*/
/*	zmp_fun								*/
/*--------------------------------------*/
/*	calculate Yzmp (0 to peak)			*/
/* 	time = walk period / 2				*/
/*	2 * r = distance between two legs	*/
float 	zmp_fun( float time, float r )
{
	float	c1;
	float	w;

	w	=	sqrtf( GRAVITY / xp_mv_walk.h_cog ) * time / 2.f;
	c1	=	-r / ( expf( w ) + expf( -w ) );

	return		2.f * c1 + r;
}
