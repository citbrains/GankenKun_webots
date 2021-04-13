/*----------------------------------------------------------*/
/*	servo motor data initilaize								*/
/*															*/
/*															*/
/*	file name	:	serv.c									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.7								*/
/*----------------------------------------------------------*/
#define		_SERV_INIT_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	"var.h"
#include	"serv_init.h"
#include	"serv.h"


/*--------------------------------------*/
/*	serv_init							*/
/*--------------------------------------*/
void 	serv_init( void )
{
	short	i;

	for( i=0; i<SERV_NUM; i++ )
	{
		xp_sv[i].deg_lim_h		=	180;
		xp_sv[i].deg_lim_l		=	-180;

		/***	RS405CB	***/
//		-150deg=0xFA24(-1500),0deg=0x0000(0),150deg=0x05DC(1500) => 10.00 bit/deg
		xp_sv[i].deg2pls		=	1000;
		xp_sv[i].deg_lim_offset	=	0;

		xp_sv[i].deg_sign		=	1;
		xp_sv[i].deg_offset		=	0;

		xv_sv[i].d				=	0;
		xv_sv[i].deg_sw_out		=	0;
		xv_sv[i].deg_sv			=	0;
		xv_sv[i].deg_lim		=	0;
		xv_sv[i].pls			=	0;
		xv_sv[i].pls_out		=	0;		/*	analogue servo	*/

		xv_ref.d[i]				=	0.f;
		xv_ref.d_ref[i]			=	0.f;
	}

	xp_sv[ 0].deg_sign		=	-1;				//	left	foot (roll)
	xp_sv[ 0].deg_offset	=	0;
	xp_sv[ 1].deg_sign		=	1;				//	left	foot (pitch)
	xp_sv[ 1].deg_offset	=	0;
	xp_sv[ 2].deg_sign		=	1;				//	left	knee
	xp_sv[ 2].deg_offset	=	0;
	xp_sv[ 3].deg_sign		=	1;				//	left	leg
	xp_sv[ 3].deg_offset	=	0;
	xp_sv[ 4].deg_sign		=	1;				//	left 	hip
	xp_sv[ 4].deg_offset	=	0;
	xp_sv[ 5].deg_sign		=	-1;				//	left	waist
	xp_sv[ 5].deg_offset	=	0;
	xp_sv[ 6].deg_sign		=	1;				//	right	foot (roll)
	xp_sv[ 6].deg_offset	=	0;
	xp_sv[ 7].deg_sign		=	-1;				//	right	foot (pitch)
	xp_sv[ 7].deg_offset	=	0;
	xp_sv[ 8].deg_sign		=	-1;				//	right	knee
	xp_sv[ 8].deg_offset	=	0;

	xp_sv[ 9].deg_sign		=	-1;				//	right	leg
	xp_sv[ 9].deg_offset	=	0;
	xp_sv[10].deg_sign		=	-1;				//	right 	hip
	xp_sv[10].deg_offset	=	0;
	xp_sv[11].deg_sign		=	-1;				//	right	waist
	xp_sv[11].deg_offset	=	0;
	xp_sv[12].deg_sign		=	-1;				//	body (pitch)
	xp_sv[12].deg_offset	=	0;
	xp_sv[13].deg_sign		=	1;				//	body (yaw)
	xp_sv[13].deg_offset	=	0;
	xp_sv[14].deg_sign		=	-1;				//	left	shoulder (pitch)
	xp_sv[14].deg_offset	=	0;
	xp_sv[15].deg_sign		=	-1;				//	left	shoulder (roll)
	xp_sv[15].deg_offset	=	0;
	xp_sv[16].deg_sign		=	-1;				//	left	elbow (yaw)
	xp_sv[16].deg_offset	=	0;
	xp_sv[17].deg_sign		=	1;				//	left	elbow (pitch)
	xp_sv[17].deg_offset	=	0;

	xp_sv[18].deg_sign		=	1;				//	right	shoulder (pitch)
	xp_sv[18].deg_offset	=	0;
	xp_sv[19].deg_sign		=	1;				//	right	shoulder (roll)
	xp_sv[19].deg_offset	=	0;
	xp_sv[20].deg_sign		=	1;				//	right	elbow (yaw)
	xp_sv[20].deg_offset	=	0;
	xp_sv[21].deg_sign		=	1;				//	right	elbow (pitch)
	xp_sv[21].deg_offset	=	0;

	xp_sv[22].deg_sign		=	1;				//	head	(yaw)(pan)
	xp_sv[22].deg_offset	=	0;
	xp_sv[23].deg_sign		=	1;				//	head	(pitch)(tilt)
	xp_sv[23].deg_offset	=	0;
	xp_sv[24].deg_sign		=	1;				//
	xp_sv[24].deg_offset	=	0;
	xp_sv[25].deg_sign		=	1;				//
	xp_sv[25].deg_offset	=	0;
	xp_sv[26].deg_sign		=	1;				//
	xp_sv[26].deg_offset	=	0;
	xp_sv[27].deg_sign		=	1;				//
	xp_sv[27].deg_offset	=	0;
	xp_sv[28].deg_sign		=	1;				//
	xp_sv[28].deg_offset	=	0;
}
