/*----------------------------------------------------------*/
/*	servo motor RS405CB driver								*/
/*															*/
/*															*/
/*	file name	:	servo_rs.c								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2004.8.29								*/
/*----------------------------------------------------------*/
#define _SERVO_RS_C_

#include "var.h"
#include "func.h"
#include "servo_rs.h"
#include "serv.h"
#include "b3m.h"
#include "sq_motion.h"

/*	SCIF=460800bps	*/
#define		SCIF_1BYTE		(21)		/*	[us]	*/
#define		SCIF_1P5BYTE	(33)		/*	[us]	*/
#define		SCIF_2BYTE		(43)		/*	[us]	*/

extern int	servo_offset[SERV_NUM];

/*--------------------------------------*/
/*	rs405_TxPacket_scif2				*/
/*--------------------------------------*/
short  	rs405_TxPacket_scif2( unsigned char bID,
							unsigned char bInstruction,
							unsigned char bCount,
							unsigned char *bpParameter,
							unsigned char bParameterLength,
							unsigned char *bpTxBuffer,
							short bPacketPointer )
{
	short	Length;

  	Length	=	rs405_TxPacket_buf( bID, bInstruction, bCount, bpParameter, bParameterLength, bpTxBuffer, bPacketPointer );
	return	Length;
}


/*--------------------------------------*/
/*	rs405_TxPacket_scif3				*/
/*--------------------------------------*/
short  	rs405_TxPacket_scif3( unsigned char bID,
							unsigned char bInstruction,
							unsigned char bCount,
							unsigned char *bpParameter,
							unsigned char bParameterLength,
							unsigned char *bpTxBuffer,
							short bPacketPointer )
{
	short	Length;

  	Length	=	rs405_TxPacket_buf( bID, bInstruction, bCount, bpParameter, bParameterLength, bpTxBuffer, bPacketPointer );
	return	Length;
}


/*--------------------------------------*/
/*	rs405_TxPacket_buf					*/
/*--------------------------------------*/
short  	rs405_TxPacket_buf( unsigned char bID,
							unsigned char bInstruction,
							unsigned char bCount,
							unsigned char *bpParameter,
							unsigned char bParameterLength,
							unsigned char *bpTxBuffer,
							short bPacketPointer )
{
	short		i;
	unsigned char	bChecksum, bPacketLength;

//	futaba RS servo
	*(bpTxBuffer+bPacketPointer+0)	=	0xFA;
	*(bpTxBuffer+bPacketPointer+1)	=	0xAF;
	*(bpTxBuffer+bPacketPointer+2)	=	bID;
	*(bpTxBuffer+bPacketPointer+3)	=	bInstruction;		/*	flag	*/
	*(bpTxBuffer+bPacketPointer+4)	=	bpParameter[0];		/*	address	*/
	*(bpTxBuffer+bPacketPointer+5)	=	bParameterLength;	//	Length=Data
	*(bpTxBuffer+bPacketPointer+6)	=	bCount;				/*	Count	*/

	for( i=1 ; i <= bParameterLength * bCount; i++ )
	{
		*(bpTxBuffer+bPacketPointer+6+i)	=	bpParameter[i];
	}

	bChecksum	=	0;
	bPacketLength	=	bParameterLength * bCount + 8;
	for( i=2 ; i < bPacketLength - 1; i++ )	//	except 0xFA,0xAF,Checksum
	{
		bChecksum	^=	*(bpTxBuffer+bPacketPointer+i);		/*	xor		*/
	}
	*(bpTxBuffer+bPacketPointer+i)	=	(bChecksum & 0xFF);

	return	bPacketLength;
}

/*--------------------------------------*/
/*	write_servo_rs_all					*/
/*--------------------------------------*/
void	write_servo_rs_all( unsigned char addr, unsigned short *pdata, short len )
{
	int ret;

	ret = Write_Servo_B3M_All_2Kport(addr, pdata, len );
}

/*--------------------------------------*/
/*	read_servo_rs_all					*/
/*--------------------------------------*/
void	read_servo_rs_all( unsigned char addr, unsigned short *pdata, short len )
{
}

/*--------------------------------------*/
/*	servo_rs_fun						*/
/*--------------------------------------*/
void	servo_rs_fun( void )
{
	if( flag_servo_off )
	{
		flag_servo_off	=	OFF;
		flag_servo_on	=	OFF;

		/***	write_servo_off	***/
		write_servo_rs_all( B3M_SERVO_SERVO_MODE, &xp_servo_rs.free_mode[0], 1 );
	}
	else if( flag_servo_output_wait )
	{
		flag_servo_output_wait	=	0;		/*	wait	*/
	}
	else if( flag_servo_on )
	{
		/***	write_servo_position	***/
		int i;
		unsigned short position[SERV_NUM];
		for(i = 0; i < SERV_NUM; i ++){
			position[i] = xv_servo_rs.goal_position[i] + servo_offset[i];
		}

		if ( flag_motion_select == 3 || flag_motion_select == 4 ) { // Increase gain at stand up motions
			if ( mode_sq_motion == SQ_MODE2_1A ) {
				xp_servo_rs.gain_preset[ARM_PITCH_R]	=   B3M_CONTROL_GAIN_PRESET_DEF;
				xp_servo_rs.gain_preset[ARM_PITCH_L]	=   B3M_CONTROL_GAIN_PRESET_DEF;
				xp_servo_rs.gain_preset[ARM_ROLL_R]	        =   B3M_CONTROL_GAIN_PRESET_DEF;
				xp_servo_rs.gain_preset[ARM_ROLL_L]	        =   B3M_CONTROL_GAIN_PRESET_DEF;

				write_servo_rs_all( B3M_CONTROL_GAIN_PRESETNO, &xp_servo_rs.gain_preset[0], 1 );
			}
			else if ( mode_sq_motion == SQ_MODE2_END ) {
				xp_servo_rs.gain_preset[ARM_PITCH_R]	=   B3M_CONTROL_GAIN_PRESET_LOW;
				xp_servo_rs.gain_preset[ARM_PITCH_L]	=   B3M_CONTROL_GAIN_PRESET_LOW;
				xp_servo_rs.gain_preset[ARM_ROLL_R]	        =   B3M_CONTROL_GAIN_PRESET_LOW;
				xp_servo_rs.gain_preset[ARM_ROLL_L]	        =   B3M_CONTROL_GAIN_PRESET_LOW;

				write_servo_rs_all( B3M_CONTROL_GAIN_PRESETNO, &xp_servo_rs.gain_preset[0], 1 );
			}
			else {
				write_servo_rs_all( B3M_SERVO_DESIRED_POSITION, &position[0], 2 );
			}
		}
		else if ( flag_motion_select == 10 || flag_motion_select == 11 || flag_motion_select == 50 ) {
			if ( mode_sq_motion == SQ_MODE2_1A ) {
				xp_servo_rs.gain_preset[ARM_PITCH_R]	=   B3M_CONTROL_GAIN_PRESET_DEF;
				xp_servo_rs.gain_preset[ARM_PITCH_L]	=   B3M_CONTROL_GAIN_PRESET_DEF;

				write_servo_rs_all( B3M_CONTROL_GAIN_PRESETNO, &xp_servo_rs.gain_preset[0], 1 );
			}
			else if ( mode_sq_motion == SQ_MODE2_END ) {
				xp_servo_rs.gain_preset[ARM_PITCH_R]	=   B3M_CONTROL_GAIN_PRESET_LOW;
				xp_servo_rs.gain_preset[ARM_PITCH_L]	=   B3M_CONTROL_GAIN_PRESET_LOW;

				write_servo_rs_all( B3M_CONTROL_GAIN_PRESETNO, &xp_servo_rs.gain_preset[0], 1 );
			}
			else {
				write_servo_rs_all( B3M_SERVO_DESIRED_POSITION, &position[0], 2 );
			}
		}
		else {
			write_servo_rs_all( B3M_SERVO_DESIRED_POSITION, &position[0], 2 );
		}
	}

	/*	measure servo on total time for protect LI-Po battery discharge		*/
	if( flag_servo_on )
	{
		servo_on_totaltime += RTC_TIME_SEC;
	}
}


/*--------------------------------------*/
/*	servopos_to_jointdeg				*/
/*--------------------------------------*/
void 	servopos_to_jointdeg( void )
{
	short	i, j;
	short	_id_scif2, _id_scif3;

	for( i = 0; i < SERVO_RS_ID_NUM; i++ )
	{
		_id_scif2	=	servo_rs_id_scif2[i];				/*	ID		*/
		_id_scif3	=	servo_rs_id_scif3[i];				/*	ID		*/

		if( _id_scif2 > 0 )
		{
			j	=	_id_scif2 - 1;
			xv_pv.deg[j]	=	(((float)(*(short *)&xv_servo_rs.present_position[j]) * 100.f / (float)xp_sv[j].deg2pls) - (float)xp_sv[j].deg_offset - (float)xp_sv[j].deg_lim_offset )
								 * (float)xp_sv[j].deg_sign;
		}
		if( _id_scif3 > 0 )
		{
			j	=	_id_scif3 - 1;
			xv_pv.deg[j]	=	(((float)(*(short *)&xv_servo_rs.present_position[j]) * 100.f / (float)xp_sv[j].deg2pls) - (float)xp_sv[j].deg_offset - (float)xp_sv[j].deg_lim_offset )
								 * (float)xp_sv[j].deg_sign;
		}
	}
}


/*--------------------------------------*/
/*	servotemp_to_temp					*/
/*--------------------------------------*/
void 	servotemp_to_temp( void )
{
	short	i, j;
	short	_id_scif2, _id_scif3;

	for( i = 0; i < SERVO_RS_ID_NUM; i++ )
	{
		_id_scif2	=	servo_rs_id_scif2[i];				/*	ID		*/
		_id_scif3	=	servo_rs_id_scif3[i];				/*	ID		*/

		if( _id_scif2 > 0 )
		{
			j	=	_id_scif2 - 1;
			xv_pv.temp[j]	=	xv_servo_rs.present_temp[j];
		}
		if( _id_scif3 > 0 )
		{
			j	=	_id_scif3 - 1;
			xv_pv.temp[j]	=	xv_servo_rs.present_temp[j];
		}
	}
}

//	begin 2010.5.8
/*--------------------------------------*/
/*	servocurrent_to_current				*/
/*--------------------------------------*/
void 	servocurrent_to_current( void )
{
	short	i, j;
	short	_id_scif2, _id_scif3;
	for( i = 0; i < SERVO_RS_ID_NUM; i++ ){
		_id_scif2	=	servo_rs_id_scif2[i];				/*	ID		*/
		_id_scif3	=	servo_rs_id_scif3[i];				/*	ID		*/

		if( _id_scif2 > 0 ){
			j	=	_id_scif2 - 1;
			xv_pv.current[j]	=	xv_servo_rs.present_current[j];
		}
		if( _id_scif3 > 0 ){
			j	=	_id_scif3 - 1;
			xv_pv.current[j]	=	xv_servo_rs.present_current[j];
		}
	}
}
//	end 2010.5.8

