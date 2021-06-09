/*----------------------------------------------------------*/
/*	servo motor RS405CB driver								*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	servo_rs.c								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2004.8.29								*/
/*----------------------------------------------------------*/
#ifndef _SERVO_RS_H_
#define _SERVO_RS_H_

#ifdef _SERVO_RS_C_
#define	extern
#endif /* _SERVO_RS_C_ */
#include	"var.h"
#include 	"b3m.h"


/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
/*	futaba RS servo	*/

/*	FA AF Flags Address Length Count Data Sum	*/
/*	Length * Count = Data size	*/

/*	Flags	*/
#define		SERVO_RS_INST_WRITE_DATA	(0x00)
#define		SERVO_RS_INST_READ_DATA		(0x0F)				/*	read from address	*/
#define		SERVO_RS_INST_FLASHWRITE_RESTART	(0x60)
#define		SERVO_RS_INST_FLASHWRITE	(0x40)
#define		SERVO_RS_INST_RESTART		(0x20)
/*	Count	*/
#define		SERVO_RS_CNT_WRITE			(0x01)				/*	number of servos to write	*/
#define		SERVO_RS_CNT_READ			(0x00)				/*	number of servos to write	*/
#define		SERVO_RS_CNT_FLASHWRITE		(0x00)				/*	number of servos to write	*/
/*	ID		*/
#define		SERVO_RS_ID_BROADCAST		(0x00)


/*	ROM Area	*/
#define		SERVO_RS_ADDR_MODEL_NO				(0x00)		/*	2bytes	*/
#define		SERVO_RS_ADDR_MODEL_NO_H			(0x01)
#define		SERVO_RS_ADDR_FIRMWARE_VERSION		(0x02)
															/*	reserved	*/
#define		SERVO_RS_ADDR_ID					(0x04)		/*	0x01		*/
#define		SERVO_RS_ADDR_REVERSE				(0x05)		/*	0x00		*/
#define		SERVO_RS_ADDR_BAUD_RATE				(0x06)		/*	0x07		*/
#define		SERVO_RS_ADDR_RETURN_DELAY_TIME		(0x07)		/*	0x00		*/
#define		SERVO_RS_ADDR_CW_ANGLE_LIMIT		(0x08)		/*	2bytes	0xDC	*/
#define		SERVO_RS_ADDR_CW_ANGLE_LIMIT_H		(0x09)		/*			0x05	*/
#define		SERVO_RS_ADDR_CCW_ANGLE_LIMIT		(0x0A)		/*	2bytes	0x24	*/
#define		SERVO_RS_ADDR_CCW_ANGLE_LIMIT_H		(0x0B)		/*			0xFA	*/
															/*	reserved	*/
															/*	reserved	*/
#define		SERVO_RS_ADDR_HIGHEST_LIMIT_TEMP	(0x0E)		/*	2bytes	0x69=105[deg]	*/
#define		SERVO_RS_ADDR_HIGHEST_LIMIT_TEMP_H	(0x0F)		/*			0x00	*/
															/*	reserved	*/
															/*	reserved	*/
															/*	reserved	*/
															/*	reserved	*/
#define		SERVO_RS_ADDR_DAMPER				(0x14)		/*	0x10		*/
															/*	reserved	*/
#define		SERVO_RS_ADDR_TORQUE_IN_SILENCE		(0x16)		/*	PWM only 0x00	*/
#define		SERVO_RS_ADDR_WARMUP_TIME			(0x17)		/*	PWM only 0xC8	*/
#define		SERVO_RS_ADDR_CW_COMPLIANCE_MARGIN	(0x18)		/*	0x01		*/
#define		SERVO_RS_ADDR_CCW_COMPLIANCE_MARGIN	(0x19)		/*	0x01		*/
#define		SERVO_RS_ADDR_CW_COMPLIANCE_SLOPE	(0x1A)		/*	0x04		*/
#define		SERVO_RS_ADDR_CCW_COMPLIANCE_SLOPE	(0x1B)		/*	0x04		*/
#define		SERVO_RS_ADDR_PUNCH					(0x1C)		/*	2bytes	0x14	*/
#define		SERVO_RS_ADDR_PUNCH_H				(0x1D)		/*			0x05	*/
/*	RAM Area	*/
#define		SERVO_RS_ADDR_GOAL_POSITION			(0x1E)		/*	2bytes	0x00	*/
#define		SERVO_RS_ADDR_GOAL_POSITION_H		(0x1F)		/*			0x00	*/
#define		SERVO_RS_ADDR_GOAL_TIME				(0x20)		/*	2bytes	0x00	*/
#define		SERVO_RS_ADDR_GOAL_TIME_H			(0x21)		/*			0x00	*/
															/*	reserved	*/
#define		SERVO_RS_ADDR_MAX_TORQUE			(0x23)		/*	0x64		*/
#define		SERVO_RS_ADDR_TORQUE_ENABLE			(0x24)		/*	0x00		*/
															/*	reserved	*/
															/*	reserved	*/
															/*	reserved	*/
															/*	reserved	*/
															/*	reserved	*/
#define		SERVO_RS_ADDR_PESENT_POSITION		(0x2A)		/*	2bytes	0x00	*/
#define		SERVO_RS_ADDR_PESENT_POSITION_H		(0x2B)		/*			0x00	*/
#define		SERVO_RS_ADDR_PESENT_TIME			(0x2C)		/*	2bytes	0x00	*/
#define		SERVO_RS_ADDR_PESENT_TIME_H			(0x2D)		/*			0x00	*/
#define		SERVO_RS_ADDR_PESENT_SPEED			(0x2E)		/*	2bytes	0x00	*/
#define		SERVO_RS_ADDR_PESENT_SPEED_H		(0x2F)      /*			0x00	*/
#define		SERVO_RS_ADDR_PESENT_CURRENT		(0x30)		/*	2bytes	0x00	*/
#define		SERVO_RS_ADDR_PESENT_CURRENT_H		(0x31)      /*			0x00	*/
#define		SERVO_RS_ADDR_PESENT_TEMP			(0x32)		/*	2bytes	0x00	*/
#define		SERVO_RS_ADDR_PESENT_TEMP_H			(0x33)      /*			0x00	*/
#define		SERVO_RS_ADDR_PESENT_VOLT			(0x34)		/*	2bytes	0x00	*/
#define		SERVO_RS_ADDR_PESENT_VOLT_H			(0x35)      /*			0x00	*/
															/*	reserved	*/
															/*	reserved	*/
															/*	reserved	*/
															/*	reserved	*/
															/*	reserved	*/
															/*	reserved	*/


#define		SERVO_RS_DATA_ON					(1)
#define		SERVO_RS_DATA_OFF					(0)
#define		SERVO_RS_DATA_BRAKE					(2)			/*	torque break mode	*/
#define		SERVO_RS_DATA_TORQUE_HIGH			(100)		/*	100[%]		*/
#define		SERVO_RS_DATA_TORQUE_LOW			(40)		/*	40[%]		*/
#define		SERVO_RS_DATA_GOAL_TIME_FAST		(2)			/*	0.02[sec]		*/
#define		SERVO_RS_DATA_GOAL_TIME_SLOW		(200)		/*	2[sec]		*/
#define		SERVO_RS_DATA_HIGHEST_LIMIT_TEMP	(105)		/*	105[deg]	*/
#define		SERVO_RS_DATA_RETURN_DELAY_TIME		(0x00)		/*	0x00		*/


#define		SERVO_RS_ID_NUM				(13)	/*	12+1	*/


typedef struct st_xp_servo_rs
{
	unsigned short	goal_time_slow[SERV_NUM];
	unsigned short  free_mode[SERV_NUM];
	unsigned short  normal_mode[SERV_NUM];
	unsigned short  control_mode[SERV_NUM];
	unsigned short  run_or_control[SERV_NUM];
	unsigned short  trajectory_normal[SERV_NUM];
	unsigned short  trajectory_even[SERV_NUM];
	unsigned short  gain_preset[SERV_NUM];
	unsigned short	deadband_width[SERV_NUM];
	unsigned short  control_kp0[SERV_NUM];
	unsigned short  control_kd0[SERV_NUM];
	unsigned short  control_ki0[SERV_NUM];
	unsigned short  control_static_friction0[SERV_NUM];
	unsigned short  control_dynamic_friction0[SERV_NUM];
	unsigned short  control_kp1[SERV_NUM];
	unsigned short  control_kd1[SERV_NUM];
	unsigned short  control_ki1[SERV_NUM];
	unsigned short  control_static_friction1[SERV_NUM];
	unsigned short  control_dynamic_friction1[SERV_NUM];
	unsigned short  control_kp2[SERV_NUM];
	unsigned short  control_kd2[SERV_NUM];
	unsigned short  control_ki2[SERV_NUM];
	unsigned short  control_static_friction2[SERV_NUM];
	unsigned short  control_dynamic_friction2[SERV_NUM];
}	tp_xp_servo_rs;

typedef struct st_xv_servo_rs
{
	unsigned short	present_temp[SERV_NUM];
	unsigned short	present_position[SERV_NUM];
	unsigned short	goal_position[SERV_NUM];
	unsigned short	present_current[SERV_NUM];
}	tp_xv_servo_rs;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern 	short			servo_rs_id_scif2[SERVO_RS_ID_NUM]		;
extern 	short			servo_rs_id_scif3[SERVO_RS_ID_NUM]		;


extern	unsigned char 	rs405_bpTxBuffer_scif2[2048];
extern	unsigned char 	rs405_bpTxBuffer_scif3[2048];
extern	unsigned char 	rs405_bpParameter[96];
extern	unsigned char 	rs405_bpRxBuffer_scif2[64];
extern	unsigned char 	rs405_bpRxBuffer_scif3[64];
extern	short			rs405_bCount;
extern	short			servo_rs_id;
//extern	short			servo_rs_address;
extern	short			servo_rs_len;
extern	short			servo_rs_sdata;
extern	short			servo_rs_rdata;
extern	short			flag_write_servo_rs_scif2;
extern	short			flag_write_servo_rs_scif3;
extern	short			flag_read_servo_rs_scif2;
extern	short			flag_read_servo_rs_scif3;
extern	short			flag_write_servo_rs_all;
extern	short			flag_read_servo_rs_all;
extern	short			flag_servo_rs_test;
extern 	short			flag_servo_on;
extern 	short			flag_servo_off;
extern 	short			flag_servo_output_wait;
extern	short			flag_read_servo_position;
extern 	float			servo_on_totaltime;
extern 	tp_xp_servo_rs	xp_servo_rs;
extern 	tp_xp_servo_rs	xp_servo_rs_soft;
extern	tp_xp_servo_rs	xp_servo_rs_motion;
extern	tp_xv_servo_rs	xv_servo_rs;
extern 	short			flag_ukemi;

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern	void	servo_rs_init( void );
extern	short  	rs405_TxPacket_scif2( unsigned char bID,
							unsigned char bInstruction,
							unsigned char bCount,
							unsigned char *bpParameter,
							unsigned char bParameterLength,
							unsigned char *bpTxBuffer,
							short bPacketPointer );

extern	short  	rs405_TxPacket_scif3( unsigned char bID,
							unsigned char bInstruction,
							unsigned char bCount,
							unsigned char *bpParameter,
							unsigned char bParameterLength,
							unsigned char *bpTxBuffer,
							short bPacketPointer );

extern	short  	rs405_TxPacket_buf( unsigned char bID,
							unsigned char bInstruction,
							unsigned char bCount,
							unsigned char *bpParameter,
							unsigned char bParameterLength,
							unsigned char *bpTxBuffer,
							short bPacketPointer );

extern	short	rs405_RxPacket_scif2_scif3( unsigned char *bpRxBuffer1, unsigned char *bpRxBuffer2, short Length );

extern	short	write_servo_rs_scif2( void );
extern	short	write_servo_rs_scif3( void );
extern	short	read_servo_rs_scif2( void );
extern	short	read_servo_rs_scif3( void );
extern	void 	write_servo_rs_all( unsigned char, unsigned short *, short );
extern	void 	read_servo_rs_all( unsigned char, unsigned short *, short );
extern	void	servo_rs_fun( void );
extern 	void 	servopos_to_jointdeg( void );
extern 	void 	servotemp_to_temp( void );
extern	void 	servocurrent_to_current( void );	//2010.5.8
extern 	short	flashwrite_servo_rs_scif2( void );
extern 	short	flashwrite_servo_rs_scif3( void );

#ifdef _SERVO_RS_C_
#undef	extern
#endif /* _SERVO_RS_C_ */

#endif /* _SERVO_RS_H_ */
