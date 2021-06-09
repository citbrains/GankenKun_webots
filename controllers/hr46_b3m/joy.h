/*----------------------------------------------------------*/
/*	command interface										*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	joy.h									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.8.22								*/
/*----------------------------------------------------------*/
#ifndef 	_JOY_H_
#define 	_JOY_H_

/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
typedef struct st_xv_comm
{
	unsigned char	cmd;
	unsigned char	para1;
	unsigned char	para2;
	unsigned char	para3;
	unsigned char	para4;
	unsigned char	para5;
}	tp_xv_comm;

typedef struct st_xv_comm_bin
{
	short	cmd;
	short	para1;
	short	para2;
	short	para3;
	short	para4;
	short	para5;
}	tp_xv_comm_bin;

typedef struct st_joy_status
{
	unsigned char	cmd;
	unsigned char	para1;
	unsigned char	para2;
	unsigned char	para3;
	unsigned char	para4;
	unsigned char	para5;
}	tp_joy_status;


typedef struct st_xv_joy
{
	short		walk_num;					/*	[step]		*/
	float		walk_time;					/*	[sec]		*/
	float		walk_x_percent;				/*	-1<x<1 [1]	*/
	float		walk_y_percent;				/*	-1<x<1 [1]	*/
	float		walk_theta_percent;			/*	-1<x<1 [1]	*/
	float		walk_zmp;					/*	[mm]		*/
	float		walk_time_dutyfactor;		/*	0<x<1 [-]	*/
 	float		walk_step_len_offset;		/*	[mm]		*/
}	tp_xv_joy;

// コマンドの設定
void set_xv_comm(tp_xv_comm *xv,
				 unsigned char	cmd,
				 unsigned char	para1,
				 unsigned char	para2,
				 unsigned char	para3,
				 unsigned char	para4,
				 unsigned char	para5
				 );

// 送信用のコマンドに変更
void convert_bin(tp_xv_comm_bin *xv_bin, tp_xv_comm *xv);

/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/

extern 	unsigned short	count_joy;
extern 	tp_xv_comm		xv_comm;
extern 	tp_xv_comm_bin	xv_comm_bin;
extern 	tp_joy_status	joy_status;
extern 	tp_xv_joy		xv_joy;
extern 	char			xv_comm_response[];
extern  int				is_walk_change;			// 歩行を変更したかどうかのフラグ

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern void 	joy_init( void );
extern void 	joy( void );
extern short	ascii2bin( unsigned char );
extern void 	copy_joy_parameter( void );
extern void	    joy_read();

#define		EOF_CODE		('\0')
#define		EOF_CODE2		('#')
#define		EOF_CODE3		(0x0a)

#define		RFMT_SIZE		(256)
#define		SFMT_SIZE		(256)
extern char		rfmt[RFMT_SIZE];
extern char		sfmt[SFMT_SIZE];

#endif 		/* _JOY_H_ */
