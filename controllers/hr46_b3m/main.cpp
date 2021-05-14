/*----------------------------------------------------------*/
/*	HAJIME ROBOT 42	for win									*/
/*	main program											*/
/*															*/
/*	file name	:	main.c									*/
/*	target		:	Renesas SH2A/7211						*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2007.7.15								*/
/*	note		:	editor tab = 4							*/
/*															*/
/*	memo		:	����C�y����Windows����̕��s�p�^�[����	*/
/*					�����̂��߂ɕύX��						*/
/*	date		:	2011.12.29								*/
/*					2012.01.14	�������Ő��䂷�邽�߂ɒǉ�*/
/*----------------------------------------------------------*/

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/

#include <stdio.h>
#include <assert.h>
#include <stdio.h>
//#include <math.h>
//#include <KSerialPort.h>

#include <boost/thread.hpp>
#include <string>
#include <HCIPC.h>

#define WEBOTS_GANKEN_SIMULATOR

#ifdef WEBOTS_GANKEN_SIMULATOR
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
#include <cmath>
#include <set>
#include <exception>
#include <cstdint>
#include <tuple>
#include "picture.pb.h"
#include <boost/interprocess/ipc/message_queue.hpp>
#include "angles.pb.h"
#endif

#include "pc_motion.h"
//#include "ADIS16375.h"
#include "OrientationEstimator.h"
//#include <RTIMULib.h>
//#include <RTIMUSettings.h>

extern "C"
{
#include "var.h"
#include "func.h"
#include "servo_rs.h"
#include "sq_walk.h"
#include "cntr.h"
#include "kine.h"
#include "serv.h"
#include "serv_init.h"
#include "calc_mv.h"
#include "mvtbl.h"
#include "sq_motion.h"
#include "sq_start.h"
#include "sq_straight.h"
#include "sq_ready.h"
#include "sq_walk.h"
#include "motion.h"
#include "joy.h"

#include "acc.h"
#include "gyro.h"
#include "b3m.h"
}

#pragma comment(lib, "winmm.lib")

constexpr int32_t FRAME_RATE = 10;

using namespace std;
using namespace boost;

static boost::mutex lock_obj;
static string cmd;
static bool response_ready = false;
static string res;

char ParamTableAlte[53] =
	{
		'z', 'y', 'x', 'w', 'v', 'u', 't', 's', 'r', 'q', 'p', 'o', 'n', 'm', 'l', 'k', 'j', 'i', 'h', 'g', 'f', 'e', 'd', 'c', 'b', 'a', // -26 - -1
		'0',																															  // 0
		'1', '2', '3', '4', '5', '6', '7', '8', '9', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'  // 1 - 26
};

extern "C"

	int
	scif1_tx_fun()
{
	boost::mutex::scoped_lock look(lock_obj);

	int len = 0;
	for (int i = 0; i < SFMT_SIZE - 1 && sfmt[i] != EOF_CODE && sfmt[i] != EOF_CODE3; i++)
	{
		len++;
	}
	res = string(sfmt, len);
	memset(sfmt, EOF_CODE, SFMT_SIZE);
	response_ready = true;

	return 1;
}

string recvHajimeCommand(const string &str, void *context)
{
	static const int CMD_TIMEOUT_MS = 100;
	int cnt = 0;
	if (str.size() > 0)
	{
		boost::mutex::scoped_lock look(lock_obj);
		cmd = str;
		response_ready = false;
	}
	while (!response_ready && cnt++ < CMD_TIMEOUT_MS)
	{
#ifdef WEBOTS_GANKEN_SIMULATOR
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
#else
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
#endif
	}
	{
		if (cnt >= CMD_TIMEOUT_MS)
		{
			std::cerr << "recvHajimeCommand: timeout" << std::endl;
		}
		boost::mutex::scoped_lock look(lock_obj);
		return res;
	}
}

void ipcthread(int argc, char *argv[], int id)
{
	HCIPCServer *hcipc = HCIPCServer::createServer(argc, argv, 0, HCIPC_DEFAULT_PORT + id);
	hcipc->setCallback(recvHajimeCommand, NULL);
	hcipc->wait();
}

extern "C" int servo_offset[SERV_NUM]; // �I�t�Z�b�g�ۑ��p

//========================
// �I�t�Z�b�g���́i���@����̃f�[�^�j
//========================
int offset_load(char *filename, int servo_offset[SERV_NUM])
{
	char off_tmp[100];
	int i, ser_ID, off_angle;
	FILE *fp;

	for (i = 0; i < SERV_NUM; i++)
	{
		servo_offset[i] = 0;
	}

	if (NULL == (fp = fopen(filename, "r")))
	{
		fprintf(stderr, "cannot open (offset_load): %s\n", filename);
		return -1;
	}
	while (fscanf(fp, "%d %d %s", &ser_ID, &off_angle, off_tmp) != EOF)
	{
		servo_offset[ser_ID] = off_angle;
	}
	fclose(fp);

	return 0;
}

//========================
// eeprom load
//========================
int eeprom_load(char *filename)
{
	char buff_tmp[100];
	int buff_num;
	float buff_val, eeprom_buff[111];
	FILE *fp;

	if (NULL == (fp = fopen(filename, "r")))
	{
		fprintf(stderr, "cannot open (eeprom_load): %s\n", filename);
		return -1;
	}
	while (fscanf(fp, "%d %f %s", &buff_num, &buff_val, buff_tmp) != EOF)
	{
		eeprom_buff[buff_num] = buff_val;
	}
	fclose(fp);

	//flag_eeprom_para_ok			=	eeprom_buff[ 0];
	//sw_soft1						=	eeprom_buff[ 1];
	//sw_soft2						=	eeprom_buff[ 2];
	//sw_soft3						=	eeprom_buff[ 3];
	//sw_soft4						=	eeprom_buff[ 4];
	xp_acc.acc_k1 = eeprom_buff[5];
	xp_acc.acc_k2 = eeprom_buff[6];
	xp_acc.acc_k3 = eeprom_buff[7];
	xp_acc.ad_volt_offset1 = eeprom_buff[8];
	xp_acc.ad_volt_offset2 = eeprom_buff[9];
	xp_acc.ad_volt_offset3 = eeprom_buff[10];
	xp_acc.t1 = eeprom_buff[11];
	xp_acc.t2 = eeprom_buff[12];
	xp_acc.fall_fwd = eeprom_buff[13];
	xp_acc.fall_bwd = eeprom_buff[14];
	xp_acc.fall_right = eeprom_buff[15];
	xp_acc.fall_left = eeprom_buff[16];
	xp_acc.fall_check_time = eeprom_buff[17];
	xp_acc.fall_pitch = eeprom_buff[18];
	xp_acc.fall_roll = eeprom_buff[19];
	xp_acc.fall_pitch_oblique = eeprom_buff[20];
	xp_acc.fall_roll_oblique = eeprom_buff[21];

	xp_gyro.kp1_foot = eeprom_buff[25];
	xp_gyro.kp2_foot = eeprom_buff[26];
	xp_gyro.kp1_hip = eeprom_buff[27];
	xp_gyro.kp2_hip = eeprom_buff[28];
	xp_gyro.kp1_arm = eeprom_buff[29];
	xp_gyro.kp2_arm = eeprom_buff[30];
	xp_gyro.kp2_waist = eeprom_buff[31];
	xp_gyro.kp3_waist = eeprom_buff[32];
	xp_gyro.gyro_k1 = eeprom_buff[33];
	xp_gyro.gyro_k2 = eeprom_buff[34];
	xp_gyro.gyro_k3 = eeprom_buff[35];
	xp_gyro.ad_volt_offset1 = eeprom_buff[36];
	xp_gyro.ad_volt_offset2 = eeprom_buff[37];
	xp_gyro.ad_volt_offset3 = eeprom_buff[38];
	xp_gyro.t1 = eeprom_buff[39];
	xp_gyro.t2 = eeprom_buff[40];
	xp_gyro.gyro_data3_flt2_t1 = eeprom_buff[41];
	xp_gyro.yaw_cntl_gain = eeprom_buff[42];
	xp_gyro.yaw_cntl_dead = eeprom_buff[43];
	xp_gyro.yaw_cntl_theta = eeprom_buff[44];
	xp_gyro.gyro_omega = eeprom_buff[45];
	xp_gyro.fall_roll_deg1 = eeprom_buff[46];
	xp_gyro.fall_pitch_deg1 = eeprom_buff[47];
	flag_gyro.fall_cntl = (short)eeprom_buff[48];

	xp_mv_straight.time = eeprom_buff[50];
	xp_mv_straight.z3 = eeprom_buff[51];
	xp_mv_straight.arm_sh_pitch = eeprom_buff[52];
	xp_mv_straight.arm_sh_roll = eeprom_buff[53];
	xp_mv_straight.arm_el_yaw = eeprom_buff[54];
	xp_mv_straight.arm_el_pitch = eeprom_buff[55];

	xp_mv_ready.time = eeprom_buff[56];
	xp_mv_ready.z3 = eeprom_buff[57];
	xp_mv_ready.arm_sh_pitch = eeprom_buff[58];
	xp_mv_ready.arm_sh_roll = eeprom_buff[59];
	xp_mv_ready.arm_el_yaw = eeprom_buff[60];
	xp_mv_ready.arm_el_pitch = eeprom_buff[61];
	xp_mv_ready.pitch = eeprom_buff[62];

	xp_mv_walk.num = (long)eeprom_buff[65];
	xp_mv_walk.h_cog = eeprom_buff[66];
	xp_mv_walk.time = eeprom_buff[67];
	xp_mv_walk.x_fwd_swg = eeprom_buff[69];
	xp_mv_walk.x_fwd_spt = eeprom_buff[70];
	xp_mv_walk.x_bwd_swg = eeprom_buff[71];
	xp_mv_walk.x_bwd_spt = eeprom_buff[72];
	xp_mv_walk.y_swg = eeprom_buff[73];
	xp_mv_walk.y_spt = eeprom_buff[74];
	xp_mv_walk.theta = eeprom_buff[75];
	xp_mv_walk.z = eeprom_buff[76];
	xp_mv_walk.y_balance = eeprom_buff[77];
	xp_mv_walk.hip_roll = eeprom_buff[78];
	xp_mv_walk.x_fwd_pitch = eeprom_buff[79];
	xp_mv_walk.x_bwd_pitch = eeprom_buff[80];
	xp_mv_walk.arm_sh_pitch = eeprom_buff[81];
	xp_mv_walk.start_time_k1 = eeprom_buff[82];
	xp_mv_walk.start_zmp_k1 = eeprom_buff[83];
	//	xp_mv_walk.start_time_k2		=	eeprom_buff[84];
	xp_mv_walk.foot_cntl_p = eeprom_buff[85];
	xp_mv_walk.foot_cntl_r = eeprom_buff[86];
	xp_mv_walk.sidestep_time_k = eeprom_buff[87];
	xp_mv_walk.sidestep_roll = eeprom_buff[88];
	xp_mv_walk.y_wide = eeprom_buff[90];
	xp_mv_walk.time_dutyfactor = eeprom_buff[91];

	xp_dlim_wait_x.dlim = eeprom_buff[92];
	xp_dlim_wait_x.wait_time = eeprom_buff[93];
	xp_dlim_wait_y.dlim = eeprom_buff[94];
	xp_dlim_wait_y.wait_time = eeprom_buff[95];
	xp_dlim_wait_theta.dlim = eeprom_buff[96];
	xp_dlim_wait_theta.wait_time = eeprom_buff[97];

	odometry_correct_para_x = eeprom_buff[98];
	odometry_correct_para_y = eeprom_buff[99];

	xp_mv_walk.x_fwd_acc_pitch = eeprom_buff[101];
	xp_mv_walk.x_bwd_acc_pitch = eeprom_buff[102];
	xp_dlim_wait_pitch.dlim = eeprom_buff[103]; // �s�b�`��ύX����䗦
	xp_mv_walk.accurate_x_percent_dlim = eeprom_buff[104];
	xp_mv_walk.accurate_y_percent_dlim = eeprom_buff[105];
	xp_mv_walk.accurate_th_percent_dlim = eeprom_buff[106];
	xp_mv_walk.arm_el_pitch = eeprom_buff[107];

	return 0;
}

class webots_motor_control
{

private:
	std::vector<std::pair<int32_t, std::string>> motors_info;
	webots::Robot *robot;
	std::vector<std::tuple<int32_t, webots::Motor *, std::string>> robot_motors;
	webots::Gyro *robot_gyro;
	webots::Accelerometer *robot_accelerometer;
	webots::Keyboard *pc_keyboard;
	webots::Camera *robot_camera;
	int32_t mTimeStep;
	std::set<std::string> reverse_motors;
	int32_t current_key;
	bool forced_wait;
	boost::interprocess::message_queue msgq;
	//ここの大きさはreceive側と同じにする必要がある
	const int32_t message_len;
	uint32_t highest_priority;
	bool forced_remove_;
	boost::interprocess::message_queue angle_q;

public:
	webots_motor_control() : mTimeStep(0), current_key(0), forced_wait(true),
							 msgq(boost::interprocess::open_or_create, "WEBOTS_PICTURE_COMMUNICATION", 1, 100), message_len(700 * 480 * 4), highest_priority(0),
																																				forced_remove_(removeQueue()),
							 angle_q(boost::interprocess::create_only, "WEBTOS_MOTIONCREATOR_COMMUNICATION", 30, 20000)
	{
		robot = new webots::Robot();
		motors_info.push_back({FOOT_ROLL_R, "right_ankle_roll_joint"});
		motors_info.push_back({LEG_PITCH_R, "right_ankle_pitch_joint"});
		motors_info.push_back({KNEE_R1, "right_ankle_pitch_mimic_joint"});
		motors_info.push_back({KNEE_R1, "right_knee_pitch_joint"});
		motors_info.push_back({KNEE_R1, "right_shin_pitch_mimic_joint"});
		motors_info.push_back({KNEE_R2, "right_knee_pitch_mimic_joint"});
		motors_info.push_back({KNEE_R2, "right_waist_pitch_joint"});
		motors_info.push_back({KNEE_R2, "right_waist_pitch_mimic_joint"});
		motors_info.push_back({LEG_ROLL_R, "right_waist_roll_joint"});
		motors_info.push_back({LEG_YAW_R, "right_waist_yaw_joint"});
		motors_info.push_back({ARM_ROLL_R, "right_shoulder_roll_joint"});
		motors_info.push_back({ARM_PITCH_R, "right_shoulder_pitch_joint"});
		motors_info.push_back({ELBOW_PITCH_R, "right_elbow_pitch_joint"});
		motors_info.push_back({FOOT_ROLL_L, "left_ankle_roll_joint"});
		motors_info.push_back({LEG_PITCH_L, "left_ankle_pitch_joint"});
		motors_info.push_back({KNEE_L1, "left_ankle_pitch_mimic_joint"});
		motors_info.push_back({KNEE_L1, "left_knee_pitch_joint"});
		motors_info.push_back({KNEE_L1, "left_shin_pitch_mimic_joint"});
		motors_info.push_back({KNEE_L2, "left_knee_pitch_mimic_joint"});
		motors_info.push_back({KNEE_L2, "left_waist_pitch_joint"});
		motors_info.push_back({KNEE_L2, "left_waist_pitch_mimic_joint"});
		motors_info.push_back({LEG_ROLL_L, "left_waist_roll_joint"});
		motors_info.push_back({LEG_YAW_L, "left_waist_yaw_joint"});
		motors_info.push_back({ARM_PITCH_L, "left_shoulder_pitch_joint"});
		motors_info.push_back({ARM_ROLL_L, "left_shoulder_roll_joint"});
		motors_info.push_back({ELBOW_PITCH_L, "left_elbow_pitch_joint"});
		motors_info.push_back({HEAD_YAW, "head_yaw_joint"});

		reverse_motors.emplace("right_knee_pitch_mimic_joint");
		reverse_motors.emplace("right_ankle_pitch_mimic_joint");
		reverse_motors.emplace("left_knee_pitch_mimic_joint");
		reverse_motors.emplace("left_ankle_pitch_mimic_joint");

		for (auto &mp : motors_info)
		{
			auto motor_ptr = robot->getMotor(mp.second);
			if (motor_ptr != nullptr)
			{
				robot_motors.emplace_back(mp.first, motor_ptr, mp.second);
			}
			else
			{
				std::cerr << " getMotor allocation error !!" << std::endl;
				std::terminate();
			}
		}

		robot_accelerometer = robot->getAccelerometer("imu/data accelerometer");
		if (robot_accelerometer == nullptr)
		{
			std::cerr << " getAccelerometer memory allocation error !!" << std::endl;
			std::terminate();
		}

		robot_gyro = robot->getGyro("imu/data gyro");
		if (robot_gyro == nullptr)
		{
			std::cerr << " getGyro memory allocation error !!" << std::endl;
			std::terminate();
		}

		pc_keyboard = robot->getKeyboard();
		if (pc_keyboard == nullptr)
		{
			std::cerr << " getKeyboard memory allocation error !!" << std::endl;
			std::terminate();
		}
		/*robot_camera = robot->getCamera("camera_sensor");
		if (robot_camera == nullptr)
		{
			std::cerr << " getCamera memory allocation error !!" << std::endl;
			std::terminate();
		}
		カメラoff*/

		mTimeStep = (int)robot->getBasicTimeStep();
		std::cout << "mTimeStep is " << mTimeStep << std::endl;
		robot_accelerometer->enable(mTimeStep);
		robot_gyro->enable(mTimeStep);
		pc_keyboard->enable(mTimeStep);
		//robot_camera->enable(mTimeStep);カメラoff
	}

	bool waitForCreateQueue()
	{
		std::cout << "call wait" << std::endl;
		while (1)
		{
			try
			{
				boost::interprocess::message_queue wait(boost::interprocess::open_only, "WEBOTS_PICTURE_COMMUNICATION");
			}
			catch (boost::interprocess::interprocess_exception ex)
			{
				std::cout << "not exist" << std::endl;
				boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
				continue;
			}
			std::cout << "arimasita!" << std::endl;
			break;
		}
		return true;
	}

	bool removeQueue()
	{
		return boost::interprocess::message_queue::remove("WEBTOS_MOTIONCREATOR_COMMUNICATION");
	}

	bool getMotionCreatorCommmand()
	{
		static std::string receive_buff;
		if (angle_q.get_num_msg() == 0)
		{
			//std::cout << "there is no message\n";
			return false;
		}
		//std::cout << "------------------------------------getcommand-----------------------\n";
		receive_buff.clear();
		receive_buff.resize(20000);
		uint32_t pri = 0;
		uint64_t receive_size = 0;
		std::cout << "will receive " << std::endl;
		angle_q.receive(&receive_buff[0], receive_buff.size(), receive_size,pri);
		std::cout << "received" << std::endl;
		SendAngles angle;
		angle.ParseFromString(receive_buff);
		std::cout << "parsed " << std::endl;
		int32_t servo_number = 0;
		webots::Motor *target_motor;
		std::string name_of_motor;
		for (int i = 0; i < std::min(angle.motor_name_size(),angle.angle_size()); ++i)
		{
			std::tie(servo_number, target_motor, name_of_motor) = robot_motors[i];
			if (reverse_motors.find(angle.motor_name(i)) != reverse_motors.end())
			{
				(target_motor)->setPosition(angle.angle(i) * (M_PI / 180.0));
				std::cout << "this is " << angle.motor_name(i) << "::" << angle.angle(i) << std::endl;
			}
			else
			{
				(target_motor)->setPosition(angle.angle(i)  * (M_PI / 180.0));
				std::cout << "this is " << angle.motor_name(i) << "::" << angle.angle(i) << std::endl;
			}
		}
		return true;
	}

	int32_t send_target_degrees()
	{
		int32_t servo_number = 0;
		webots::Motor *target_motor;
		std::string name_of_motor;
		for (auto &mp : robot_motors)
		{
			std::tie(servo_number, target_motor, name_of_motor) = mp;
			if (reverse_motors.find(name_of_motor) != reverse_motors.end())
			{
				(target_motor)->setPosition(xv_ref.d[servo_number] * (M_PI / 180.0));
			}
			else
			{
				(target_motor)->setPosition(-xv_ref.d[servo_number] * (M_PI / 180.0));
			}
		}
		return 0;
	}

	int32_t get_acc_values()
	{
		const double *val = robot_accelerometer->getValues();
		xv_acc.acc_data1 = val[0] * 0.3f * 3.1f; // x ここのスケールが正しいのかが確認できない。
		xv_acc.acc_data2 = val[1] * 0.3f * 3.1f; // y
		xv_acc.acc_data3 = val[2] * 0.3f * 3.1f; // z
		//std::cout << "acc--x: " << xv_acc.acc_data1 << " y: " << xv_acc.acc_data2 << " z: " << xv_acc.acc_data3 << "\n";
		return 0;
	}

	int32_t get_gyro_values()
	{
		const double *val = robot_gyro->getValues();
		xv_gyro.gyro_data1 = val[0] * 180.0f / M_PI; // roll	返却値が[rad/sec]らしい.
		xv_gyro.gyro_data2 = val[1] * 180.0f / M_PI; // pitch	gyro_dataは[deg/sec]なので割る.
		xv_gyro.gyro_data3 = val[2] * 180.0f / M_PI; // yaw
		//std::cout << "gyro--roll: " << xv_gyro.gyro_data1 << " pitch: " << xv_gyro.gyro_data2 << " yaw: " << xv_gyro.gyro_data3 << "\n";
		return 0;
	}

	/*
	キーボードの上下左右キーで上下左右に歩行。Jキーで右旋回、Fキーで左旋回を行う。Cキーでキャンセル。Mキーでキックモーション再生が出来るが、
	その前にキャンセルをしておかないと再生出来ない。Tキーでその場足踏み。歩行速度が遅いときは適当にstrideを変更して下さい。
	*/
	bool get_key_and_send_command()
	{
		int32_t grabbed_key = 0;
		bool get_new_command = false;
		grabbed_key = pc_keyboard->getKey();
		if (grabbed_key != current_key)
		{
			std::cout << "getkey" << std::endl;
			get_new_command = true;
			current_key = grabbed_key;
		}

		if (get_new_command)
		{
			switch (grabbed_key)
			{
			case webots::Keyboard::UP:
			{
				//motion_flag = false;
				unsigned char walk_cmd = 'A';
				unsigned char num_step = ParamTableAlte[(int)(0 + 26)];
				unsigned char period = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_x = ParamTableAlte[(int)(15 + 26)];
				unsigned char stride_y = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_th = ParamTableAlte[(int)(0 + 26)];
				set_xv_comm(&xv_comm, walk_cmd, num_step, stride_th, stride_x, period, stride_y);
				convert_bin(&xv_comm_bin, &xv_comm);
			}
			break;
			case webots::Keyboard::DOWN:
			{
				//motion_flag = false;
				unsigned char walk_cmd = 'A';
				unsigned char num_step = ParamTableAlte[(int)(0 + 26)];
				unsigned char period = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_x = ParamTableAlte[(int)(-15 + 26)];
				unsigned char stride_y = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_th = ParamTableAlte[(int)(0 + 26)];
				set_xv_comm(&xv_comm, walk_cmd, num_step, stride_th, stride_x, period, stride_y);
				convert_bin(&xv_comm_bin, &xv_comm);
			}
			break;
			case webots::Keyboard::RIGHT:
			{
				//motion_flag = false;
				unsigned char walk_cmd = 'A';
				unsigned char num_step = ParamTableAlte[(int)(0 + 26)];
				unsigned char period = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_x = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_y = ParamTableAlte[(int)(-15 + 26)];
				unsigned char stride_th = ParamTableAlte[(int)(0 + 26)];
				set_xv_comm(&xv_comm, walk_cmd, num_step, stride_th, stride_x, period, stride_y);
				convert_bin(&xv_comm_bin, &xv_comm);
			}
			break;
			case webots::Keyboard::LEFT:
			{
				//motion_flag = false;
				unsigned char walk_cmd = 'A';
				unsigned char num_step = ParamTableAlte[(int)(0 + 26)];
				unsigned char period = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_x = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_y = ParamTableAlte[(int)(15 + 26)];
				unsigned char stride_th = ParamTableAlte[(int)(0 + 26)];
				set_xv_comm(&xv_comm, walk_cmd, num_step, stride_th, stride_x, period, stride_y);
				convert_bin(&xv_comm_bin, &xv_comm);
			}
			break;
			case 'T':
			{
				//motion_flag = false;
				unsigned char walk_cmd = 'A';
				unsigned char num_step = ParamTableAlte[(int)(0 + 26)];
				unsigned char period = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_x = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_y = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_th = ParamTableAlte[(int)(0 + 26)];
				set_xv_comm(&xv_comm, walk_cmd, num_step, stride_th, stride_x, period, stride_y);
				convert_bin(&xv_comm_bin, &xv_comm);
			}
			break;
			case 'J':
			{
				//motion_flag = false;
				unsigned char walk_cmd = 'A';
				unsigned char num_step = ParamTableAlte[(int)(0 + 26)];
				unsigned char period = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_x = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_y = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_th = ParamTableAlte[(int)(20 + 26)];
				set_xv_comm(&xv_comm, walk_cmd, num_step, stride_th, stride_x, period, stride_y);
				convert_bin(&xv_comm_bin, &xv_comm);
			}
			break;
			case 'F':
			{
				//motion_flag = false;
				unsigned char walk_cmd = 'A';
				unsigned char num_step = ParamTableAlte[(int)(0 + 26)];
				unsigned char period = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_x = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_y = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_th = ParamTableAlte[(int)(-20 + 26)];
				set_xv_comm(&xv_comm, walk_cmd, num_step, stride_th, stride_x, period, stride_y);
				convert_bin(&xv_comm_bin, &xv_comm);
			}
			break;
			case 'M':
			{
				//motion_flag = false;
				std::cout << "get M key" << std::endl;
				unsigned char walk_cmd = 'M';
				unsigned char num_step = ParamTableAlte[(int)(0 + 26)];
				unsigned char period = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_x = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_y = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_th = ParamTableAlte[(int)(3 + 26)];
				set_xv_comm(&xv_comm, walk_cmd, num_step, stride_th, stride_x, period, stride_y);
				convert_bin(&xv_comm_bin, &xv_comm);
			}
			break;
			case 'C':
			{
				//motion_flag = false;
				unsigned char walk_cmd = 'C';
				unsigned char num_step = ParamTableAlte[(int)(0 + 26)];
				unsigned char period = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_x = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_y = ParamTableAlte[(int)(0 + 26)];
				unsigned char stride_th = ParamTableAlte[(int)(0 + 26)];
				set_xv_comm(&xv_comm, walk_cmd, num_step, stride_th, stride_x, period, stride_y);
				convert_bin(&xv_comm_bin, &xv_comm);
			}
			break;
			default:
				break;
			}
		}
		return get_new_command;
	}

	void get_and_send_image(int64_t current_loop)
	{
		//highest_priorityがオーバーフローした場合壊れる。しかしuint32の為オーバーフローするのは
		//シミュレーション時間で9544時間連続起動した場合なので問題ないはず。message_queueの動きがよく分からない為
		//現状priorityは0固定。早い周期で送るとpriority_queueの並べ替えが間に合わないのかもしれない。
		webotsvision::CameraMeasurement picture;
		std::string send_data;
		//std::cout << "call image " << std::endl;
		send_data.resize(message_len - 100);
		//std::cout << "resize ok" << std::endl;
		//std::cout << "string construct" << std::endl;
		std::string in(reinterpret_cast<const char *>(robot_camera->getImage()));
		//std::cout << "in " << in.size() << std::endl;
		picture.set_image(in);
		//std::cout << "set_image ok" << std::endl;
		picture.set_simtime(current_loop * mTimeStep);
		send_data = picture.SerializeAsString();
		//std::cout << "serialize ok" << std::endl;
		try
		{
			if (!msgq.try_send(&send_data[0], send_data.size(), 0))
			{
				std::cout << "-------------------------buffer is full-----------------------------------\n";
			}
		}
		catch (boost::interprocess::interprocess_exception eee)
		{
			std::cout << "-----------------------------buffer is full ----------------------------------\n";
			//std::cout << eee.what() << std::endl;
		}
		//std::cout << "send ok" << std::endl;
		//++highest_priority;
	}

	bool step()
	{
		return robot->step(mTimeStep) != -1;
	}

	int32_t getmTimeStep()
	{
		return mTimeStep;
	}
};

/*--------------------------------------*/
/*	PC simulation main					*/
/*--------------------------------------*/
int main(int argc, char *argv[])
{
	int id = 0;
	short j;
	int shutdown_flag = 0;
#ifdef WEBOTS_GANKEN_SIMULATOR

	webots_motor_control wb_ganken;

	int64_t keyboard_loop = 0;

	OrientationEstimator orientationEst((double)(1000.0 / wb_ganken.getmTimeStep()) / 1000.0, 0.1);

#endif
	if (argc > 1)
	{
		//servo_port = argv[1];
		id = argv[1][3] - '0';
	}
	boost::thread thread(boost::bind(ipcthread, argc, argv, id));
	boost::posix_time::ptime ptime = boost::posix_time::microsec_clock::local_time();
	const char *servo_port = "/dev/kondoservo";

	var_init();		// �ϐ��̏�����
	serv_init();	// �T�[�{���[�^�̏�����
	calc_mv_init(); // �����̌v�Z�̏�����
	load_pc_motion("motions");
	offset_load((char *)"offset_angle.txt", servo_offset);
	eeprom_load((char *)"eeprom_list.txt");
	flag_gyro.zero = ON;

	// loop start
	for (count_time_l = 0; wb_ganken.step(); count_time_l++)
	{
		bool cmd_accept = false;
		{
			// accept command
			boost::mutex::scoped_lock look(lock_obj);
			if (cmd.size() > 0)
			{
				memcpy(rfmt, &cmd[0], cmd.size());
				joy_read();
				cmd_accept = true;
				cmd = "";
			}
		}
		if (!shutdown_flag)
			cntr();

		static unsigned long last_pan_update = 0;
		if ((fabs(xv_gyro.gyro_roll) > 30) || (fabs(xv_gyro.gyro_pitch) > 30))
		{
			if (abs((long)(count_time_l - last_pan_update)) > 10)
			{
				set_xv_comm(&xv_comm, 'H', '0', '0', '0', '1', '0');
				convert_bin(&xv_comm_bin, &xv_comm);
				last_pan_update = count_time_l;
			}
		}

		for (j = 0; j < SERV_NUM; j++)
		{
			if (xv_sv[j].deg_sv > xp_sv[j].deg_lim_h * 100 || xv_sv[j].deg_sv < xp_sv[j].deg_lim_l * 100)
				printf("*******ERROR**** xv_sv[%d].deg_sv=%f\n", j, xv_sv[j].deg_sv / 100.0f);
		}
#ifdef WEBOTS_GANKEN_SIMULATOR
		{

			//ここで指令を出す
			if(!wb_ganken.getMotionCreatorCommmand())
			{
				//wb_ganken.send_target_degrees();
			}

			if (count_time_l > 100)
			{

				wb_ganken.get_gyro_values();
				wb_ganken.get_acc_values();
				//				printf("R:%lf\tP:%lf\tY:%lf\n",xv_gyro.gyro_data1, xv_gyro.gyro_data2, xv_gyro.gyro_data3);
				//				printf("X:%f\tY:%f\tZ:%f\n",xv_acc.acc_data1, xv_acc.acc_data2, xv_acc.acc_data3);

				if ((xv_acc.acc_data1 != 0.0) || (xv_acc.acc_data2 != 0.0) || (xv_acc.acc_data3 != 0.0))
				{
					orientationEst.update(xv_gyro.gyro_data1 * M_PI / 180.0, xv_gyro.gyro_data2 * M_PI / 180.0, xv_gyro.gyro_data3 * M_PI / 180.0,
										  xv_acc.acc_data1 * 9.8, xv_acc.acc_data2 * 9.8, xv_acc.acc_data3 * 9.8);

					xv_gyro.gyro_roll =
						xv_gyro.gyro_roll2 = orientationEst.getRoll() * 180.0 / M_PI;
					xv_gyro.gyro_pitch =
						xv_gyro.gyro_pitch2 = orientationEst.getPitch() * 180.0 / M_PI;
					xv_gyro.gyro_yaw =
						xv_gyro.gyro_yaw2 = orientationEst.getYaw() * 180.0 / M_PI;
					orientationEst.getQuaternion(&xv_gyro.quaternion[0], &xv_gyro.quaternion[1], &xv_gyro.quaternion[2], &xv_gyro.quaternion[3]);
					//					printf("Roll:%f\tPitch:%f\tYaw:%f\n",xv_gyro.gyro_roll, xv_gyro.gyro_pitch, xv_gyro.gyro_yaw);
				}
			}

			if (count_time_l > (keyboard_loop + 30))
			{

				if (wb_ganken.get_key_and_send_command())
				{
					keyboard_loop = count_time_l;
				}
			}
			//wb_ganken.get_and_send_image(count_time_l);カメラoff
			/*boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time(); 
			boost::posix_time::time_duration diff = now - ptime;
			while(diff.total_milliseconds() < wb_ganken.getmTimeStep()){
			
				boost::this_thread::sleep(boost::posix_time::milliseconds((wb_ganken.getmTimeStep()) - diff.total_milliseconds()));
				now = boost::posix_time::microsec_clock::local_time();
				diff = now - ptime;
			}*/
		}
#endif // WEBOTS_GANKEN_SIMULATOR

		//		printf("cnt:%05d mode:%d%d%d%d%d\n", count_time_l, sq_flag.start, sq_flag.straight, sq_flag.ready, sq_flag.walk, sq_flag.motion);
	}
}
