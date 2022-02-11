#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <set>
#include <string>
#include <vector>
#include <tuple>
#include <exception>
#include <map>
#include <cmath>
#define SERV_NUM (19)

struct st_xv_ref
{
	float d[SERV_NUM];
	float d_ref[SERV_NUM];
};

struct st_xv_acc
{
	float	acc_data1;				/*	x[G]			*/
	float	acc_data2;				/*	y[G]			*/
	float	acc_data3;				/*	z[G]			*/
	float	acc_data1_d;			/*	x[G/sec]		*/
	float	acc_data2_d;			/*	y[G/sec]		*/
	float	acc_data3_d;			/*	z[G/sec]		*/
	float	acc_data1_flt;			/*	filer x[G]		*/
	float	acc_data2_flt;			/*	filter y[G]		*/
	float	acc_data3_flt;			/*	filter z[G]		*/
	float	acc_pitch;				/*	pitch angle [deg]			*/
	float	acc_roll;				/*	roll angle [deg]			*/
	float	acc_pitch_d;			/*	pitch angle velocity [deg/s]*/
	float	acc_roll_d;				/*	roll angle velocity [deg/s]	*/
	float	acc_pitch2;				/*	pitch angle [deg]			*/
	float	acc_roll2;				/*	roll angle [deg]			*/
	short	fall_fwd_work;			/*	[bit]			*/
	short	fall_bwd_work;			/*	[bit]			*/
	short	fall_right_work;		/*	[bit]			*/
	short	fall_left_work;			/*	[bit]			*/
};

struct st_xv_gyro
{
	float	gyro_data1;				/*	[deg/sec]		*/
	float	gyro_data2;				/*	[deg/sec]		*/
	float	gyro_data3;				/*	[deg/sec]		*/
	float	gyro_data1_d;			/*	[deg/sec/sec]	*/
	float	gyro_data2_d;			/*	[deg/sec/sec]	*/
	float	gyro_data3_d;			/*	[deg/sec/sec]	*/
	float	gyro_data1_flt;			/*	[deg/sec]		*/
	float	gyro_data2_flt;			/*	[deg/sec]		*/
	float	gyro_data3_flt;			/*	[deg/sec]		*/
	float	gyro_data3_flt2;		/*	[deg/sec]		*/
	float	gyro_roll;				/*	roll [deg]		*/
	float	gyro_pitch;				/*	pitch [deg]		*/
	float	gyro_yaw;				/*	yaw [deg]		*/
	float	gyro_yaw2;				/*	yaw [deg]		*/
	float	gyro_roll2;				/*	roll [deg]		*/
	float	gyro_pitch2;			/*	pitch [deg]		*/
	float	deg_foot_roll;			/*	[deg]			*/
	float	deg_foot_pitch;			/*	[deg]			*/
	float	deg_hip_roll;			/*	[deg]			*/
	float	deg_hip_pitch;			/*	[deg]			*/
	float	deg_arm_roll;			/*	[deg]			*/
	float	deg_arm_pitch;			/*	[deg]			*/
	float	deg_waist_pitch;		/*	[deg]			*/
	float	deg_waist_yaw;			/*	[deg]			*/
	float	yaw_cntl_ref;			/*	reference [deg]	*/
	float	yaw_cntl_fb;			/*	feedback [deg]	*/
	float	quaternion[4];			/*  quaternion (w, x, y, z) */
};

st_xv_ref xv_ref;
st_xv_acc xv_acc;
st_xv_gyro xv_gyro;


class webots_motor_control
{

private:
    webots::Robot *robot;
    webots::Gyro *robot_gyro;
	webots::Accelerometer *robot_accelerometer;
	int32_t mTimeStep;
    std::map<int32_t, std::string> motors_info;
    std::vector<std::tuple<int32_t, webots::Motor *, std::string>> robot_motors;
    std::set<std::string> reverse_motors;

public:
    webots_motor_control() : mTimeStep(0)
    {
        robot = new webots::Robot();
        
		motors_info.emplace(0, "right_ankle_roll_joint");
		motors_info.emplace(1, "right_ankle_pitch_joint");
		motors_info.emplace(2, "right_knee_pitch_joint");
		motors_info.emplace(3, "right_waist_pitch_joint");
		motors_info.emplace(4, "right_waist_roll_joint [hip]");
		motors_info.emplace(5, "right_waist_yaw_joint");
		motors_info.emplace(6, "right_shoulder_roll_joint");
		motors_info.emplace(7, "right_shoulder_pitch_joint [shoulder]");
		motors_info.emplace(8, "right_elbow_pitch_joint");
		motors_info.emplace(9, "left_ankle_roll_joint");
		motors_info.emplace(10, "left_ankle_pitch_joint");
		motors_info.emplace(11, "left_knee_pitch_joint");
		motors_info.emplace(12, "left_waist_pitch_joint");
		motors_info.emplace(13, "left_waist_roll_joint [hip]");
		motors_info.emplace(14, "left_waist_yaw_joint");
		motors_info.emplace(15, "left_shoulder_pitch_joint [shoulder]");
		motors_info.emplace(16, "left_shoulder_roll_joint");
		motors_info.emplace(17, "left_elbow_pitch_joint");
		motors_info.emplace(18, "head_yaw_joint");

		reverse_motors.emplace("right_knee_pitch_joint");
		reverse_motors.emplace("right_waist_pitch_joint");
		reverse_motors.emplace("left_knee_pitch_joint");
		reverse_motors.emplace("left_waist_pitch_joint");
		reverse_motors.emplace("right_shoulder_roll_joint");
		reverse_motors.emplace("left_shoulder_roll_joint");

        for(auto &mp : motors_info)
        {
            auto motor_ptr = robot->getMotor(mp.second);
            if(motor_ptr != nullptr)
            {
                robot_motors.emplace_back(mp.first, motor_ptr, mp.second);
            }
            else
            {
                std::cerr << "getMotor allocation error !!!" << std::endl;
                std::terminate();
            }
        }

        robot_accelerometer = robot->getAccelerometer("accelerometer");
		if (robot_accelerometer == nullptr)
		{
			std::cerr << " getAccelerometer memory allocation error !!" << std::endl;
			std::terminate();
		}

		robot_gyro = robot->getGyro("gyro");
		if (robot_gyro == nullptr)
		{
			std::cerr << " getGyro memory allocation error !!" << std::endl;
			std::terminate();
		}

        mTimeStep = 8; // timestepは8固定
		std::cout << "mTimeStep is " << mTimeStep << std::endl;
		robot_accelerometer->enable(mTimeStep);
		robot_gyro->enable(mTimeStep);
    }

	int32_t send_target_degrees()
	{
		int32_t servo_number = 0;
		webots::Motor * target_motor;
		std::string name_of_motors;
		for(auto &mp : robot_motors)
		{
			std::tie(servo_number, target_motor, name_of_motors) = mp;
			if(reverse_motors.find(name_of_motors) != reverse_motors.end())
			{
				(target_motor)->setPosition((xv_ref.d[servo_number]) * (M_PI / 180.0));
			}
			else
			{
				(target_motor)->setPosition((-xv_ref.d[servo_number] + (M_PI / 180.0)));
			}
		}
		return 0;
	}

	int32_t get_acc_values()
	{
		const double *acc_val = robot_accelerometer->getValues();
		std::cout << acc_val << std::endl;
		return 0;
	}

	int32_t get_gyro_values()
	{
		const double *gyro_val = robot_gyro->getValues();
		std::cout << gyro_val << std::endl;
		return 0;
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