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

	int32_t send_target_degrees(std::vector<std::pair<uint32_t, double>> getMotorDegrees)
	{
		int32_t servo_number = 0;
		webots::Motor * target_motor;
		std::string name_of_motors;
		//server.hpp側から与えられたサーボIDとdegをwebotsのものに並び替える
		std::sort(getMotorDegrees.begin(), getMotorDegrees.end());

		/*
		[実装できなかったこと]

		与えられたgetMotorDegreesのpairのvectorから角度データ(deg)を抜き取って、
		下のfor文のsetPositionにデータ入れたい。

		*/

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

	std::vector<double> get_acc_values()
	{
		const double *val = robot_accelerometer->getValues();
		std::vector<double> acc_val;
		acc_val.emplace_back(val[0]);
		acc_val.emplace_back(val[1]);
		acc_val.emplace_back(val[2]);
		return acc_val;
	}

	std::vector<double> get_gyro_values()
	{
		const double *val = robot_gyro->getValues();
		std::vector<double> gyro_val;
		gyro_val.emplace_back(val[0]);
		gyro_val.emplace_back(val[1]);
		gyro_val.emplace_back(val[2]);
		return gyro_val;
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