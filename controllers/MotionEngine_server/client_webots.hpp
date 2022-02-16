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

enum {
	FOOT_ROLL_R 	= 	0,
	LEG_PITCH_R 	= 	1,
	KNEE_R1		 	= 	2,
	KNEE_R2 		= 	3,
	LEG_ROLL_R	 	= 	4,
	LEG_YAW_R 		= 	5,
	ARM_PITCH_R		= 	6,
	ARM_ROLL_R		= 	7,
	ELBOW_PITCH_R	= 	8,
	FOOT_ROLL_L 	= 	9,
	LEG_PITCH_L 	= 	10,
	KNEE_L1 		= 	11,
	KNEE_L2 		= 	12,
	LEG_ROLL_L	 	= 	13,
	LEG_YAW_L 		= 	14,
	ARM_PITCH_L		= 	15,
	ARM_ROLL_L		= 	16,
	ELBOW_PITCH_L	= 	17,
	HEAD_YAW		= 	18,
};
class webots_motor_control
{

private:
    webots::Robot *robot;
    webots::Gyro *robot_gyro;
	webots::Accelerometer *robot_accelerometer;
	int32_t mTimeStep;
    std::map<int32_t, std::string> motors_info;
    std::map<int32_t, webots::Motor *> robot_motors;
    std::set<int32_t> reverse_motors;

public:
    webots_motor_control() : mTimeStep(0)
    {
        robot = new webots::Robot();
        //enum の要素にする
		motors_info.emplace(FOOT_ROLL_R , "right_ankle_roll_joint");
		motors_info.emplace(LEG_PITCH_R, "right_ankle_pitch_joint");
		motors_info.emplace(KNEE_R1, "right_knee_pitch_joint");
		motors_info.emplace(KNEE_R2, "right_waist_pitch_joint");
		motors_info.emplace(LEG_ROLL_R, "right_waist_roll_joint [hip]");
		motors_info.emplace(LEG_YAW_R , "right_waist_yaw_joint");
		motors_info.emplace(ARM_ROLL_R, "right_shoulder_roll_joint");
		motors_info.emplace(ARM_PITCH_R, "right_shoulder_pitch_joint [shoulder]");
		motors_info.emplace(ELBOW_PITCH_R, "right_elbow_pitch_joint");
		motors_info.emplace(FOOT_ROLL_L, "left_ankle_roll_joint");
		motors_info.emplace(LEG_PITCH_L, "left_ankle_pitch_joint");
		motors_info.emplace(KNEE_L1, "left_knee_pitch_joint");
		motors_info.emplace(KNEE_L2, "left_waist_pitch_joint");
		motors_info.emplace(LEG_ROLL_L, "left_waist_roll_joint [hip]");
		motors_info.emplace(LEG_YAW_L, "left_waist_yaw_joint");
		motors_info.emplace(ARM_PITCH_L, "left_shoulder_pitch_joint [shoulder]");
		motors_info.emplace(ARM_ROLL_L, "left_shoulder_roll_joint");
		motors_info.emplace(ELBOW_PITCH_L, "left_elbow_pitch_joint");
		motors_info.emplace(HEAD_YAW, "head_yaw_joint");

		reverse_motors.emplace(KNEE_R1);
		reverse_motors.emplace(KNEE_R2);
		reverse_motors.emplace(KNEE_L1);
		reverse_motors.emplace(KNEE_L2);
		reverse_motors.emplace(ARM_ROLL_R);
		reverse_motors.emplace(ARM_ROLL_L);

        for(auto &mp : motors_info)
        {
            auto motor_ptr = robot->getMotor(mp.second);
            if(motor_ptr != nullptr)
            {
                robot_motors.emplace(mp.first, motor_ptr);
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

		for(auto& [id, deg] : getMotorDegrees)
		{
			if(reverse_motors.find(id) != reverse_motors.end())
			{
				robot_motors.at(id)->setPosition(deg * (M_PI / 180.0));
			}
			else
			{
				robot_motors.at(id)->setPosition(-deg * (M_PI / 180.0));
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