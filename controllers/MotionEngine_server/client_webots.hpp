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
	HEAD_PITCH		= 	19,
	SPARE12			= 	20,
	SPARE13			= 	21,
	SPARE17			= 	22,
	SPARE21			= 	23,
	SPARE24			= 	24,
	SPARE25			= 	25,
	SPARE26			= 	26,
	SPARE27			= 	27,
	SPARE28			= 	28
};
// enum {
// 	FOOT_ROLL_L 	= 	0,			//! 左足首のロール軸
// 	KNEE_L1 	= 	1,			//! 左足膝下ピッチ軸（平行リンク）
// 	KNEE_L2 	= 	2,			//! 左足膝上ピッチ軸（平行リンク）
// 	LEG_PITCH_L 	= 	3,			//! 左股のピッチ軸（シリアルリンク，股のピッチ軸と合わせて足首の向きを変更）
// 	LEG_ROLL_L	= 	4,			//! 左股のロール軸
// 	LEG_YAW_L 	= 	5,			//! 左股のヨー軸
// 	FOOT_ROLL_R 	= 	6,			//! 右足首のロール軸
// 	KNEE_R1		= 	7,			//! 右足膝下ピッチ軸（平行リンク）
// 	KNEE_R2 	= 	8,			//! 右足膝上ピッチ軸（平行リンク）
// 	LEG_PITCH_R 	= 	9,			//! 右股のピッチ軸（シリアルリンク，股のピッチ軸と合わせて足首の向きを変更）
// 	LEG_ROLL_R	= 	10,			//! 右股のロール軸
// 	LEG_YAW_R 	= 	11,			//! 右股のヨー軸 
// 	// 12 13
// 	ARM_PITCH_L	= 	14,			//! 左腕のピッチ軸
// 	ARM_ROLL_L	= 	15,			//! 左腕のロール軸
// 	ELBOW_PITCH_L = 16,
// 	// 16 17
// 	ARM_PITCH_R	= 	18,			//! 右腕のピッチ軸
// 	ARM_ROLL_R	= 	19,			//! 右腕のロール軸
// 	ELBOW_PITCH_R = 20,
// 	HEAD_YAW	= 	22,			//! 首のヨー軸
// 	HEAD_PITCH	= 	23,			//! 首のピッチ軸（Acceliteは無い）
// 	JOINT_NUM
// };


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

		// reverse_motors.emplace(ARM_ROLL_R);
		// reverse_motors.emplace(ELBOW_PITCH_R);
		// reverse_motors.emplace(ARM_PITCH_R);

		// reverse_motors.emplace(ARM_ROLL_L);
		
		reverse_motors.emplace(KNEE_L1);
		reverse_motors.emplace(KNEE_L2);
		// reverse_motors.emplace(LEG_ROLL_L);
		// reverse_motors.emplace(FOOT_ROLL_L);
		// reverse_motors.emplace(LEG_PITCH_L);
		// reverse_motors.emplace(LEG_YAW_L);
		
		// reverse_motors.emplace(LEG_ROLL_R);

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

        mTimeStep = 10; // timestepは8固定
		std::cout << "mTimeStep is " << mTimeStep << std::endl;
		robot_accelerometer->enable(mTimeStep);
		robot_gyro->enable(mTimeStep);
    }

	int32_t send_target_degrees(std::vector<std::pair<uint32_t, double>> getMotorDegrees)
	{

		for(auto& [id, rad] : getMotorDegrees)
		{
			if(reverse_motors.find(id) != reverse_motors.end())
			{
				robot_motors.at(id)->setPosition(rad);
			}
			else
			{
				robot_motors.at(id)->setPosition(-rad);
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