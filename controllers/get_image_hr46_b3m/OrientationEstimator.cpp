/*!
 * @file OrientationEstimator.cpp
 * @brief 姿勢を推定するプログラム
 * @date 2013.12.31
 * @author Yasuo Hayashibara
 *
 * 以下の論文のアルゴリズムを利用しています．
 * S. O. H. Madgwick, “An efficient orientation filter for inertial and inertial/magnetic sensor arrays”, 
 * Technical report, University of Bristol University, UK, (2010). 
 */

#include <boost/math/constants/constants.hpp>
#include "OrientationEstimator.h"
#include "stdlib.h"

/*!
 * @brief コンストラクタ
 * 変数の初期化
 *
 * @param[in] period updateを呼び出す周期(s)
 * @param[in] gyroscopeMeasurementError 一秒間にジャイロセンサがずれる誤差(deg/s)
 */
OrientationEstimator::OrientationEstimator(double period, double gyroError)
{
	q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
	roll = pitch = yaw = 0.0;

	dt = period;
	this->gyroError = gyroError;
	double pi = boost::math::constants::pi<double>();
	beta = sqrt(3.0 / 4.0) * (pi * (gyroError / 180.0));
}

/*!
 * @brief データのアップデート
 * 姿勢（クオータニオン）の計算
 *
 * @param[in] wx x軸方向のジャイロの値 (rad/s)
 * @param[in] wy y軸方向のジャイロの値 (rad/s)
 * @param[in] wz z軸方向のジャイロの値 (rad/s)
 * @param[in] ax x軸方向の加速度の値 (m/s^2)
 * @param[in] ay y軸方向の加速度の値 (m/s^2)
 * @param[in] az z軸方向の加速度の値 (m/s^2)
 */
void OrientationEstimator::update(double wx, double wy, double wz, double ax, double ay, double az) {
	double pi = boost::math::constants::pi<double>();
	const double eps = 1e-8;    // 最小値
	double qd[4];				// ジャイロの値から計算されるクオータニオンが変動する割合
	double f[3];				// 目的関数
	double J[3][4];				// ヤコビ行列
	double nabla_f[4];			// ナブラ
	double norm;

	// ジャイロの計測値からクオータニオンの変動する割合を計算 (eq.11)
	qd[0] =                    - 0.5 * q[1] * wx - 0.5 * q[2] * wy - 0.5 * q[3] * wz ;
	qd[1] =  0.5 * q[0] * wx                     + 0.5 * q[2] * wz - 0.5 * q[3] * wy ;
	qd[2] =  0.5 * q[0] * wy - 0.5 * q[1] * wz                     + 0.5 * q[3] * wx ;
	qd[3] =  0.5 * q[0] * wz + 0.5 * q[1] * wy - 0.5 * q[2] * wx                     ;

	// 加速度の正規化 (eq.24)
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm < eps) norm = eps;
	ax /= norm; ay /= norm; az /= norm;

	// 目的関数の計算 (eq.25)
	f[0] = 2.0 * (          q[1] * q[3] - q[0] * q[2]) - ax;
	f[1] = 2.0 * (          q[0] * q[1] + q[2] * q[3]) - ay;
	f[2] = 2.0 * (1.0/2.0 - q[1] * q[1] - q[2] * q[2]) - az;

	// ヤコビ関数の計算 (eq.26)
	J[0][0] = -2.0 * q[2]; J[0][1] =  2.0 * q[3]; J[0][2] = -2.0 * q[0]; J[0][3] =  2.0 * q[1];
	J[1][0] =  2.0 * q[1]; J[1][1] =  2.0 * q[0]; J[1][2] =  2.0 * q[3]; J[1][3] =  2.0 * q[2];
	J[2][0] =  0.0       ; J[2][1] = -4.0 * q[1]; J[2][2] = -4.0 * q[2]; J[2][3] =  0.0       ;

	// ナブラを計算 (eq.34)
	nabla_f[0] = J[0][0] * f[0] + J[1][0] * f[1] + J[2][0] * f[2];
	nabla_f[1] = J[0][1] * f[0] + J[1][1] * f[1] + J[2][1] * f[2];
	nabla_f[2] = J[0][2] * f[0] + J[1][2] * f[1] + J[2][2] * f[2];
	nabla_f[3] = J[0][3] * f[0] + J[1][3] * f[1] + J[2][3] * f[2];

	// 勾配を正規化 (eq.33)
	norm = sqrt(nabla_f[0] * nabla_f[0] + nabla_f[1] * nabla_f[1] + nabla_f[2] * nabla_f[2] + nabla_f[3] * nabla_f[3]);
	if (norm < eps) norm = eps;
	nabla_f[0] /= norm; nabla_f[1] /= norm; nabla_f[2] /= norm; nabla_f[3] /= norm;

	// 推定したクオータニオンの変動量を積分 (eq.43)
	q[0] += (qd[0] - (beta * nabla_f[0])) * dt;
	q[1] += (qd[1] - (beta * nabla_f[1])) * dt;
	q[2] += (qd[2] - (beta * nabla_f[2])) * dt;
	q[3] += (qd[3] - (beta * nabla_f[3])) * dt;

	// クオータニオンを正規化
	norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (norm < eps) norm = eps;
	q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;

	// Roll, Yaw, Pitchの計算
	double x = 2.0 * q[1] * q[3] - 2.0 * q[0] * q[2];
	double y = 2.0 * q[2] * q[3] + 2.0 * q[0] * q[1];
	double z = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	roll   = atan2(y, sqrt(x * x + z * z));
	pitch  = -atan2(x, sqrt(y * y + z * z));
	if ((abs(roll) < pi/4.0)&&(abs(pitch) < pi/4.0f)){				// 垂直から45degを超えた時にはyaw方向の角度を更新しない．
		yaw = -atan2(2.0 * q[1] * q[2] - 2.0 * q[0] * q[3], q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	}
}

/*!
 * @brief クオータニオンの値取得
 *
 * @param[out] w,x,y,z クオータニオンの値
 *
 * @return 0:成功，-1:失敗
 */
int OrientationEstimator::getQuaternion(float *w, float *x, float *y, float *z)
{
	*w = q[0];
	*x = q[1];
	*y = q[2];
	*z = q[3];
	return 0;
}


/*!
 * @brief ロール角の取得
 *
 * @return ロール角(rad)
 */
double OrientationEstimator::getRoll(void)
{
	return roll;
}

/*!
 * @brief ピッチ角の取得
 *
 * @return ピッチ角(rad)
 */
double OrientationEstimator::getPitch(void)
{
	return pitch;
}

/*!
 * @brief ヨー角の取得
 *
 * @return ヨー角(rad)
 */
double OrientationEstimator::getYaw(void)
{
	return yaw;
}

/*!
 * @brief 初期化
 */
void OrientationEstimator::reset(void)
{
	q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
	roll = pitch = yaw = 0.0;
}

#if 0
/*
 * @brief OrientationEstimatorクラスのチェックプログラム
 * コンパイル方法
 * gcc -o OrientationEstimator OrientationEstimator.cpp -lm -lstdc++
 */
int main(void)
{
	const int div = 20;
	const double pi = 3.14159;
	{
		// ジャイロセンサのみのチェック
		OrientationEstimator oe(1.0/div, 0.0);					// 1/div(s)毎に呼び出す. 1.0deg/sのエラー

		// rotate X-axis
		for(int i = 0; i < div; i ++) oe.update(1, 0, 0, 0, 0, 0);
		printf("Rotate X-axis : (1,0,0) %f, %f, %f\n", oe.getRoll(), oe.getPitch(), oe.getYaw());
		oe.reset();

		// rotate Y-axis
		for(int i = 0; i < div; i ++) oe.update(0, 1, 0, 0, 0, 0);
		printf("Rotate Y-axis : (0,1,0) %f, %f, %f\n", oe.getRoll(), oe.getPitch(), oe.getYaw());
		oe.reset();

		// rotate Z-axis
		for(int i = 0; i < div; i ++) oe.update(0, 0, 1, 0, 0, 0);
		printf("Rotate Z-axis : (0,0,1) %f, %f, %f\n", oe.getRoll(), oe.getPitch(), oe.getYaw());
		oe.reset();
	}
	{
		// ジャイロセンサと加速度センサのチェック
		OrientationEstimator oe(0.05, 10.0);					// 1/div(s)毎に呼び出す. 1.0deg/sのエラー

		// rotate X-axis
		for(int i = 0; i < div; i ++) oe.update(1, 0, 0, 0, sin((double)i/div), cos((double)i/div));
		printf("Rotate X-axis : (1,0,0) %f, %f, %f\n", oe.getRoll(), oe.getPitch(), oe.getYaw());
		oe.reset();

		// rotate Y-axis
		for(int i = 0; i < div; i ++) oe.update(0, 1, 0, -sin((double)i/div), 0, cos((double)i/div));
		printf("Rotate Y-axis : (0,1,0) %f, %f, %f\n", oe.getRoll(), oe.getPitch(), oe.getYaw());
		oe.reset();

		// rotate Z-axis
		for(int i = 0; i < div; i ++) oe.update(0, 0, 1, 0, 0, 1);
		printf("Rotate Z-axis : (0,0,1) %f, %f, %f\n", oe.getRoll(), oe.getPitch(), oe.getYaw());
		oe.reset();
	}
	{
		// 倒れて回転して起き上がった場合
		OrientationEstimator oe(0.05, 1.0);					// 1/div(s)毎に呼び出す. 1.0deg/sのエラー

		// rotate Y-axis
		for(int i = 0; i < div; i ++) oe.update(0,pi/2,0,-sin(pi/2*i/div),0,cos(pi/2*i/div));
		printf("Rotate Y-axis : (0,pi/2,0) %f, %f, %f\n", oe.getRoll(), oe.getPitch(), oe.getYaw());

		// rotate X-axis
		for(int i = 0; i < div; i ++) oe.update(1,0,0,-1,0,0);
		printf("Rotate X-axis : (0,pi/2,-1) %f, %f, %f\n", oe.getRoll(), oe.getPitch(), oe.getYaw());

		// rotate Y-axis
		for(int i = 0; i < div; i ++) oe.update(0,-pi/2,0,-sin(-pi/2*(div-i)/div),0,cos(-pi/2*(div-i)/div));
		printf("Rotate Y-axis : (0,0,-1) %f, %f, %f\n", oe.getRoll(), oe.getPitch(), oe.getYaw());
	}
	getchar();
	return 0;
}
#endif

