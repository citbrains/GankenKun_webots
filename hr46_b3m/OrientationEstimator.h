/*!
 * @file OrientationEstimator.h
 * @brief 姿勢を推定するプログラム
 * @date 2013.12.31
 * @author Yasuo Hayashibara
 *
 * 以下の論文のアルゴリズムを利用しています．
 * S. O. H. Madgwick, “An efficient orientation filter for inertial and inertial/magnetic sensor arrays”, 
 * Technical report, University of Bristol University, UK, (2010). 
 */

#pragma once

/*!
 * @class OrientationEstimate
 */
class OrientationEstimator{

public:

    /*!
     * @brief コンストラクタ
     * 変数の初期化
     *
     * @param[in] period updateを呼び出す周期(s)
	 * @param[in] gyroscopeMeasurementError 一秒間にジャイロセンサがずれる誤差(deg/s)
     */
    OrientationEstimator(double period = 0.1, double gyroscopeMeasurementError = 1.0);

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
    void update(double wx, double wy, double wz, double ax, double ay, double az);

    /*!
     * @brief クオータニオンの値取得
     *
     * @param[out] w,x,y,z クオータニオンの値
     *
     * @return 0:成功，-1:失敗
     */
    int getQuaternion(float *w, float *x, float *y, float *z);

	/*!
     * @brief ロール角の取得
     *
     * @return ロール角(rad)
     */
    double getRoll(void);

    /*!
     * @brief ピッチ角の取得
     *
     * @return ピッチ角(rad)
     */
    double getPitch(void);

    /*!
     * @brief ヨー角の取得
     *
     * @return ヨー角(rad)
     */
    double getYaw(void);

    /*!
     * @brief 初期化
     */
    void reset(void);

private:
    double q[4];					//! 推定したクオータニオンの値
    double dt;						//! サンプリングタイム(s)
    double gyroError;				//! 一秒間にジャイロセンサがずれる誤差(deg/s)
    double beta;					//! チューニングのための定数

    double roll;					//! ロールの値
    double pitch;					//! ピッチの値
    double yaw;						//! ヨーの値

};
