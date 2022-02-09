#include "zmq.hpp"
#include <string>
#include <iostream>
#include <future>
#include <array>
#include <cstdint>
#include <cstddef>
#include <cassert>
#include <vector>
#include <utility>
static const std::string adress("ipc://WebotsMotionEngine");

class MotionEngineCom
{
    MotionEngineCom() : ctx_(), reply_(ctx_, zmq::socket_type::rep)
    {
        reply_.bind(adress);
    }
    /**
     * @brief モーターの角度指令値を受け取る.
     * 
     * @return std::vector<std::pair<int32_t, double>> モーター番号とそれを動かす角度指令値[rad] 
     */
    std::vector<std::pair<int32_t, double>> getMotorDgrees(){

    };
    /**
     * @brief webotsで取得したジャイロと加速度計の値を送信する。こちらを呼び出す前には必ずgetMotorDegreesを呼ぶ.
     * 
     * @param gyro x,y,zの3要素 [rad/s]
     * @param accelerometer x,y,zの3要素 [m/s^2] 
     */
    void sendIMUData(const std::vector<double>& gyro,const std::vector<double>& accelerometer){

    }
    private : zmq::context_t ctx_;
    zmq::socket_t reply_;
    const size_t buf_size_ = 1024;
};

