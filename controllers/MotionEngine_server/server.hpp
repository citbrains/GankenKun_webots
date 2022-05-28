#include "zmq.hpp"
#include <string>
#include <iostream>
#include <array>
#include <cstdint>
#include <cstddef>
#include <vector>
#include <utility>
#include <string_view>
#include "message.pb.h"
static const std::string adress("ipc:///tmp/WebotsMotionEngine");

class MotionEngineCom
{
public:
    MotionEngineCom() : ctx_(), reply_(ctx_, zmq::socket_type::rep)
    {
        reply_.bind(adress);
    }
    /**
     * @brief モーターの角度指令値を受け取る.
     *
     * @return std::vector<std::pair<int32_t, double>> モーター番号とそれを動かす角度指令値[rad]
     */
    std::vector<std::pair<uint32_t, double>> getMotorDegrees()
    {
        std::array<char, buf_size_> data;
        data.fill(static_cast<char>(0));
        auto buf = zmq::buffer(data.data(), data.size());
        reply_.recv(buf);
        webotsMotionEngine::degrees deg;
        std::string s(reinterpret_cast<const char *>(buf.data()),buf.size());
        deg.ParseFromString(s);
        std::cout << deg.DebugString();
        std::vector<std::pair<uint32_t, double>> return_val;
        for (size_t i = 0; i < std::min(deg.motor_number_size(), deg.motor_degs_size()); ++i)
        {
            return_val.emplace_back(std::make_pair<uint32_t, double>(deg.motor_number(i), deg.motor_degs(i)));
            std::cout << deg.motor_number(i) << " deg ::" << deg.motor_degs(i) << std::endl;
        }
        return return_val;
    };
    /**
     * @brief webotsで取得したジャイロと加速度計の値を送信する。こちらを呼び出す前には必ずgetMotorDegreesを呼ぶ.
     *
     * @param gyro x,y,zの3要素 [rad/s]
     * @param accelerometer x,y,zの3要素 [m/s^2]
     */
    void sendIMUData(const std::vector<double> &gyro, const std::vector<double> &accelerometer)
    {
        webotsMotionEngine::sensorData sensor;
        for (const auto &itm : gyro)
        {
            sensor.add_gyro(itm);
        }
        for (const auto &itm : accelerometer)
        {
            sensor.add_accelerometer(itm);
        }
        std::string data(sensor.SerializeAsString());
        std::cout << sensor.DebugString();
        auto buf = zmq::buffer(data);
        reply_.send(buf);
    }

private:
    zmq::context_t ctx_;
    zmq::socket_t reply_;
    inline static constexpr size_t buf_size_ = 2048;
};
