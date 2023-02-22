#include "zmq.hpp"
#include <string>
#include <iostream>
#include <array>
#include <cstdint>
#include <cstddef>
#include <vector>
#include <utility>
#include <string_view>
#include "client_webots.hpp"
#include "message.pb.h"
static const std::string adress("ipc:///tmp/WebotsMotionEngine");
static const std::string adress_camera("ipc:///tmp/WebotsMotionEngine_Camera");

class MotionEngineCom
{
public:
    MotionEngineCom() : ctx_(), reply_(ctx_, zmq::socket_type::rep),publish_(ctx_,zmq::socket_type::pub)
    {
        reply_.bind(adress);
        publish_.bind(adress_camera);
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
        // std::cout << deg.DebugString();
        std::vector<std::pair<uint32_t, double>> return_val;
        for (size_t i = 0; i < std::min(deg.motor_number_size(), deg.motor_degs_size()); ++i)
        {
            return_val.emplace_back(std::make_pair<uint32_t, double>(deg.motor_number(i), deg.motor_degs(i)));
            // std::cout << deg.motor_number(i) << " deg ::" << deg.motor_degs(i) << std::endl;
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
        // std::cout << sensor.DebugString();
        auto buf = zmq::buffer(data);
        reply_.send(buf);
    }

    void sendCameraData(const camera_sensor_data& data){
        webotsMotionEngine::cameraData camera_data;
        camera_data.set_height(data.height);
        camera_data.set_width(data.width);
        camera_data.set_current_total_timestep(data.current_total_timestep);
        camera_data.set_raw_data(std::move(data.raw_data));
        std::string s(camera_data.SerializeAsString());
        auto buf = zmq::buffer(s);
        try
        {
            auto err = publish_.send(buf,zmq::send_flags::dontwait);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        return;
    }

private:
    zmq::context_t ctx_;
    zmq::socket_t reply_;
    zmq::socket_t publish_;
    inline static constexpr size_t buf_size_ = 2048;
};
