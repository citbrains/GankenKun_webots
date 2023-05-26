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


#if (defined WIN32) || (defined COMM_WITH_WINDOWS_WEBOTS) 
    static const std::string adress("tcp://*:7650");
    static const std::string adress_camera("tcp://*:8760");
#else
    static const std::string adress("ipc:///tmp/WebotsMotionEngine");
    static const std::string adress_camera("ipc:///tmp/WebotsMotionEngine_Camera");
#endif // DEBUG

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
        auto ret = reply_.recv(buf, zmq::recv_flags::none);
        if(!ret.has_value()){
            std::cout << "no value!" << std::endl;
            return {};
        }
        else if(ret.value().truncated()){
            std::cout << "truncated!" << std::endl;
            return {};
        }
        std::string parse_data(reinterpret_cast<const char *>(buf.data()),buf.size());
        webotsMotionEngine::degrees receive_degs;
        if(!receive_degs.ParseFromString(parse_data))
        {
            // std::cout << receive_degs.DebugString() << std::endl;
            // std::cout << "parse error..\n";
            // return {};
        }
        std::vector<std::pair<uint32_t, double>> return_val;
        return_val.reserve(receive_degs.motor_number_size());
        // std::cout << "reserve" << receive_degs.motor_number_size() << std::endl;
        for (size_t i = 0; i < std::min(receive_degs.motor_number_size(), receive_degs.motor_degs_size()); ++i)
        {
            return_val.emplace_back(std::make_pair<uint32_t, double>(receive_degs.motor_number(i), receive_degs.motor_degs(i)));
            // std::cout << receive_degs.motor_number(i) << " receive_degs ::" << receive_degs.motor_degs(i) << std::endl;
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
    inline static constexpr size_t buf_size_ = 8192;
};
