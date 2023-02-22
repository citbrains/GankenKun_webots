#include "server.hpp"
#include "client_webots.hpp"
#include <boost/thread.hpp>

int main(int argc, char const *argv[])
{
    volatile unsigned long 	count_time_l;

    webots_motor_control wb_gankenkun;
    MotionEngineCom MotionEngine_webots;

    for(count_time_l = 0; wb_gankenkun.step(); count_time_l++)
    {
        wb_gankenkun.send_target_degrees(MotionEngine_webots.getMotorDegrees());

        // if(count_time_l > 100)
        {
            MotionEngine_webots.sendIMUData(wb_gankenkun.get_gyro_values(), wb_gankenkun.get_acc_values());
        }
#ifdef CAMERA_ENABLE
        auto camera_data = wb_gankenkun.get_camera_image();
        MotionEngine_webots.sendCameraData(camera_data);
#endif // CAMERA_ENABLE
    }
    return 0;
}
