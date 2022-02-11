#include "server.hpp"
#include "client_webots.hpp"
#include <boost/thread.hpp>

int main(int argc, char const *argv[])
{
    //int id = 0;
    //short joint;
    //int shutdown_flag = 0;
    volatile unsigned long 	count_time_l;

    webots_motor_control wb_gankenkun;
    //MotionEngineCom MotionEngine_webots;

    for(count_time_l = 0; wb_gankenkun.step(); count_time_l++)
    {
        wb_gankenkun.send_target_degrees();
        if(count_time_l > 100)
        {
            wb_gankenkun.get_acc_values();
            wb_gankenkun.get_gyro_values();
        }
    }
    return 0;
}
