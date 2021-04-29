#include <boost/interprocess/ipc/message_queue.hpp>
#include <iostream>

int main(int argc, char const *argv[])
{
    if(boost::interprocess::message_queue::remove("WEBOTS_PICTURE_COMMUNICATION"))
    {
        std::cout << "success removing  message_queue which has key of [WEBOTS_PICTURE_COMMUNICATION] " << std::endl;
    }
    else{
        std::cout << "fail removing message_queue" << std::endl;
    }
    return 0;
}
