#include <boost/interprocess/ipc/message_queue.hpp>
#include <iostream>

int main(int argc, char const *argv[])
{
    boost::interprocess::message_queue::remove("WEBOTS_PICTURE_COMMUNICATION");
    std::cout << "remove message_queue which has key of [WEBOTS_PICTURE_COMMUNICATION] " << std::endl;
    return 0;
}
