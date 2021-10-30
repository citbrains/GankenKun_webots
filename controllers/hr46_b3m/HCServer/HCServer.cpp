#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <Ice/Ice.h>
#include <HCIPC.h>
#include "KSerialPort.h"

using namespace std;
using namespace hc;
using namespace boost;
using namespace boost::asio;

static boost::mutex lock_obj;
static string cmd;
static string res;
static bool received = false;

const size_t BUF_SIZE = 1024;

void thread_run()
{
	boost::system::error_code err;
	char buf[1024];
	KSerialPort port;
	if (port.open("COM1")) {
		std::cerr << "cannot open port" << std::endl;
		return;
	}
	port.setBaudRate(115200);
	while(true) {
		boost::mutex::scoped_lock look(lock_obj);
		if (cmd.size() > 0) {
			memset(buf, 0, sizeof(buf));
			port.write_some(cmd.c_str(), cmd.size(), err);
			std::cout << cmd << std::endl;
			int numread = port.read_some(buf, 1023, err);
			//int numread = port.readLine(buf, 1023, err);
			res = string(buf, numread);
			std::cout << res;
			received = true;
			cmd = "";
		}
	}
}

string recvCommand(const string &str, void *context)
{
	if (str.size() > 0) {
		boost::mutex::scoped_lock look(lock_obj);
		cmd = str;
		received = false;
	}
	while (!received) {
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	}
	{
		boost::mutex::scoped_lock look(lock_obj);
		return res;	
	}
}

int
main(int argc, char* argv[])
{
	boost::thread thread(thread_run);
	HCIPCServer *hcipc = HCIPCServer::createServer(argc, argv);
	hcipc->setCallback(recvCommand, NULL);
	hcipc->wait();

	delete hcipc;
    return 0;
}
