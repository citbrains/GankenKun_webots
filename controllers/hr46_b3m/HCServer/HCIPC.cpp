#include "HCIPC.h"
#include <boost/thread.hpp>

using namespace std;
using namespace hc;

static boost::mutex mut;
static hc::StringSenderPrx prx = 0;

#define MAX_CMD_LEN 256

HCIPCServer::HCIPCServer()
{
}

HCIPCServer::~HCIPCServer()
{
    if (ic) {
        try {
            ic->destroy();
        } catch (const Ice::Exception& e) {
            cerr << e << endl;
        }
    }
}

HCIPCServer*
HCIPCServer::createServer(int argc, char *argv[], int *out_status, unsigned short in_port)
{
	HCIPCServer *ret;
	int status = 0;
    Ice::ObjectPtr object = NULL;
    Ice::CommunicatorPtr ic;
    try {
        ic = Ice::initialize(argc, argv);
		char portname[256]; 
		sprintf(portname, "default -p %d", in_port);
        Ice::ObjectAdapterPtr adapter =
            ic->createObjectAdapterWithEndpoints("HCIPCAdapter", portname);
		ret = new HCIPCServer();
        Ice::ObjectPtr object = ret;
        adapter->add(object, ic->stringToIdentity("HCIPCServer"));
        adapter->activate();
		ret->ic = ic;
    } catch (const Ice::Exception& e) {
        cerr << e << endl;
        status = 1;
    } catch (const char* msg) {
        cerr << msg << endl;
        status = 1;
    }

	if (out_status) {
		*out_status = status;
	}
	return ret;
}

void
HCIPCServer::wait()
{
	ic->waitForShutdown();
}

std::string
HCIPCServer::sendString(const std::string& s, const Ice::Current&)
{
	std::string ret;
	boost::mutex::scoped_lock loc(mut);
	if (pCallback) { 
		ret = pCallback(s, context);
	}
	return ret;
}

HCIPCClient::HCIPCClient(const std::string &in_host, unsigned short in_port)
: ic(0), hostname(in_host), port(in_port)
{
    try {
        ic = Ice::initialize();
    } catch (const Ice::Exception& ex) {
        cerr << ex << endl;
    } catch (const char* msg) {
        cerr << msg << endl;
    }
}

HCIPCClient::~HCIPCClient()
{
	if (ic) {
		ic->destroy();
		ic = 0;
	}
}

std::string
HCIPCClient::sendString(const std::string &str)
{
	boost::mutex::scoped_lock loc(mut);
	std::string ret;
	hc::StringSenderPrx prx = getProxy();
	if (prx) {
		try {
			ret = prx->sendString(str);
		} catch (const Ice::Exception&) {
			prx = 0;
		}
	}
	return ret;
}

StringSenderPrx
HCIPCClient::getProxy()
{
	if (!ic) {
		try {
			ic = Ice::initialize();
		} catch (const Ice::Exception& ex) {
			cerr << ex << endl;
		} catch (const char* msg) {
			cerr << msg << endl;
		}
	}

	if (!prx) {
		try {
			char proxystr[256];
			sprintf(proxystr, "HCIPCServer:default -h %s -p %d", hostname.c_str(), port);
			Ice::ObjectPrx base = ic->stringToProxy(proxystr);
			prx = hc::StringSenderPrx::checkedCast(base);
			if (!prx)
				throw "Invalid proxy";
		} catch (const Ice::Exception& ex) {
			cerr << ex << endl;
		} catch (const char* msg) {
			cerr << msg << endl;
		}
	}
	return prx;
}

int
HCIPCClient::getRobotStatus( RobotStatus *out_status )
{
	string cmdstr(MAX_CMD_LEN,' ');

	StatusReadCommand(&cmdstr[0]);
	string res = sendString(cmdstr);
	if(ParseRobotStatusResponse(out_status, res.c_str(), res.size()) == RESPONSE_READ_ERR_NOERROR){
		return HCIPC_SUCCESS;
	}

	return HCIPC_FAILURE;
}

int
HCIPCClient::getRobotStatusQuaternion( RobotStatus *out_status )
{
	string cmdstr(MAX_CMD_LEN,' ');

	StatusReadQuaternionCommand(&cmdstr[0]);
	string res = sendString(cmdstr);
	if(ParseRobotStatusQuaternionResponse(out_status, res.c_str(), res.size()) == RESPONSE_READ_ERR_NOERROR){
		return HCIPC_SUCCESS;
	}

	return HCIPC_FAILURE;
}

void HCIPCClient::setHostPort(const std::string &in_host, int in_port)
{
	hostname = in_host;
	port = in_port;

	if (prx) {
		ic->shutdown();
		prx = 0;
		ic = 0;
	}
}
