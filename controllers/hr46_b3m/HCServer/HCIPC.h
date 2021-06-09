#pragma once

#ifndef HCIPC_H_
#define HCIPC_H_

#include <Ice/Ice.h>
#include <StringSender.h>
#include "hajime_command.h"

#define HCIPC_SUCCESS 0
#define HCIPC_FAILURE -1
#define HCIPC_DEFAULT_PORT 10130

class HCIPCServer : public hc::StringSender
{
	//TODO: use singleton pattern
public:
	~HCIPCServer();
	virtual std::string sendString(const std::string& s, const Ice::Current&);

	static HCIPCServer* createServer(int argc, char *argv[], int *out_status = NULL, unsigned short port = HCIPC_DEFAULT_PORT);
	void setCallback(std::string (*in_cb)(const std::string&, void *), void *in_ctx) {
		pCallback = in_cb;
		context = in_ctx; //TODO: use boost::function
	}
	void wait();
private:
	HCIPCServer();
	Ice::CommunicatorPtr ic;
	std::string (*pCallback)(const std::string &str, void *context);
	void *context;
};

class HCIPCClient
{
public:
	HCIPCClient(const std::string &in_host = "localhost", unsigned short in_port = HCIPC_DEFAULT_PORT);
	~HCIPCClient();
	std::string sendString(const std::string &str);
	int getRobotStatus(RobotStatus *out_status);
	int getRobotStatusQuaternion(RobotStatus *out_status);
	void setHostPort( const std::string &in_host, int in_port );
private:
    Ice::CommunicatorPtr ic;
	hc::StringSenderPrx getProxy();
	unsigned short port;
	std::string hostname;
};

#endif // HCIPC_H_
