/*******************************************************************
 * Copyright (c) 2011 Kiyoshi Irie and CIT Brains All Rights Reserved.
 *
 * @file QHuSendCom.cpp
 * @brief Hajime Command Sender
 * @author Kiyoshi Irie
 * @date 2011-12-08
 *******************************************************************/

#include "KTCPSocket.h"
#include "HCDelegate.h"
#include "hajime_command.h"
#include <HCIPC.h>

using namespace std;

static KTCPSocket tcpsocket;
static HCIPCServer *hcserver;
static boost::mutex mut;
static string cmd;
static string res;
static bool received;

string recvCommand(const string &str, void *context)
{
	if (str.size() > 0) {
		boost::mutex::scoped_lock look(mut);
		cmd = str;
		received = false;
	}
	while (!received) {
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	}
	{
		boost::mutex::scoped_lock look(mut);
		return res;	
	}
}

static std::string handleHCRequest(const std::string& str, void *ctx)
{
	std::string res;
	int len = str.size();
	if (len > 0) {
		boost::mutex::scoped_lock look(mut);
		cmd = str;
		received = false;
	}
	while (!received) {
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	}
	{
		boost::mutex::scoped_lock lock(mut);
		return res;	
	}
}

static void hcserver_thread()
{
	hcserver = HCIPCServer::createServer(0, 0);
	hcserver->setCallback(handleHCRequest, 0);
	hcserver->wait();
}

HCDelegate::HCDelegate(QWidget *parent, Qt::WFlags flags)
: QMainWindow(parent, flags), connected(false)
{
	ui.setupUi(this);
	connectSignals();
	server_th = boost::thread(&hcserver_thread);
}

void
HCDelegate::connectSignals()
{
	bool ret;
	ret = connect(ui.pushConnect, SIGNAL(clicked()), this, SLOT(onPushConnect()));
	assert(ret);
	ret = connect(&timer, SIGNAL(timeout()), this, SLOT(intervalFunc()));
	assert(ret);
}

void HCDelegate::onPushConnect()
{
	if (!connected) {
		if (ui.radioTCP->isChecked()) {
			connectToTCPServer(ui.linePort->text());
		} else {
			assert(ui.radioSerialPort->isChecked());
			//TODO: imeplent it later
		}
		ui.pushConnect->setText("Disconnect");
	} else {
		boost::mutex::scoped_lock lock(mut);
		tcpsocket.close();
		ui.pushConnect->setText("Connect");
	}
}

bool HCDelegate::connectToTCPServer( const QString &str, unsigned short port )
{
	boost::mutex::scoped_lock lock(mut);
	tcpsocket.close();
	tcpsocket.connect(str.toStdString(), port);
	timer.setInterval(3);
	timer.start();
	return true;
}

void HCDelegate::intervalFunc()
{
	boost::mutex::scoped_lock lock(mut);
	if (cmd.size() > 0) {
		tcpsocket.send(cmd + '\n');
		tcpsocket.read_some(res);
		cmd = "";
		received = true;
	}	
}
