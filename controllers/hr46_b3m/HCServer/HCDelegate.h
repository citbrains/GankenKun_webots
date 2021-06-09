/*******************************************************************
 * Copyright (c) 2012 Kiyoshi Irie and CIT Brains All Rights Reserved.
 *
 * @file HCDelegate.h
 * @brief Proxy for HCServer
 * @author Kiyoshi Irie
 * @date 2012-06-17
 *******************************************************************/

#pragma once

#include <QtGui/QMainWindow>
#include <QTimer>
#include "ui_HCDelegate.h"
#ifndef Q_MOC_RUN
#include <boost/thread.hpp>
#endif

class HCDelegate : public QMainWindow
{
	Q_OBJECT
public:
	HCDelegate(QWidget *parent = 0, Qt::WFlags flags = 0);
	
public slots:
	void onPushConnect();
	void intervalFunc();

private:
	bool connectToTCPServer(const QString &str, unsigned short port = 20125);

	bool connected;
	void connectSignals();
	boost::thread server_th;
	Ui::HCDelegateMainWindow ui;
	QStatusBar statusBar;
	QTimer timer;
};

