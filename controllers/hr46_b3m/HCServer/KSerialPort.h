/*******************************************************************
 * Copyright (c) 2012 Kiyoshi Irie  All Rights Reserved.
 *
 * @file KSerialPort.h
 * @brief Serial Port I/O
 * @author Kiyoshi Irie
 * @date 2012-02-23
 *******************************************************************/

#ifndef KSERIALPORT_H_
#define KSERIALPORT_H_

#include <iostream>
#include <boost/utility.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

class KSerialPort : public boost::noncopyable
{
public:
	KSerialPort() : mport(io_srv) {
	}

	typedef enum {
		KS_PARITY_NONE,
		KS_PARITY_EVEN,
		KS_PARITY_ODD
	} ParityType;

	boost::system::error_code open(const std::string portname) {
		boost::system::error_code err;
		mport.open(portname, err);
		if (!err) {
			mport.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
			mport.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		}
		return err;
	}
	void close() {
		mport.close();
	}
	void setBaudRate(int rate) {
		mport.set_option(boost::asio::serial_port_base::baud_rate(rate));
	}
	void setParity(ParityType p) {
		switch(p) {
		case KS_PARITY_NONE:
			mport.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
			break;
		case KS_PARITY_EVEN:
			mport.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even));
			break;
		case KS_PARITY_ODD:
			mport.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::odd));
			break;
		}
	}
#if 0
	void set_option(const SettableSerialPortOption& option) {
		mport.set_option(option);
	}
#endif
	int read_some(std::vector<char> &buffers, boost::system::error_code& ec) {
		return mport.read_some(boost::asio::buffer(buffers, buffers.size()), ec);
	}
	int read_some(char *buffers, int len, boost::system::error_code& ec) {
		return mport.read_some(boost::asio::buffer(buffers, len), ec);
	}

	void timeout_callback(boost::asio::serial_port& port, const boost::system::error_code& error) {
		if (!error) {
			//timeout
			port.cancel();
			std::cout << "serial read timeout" << std::endl;
		}
	}
	int read(char *buffers, unsigned int len, boost::system::error_code& ec) {
		size_t numread = 0;
		boost::asio::deadline_timer timeout(io_srv);
		while (numread < len) {
			size_t async_numread = 0;
			//std::cout << "i";

			timeout.expires_from_now(boost::posix_time::milliseconds(5));
			timeout.async_wait(boost::bind(&KSerialPort::timeout_callback, this, boost::ref(mport), boost::asio::placeholders::error));
			boost::asio::async_read(mport, boost::asio::buffer(&buffers[numread], len-numread), boost::asio::transfer_at_least(len-numread), boost::bind(&KSerialPort::async_read_callback, this, boost::ref(timeout), boost::ref(async_numread), _1, _2));
			io_srv.run();
			//std::cout << "/o ";
			if (async_numread == 0) {
				break;
			}
			numread += async_numread;
		}
		io_srv.reset();
		//std::cout << "\nread: " << std::string(buffers, numread) << std::endl;
		return numread;
	}
	int readLine(char *buffers, unsigned int len, boost::system::error_code& ec) {
		size_t numread = 0;
		boost::asio::deadline_timer timeout(io_srv);
		while (numread < len) {
			size_t async_numread = 0;

			timeout.expires_from_now(boost::posix_time::milliseconds(100));
			timeout.async_wait(boost::bind(&KSerialPort::timeout_callback, this, boost::ref(mport), boost::asio::placeholders::error));
			boost::asio::async_read(mport, boost::asio::buffer(&buffers[numread], 1), boost::asio::transfer_at_least(1), boost::bind(&KSerialPort::async_read_callback, this, boost::ref(timeout), boost::ref(async_numread), _1, _2));
			io_srv.run();
			if (async_numread == 0) {
				break;
			} else if (buffers[numread] == '\n') {
				break;
			}
			numread += async_numread;
		}
		io_srv.reset();
		return numread;
	}
	int write_some(const std::vector<char> &buffers, boost::system::error_code& ec) {
		return mport.write_some(boost::asio::buffer(buffers, buffers.size()), ec);
	}
	int write_some(const char *buffers, int len, boost::system::error_code& ec) {
		return mport.write_some(boost::asio::buffer(buffers, len), ec);
	}

private:
	void async_read_callback(boost::asio::deadline_timer &timeout, std::size_t &out_numread, const boost::system::error_code& error, std::size_t bytes_transferred) {
		timeout.cancel();
		if (!error) {
			out_numread = bytes_transferred;
		}
	}

private:
	boost::asio::io_service io_srv;
	boost::asio::serial_port mport;
};

#endif // KSERIALPORT_H_

