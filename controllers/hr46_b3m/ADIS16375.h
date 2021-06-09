#pragma once
#include <KSerialPort.h>
#include <string>
#include <iostream>
#include <sstream>
#include <bitset>

#include <boost/math/constants/constants.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

static boost::mutex lock_imu_recv;

class ADIS16375
{
public:
	ADIS16375() : terminated(false) {
		for(int i=0; i<9; i++) {
			data[i] = 0;
		}
	}
	//accel(3), angvel(3), dir(3)
	void setPortName(const char *in_pname) {
		portname = in_pname;
	}
	void start() {
		mthread = boost::thread(boost::bind(&ADIS16375::thread_run, this));
	}
	void getAccel(double *ax, double *ay, double *az) {
		boost::mutex::scoped_lock look(lock_imu_recv);
		*ax = data[0];	
		*ay = data[1];	
		*az = data[2];
	}
	void getAngVel(double *ax, double *ay, double *az) {
		boost::mutex::scoped_lock look(lock_imu_recv);
		*ax = data[3];	
		*ay = data[4];	
		*az = data[5];
	}
private:	
	enum {
		DATA_LENGTH = 25,
	};
	short readhex_4char(char *buf) {
		short val;
		char buf2[20];
		memset(buf2, 0, sizeof(buf2));
		strncpy(buf2, buf, 4);
		sscanf(buf2, "%hx", &val);
		return val;
	}

	void thread_run() {
		boost::system::error_code err = port.open(portname);
		
		if (err) {
			std::cerr << portname << " open failed" << std::endl;
			return;
		}
		port.setBaudRate(115200);
		port.setParity(KSerialPort::KS_PARITY_NONE);
//		double pi = boost::math::constants::pi<double>();
		double gyro_lsb  = 0.01311;  // deg/sec per LSB
		double dir_lsb   = 0.01311;  // deg per LSB
		double accel_lsb = 0.8192;   // mg per LSB
		int error_count = 0;
		int data_len = DATA_LENGTH;
		while(!terminated) {
			boost::system::error_code err;
			char buf[DATA_LENGTH+1];
			port.read(&buf[DATA_LENGTH - data_len], data_len, err);
			if (strlen(buf) == DATA_LENGTH && buf[DATA_LENGTH-1] == '\n') {
				boost::mutex::scoped_lock look(lock_imu_recv);
				for(int i=0; i<3; i++) {
					double rval = readhex_4char(&buf[i*4]) * accel_lsb * 9.81 / 1000;
					data[i] = rval;
				}
				for(int i=3; i<6; i++) {
					double rval = readhex_4char(&buf[i*4]) * gyro_lsb;
					data[i] = rval;
				}
				memset(buf, 0, sizeof(buf));
				data_len = DATA_LENGTH;
				if (error_count > 0){
					FILE *fp = fopen("/var/tmp/error.txt","a");
					if (fp != NULL){
						time_t now = time(NULL);
						struct tm *pnow = localtime(&now);
						fprintf(fp, "%d:%d:%d IMU ERROR %d TIMES\r\n",
						pnow->tm_hour, pnow->tm_min, pnow->tm_sec,
						error_count);
						fclose(fp);
						error_count = 0;
					}
				}
			} else {
				std::stringstream ss(buf);
				std::string buffer;
				while (std::getline(ss, buffer));
				memset(buf, 0, sizeof(buf));
				if (buffer.size() != DATA_LENGTH){
					for(int i = 0; i < buffer.size(); i ++) buf[i] = buffer[i];
					data_len = DATA_LENGTH - buffer.size();
				}
				for(int i = 0; i < 6; i ++) data[i] = 0;
				error_count ++;
			}
		}
	}
	KSerialPort port;
	std::string portname;
	bool terminated;
	double data[9];
	boost::thread mthread;
};

