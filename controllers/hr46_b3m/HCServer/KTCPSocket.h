#pragma once
#include <boost/array.hpp>
#include <boost/asio.hpp>

class KTCPSocket{
private:
	boost::asio::io_service io_service;
	boost::asio::ip::tcp::socket sock;

public:
	static const size_t BUF_SIZE = 10240;
	KTCPSocket()
		: sock(io_service){
	}
	~KTCPSocket(){
		close();
	}
	bool is_open() {
		return sock.is_open();
	}
	void connect(const std::string &ipaddr, unsigned short port){
		sock.open(boost::asio::ip::tcp::v4());
		sock.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ipaddr), port));
	}
	void close(){
		sock.close();
	}

	size_t send(const std::string &str){
		size_t len = 0;
		len = sock.send(boost::asio::buffer(str));
		return len;
	}
	size_t read_some(std::string &response){
		size_t len = 0;
		boost::array<char, BUF_SIZE> buf;
		len = sock.read_some(boost::asio::buffer(buf));
		response.assign(buf.data(), len);
		return len;
	}

	bool bind( const std::string &ipaddr, unsigned short port ) 
	{
		sock.open(boost::asio::ip::tcp::v4());
		sock.bind(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ipaddr), port));
		return true;
	}

	size_t receive(std::string &response){
		boost::array<char, BUF_SIZE> buf;
		size_t len = sock.receive(boost::asio::buffer(buf));
		response.assign(buf.data(), len);
		return len;
	}
};

