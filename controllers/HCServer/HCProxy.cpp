#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <KTCPSocket.h>
#include <HCIPC.h>

using namespace std;
using boost::asio::ip::tcp;

typedef boost::shared_ptr<tcp::socket> socket_ptr;

static const int max_length = 10240;

void session(socket_ptr sock)
{
	HCIPCClient client;
	try
	{
		for (;;)
		{
			char data[max_length];

			boost::system::error_code error;
			size_t length = sock->read_some(boost::asio::buffer(data), error);
			if (error == boost::asio::error::eof)
				break; // Connection closed cleanly by peer.
			else if (error)
				throw boost::system::system_error(error); // Some other error.

			string cmd = string(data, length) + '0';
			//std::string res = "hoge";
			std::string res = client.sendString(cmd) + '0';
			std::cout <<  cmd << std::endl;;
			boost::asio::write(*sock, boost::asio::buffer(res, res.size()));
			std::cout <<  res << std::endl;;
		}
	}
	catch (std::exception& e)
	{
		std::cerr << "Exception in thread: " << e.what() << "\n";
	}
}

int main()
{
	boost::asio::io_service ios;
	KTCPSocket tcpsocket;

	tcp::acceptor a(ios, tcp::endpoint(tcp::v4(), 20125));
	for (;;)
	{
		socket_ptr sock(new tcp::socket(ios));
		a.accept(*sock);
	    boost::thread t(boost::bind(session, sock));
	}
	return 0;
}

