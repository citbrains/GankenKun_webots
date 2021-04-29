#include <Ice/Ice.h>
#include <HCIPC.h>

using namespace std;
using namespace hc;

int
main(int argc, char* argv[])
{
    int status = 0;
    Ice::CommunicatorPtr ic;
    try {
        ic = Ice::initialize(argc, argv);
        Ice::ObjectPrx base = ic->stringToProxy("HCIPC:default -p 10000");
        StringSenderPrx stringsender = StringSenderPrx::checkedCast(base);
        if (!stringsender)
            throw "Invalid proxy";

        string ret = stringsender->sendString("*0830PA0000");
		cout << ret << std::endl;
    } catch (const Ice::Exception& ex) {
        cerr << ex << endl;
        status = 1;
    } catch (const char* msg) {
        cerr << msg << endl;
        status = 1;
    }
    if (ic)
        ic->destroy();
    return status;
}

