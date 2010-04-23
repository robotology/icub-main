#include "yarp_viewer.h"
#include <cstring>

int main(int argc, char *argv[])
{
	Network yarp;
	std::string in;
    string arg;
    if(argc > 1)
        arg = argv[1];
    else
        arg = "logpolar";
#ifdef _DEBUG
	cout << "Create a C_yarpViewer instance" << endl;
#endif
	C_cFramePrinter cFP;
#ifdef _DEBUG
	cout << "Register the instance to the callback" << endl;
#endif
	cFP.useCallback();
#ifdef _DEBUG
	cout << "Open the port for listening" << endl;
#endif
    string _port = "/"+arg+"/in";
	cFP.open(_port.c_str());
#ifdef _DEBUG
	cout << "Connect the 'read' and 'write port'" << endl;
#endif
    arg = "/image/"+arg;
	Network::connect(arg.c_str(),_port.c_str());
    bool end = false;
	//cvNamedWindow("Events", 0);
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
#ifdef _DEBUG
	cout << "Close the listening port" << endl;
#endif
	cFP.close();
#ifdef _DEBUG
	cout << "Unregister form the callback" << endl;
#endif
    cFP.disableCallback();
    return EXIT_SUCCESS;
}
