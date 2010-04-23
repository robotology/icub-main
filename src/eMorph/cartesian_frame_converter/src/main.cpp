
#include "cartesian_frame_converter.h"

int main(void)
{
	Network yarp;
	std::string in;
#ifdef _DEBUG
	cout << "Create a C_yarpViewer instance" << endl;
#endif
	C_cframeConverter cFC;
#ifdef _DEBUG
	cout << "Register the instance to the callback" << endl;
#endif
	cFC.useCallback();
#ifdef _DEBUG
	cout << "Open the port for listening" << endl;
#endif
	cFC.open("/converter/in");
#ifdef _DEBUG
	cout << "Connect the 'read' and 'write port'" << endl;
#endif
	//Network::connect("/DV128/out","/converter/in");
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
	cFC.close();
#ifdef _DEBUG
	cout << "Unregister form the callback" << endl;
#endif
    cFC.disableCallback();
    return 0;
}
