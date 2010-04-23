#include "logpolar_frame_converter.h"
//#include <iostream>
//#include <String>
//#include <cv.h>
//#include <cvaux.h>
//#include <highgui.h>
int main(void)
{
	Network yarp;
	std::string in;
#ifdef _DEBUG
	cout << "Create a C_yarpViewer instance" << endl;
#endif
	C_lFrameConverter lFC;
#ifdef _DEBUG
	cout << "Register the instance to the callback" << endl;
#endif
	lFC.useCallback();
#ifdef _DEBUG
	cout << "Open the port for listening" << endl;
#endif
	lFC.open("/logconverter_add/in");
#ifdef _DEBUG
	cout << "Connect the 'read' and 'write port'" << endl;
#endif
	Network::connect("/DV128/out","/logconverter_add/in");
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
	lFC.close();
#ifdef _DEBUG
	cout << "Unregister form the callback" << endl;
#endif
    lFC.disableCallback();
    return 0;
}
