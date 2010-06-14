#include <yarp/os/Network.h>
using namespace yarp;

#include "drumPositionTester.h"

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

	DrumPositionTester drumPositionTester;
	
	return drumPositionTester.runModule(argc, argv);
}


      
