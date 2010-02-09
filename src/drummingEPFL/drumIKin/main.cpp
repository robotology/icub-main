#include <yarp/os/Network.h>
using namespace yarp;

#include "DrumIKin.h"

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

	DrumIKin drumIKin;
	
	return drumIKin.runModule(argc, argv);
}


      
