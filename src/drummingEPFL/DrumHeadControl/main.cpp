
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


#include "DrumHeadControl.h"

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

	DrumHeadControl drumHeadControl;
	
	return drumHeadControl.runModule(argc, argv);;
}


      
