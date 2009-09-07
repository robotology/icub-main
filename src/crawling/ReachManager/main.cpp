
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>
//
//#include <iCub/iKinSlv.h>
//
//#include <iostream>
//#include <iomanip>
//#include <string>
//
//using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
//using namespace yarp::math;
//using namespace iKin;

#include "ReachManagerModule.h"

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

	ReachManagerModule reachManagerModule;
	
	return reachManagerModule.runModule(argc, argv);;
}


      
