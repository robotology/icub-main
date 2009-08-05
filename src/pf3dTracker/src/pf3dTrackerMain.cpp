//#######################
//# PF3DTrackerMain.cpp #
//#######################
//
//
//

#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <iCub/pf3dTracker.hpp>

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[])
{
    Network yarp;  //set up yarp.
    PF3DTracker tracker; //instanciate the tracker.
    tracker.setName("/PF3DTracker"); // set default name for the tracker.
    return tracker.runModule(argc,argv); //execute the tracker.
}
