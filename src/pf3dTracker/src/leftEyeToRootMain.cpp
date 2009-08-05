//#############################
//# LeftEyeToRootMain.cpp #
//#############################
//
//
//

#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <iCub/leftEyeToRoot.hpp>

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[])
{
    Network yarp;  //set up yarp.
    LeftEyeToRoot moduleID; //instanciate the module.
    moduleID.setName("/LeftEyeToRoot"); // set default name for the module.
    return moduleID.runModule(argc,argv); //execute the module.
}
