#include <yarp/os/Network.h>
using namespace yarp;

#include "DrumIKinSolverModule.h"

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("drummingEpfl/conf/DrumIKinCartesianSolver/");
    rf.setDefaultConfigFile("config.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    DrumIKinSolverModule mod;

    return mod.runModule(rf);
}
