/** 
\defgroup icub_pf3dBottomup pf3dBottomup
@ingroup icub_module   

A bottom-up approach for generating particles for the \ref icub_pf3dTracker "pf3dTracker" module

Copyright (C) 2010 RobotCub Consortium
 
Author: Martim Brandao

Note: Should you use or reference my work on your own research, please let me know (mbrandao _AT_ isr.ist.utl.pt)

Image sequence as input, N particles (3D position of balls) as output.

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
<b>For an explanation on how to configure the tracker and bottom up modules, how to connect them, how to run them through the application manager, etc., please have a look at <A HREF="http://mediawiki.isr.ist.utl.pt/wiki/3D_ball_tracker">this page</A>.</b>

*/

// yarp
#include <yarp/os/Network.h>

// iCub
#include <iCub/pf3dBottomup.hpp>

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp::os::Network::checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("pf3dBottomup");
    rf.setDefaultConfigFile("pf3dBottomup.ini");
    rf.configure(argc, argv);

    pf3dBottomup bottomup;
    bottomup.setName("/pf3dBottomup");
    return bottomup.runModule(rf);
}

