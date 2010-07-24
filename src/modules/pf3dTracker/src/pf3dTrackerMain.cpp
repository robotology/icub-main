/** 
\defgroup icub_pf3dTracker pf3dTracker
@ingroup icub_module   

An object 3d position tracker implementing the particle filter.

Copyright (C) 2009 RobotCub Consortium
 
Author: Matteo Taiana

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
... 
 
\section lib_sec Libraries 
... 
 
\section parameters_sec Parameters
... 
 
\section portsa_sec Ports Accessed
... 
 
\section portsc_sec Ports Created 
... 
 
\section in_files_sec Input Data Files
...

\section out_data_sec Output Data Files 
...

\section tested_os_sec Tested OS
...

\author Matteo Taiana
*/ 

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


