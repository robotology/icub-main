/** 
\defgroup icub_trackerexpressions trackerExpressions
@ingroup icub_module   

A module that controls the iCub facial expressions based on whether a tracker is tracking the object of interest or not.

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
#include <iCub/trackerExpressions.hpp>

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[])
{
    Network yarp;  //set up yarp.
    TrackerExpressions moduleID; //instanciate the module.
    moduleID.setName("/TrackerExpressions"); // set default name for the module.
    return moduleID.runModule(argc,argv); //execute the module.
}
