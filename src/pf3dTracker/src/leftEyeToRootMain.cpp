/** 
\defgroup leftEyeToRoot leftEyeToRoot
 
@ingroup icub_module  
 
A kinematic translator which returns the equivalent 3-d position
wrt the root frame of the robot starting from a given 3-d 
position in the frame attached to the left eye.

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
