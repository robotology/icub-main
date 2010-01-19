
/**
*
@ingroup icub_module
\defgroup icub_retinalSlipBridge retinalSlipBridge

Covert the pixel value of a target in azimut angle by using the camera parameters, and send this value to a velocityObserver module.

\section intro_sec Description

This module takes in input the pixel value of the tracked object from 3D Particle Filter Tracker module and converts this value in azimut angle. Then send this value to a velocityObserver module.


\section lib_sec Libraries

- YARP libraries.

\section parameters_sec Parameters

none.

\section portsa_sec Ports Accessed

- /icub/PF3DTracker/dataOut

\section portsc_sec Ports Created

Output ports:

- /portForVel: streams out a yarp::sig::vector which contains the azimut angle of the target.

Input ports:

- /portInFromPf3 : input port to take the pixel value of the target

\section in_files_sec Input Data Files

None.

\section out_data_sec Output Data Files

None.
 
\section conf_file_sec Configuration Files

None.

\section tested_os_sec Tested OS

Linux and Windows.

\section example_sec Example Instantiation of the Module

retinalSlipBridge

\author Egidio Falotico and Davide Zambrano

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/retinalSlipBridge/main.cpp.
**/




// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include "camera.h"

#include <yarp/sig/Vector.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;


//
int main(int argc, char *argv[]) 
{

Network yarp;
BufferedPort<Bottle> portInFromPf3;

	portInFromPf3.open("/portInFromPf3");
	Network ::connect("/icub/PF3DTracker/dataOut","/portInFromPf3");

    BufferedPort <Vector> port;
    port.open("/portForVel");
    Network ::connect("/portForVel","/objVel/pos:i");

    while (1){    
    Bottle* input =portInFromPf3.read();

    double xpix=input->get(4).asDouble();
    bool res=false;
    
    Property propCamera;
    res=propCamera.fromConfigFile("/camCalib.ini");

    Camera *camera=new Camera();

    camera->open(propCamera);
    camera->setImageSize(320,240);

    double az=0;
    double el=0;
    camera->pixel2angle(xpix,0,az,el);

    az=-az;

    Vector & processed = port.prepare();    
    processed.resize(1);
    processed[0]=az;
    port.write();


    
     
    
    
    }



return 0;
}
