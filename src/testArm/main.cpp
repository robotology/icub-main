// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
@ingroup icub_module
\defgroup icub_testArm testArm

This module shows how to get interfaces to control the arm.

\section intro_sec Description
This module shows how to acquire interfaces which provide information 
about the arm (encoder values) and control the motors. This can be 
easily generalized to other parts of the robot.

\section lib_sec Libraries
YARP.

\section parameters_sec Parameters
--robot: set robot name (e.g. icub or icubSim)
 
\section portsa_sec Ports Accessed
Ports instantiated by \ref icub_iCubInterface.
                      
\section portsc_sec Ports Created
/test/client/rpc:o
/test/client/command:i
/test/client/state:i

These output ports connect directly to the remote 
counterparts in \ref icub_iCubInterface.

\section in_files_sec Input Data Files
None

\section out_data_sec Output Data Files
NOne
 
\section conf_file_sec Configuration Files
None

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
testArm --robot icub

\author Lorenzo Natale

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/testArm/main.cpp.
**/


#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

int main(int argc, char *argv[]) 
{
    Network yarp;

    Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return -1;
    }
    std::string robotName=params.find("robot").asString().c_str();
    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/right_arm";

    std::string localPorts="/test/client";

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionControl *pos;
    IEncoders *encs;

    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    int nj=0;
    pos->getAxes(&nj);
    Vector encoders;
    Vector command;
    Vector tmp;
    encoders.resize(nj);
    tmp.resize(nj);
    command.resize(nj);
    
    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = 50.0;
    }
    pos->setRefAccelerations(tmp.data());

    for (i = 0; i < nj; i++) {
         tmp[i] = 10.0;
    }
    pos->setRefSpeeds(tmp.data());
    command=0;
    pos->positionMove(command.data());
    
    bool done=false;

    while(!done)
        {
            pos->checkMotionDone(&done);
            Time::delay(0.1);
        }

    int times=0;
    while(true)
    {
        times++;
        if (times%2)
        {
             command[0]=50;
             command[1]=20;
             command[2]=-10;
             command[3]=50;
        }
        else
        {
            command=0;
        }

        pos->positionMove(command.data());

        int count=50;
        while(count--)
            {
                Time::delay(0.1);
                encs->getEncoders(encoders.data());
                printf("%.1lf %.1lf %.1lf %.1lf\n", encoders[0], encoders[1], encoders[2], encoders[3]);
            }
    }

    robotDevice.close();
    
    return 0;
}
