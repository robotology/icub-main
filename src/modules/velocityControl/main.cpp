// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Copyright (C) 2008 RobotCub Consortium
// Author: Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

/**
*
@ingroup icub_module
\defgroup icub_velocityControl velocityControl

Perform position control using a velocity control loop.

\section intro_sec Description
This module performs joint space position control using a velocity loop. 
It basically implements a pid control converting a position error into a 
velocity signal that sent to the control board. It allows commanding 
trajectories at fine temporal scale (~10-20ms).

Note: for safey reasons the module initially starts with 0 gain and 
0 max velocity, so you first have to set them to correct values.

Use yarp rpc and type for example:
\code
gain 0 10
svel 0 10
\endcode

This set gain and maximum velocity to 10 for the joint 0.

There are maximum values for gain and velocity set in the code 
(check velControlThread.cpp). At the moment they are set to 10 and 50 deg/sec respectively.

\section lib_sec Libraries
YARP

\section parameters_sec Parameters

--robot: specifies the name of the robot. It will be used to form 
the names of the ports created and accessed by module.

--part: part to control (e.g. head,arm_right, arm_left, lef_right...), it 
will be used to form the names of the ports created and accessed by the 
module.

--period: the periodicity of the velocity control loop, in milliseconds 
(possible values are 20 or 10).

\section portsa_sec Ports Accessed
It assumes \ref icub_iCubInterface runs. It accesses velocity and 
encoder ports created by iCubInterface.

\section portsc_sec Ports Created
The module instantiates a control_board device, which opens the
usual ports. The pattern of the name is as follow:
- /robot/vc/part/client/rpc:o
- /robot/vc/part/client/command:o
- /robot/vc/part/client/state:i

where robot is the name of the robot as specified with --robot and 
part is the required part as specified with --part (see below).

- /robot/vc/part/input: input port of the module
    -   [susp]: suspend the controller (command zero velocity)
    -   [run]: start (and resume after being suspended) the controller
    -   [quit]: quit the module (exit)
    -   [set] j p: move joint j to p (degrees)
    -   [svel] j v: set maximum speed for joint j to v (deg/sec)
    -   [gain] j k: set gain for joint j to k
    -   [help] to get this help

Note: commands to the module through /robot/vc/part/input are not
fast, use /robot/vc/part/fastCommand or /robot/vc/part/command instead.

- /robot/vc/part/fastCommand: accept a vector of positions as a bottle 
of int-double pairs, in the form j,p (where j is the joint and p is the 
requested position). Messages in this format are deprecated, 
use /robot/vc/part/command instead.

- /robot/vc/part/command: a a vector of positions which specifies the references
for each joint.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.

\section conf_file_sec Configuration Files
None.

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module

velocityControl --robot icub --part head --period 10

Starts the velocityControl module using the robot icub to control
the head. The period of the velocity control loop will be 10ms.

\author Lorenzo Natale

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/velocityControl/main.cpp.
**/

#include <stdio.h>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include "velControlThread.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <string.h>

/*default rate for the control loop*/
const int CONTROL_RATE=20;

using namespace yarp::os;
using namespace yarp::dev;

class VelControlModule: public RFModule {
private:
    PolyDriver driver;
    velControlThread *vc;
    char partName[255];
  
    //added by ludovic
    Port input_port;
    ////
public:
  
    virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
    {
        reply.clear(); 
        yDebug("receiving command from port\n");
        int index = 0;
        int cmdSize = command.size();
    
        while(cmdSize>0)
            {
                switch(command.get(index).asVocab())
                {
                    case yarp::os::createVocab('s','u','s','p'):
                        reply.addVocab(Vocab::encode("ack"));
                        vc->halt();
                        cmdSize--;
                        index++;
                    break;
                    case yarp::os::createVocab('r','u','n'):
                        reply.addVocab(Vocab::encode("ack"));
                        vc->go();
                        cmdSize--;
                        index++;
                    break;
                    //this set current position reference
                    case yarp::os::createVocab('s','e','t'):
                        if (command.size()>=3)
                            {
                                int i=command.get(index+1).asInt();
                                double pos=command.get(index+2).asDouble();
                                vc->setRef(i, pos);
                                index +=3;
                                cmdSize-=3;
                            }
                        else
                            {
                                cmdSize--;
                                index++;
                                yError("Invalid set message, ignoring\n");
                            }
                        reply.addVocab(Vocab::encode("ack"));
                    break;
                    //this set maximum velocity (limiter)
                    case yarp::os::createVocab('s','v','e','l'):
                        if(command.size()>=3)
                            {
                                int i=command.get(index+1).asInt();
                                double vel = command.get(index+2).asDouble();
                                vc->setVel(i,vel);
                                index += 3;
                                cmdSize-=3;;
                                reply.addVocab(Vocab::encode("ack"));
                            }
                        else
                            {
                                cmdSize--;
                                index++;
                                yError("Invalid set vel message, ignoring\n");
                                reply.addVocab(Vocab::encode("fail"));
                            }
                    break;
                    case yarp::os::createVocab('g','a','i','n'):
                        if(command.size()>=3)
                            {
                                int i=command.get(index+1).asInt();
                                double gain = command.get(index+2).asDouble();
                                vc->setGain(i,gain);
                                index+=3;
                                cmdSize-=3;
                                reply.addVocab(Vocab::encode("ack"));
                            }
                        else
                            {
                                cmdSize--;
                                index++;
                                yError("Invalid set gain message, ignoring\n");
                                reply.addVocab(Vocab::encode("fail"));
                            }
                    break;
                    case yarp::os::createVocab('h','e','l','p'):
                        fprintf(stdout,"VelocityControl module, valid commands are:\n");
                        fprintf(stdout,"-   [susp]         suspend the controller (command zero velocity)\n");
                        fprintf(stdout,"-   [run]          start (and resume after being suspended) the controller\n");
                        fprintf(stdout,"-   [quit]         quit the module (exit)\n");
                        fprintf(stdout,"-   [set]  <j> <p> move joint j to p (degrees)\n");
                        fprintf(stdout,"-   [svel] <j> <v> set maximum speed for joint j to v (deg/sec)\n");
                        fprintf(stdout,"-   [gain] <j> <k> set P gain for joint j to k\n");
                        fprintf(stdout,"-   [help] to get this help\n");
                        fprintf(stdout,"\n typical commands:\n gain 0 10\n svel 0 10\n run\n set 0 x\n\n");
                        
                        reply.addVocab(Vocab::encode("many"));
                        reply.addVocab(Vocab::encode("ack"));
                        reply.addString("VelocityControl module, valid commands are:");
                        reply.addString("-   [susp]         suspend the controller (command zero velocity)");
                        reply.addString("-   [run]          start (and resume after being suspended) the controller");
                        reply.addString("-   [quit]         quit the module (exit)");
                        reply.addString("-   [set]  <j> <p> move joint j to p (degrees)");
                        reply.addString("-   [svel] <j> <v> set maximum speed for joint j to v (deg/sec)");
                        reply.addString("-   [gain] <j> <k> set P gain for joint j to k");
                        reply.addString("-   [help] to get this help");
                        reply.addString("\n typical commands:\n gain 0 10\n svel 0 10\n run\n set 0 x\n\n");
                    break;
                    default:
                        yError("Invalid command, ignoring\n");
                        reply.addVocab(Vocab::encode("fail"));
                        //cmdSize--;
                        //index++;
                        //return respond(command, reply); // call default
                    break;
                }
                return true;
            }

        return false;
    }
  
    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        Property options;
        options.fromString(rf.toString());
        char robotName[255];
        options.put("device", "remote_controlboard");
        if(options.check("robot"))
            strncpy(robotName, options.find("robot").asString().c_str(),sizeof(robotName));
        else
            strncpy(robotName, "icub", sizeof(robotName));
    
        if(options.check("part"))
            {
                char tmp[800];
                snprintf(tmp, sizeof(tmp), "/%s/vc/%s/client",
                        robotName,
                        options.find("part").asString().c_str());
                options.put("local",tmp);
        
                snprintf(tmp, sizeof(tmp),"/%s/%s",
                        robotName,
                        options.find("part").asString().c_str());
                options.put("remote", tmp);
        
                snprintf(tmp, sizeof(tmp), "/%s/vc/%s/input",
                        robotName,
                        options.find("part").asString().c_str());
                input_port.open(tmp);
            
                options.put("carrier", "mcast");
            
                attach(input_port);
            }
        else
            {
                yError("Please specify part (e.g. --part head)\n");
                return false;
            }
        ////end of the modif////////////
    
        if (!driver.open(options))
            {
                yError("Error opening device, check parameters\n");
                return false;
            }

        ///we start the thread
        int period = CONTROL_RATE;
        if(options.check("period"))
            period = options.find("period").asInt();
        
        yInfo("control rate is %d ms",period);

        if (!options.check("part"))
            return false;
        
        snprintf(partName, sizeof(partName), "%s", options.find("part").asString().c_str());

        vc=new velControlThread(period);
        vc->init(&driver, partName,
                 robotName);

        vc->start();
        return true;
    }

    virtual bool close()
    {
        yInfo("Closing module [%s]\n", partName);
        vc->stop();
        vc->halt();
        delete vc;
        yInfo("Thead [%s] stopped\n", partName);

        driver.close();
        input_port.close();

        yInfo("Module [%s] closed\n", partName);
        return true;
    }

    bool updateModule()
    {
        return true;
    }

};

int main(int argc, char *argv[])
{
    Network yarp;
    VelControlModule mod;
    ResourceFinder rf;
    
    rf.configure(argc, argv);
    if (mod.configure(rf)==false) return 1;
    mod.runModule();
    yInfo("Main returning\n");
    return 0;
}

