// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 *
@ingroup icub_lasaImitation_module
\defgroup icub__lasaImitation_VelocityControllerModule Extended Velocity Controller Module

This module is an revisited version of the velocity controler module with some useful extensions for realtime control.

\section intro_sec Description

This module is an revisited version of the velocity controler module with some useful extensions for realtime control. It mainly allows:

- To instantiate several body parts simultaneously and to control them through a single set of i/o ports
- To simultaneously send position and velocity order which will be executed according to some gains
- To help solving the shoulder coupling problem through a high level "reverse egnineered hack" of the low-level controller. It is not perfect but greatly helps for motions involving the three shoulder joints.
- To be protected against communication breakdown, i.e., the module assumes continuous streams of orders, so if some timeout occurs, the module stops the robot. This module should preferentially be run on the pc104.


\section dependencies_sec Dependencies

- YARP

\section parameters_sec Parameters

\verbatim
--robot <string>:      the name ofthe robot to be controlled (icubSim by default)
--name <string>:       the module base name for port creation (VelocityController by default)
--period <double>:     control loop period (0.01 sec by default)
--part: <string list>: which parts do you want to instantiate (multiple instances is possible)?
                        [right_arm | left_arm | right_leg | left_leg | head | torso]+
--amp:                 optional speed amplitude factor (1.0 by default)
\endverbatim

\section portsa_sec Ports Accessed

The module assumes a robot interface running (for instance either iCubInterface or the simulator). It accesses velocity and encoder ports.

\section portsc_sec Ports Created

The module instantiates a control_board device for each instantiated body part. This opens the usual ports according to the following pattern:
    - /moduleName/part/rpc:o
    - /moduleName/part/command:o
    - /moduleName/part/state:i

Input ports:

    - /moduleName/rpc: single general command entry port to the module
          - [run]: run the controller
          - [susp]: suspend the controller (command zero velocity)
          - [rest]: move back to the original robot position (that seen first when the controller was run)
          - [quit]: quit the module (exit)
          - [mask all]: set active mask to all joints (useful for setting position kp and kd gain values)
          - [mask none]: set inactive mask to all joints
          - [mask] m1 m2 ...mn: set joint mask list of values
          - [kp] kp: set given kp values to active masked joints (2.0 by default)
          - [kd] kd: set given kd values to active masked joints (0.0 by default)

    - /moduleName/targetPosition: input target position for all joints (The order follow that given in the parameters section)
    - /moduleName/targetVelocity: input target velocity for all joints

Output ports:

    * /moduleName/position: output current position for all joints
    * /moduleName/velocity: output current velocity for all joints (the velocity feedback shoud be turned on on the interface configuration file)


\section in_files_sec Input Data Files

None

\section out_data_sec Output Data Files

None

\section conf_file_sec Configuration Files

None

\section tested_os_sec Tested OS

Linux

\section example_sec Example Instantiation of the Module

\verbatim
VelocityControllerExt --robot icub --period 0.02 --right_arm --left_arm
\endverbatim

\author Eric Sauser

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/lasaImitation/VelocityControllerExt/src/VelocityControllerModule.h.
**/


#ifndef VELOCITYCONTROLLERMODULE_H_
#define VELOCITYCONTROLLERMODULE_H_

#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;

#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

#include "VelocityController.h"

class VelocityControllerThread;

class VelocityControllerModule: public Module
{
private:
    BufferedPort<Bottle>        mRpcPort;
      
    VelocityControllerThread   *mThread;
      
    vector<PolyDriver*>         mDrivers;
    vector<VelocityController*> mControllers;
      
    Property                    mParams;
      
    char                        mRobotName[256];
    char                        mModuleName[256];
    double                      mPeriod;
    
    bool                        bIsReady;

    bool                        mParts[8];
    
    bool                        bFakeDrivers;
public:
    VelocityControllerModule();

    virtual bool open(Searchable &s);
    virtual bool close();

    virtual bool respond(const Bottle& command, Bottle& reply);

    virtual double getPeriod();
    virtual bool   updateModule();
  
    virtual int runModule(int argc, char *argv[], bool skipFirst = true);
    
};



class VelocityControllerThread: public RateThread
{
private:
    vector<VelocityController*>    *mControllersPtr;
    int                             mPeriod;
    char                            mModuleName[256];

    BufferedPort<Vector>        mTargetPosPort;
    BufferedPort<Vector>        mTargetVelPort;
    BufferedPort<Vector>        mPosPort;
    BufferedPort<Vector>        mVelPort;

    double                      mTime;
    double                      mPrevTime;    

public:
            VelocityControllerThread(int period, const char* moduleName, vector<VelocityController*> *controllers);
    virtual ~VelocityControllerThread();

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();

            Vector& DispatchVector(Vector& src, int cId, Vector &res);
};

#endif /*VELOCITYCONTROLLERMODULE_H_*/

