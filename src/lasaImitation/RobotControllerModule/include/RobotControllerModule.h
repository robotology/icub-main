// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
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
\defgroup icub__lasaImitation_RobotControllerModule Robot Controller Module

Main robot controller module used for the concurrent management of forward-inverse kinematics, joint-based control and pseudo tactile feedback.

\section intro_sec Description

This module concentrate the information related to the robot state and desired state. Since they should be both operational and joint space
the module also performs the necessary conversions and at a final stage, generate motion commands in joint space in order
to fulfill a maximum of constraints.

If several constraints are simultaneously activated, the following priority is taken for each arm: end-effector > wrist > joint-space

This module can also receive and process information provided by external sources such as touch sensors or other 3D devices through 
specific ports. It then acts accordingly to provoke motions on arm joints of the robot.

Actually, this module controls both robot arms.

In brief, this is the core robot controller. 

\section dependencies_sec Dependencies

- YARP
- iKinFwd

\section parameters_sec Parameters

\verbatim
--name <string>:    the module base name for port creation
--period <double>:  control loop period (0.02 sec by default)
\endverbatim

\section portsa_sec Ports Accessed

\section portsc_sec Ports Created

Input ports:

    - /RobotController/moduleName/rpc: single general command entry port to the module
          - [run]: run the controller
          - [susp]: suspend the controller (send zero output)
          - [quit]: quit the module (exit)
          - [jnt] <L/R>: activate/suspend joint control for left or right arm
          - [cart] <L/R>: activate/suspend cartesian control for left or right arm
          - [wrst] <L/R>: activate/suspend wrist control for left or right arm
          - [kj]  k: set gain value joint control
          - [kc]  k: set gain value cartesian control
          - [kw]  k: set gain value wrist control

    - /RobotController/moduleName/currentJointPosition: current joint position for both arm 
    - /RobotController/moduleName/currentJointVelocity: current joint velocity for both arm 

    - /RobotController/moduleName/desiredJointPositionL: desired joint position for the left arm 
    - /RobotController/moduleName/desiredJointPositionR: desired joint position for the right arm

    - /RobotController/moduleName/desiredCartPositionL: desired cartesian position and orientation for the left arm end effector
    - /RobotController/moduleName/desiredCartPositionR: desired cartesian position and orientation for the right arm end effector

    - /RobotController/moduleName/desiredWristCartPositionL: desired cartesian position and orientation for the left arm wrist
    - /RobotController/moduleName/desiredWristCartPositionR: desired cartesian position and orientation for the right arm wrist


Output ports:

    - /RobotController/moduleName/targetJointPosition: target joint position for both arm (to the velocity controller)
    - /RobotController/moduleName/targetJointVelocity: target joint velocity for both arm (to the velocity controller)

    - /RobotController/moduleName/currentCartPositionL: current cartesian position and orientation of the left arm end effector
    - /RobotController/moduleName/currentCartPositionR: current cartesian position and orientation of the right arm end effector

    - /RobotController/moduleName/currentWristPositionL: current cartesian position and orientation of the left arm wrist
    - /RobotController/moduleName/currentWristPositionR: current cartesian position and orientation of the right arm wrist

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
RobotControllerModule --period 0.02
\endverbatim

\note This module is still heavily under development.

\author Eric Sauser

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/lasaImitation/RobotControllerModule/src/RobotControllerModule.h.
**/


#ifndef RobotControllerMODULE_H_
#define RobotControllerMODULE_H_

#include <yarp/os/Module.h>

using namespace yarp;
using namespace yarp::os;

#include "RobotControllerThread.h"

class RobotControllerModule: public Module
{
private:
    Property                    mParams;
    double                      mPeriod;
    bool                        bIsReady;

    BufferedPort<Bottle>        mControlPort;

    RobotControllerThread      *mThread;
    
public:
            RobotControllerModule();
    virtual ~RobotControllerModule();


    virtual bool    open(Searchable &s);
    virtual bool    close();
    virtual bool    respond(const Bottle& command, Bottle& reply);
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual int     runModule(int argc, char *argv[], bool skipFirst = true);
};

#endif

