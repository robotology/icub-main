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
\defgroup icub__lasaImitation_TouchControllerModule Touch Controller Module

This module provides 3d-cartesian touch feedback from external touch sensors. 

\section intro_sec Description

This module provides 3d-cartesian touch feedback from external touch sensors which should be usb-mice based interfaces. Feedback provided is 6-dof velocity feedback, i.e., 3 translations around the 3 canonical axes and 3 rotations around these same axes.

Tested interfaces are "GlidePoint" touch sensors arranged as a grid around the iCub's wrist as well as a 3Dconnexion 6-dof mouse.

For more information, one can refer to the following paper under preparation: 
<a href="http://lasa.epfl.ch/~sauser/RobotCub/ArgallEtAl_TactileGuidance.pdf"> 
Argall, Sauser and Billard. Tactile Guidance for Policy Refinement and Reuse. </a>

\section dependencies_sec Dependencies

- YARP
- \ref icub__lasaImitation_MultipleMiceDriver "Multiple Mice Driver"

\section parameters_sec Parameters

\verbatim
--name <string>:   the module base name for port creation (Touch000 by default)
--period <double>: control loop period (0.05 sec by default)
--type <string>:   type of input device ("3Dmouse" or "touchpad")
--device <string>: generic name of the mouse device
\endverbatim

\section portsa_sec Ports Accessed

\section portsc_sec Ports Created

Input ports:

    - /TouchController/moduleName/rpc: single general command entry port to the module
          - [run]: run the controller
          - [susp]: suspend the controller (send zero output)
          - [quit]: quit the module (exit)
          - [kpt] kp: set gain values for translational feedback
          - [kpr] kp: set gain values for rotational feedback
          - [limt] lim: set limit values for translational feedback
          - [limr] lim: set limit values for rotational feedback
          - [sid] id: set the next touched mouse with given id

    - /TouchController/moduleName/frameOfRef: externally provided frame of reference for the touch device

Output ports:

    - /TouchController/moduleName/velocity: a vector of size 6 providing sensed velocities for 6-dof


\section in_files_sec Input Data Files

mice_moduleName.map (load last mice configuration)

\section out_data_sec Output Data Files

mice_moduleName.map (save last mice configuration)

\section conf_file_sec Configuration Files

None

\section tested_os_sec Tested OS

Linux

\section example_sec Example Instantiation of the Module

\verbatim
TouchControllerModule --name TouchRight --period 0.02 --type 3dmouse --device 3Dconnexion
\endverbatim

\note This module is still heavily under development.

\author Eric Sauser

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/lasaImitation/TouchControllerModule/src/TouchControllerModule.h.
**/

#ifndef TOUCHCONTROLLERMODULE_H_
#define TOUCHCONTROLLERMODULE_H_

#include <yarp/os/Module.h>

using namespace yarp;
using namespace yarp::os;

#include "TouchControllerThread.h"

class TouchControllerModule: public Module
{
private:
    Property                    mParams;
    double                      mPeriod;
    bool                        bIsReady;

    BufferedPort<Bottle>        mControlPort;

    TouchControllerThread      *mThread;
    
    int                         mType;
    char                        mDeviceName[256];
    
public:
            TouchControllerModule();
    virtual ~TouchControllerModule();


    virtual bool    open(Searchable &s);
    virtual bool    close();
    virtual bool    respond(const Bottle& command, Bottle& reply);
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual int     runModule(int argc, char *argv[], bool skipFirst = true);
};

#endif

