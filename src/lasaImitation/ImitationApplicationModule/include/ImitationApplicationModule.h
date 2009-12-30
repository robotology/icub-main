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
\defgroup icub__lasaImitation_ImitationApplicationModule Imitation Application Control Module

State machine module which controls the activity and processes of the modules involved in the 
\ref icub_lasaImitation "Imitation learning, refinement and reuse" application.

\section intro_sec Description

This module is the application state machine and drives each module involved in the application
\ref icub_lasaImitation "Imitation learning, refinement and reuse".

\section dependencies_sec Dependencies

- YARP

\section parameters_sec Parameters

\verbatim
--name <string>:    the module base name for port creation
--period <double>:  control loop period (0.1 sec by default)
\endverbatim

\section portsa_sec Ports Accessed

\section portsc_sec Ports Created

Input ports:

- /ImitationApplication/moduleName/rpc: The list of commands listed in the application \ref icub_lasaImitation "Imitation learning, refinement and reuse" are considered 

Output ports:

- /ImitationApplication/moduleName/commandToX : series of output ports where "X" are modules involved in the application \ref icub_lasaImitation "Imitation learning, refinement and reuse"


\section in_files_sec Input Data Files

See \ref icub_lasaImitation "Imitation learning, refinement and reuse" application 

\section out_data_sec Output Data Files

See \ref icub_lasaImitation "Imitation learning, refinement and reuse" application 

\section conf_file_sec Configuration Files

See \ref icub_lasaImitation "Imitation learning, refinement and reuse" application 

\section tested_os_sec Tested OS

Linux

\section example_sec Example Instantiation of the Module

\verbatim
ImitationApplicationModule --period 0.1
\endverbatim

However, don't forget to run the application script.


\note This module is still heavily under development.

\author Eric Sauser

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/lasaImitation/ImitationApplicationModule/src/ImitationApplicationModule.h.
**/


#ifndef ImitationApplicationMODULE_H_
#define ImitationApplicationMODULE_H_

#include <yarp/os/Module.h>

using namespace yarp;
using namespace yarp::os;

#include "ImitationApplicationThread.h"

class ImitationApplicationModule: public Module
{
private:
    Property                    mParams;
    double                      mPeriod;
    bool                        bIsReady;

    BufferedPort<Bottle>        mControlPort;

    ImitationApplicationThread      *mThread;
    
public:
            ImitationApplicationModule();
    virtual ~ImitationApplicationModule();


    virtual bool    open(Searchable &s);
    virtual bool    close();
    virtual bool    respond(const Bottle& command, Bottle& reply);
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual int     runModule(int argc, char *argv[], bool skipFirst = true);
};

#endif

