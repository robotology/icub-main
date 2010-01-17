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
\defgroup icub__lasaImitation_WiimoteServerModule Wiimote Server Module

Module that connects to a wiimote and send button state or events over the network

\section intro_sec Description

Module that connects to a wiimote and send button state or events over the network. It requires 
the wiiuse library (http://wiiuse.net). Version 0.12 works...

The wiimote should be runned at module startup. If no wiimote is found, the module exits, same if the 
wiimote gets disconnected.

There is two mode: event-based (default) and streaming. The event one sends strict messages to the output 
port only when a buton is pressed or released, while
the streamed one sends a continuous flow of the buttons state.


\section dependencies_sec Dependencies

- YARP
- wiiuse library (http://wiiuse.net) tested with version 0.12

\section parameters_sec Parameters

\verbatim
--name <string>:    the module base name for port creation
--period <double>:  control loop period (0.1 sec by default)
\endverbatim

\section portsa_sec Ports Accessed

\section portsc_sec Ports Created

Input ports:

- /WiimoteServer/moduleName/rpc: single general command entry port to the module
          - [emod]: event mode (default)
          - [smod]: stream mode
          - [quit]: quit the module (exit)

Output ports:

- /WiimoteServer/moduleName/output: A 11-sized vector containing the state of each button
            - the vector position corresponding to each button is: 
              - A, B, Up, Down, Left, Right, Minus, Plus, Home, One, Two
            - in event mode, vector values are:
              - 0 -> nothing, 1 -> button pressed, -1 -> button released
            - in stream mode, vector values are:
              - 0 -> nothing, 1 -> button pressed


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
WiimoteServerModule --period 0.1 --name Wiimote1
\endverbatim

\author Eric Sauser

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/lasaImitation/WiimoteServerModule/src/WiimoteServerModule.h.
**/


#ifndef WiimoteServerMODULE_H_
#define WiimoteServerMODULE_H_

#include <yarp/os/Module.h>

using namespace yarp;
using namespace yarp::os;

#include "WiimoteServerThread.h"




class WiimoteServerModule: public Module
{
private:
    Property                    mParams;
    double                      mPeriod;
    bool                        bIsReady;

    BufferedPort<Bottle>        mControlPort;

    WiimoteServerThread      *mThread;


public:
            WiimoteServerModule();
    virtual ~WiimoteServerModule();


    virtual bool    open(Searchable &s);
    virtual bool    close();
    virtual bool    respond(const Bottle& command, Bottle& reply);
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual int     runModule(int argc, char *argv[], bool skipFirst = true);
    
    


};

#endif

