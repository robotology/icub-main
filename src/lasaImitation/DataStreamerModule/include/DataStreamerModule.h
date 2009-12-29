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
\defgroup icub__lasaImitation_DataStreamerModule Data Streaming Module

This module allows either to stream numerical data from a matrix data 
structure to a port or to record a stream from a port into a matrix.

\section intro_sec Description

This module allows either to stream numerical data from a matrix data structure to a port or to record a stream from a port into a matrix. 
After all, data can be loaded from or saved into a file.

\section dependencies_sec Dependencies

- YARP

\section parameters_sec Parameters

\verbatim
--name <string>: the module base name for port creation (DataStreamer000 by default)
--period <double>: control loop period (0.02 sec by default)
\endverbatim

\section portsa_sec Ports Accessed

\section portsc_sec Ports Created

Input ports:

    - /DataStreamer/moduleName/rpc: general command entry port to the module
          - [run start]: run the module and start streaming and/or recording
          - [run loop]: run the module and start streaming and/or recording in a loop (forever)
          - [run stop]: stop the module
          - [run pause]: pause the module
          - [run resume]: resume the paused module
          - [rec set]: set recording mode (when recording, input is directly sent to output port)
          - [rec unset]: cancel recording
          - [data lineSize]: set the number of numbers to be streamed at each time
          - [data maxSize]: set the capacity of the dataset
          - [data load] filename: load filename as data
          - [data save] filename: save data to filename
          - [data timeOn]: consider first column as a time index (not streamed)
          - [data timeOff]: disable timing
          - [data clear]: clear data
    - /DataStreamer/moduleName/input: input port for recording data stream

Output ports:

    - /DataStreamer/moduleName/output: output port for the data stream


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
DataStreamerModule --name myData --period 0.02
\endverbatim

\author Eric Sauser

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/lasaImitation/DataStreamerModule/src/DataStreamerModule.h.
**/

#ifndef DATASTREAMERMODULE_H_
#define DATASTREAMERMODULE_H_

#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

using namespace yarp;
using namespace yarp::os;

#include "DataStreamerThread.h"

class DataStreamerModule: public Module
{
private:
    Property                    mParams;
    double                      mPeriod;
    bool                        bIsReady;

    BufferedPort<Bottle>        mControlPort;

    DataStreamerThread          *mThread;
    
public:
            DataStreamerModule();
    virtual ~DataStreamerModule();


    virtual bool    open(Searchable &s);
    virtual bool    close();
    virtual bool    respond(const Bottle& command, Bottle& reply);
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual int     runModule(int argc, char *argv[], bool skipFirst = true);
};

#endif

