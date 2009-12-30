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
\defgroup icub__lasaImitation_GaussianMixtureModelModule Gaussian Mixture Model Module

Module devoted to hold and execute a learned task model by means of a Gaussian Mixture Model.

\section intro_sec Description

This module allows to learn and to execute a task by means of a model encoded in a Gaussian mixture.
It receives the current robot state and produce desired output command for the robot to reproduce the
task as best as possible.

This module can also be used for online correction, i.e., the model can be updated according to an exernal
control signal indicating if a correction has being applied. 

For more information, one can refer to the following paper under preparation: 
<a href="http://lasa.epfl.ch/~sauser/RobotCub/ArgallEtAl_TactileGuidance.pdf"> 
Argall, Sauser and Billard. Tactile Guidance for Policy Refinement and Reuse. </a>

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

    - /GaussianMixtureModel/moduleName/rpc: single general command entry port to the module
          - [load] file: load a model file
          - [save] file: save a model file
          - [exec]: run current model
          - [susp]: pause/resume execution
          - [stop]: stop execution
          - [corr]: set/unset correction-based learning
          - [lear]: applies last correction set

    - /GaussianMixtureModel/moduleName/correctionLevel: current level of the correction process (0: off, 1:on)
    - /GaussianMixtureModel/moduleName/currentState: current state of the robot (the state space depends on the model type)

Output ports:

    - /GaussianMixtureModel/moduleName/command: command to be sent to the robot (the command space depends on the model)


\section in_files_sec Input Data Files

Model files located in ./data/

\section out_data_sec Output Data Files

Model files located in ./data/

\section conf_file_sec Configuration Files

None

\section tested_os_sec Tested OS

Linux

\section example_sec Example Instantiation of the Module

\verbatim
GaussianMixtureModelModule --name GMM --period 0.02
\endverbatim

\note This module is still heavily under development.

\author Eric Sauser

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/lasaImitation/GaussianMixtureModelModule/src/GaussianMixtureModelModule.h.
**/


#ifndef GaussianMixtureModelMODULE_H_
#define GaussianMixtureModelMODULE_H_

#include <yarp/os/Module.h>

using namespace yarp;
using namespace yarp::os;

#include "GaussianMixtureModelThread.h"

class GaussianMixtureModelModule: public Module
{
private:
    Property                    mParams;
    double                      mPeriod;
    bool                        bIsReady;

    BufferedPort<Bottle>        mControlPort;

    GaussianMixtureModelThread      *mThread;
    
public:
            GaussianMixtureModelModule();
    virtual ~GaussianMixtureModelModule();


    virtual bool    open(Searchable &s);
    virtual bool    close();
    virtual bool    respond(const Bottle& command, Bottle& reply);
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual int     runModule(int argc, char *argv[], bool skipFirst = true);
};

#endif

