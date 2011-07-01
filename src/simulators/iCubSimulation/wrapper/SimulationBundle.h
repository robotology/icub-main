// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick, Vadim Tikhanoff
* email:   paulfitz@alum.mit.edu, vadim.tikhanoff@iit.it
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


#ifndef ICUBSIMULATION_SIMULATIONBUNDLE_INC
#define ICUBSIMULATION_SIMULATIONBUNDLE_INC

#include "Simulation.h"
#include "LogicalJoints.h"
#include "WorldManager.h"

#include <yarp/os/Module.h>

class SimulationBundle {
public:
    virtual bool onBegin() = 0;

    virtual LogicalJoints *createJoints(RobotConfig& config) = 0;

    virtual WorldManager *createWorldManager(RobotConfig& config) = 0;

    virtual Simulation *createSimulation(RobotConfig& config) = 0;

    virtual bool onEnd() = 0;
};

#endif
