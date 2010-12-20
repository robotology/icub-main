// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2010 Paul Fitzpatrick, Vadim Tikhanoff
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/


#ifndef ICUBSIMULATION_SIMULATIONBUNDLE_INC
#define ICUBSIMULATION_SIMULATIONBUNDLE_INC

#include "Simulation.h"
#include "LogicalJoints.h"

#include <yarp/os/Module.h>

class SimulationBundle {
public:
    virtual bool onBegin() = 0;

    virtual LogicalJoints *createJoints(RobotConfig& config) = 0;

    virtual Simulation *createSimulation(RobotConfig& config) = 0;

    virtual bool onEnd() = 0;
};

#endif
