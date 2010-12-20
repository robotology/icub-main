// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2010 Paul Fitzpatrick, Vadim Tikhanoff
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/

#ifndef ICUBSIMULATION_FAKESIMULATIONBUNDLE_INC
#define ICUBSIMULATION_FAKESIMULATIONBUNDLE_INC

#include "SimulationBundle.h"

class FakeSimulationBundle : public SimulationBundle {
public:
    virtual bool onBegin();

    virtual LogicalJoints *createJoints(RobotConfig& config);

    virtual Simulation *createSimulation(RobotConfig& config);

    virtual bool onEnd();
};


#endif
