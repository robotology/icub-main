// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2010 Paul Fitzpatrick, Vadim Tikhanoff
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/

#include "FakeSimulationBundle.h"

#include "FakeLogicalJoints.h"
#include "FakeSimulation.h"

bool FakeSimulationBundle::onBegin() {
    return true;
}

LogicalJoints *FakeSimulationBundle::createJoints(RobotConfig& config) {
    config.setFlags();
    return new FakeLogicalJoints;
}

Simulation *FakeSimulationBundle::createSimulation(RobotConfig& config) {
    return new FakeSimulation;
}

bool FakeSimulationBundle::onEnd() {
    return true;
}


