// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2010 Paul Fitzpatrick, Vadim Tikhanoff
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/

#include "BlenderSimulationBundle.h"

#include "BlenderLogicalJoints.h"
#include "BlenderSimulation.h"

bool BlenderSimulationBundle::onBegin() {
    return true;
}

LogicalJoints *BlenderSimulationBundle::createJoints(RobotConfig& config) {
    config.setFlags();
    return new BlenderLogicalJoints;
}

Simulation *BlenderSimulationBundle::createSimulation(RobotConfig& config) {
    return new BlenderSimulation;
}

bool BlenderSimulationBundle::onEnd() {
    return true;
}


