// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2010 Paul Fitzpatrick, Vadim Tikhanoff
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/

#include "OdeSdlSimulationBundle.h"

#include "OdeInit.h" 
#include "iCubLogicalJoints.h"
#include "iCub_Sim.h"
#include "OdeWorldManager.h"

bool OdeSdlSimulationBundle::onBegin() {
    dInitODE2(0); 
    printf("\nODE configuration: %s\n\n", dGetConfiguration());
    return true;
}

LogicalJoints *OdeSdlSimulationBundle::createJoints(RobotConfig& config) {
    OdeInit& odeinit = OdeInit::init(&config);
    odeinit.setName(config.getModuleName().c_str());

    // Set up ODE joints
    return new iCubLogicalJoints(config);
}

WorldManager *OdeSdlSimulationBundle::createWorldManager(RobotConfig& config) {
    return new OdeWorldManager();
}

Simulation *OdeSdlSimulationBundle::createSimulation(RobotConfig& config) {
    return new OdeSdlSimulation();
}

bool OdeSdlSimulationBundle::onEnd() {
    OdeInit::destroy();
    dCloseODE();
    return true;
}


