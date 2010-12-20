// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2010 Paul Fitzpatrick, Vadim Tikhanoff
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/

#include "SimulationRun.h"

#include <yarp/dev/all.h>

#include "SimulatorModule.h"
#include "iCubSimulationControl.h"
#include "SimConfig.h"

using namespace yarp::dev;
using namespace std;

bool SimulationRun::run(SimulationBundle *bundle, int argc, char *argv[]) {
    if (bundle==NULL) {
        fprintf(stderr,"Failed to allocate simulator\n");
        return false;
    }

    bundle->onBegin();

    SimConfig config;
    string moduleName;
    config.configure(argc, argv, moduleName);

    LogicalJoints *icub_joints = bundle->createJoints(config);
    if (icub_joints==NULL) {
        fprintf(stderr,"Failed to allocate joints\n");
        delete bundle;
        return false;
    }

    PolyDriver icub_joints_dev;
    icub_joints_dev.give(icub_joints,true);

    // Make sure all individual control boards route to single ode_joints driver
    Drivers::factory().add(new DriverLinkCreator("robot_joints",icub_joints_dev));

    // Provide simulated controlboard driver
    Drivers::factory().add(new DriverCreatorOf<iCubSimulationControl>("simulationcontrol", 
        "controlboard",
        "iCubSimulationControl"));

    SimulatorModule module(config,bundle->createSimulation(config));

    if (module.open()) {
        //this blocks until termination (through ctrl+c or a kill)
        module.runModule();
    }        

    module.closeModule();

    bundle->onEnd();

    delete bundle;
    bundle = NULL;

    return true;
}
