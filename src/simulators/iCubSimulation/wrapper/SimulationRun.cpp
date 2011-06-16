// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff, Paul Fitzpatrick
* email:   vadim.tikhanoff@iit.it, paulfitz@alum.mit.edu
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

    WorldManager *world = bundle->createWorldManager(config);
    if (world==NULL) {
        fprintf(stderr,"Failed to allocate world manager\n");
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

    SimulatorModule module(*world,config,bundle->createSimulation(config));

    if (module.open()) {
        //this blocks until termination (through ctrl+c or a kill)
        module.runModule();
    }        

    module.closeModule();

    bundle->onEnd();

    delete world;
    world = NULL;

    delete bundle;
    bundle = NULL;

    return true;
}
