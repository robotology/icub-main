// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 * 
 * \defgroup icub_Simulation iCubSimulation
 * 
 * This module is the physics simulator of the iCub robot. 
 *
 * \image html iCubSim.jpg
 * \image latex robotMotorGui.eps "A window of the iCub Simulator running on Linux" width=15cm
 * 
 * \section intro_sec Description
 * 
 *	The module does:
 * -	Has been designed to reproduce, as accurately as possible, the physics and the dynamics of the robot and its environment
 * -   It has been constructed collecting data directly from the robot design specifications in order to achieve an exact replication 
 * -	The software infrastructure and inter-process communication are as those used to control the physical robot
 *
 * \section lib_sec Libraries
 * YARP
 * ODE
 * SDL 
 *
 * \section parameters_sec Parameters
 * All the parameters are located in the conf/iCub_parts_activation.ini
 *
 *	[SETUP]
 * elevation on
 *
 *	[PARTS]
 *	legs on
 *	torso on 
 *	left_arm on
 *	left_hand on
 *	right_arm on 
 *	right_hand on
 *	head on
 *	fixed_hip on 
 *
 *	[VISION]
 *	cam off
 *
 *	[RENDER]
 *	objects off
 *	cover on
 * 
 * \section portsa_sec Ports Accessed
 * No ports are accessed nor needed by the iCub simulator
 *
 * \section portsc_sec Ports Created
 * list of ports created by the module:
 *
 * - /icubSim/left_leg/rpc:i : left_leg rpc port
 * - /icubSim/left_leg/command:i : left_leg command port
 * - /icubSim/left_leg/state:o : left_leg state port
 *
 * - /icubSim/right_leg/rpc:i : right_leg rpc port
 * - /icubSim/right_leg/command:i : right_leg command port
 * - /icubSim/right_leg/state:o : right_leg state port
 *
 * - /icubSim/torso/rpc:i : torso rpc port
 * - /icubSim/torso/command:i : torso command port
 * - /icubSim/torso/state:o : torso state port
 *
 * - /icubSim/left_arm/rpc:i : left arm rpc port
 * - /icubSim/left_arm/command:i : left arm command port
 * - /icubSim/left_arm/state:o : left arm state port
 *
 * - /icubSim/right_arm/rpc:i : right arm rpc port
 * - /icubSim/right_arm/command:i : right arm command port
 * - /icubSim/right_arm/state:o : right arm state port
 *
 * - /icubSim/head/rpc:i : head rpc port
 * - /icubSim/head/command:i : head command port
 * - /icubSim/head/state:o : head state port
 *
 * - /icubSim/face/eyelids : port to control the eyelids
 * - /icubSim/cam/left : streams out the data from the left camera (cartesian format 320x240 )
 * - /icubSim/cam/left/fovea : streams out the data from the left camera (fovea format 128x128 ) 
 * - /icubSim/cam/left/logpolar : streams out the data from the left camera (log polar format 252x152 ) 
 * - /icubSim/cam/right : streams out the data from the right camera (cartesian format 320x240 )
 * - /icubSim/cam/right/fovea : streams out the data from the left camera (fovea format 128x128 ) 
 * - /icubSim/cam/right/logpolar : streams out the data from the left camera (log polar format 252x152 ) 
 * - /icubSim/cam : streams out the data from the global view
 *
 * - /icubSim/world : port to manipulate the environment
 * - /icubSim/touch : streams out a sequence the touch sensors for both hands
 * - /icubSim/inertial : streams out a sequence of inertial data taken from the head
 * - /icubSim/texture : port to receive texture data to place on an object (e.g. data from a webcam etc...)
 *
 * \section in_files_sec Input Data Files
 * iCubSimulator expects the following configuration files:
 * - iCub_parts_activation.ini  : the runtime parameters
 * - Sim_camera.ini  : initialization file for the camera(s) of the iCub Simulation
 * - Sim_head.ini  : initialization file for the head of the iCub Simulation
 * - Sim_left_arm.ini  : initialization file for the left arm of the iCub Simulation
 * - Sim_left_leg.ini  : initialization file for the left leg of the iCub Simulation
 * - Sim_right_arm.ini  : initialization file for the right arm of the iCub Simulation
 * - Sim_right_leg.ini  : initialization file for the right leg of the iCub Simulation
 * - Sim_torso.ini  : initialization file for the torso of the iCub Simulation
 * - Video.ini  : initialization file for the texture port
 *
 * \section tested_os_sec Tested OS
 * Linux, Windows and Mac partially.
 *
 * \author Tikhanoff Vadim, Fitzpatrick Paul
 *
 * Copyright (C) 2007 - 2008 - 2009 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/iCubSimulation/main.cpp.
 *
 **/

//\file main.cpp
//\brief This file is responsible for the initialisation the module and yarp ports. It deals with incoming world commands such as getting information or creating new objects 

#include <yarp/os/Network.h>
#include <yarp/os/Os.h>
#include <yarp/dev/all.h>

#include "SimulatorModule.h"
#include "SimConfig.h"
#include "OdeInit.h" 
#include "iCubSimulationControl.h" 

extern OdeInit& getOdeInit();
#define odeinit (getOdeInit())

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

static SimulatorModule *simulatorModule = NULL;

void sendTouch(Bottle& report) {
	simulatorModule->sendTouch(report);
}

bool shouldSendTouch() {
	return simulatorModule->shouldSendTouch();
}

void sendInertial(Bottle& report) {
	simulatorModule->sendInertial(report);
}

bool shouldSendInertial() {
	return simulatorModule->shouldSendInertial();
}


void sendVision(){
    simulatorModule->displayStep(0);
}

static void displayStep(int pause) {
    // simulatorModule->displayStep(pause);
}

int main( int argc, char** argv) 
{		
	dInitODE2(0); 
	printf("\nODE configuration: %s\n\n", dGetConfiguration());
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    SimConfig finder;
    string moduleName;
		
    finder.configure(argc, argv, moduleName);

	odeinit._wrld->OBJNUM = 0;
	odeinit._wrld->waitOBJ = 0;
	odeinit._wrld->S_OBJNUM = 0;

  	odeinit._wrld->SPHNUM = 0;
	odeinit._wrld->waitSPH = 0;
	odeinit._wrld->S_SPHNUM = 0;
		
	odeinit._wrld->cylOBJNUM = 0;
	odeinit._wrld->waitOBJ1 = 0;
	odeinit._wrld->S_cylOBJNUM = 0;
	
	odeinit._wrld->waitMOD = 0;
	odeinit._wrld->MODEL_NUM = 0;

	odeinit._wrld->s_waitMOD = 0;
	odeinit._wrld->s_MODEL_NUM = 0;

    iCubSimulationControl::init();

    Drivers::factory().add(new DriverCreatorOf<iCubSimulationControl>("simulationcontrol", 
        "controlboard",
        "iCubSimulationControl"));

    if (!Network::checkNetwork()) {
        printf("Please start a yarp name server first\n");
        yarp::os::exit(1);
    }

    SimulatorModule module;
    simulatorModule = &module;
    module.moduleName = moduleName;
    odeinit.setName(moduleName);
    module.open();

    //this blocks until termination (through ctrl+c or a kill)
    module.runModule();

    module.closeModule();
    finder.deleteFinder();
    delete &getOdeInit();

    dCloseODE();

    return 0;
}
