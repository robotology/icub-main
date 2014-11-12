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
 *  The module does:
 * -    Has been designed to reproduce, as accurately as possible, the physics and the dynamics of the robot and its environment
 * -   It has been constructed collecting data directly from the robot design specifications in order to achieve an exact replication 
 * -    The software infrastructure and inter-process communication are as those used to control the physical robot
 *
 * \section lib_sec Libraries
 * YARP
 * ODE
 * SDL 
 *
 * \section parameters_sec Parameters
 * All the parameters are located in the iCub_parts_activation.ini
 *
 *  [SETUP]
 * elevation on
 *
 *  [PARTS]
 *  legs on
 *  torso on 
 *  left_arm on
 *  left_hand on
 *  right_arm on 
 *  right_hand on
 *  head on
 *  fixed_hip on 
 *
 *  [VISION]
 *  cam off
 *
 *  [RENDER]
 *  objects off
 *  cover on
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
#include <yarp/os/Property.h>
#include <yarp/os/Thread.h>
#include <yarp/dev/Drivers.h>

#include "SimulationRun.h"
#ifdef ICUB_SIM_ENABLE_ODESDL
#include "OdeSdlSimulationBundle.h"
#endif
#ifdef ICUB_SIM_ENABLE_FAKE
#include "FakeSimulationBundle.h"
#endif

// Needed on OSX, to hijack main()
#ifdef ICUB_SIM_ENABLE_ODESDL
#include "SDL.h" 
#endif

YARP_DECLARE_DEVICES(icubmod)


class MainThread : public yarp::os::Thread {
 public:
   SimulationBundle *bundle;
   int argc;
   char **argv;
   bool result;

   void run() {
       SimulationRun main;
       result = main.run(bundle,argc,argv);
   }
 };


int main(int argc, char** argv) {
    //yarp::os::Thread::setDefaultStackSize(3145728);
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf (stderr, "Error: could not initialize YARP network (is the nameserver running?)\n");
        return 1;
    }
    
    YARP_REGISTER_DEVICES(icubmod)
    yarp::os::Property options;
    options.fromCommand(argc,argv);

    // the "bundle" controls the implementation used for the simulation
    // (as opposed to the simulation interface).  The default is the
    // standard ODE/SDL implementation.  There is a "fake" do-nothing
    // simulation for test purposes that does nothing other than create
    // the standard ports (pass --fake to start this simulation).
    SimulationBundle *bundle = NULL;

    if (options.check("fake")) {
#ifdef ICUB_SIM_ENABLE_FAKE
        bundle = new FakeSimulationBundle;
#endif
    } else {
#ifdef ICUB_SIM_ENABLE_ODESDL
        bundle = new OdeSdlSimulationBundle;
#endif
    }
    if (options.check("verbose"))
    {
        bundle->verbose = true;
        fprintf(stdout, "Starting the simulator with verbose flag on\n");
    }
    if (bundle==NULL) {
        fprintf(stderr,"Failed to allocate simulator\n");
        return 1;
    }

    //SimulationRun main;
    //if (!main.run(bundle,argc,argv)) return 1;
   MainThread thread;
   thread.bundle = bundle;
   thread.argc = argc;
   thread.argv = argv;
#ifdef _WIN32
   //thread.setOptions(3145728);
#endif
   thread.start();
   thread.join();
   return thread.result?0:1;
}
