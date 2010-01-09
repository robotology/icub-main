// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Luis Montesano lmontesano@isr.ist.utl.pt
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * @ingroup icub_module
 *
 * \defgroup icub_demoAffv2
 *
 * This module is part of the application \ref demoAffv2 "demoAffv2"
 *
 *\section intro_sec Description 
 *
 * This module is part of the second version of the affordance demo. Its main functionality is to control the selection between a full bottom-up attention system and social engagement.
 *
 *\section lib_sec Libraries
 *
 *
 *
 *
 *\section parameters_sec Parameters
 *
 * No parameters.
 *
 *\section portssa_sec Ports Accessed 
 *
 * Input ports\n
 * (just descriptive at the time)
 *
 * gaze_control (gaze position) 
 * 
 * Output ports\n 
 * attention system inhibition of return
 * affordance Query Collector
 *
 *
 *\section portsc_sec Ports Created
 *
 *
 * Output ports\n
 *
 *\section conf_file_sec Configuration Files
 *
 *
 * --file affordance Bayes net
 *  example: --file $ICUB_ROOT/conf/BNaffordances.txt
 * Examples of such files can be found at \in conf/BNaffordances.txt
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux. Windows portability issues may arise due to its dependency on the pnl library.
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./demoAfford  --file  $ICUB_ROOT/conf/BNaffordances.txt \n
 *
 * This file can be edited at \in src/demoAfford/main.cpp 
 *
 *\authors Luis Montesano, Manuel Lopes
 *
 **/

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include "iCub/behavior.h"

using namespace std;
using namespace yarp::os;
//using namespace iCub::contrib;


int main(int argc, char *argv[]) {

    Network yarp;

    /* prepare and configure Resource Finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("behavior.ini");  // overridden by --from parameter
    rf.setDefaultContext("demoAffv2/conf");    // overridden by --context parameter
    rf.setDefault("name","/demoAffv2/behavior");

    rf.configure("ICUB_ROOT", argc, argv);

    Behavior module;
    return module.runModule(rf);
}
