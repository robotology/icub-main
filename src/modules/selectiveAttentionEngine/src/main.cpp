// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
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
 * @file stereoAttModule.cpp
 * @brief Implementation of the stereoAttModule (see header file).
 */

#include <yarp/os/all.h>
#include <iCub/selectiveAttentionModule.h>
#include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)



#include <iostream>

using namespace std;
using namespace yarp::os;


int main(int argc, char *argv[]) {
    //initialise Yarp Network
    Network yarp;
    Time::turboBoost();
    selectiveAttentionModule module; 

    YARP_REGISTER_DEVICES(icubmod)

    /* prepare and configure the resource finder */

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("selectiveAttentionLeft.ini");  //overridden by --from parameter
    rf.setDefaultContext("attentionMechanism/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);

}