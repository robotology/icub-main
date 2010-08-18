// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file main.cpp
 * @brief main for the blob finder module: this module gets edge images from the visual filter module, computes a blob
 * based segmentation, color quantization and a blob based saliency map (logpolar).
 */

#include <yarp/os/all.h>
#include <iCub/saliencyBlobFinderModule.h>

#include <iostream>
#include <string.h>
using namespace std;

#include <ippi.h>
#include <ippcore.h>

int main(int argc, char *argv[]) {
    /* important, makes IPP single threaded! */
    ippSetNumThreads(1);

    Network yarp;
    Time::turboBoost();

    saliencyBlobFinderModule module; 

    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("blobFinderLeft.ini");      //overridden by --from parameter
    rf.setDefaultContext("attentionMechanism/conf");    //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
 
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);

    return 0;
}