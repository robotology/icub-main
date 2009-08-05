// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 Alex Bernardino, Vislab, IST/ISR
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include "arMarkerDetectorModule.h"

using namespace std;
using namespace yarp::os;


int main(int argc, char *argv[]) {

    Network yarp;
    ARMarkerDetectorModule module;
    module.setName("/artkSingleTracker"); // set default name of module
    return module.runModule(argc,argv);
}
