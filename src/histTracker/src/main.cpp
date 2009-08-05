// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Plinio Moreno
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/HistTrackerModule.h>

using namespace std;
using namespace yarp::os;

using namespace iCub::contrib;

/**
 * @ingroup icub_module
 *
 * \defgroup icub_histtracker histTracker
 *
 * It tracks an image region using the mean shift algorithm.
 *
 *
 * \author Plinio Moreno
 *
 */


int main(int argc, char *argv[]) {

    Network yarp;
    HistTrackerModule module;
    module.setName("/histtracker"); // set default name of module
    return module.runModule(argc,argv);
}
