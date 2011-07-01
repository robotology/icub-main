// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */


#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include "SimulationRun.h"
#include "FakeSimulationBundle.h"

int main(int argc, char** argv) {
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
        return 1;

    yarp::os::Property options;
    options.fromCommand(argc,argv);

    SimulationBundle *bundle = new FakeSimulationBundle;
    SimulationRun main;
    if (!main.run(bundle,argc,argv)) return 1;
    return 0;
}
