/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "WholeBodyPlayerModule.h"

int main(int argc, char * argv[]) {

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return EXIT_FAILURE;
    }
    WholeBodyPlayerModule wbdPlayerModule;
    yarp::os::ResourceFinder config;
    config.configure(argc, argv);
    return wbdPlayerModule.runModule(config);
};
 
