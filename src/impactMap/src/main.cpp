// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Chris McCarthy
 *
 */


// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// OpenCV
#include <cv.h>

#include <ImpactMapModule.h>

using namespace std;
using namespace yarp::os;


int main(int argc, char *argv[]) {

     // prepare CalibTools
    Network yarp;
    ImpactMapModule module;
    module.setName("/impactMap"); // set default name of module
    return module.runModule(argc,argv);
}
