// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 02009
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/handMover.h>

using namespace std;
using namespace yarp::os;
//using namespace iCub::contrib;


/**
 * @ingroup icub_module
 * \defgroup icub_handMover handMover
 *
 * \author Vislab
 * 
 */

int main(int argc, char *argv[]) {

    Network yarp;
    HandMover module;
    module.setName("/handMover"); // set default name of module
    return module.runModule(argc,argv);
}
