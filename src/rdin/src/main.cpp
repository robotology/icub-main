// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/RdInModule.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

/**
 * @ingroup icub_module
 *
 * \defgroup icub_rdin rdin
 *
 * Redirect stdin to a yarp port.\n
 * Usage, redirect output of an arbitrary module to the yarp port <modName>/stdout : 
 * myModule --name modName [--xyz ...] | rdin --name modName
 * You might want to add some fflush(stdout)'s to your source module)
 *
 * \author Jonas Ruesch
 *
 */

int main(int argc, char *argv[]) {
    Network yarp;
    RdInModule module;
    module.setName("/rdin"); // set default name of module
    return module.runModule(argc,argv);
}
