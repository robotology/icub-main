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
#include <iCub/HeadMoverManualModule.h>

using namespace yarp::os;

/**
* @ingroup icub_module
*
* \defgroup icub_headmovermanual headmovermanual
*
* Use this module to move the iCub head manually by pressing keys (demo purpose etc..).
*
* The key map can be configured in headMoverManual.ini in $ICUB_DIR/conf.
*
* \author Jonas Ruesch
*
*/

int main(int argc, char *argv[]) {

    Network yarp;
    HeadMoverManualModule module;
    module.setName("/headMoverManual"); // set default name of module
	// don't call runModule() in order to prevent terminal being attached.
	if (!module.openFromCommand(argc,argv)) {
        cout << "Module failed to open." << endl;
        return 1;
    }
    bool ok = module.runModule();
	module.close();
    return ok?0:1;
}
