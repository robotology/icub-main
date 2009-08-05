// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2008 Boris Duran
 *	Control of iCub's head motors from DFT peaks.
 *	Usage: 
 *			> dftCtrl.exe --file dftCtrl.ini [--verbose]
 *
 */

#include <stdio.h>
// yarp
#include <yarp/os/all.h>
// iCub
#include "ctrlModule.h"

using namespace yarp::os;

int main( int argc, char* argv[] ) {
    Network yarp;
    
    CTRLmodule module;
    module.setName("/boris/ctrl"); // set default name of module
	// don't call runModule() in order to prevent terminal being attached.
	if (!module.openFromCommand(argc,argv)) {
        cout << "Module failed to open." << endl;
        return 1;
    }
    bool ok = module.runModule();	//true;	//
	module.close();
    return ok?0:1;
}
