// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <SoundLocalizationDummyModule.h>

using namespace yarp::os;

///**
// * @ingroup icub_module
// *
// * \defgroup icub_soundlocalizationdummy soundlocalizationdummy
// *
// * Sound Localization Dummy
// *
// * This module generates random sound localization data for debugging. 
// *
// * \author Jonas Ruesch
// *
// */

int main(int argc, char *argv[]) {

    Network yarp;
    SoundLocalizationDummyModule module;
    module.setName("/soundlocalization"); // set default name of module
    return module.runModule(argc,argv);
}
