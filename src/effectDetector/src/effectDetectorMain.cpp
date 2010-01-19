// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * @inmodule icub_module
 * /defgroup icub_effectDetector effectDetector
 *
 * Detect effect of an action on objects. Used in the affordance application.
 *
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

//OpenCV
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <math.h>

#include <iCub/effectDetector.h>

int main(int argc, char *argv[]) {
    Network yarp;
    yarp::os::Time::turboBoost();
	
	/* prepare and configure Resource Finder */
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("effectDetector.ini");  // overridden by --from parameter
	rf.setDefaultContext("effectDetector/conf");    // overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

    EffectDetector module;
	module.setName("/effectDetector");
    return module.runModule(rf);
}
