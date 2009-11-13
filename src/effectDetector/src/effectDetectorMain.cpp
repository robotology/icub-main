// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
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

    EffectDetector module;
    module.setName("/effectDetector"); // set default name of module
    return module.runModule(argc,argv);
}
