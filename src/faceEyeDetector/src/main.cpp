// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Plinio Moreno
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/FaceEyeDetectorModule.h>

using namespace std;
using namespace yarp::os;

using namespace iCub::contrib;

/**
 * @ingroup icub_module
 *
 * \defgroup icub_faceeyedetector faceEyeDetector
 *
 * It detects faces and eyes within each face.
 *
 * The module has two different algorithms to detect faces:
 * (1) The Viola and Jones like opencv detector, that applies a cascade
 * of boosting classifers to classify face candidate regions in the whole
 * image.
 * (2) The MPT ( Machine Perception Toolbox ) from University of 
 * California at San Diego. This is also a cascade of boosting classifiers,
 * but modelling the image as a set of patches that may/may not have faces
 * inside.
 * 
 * The eye detection option is based on the MPT source code, and can be applied
 * to both opencv face detector and MPT face detector.
 *
 *
 * \author Plinio Moreno
 *
 */


int main(int argc, char *argv[]) {

    Network yarp;
    FaceEyeDetectorModule module;
	printf("Module object created\n");
    module.setName("/faceeyedetector"); // set default name of module
    return module.runModule(argc,argv);
}
