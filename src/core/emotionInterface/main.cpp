// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include "emotionInterfaceModule.h"

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

/**
 * @ingroup icub_tools
 *
 * \defgroup icub_emotioninterface emotioninterface
 *
 * Emotion interface module: commands facial expressions
 *
 * \author Alexandre Bernardino
 *
 * Copyright (C) 2007 Alex Bernardino 
 * Recently codified by : Sunila Saqib saqib@mit.edu
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at main/src/core/emotioninterface/main.cpp.
 */



int main(int argc, char *argv[]) {


    Network yarp;

	//resource finder initialization. 
	ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultConfigFile("emotions.ini");
    rf.setDefaultContext("faceExpressions/conf");
    rf.configure("ICUB_ROOT", argc, argv);
   	EmotionInterfaceModule eiModule;
	eiModule.setName("/emotion");
	eiModule.configure(rf);

	return eiModule.runModule();
}
