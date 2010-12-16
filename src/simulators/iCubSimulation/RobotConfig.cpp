// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "RobotConfig.h"

#include <yarp/os/Property.h>

#include <iostream>

using namespace yarp::os;
using namespace std;

static void set_flag(bool& flag,const ConstString& str) {
    flag = (str=="on" || str=="1" || str=="ON" || str=="true" || str=="TRUE");
}
#define FLAGIFY(flags,name) set_flag(flags.name,name)

void RobotConfig::setFlags() {
    ConstString general = getFinder().findFile("general");
  
    Property options;
	options.fromConfigFile(general.c_str());

    RobotFlags& flags = getFlags();

	ConstString actElevation = options.findGroup("SETUP").check("elevation",Value(1),"what did the user select?").asString();
    FLAGIFY(flags,actElevation);
	
	ConstString actLegs = options.findGroup("PARTS").check("legs",Value(1),"what did the user select?").asString();
    FLAGIFY(flags,actLegs);
	ConstString actTorso = options.findGroup("PARTS").check("torso",Value(1),"what did the user select?").asString();
    FLAGIFY(flags,actTorso);
	ConstString actLArm = options.findGroup("PARTS").check("left_arm",Value(1),"what did the user select?").asString();
    FLAGIFY(flags,actLArm);
	ConstString actRArm = options.findGroup("PARTS").check("right_arm",Value(1),"what did the user select?").asString();
    FLAGIFY(flags,actRArm);
	ConstString actLHand = options.findGroup("PARTS").check("left_hand",Value(1),"what did the user select?").asString();
    FLAGIFY(flags,actLHand);
	ConstString actRHand = options.findGroup("PARTS").check("right_hand",Value(1),"what did the user select?").asString();
    FLAGIFY(flags,actRHand);
	ConstString actHead = options.findGroup("PARTS").check("head",Value(1),"what did the user select?").asString();
    FLAGIFY(flags,actHead);
	ConstString actfixedHip = options.findGroup("PARTS").check("fixed_hip",Value(1),"what did the user select?").asString();
    FLAGIFY(flags,actfixedHip);
	ConstString actVision = options.findGroup("VISION").check("cam",Value(1),"What did the user select?").asString();
    FLAGIFY(flags,actVision);
    
    ConstString actPressure = options.findGroup("SENSORS").check("pressure",Value(1),"What did the user select?").asString();
    FLAGIFY(flags,actPressure);

	ConstString actWorld = options.findGroup("RENDER").check("objects",Value(1),"What did the user select?").asString();
    FLAGIFY(flags,actWorld);
	ConstString actCover = options.findGroup("RENDER").check("cover",Value(1),"What did the user select?").asString();
    FLAGIFY(flags,actCover);

    flags.valid = true;
	
	cout << "The iCub simulator will start with the following configuration: " << endl << endl <<
		"Elevation : " << actElevation << endl <<
		"Legs : " << actLegs << endl <<
		"Torso : " << actTorso << endl <<
		"Left arm : " << actLArm << endl <<
		"Left hand : " << actLHand << endl <<
		"Right arm : " << actRArm << endl <<
		"Right hand : " << actRHand << endl <<
		"Head : " << actHead << endl <<
		"Fixed Hip : " << actfixedHip << endl << endl << 
        "Pressure sensors: " << actPressure << endl <<
		"Cameras :" << actVision << endl  <<
		"Objects : " << actWorld << endl <<
		"Cover : " << actCover << endl << endl;
}
