// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick, Vadim Tikhanoff
* email:   paulfitz@alum.mit.edu, vadim.tikhanoff@iit.it
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include "RobotConfig.h"

#include <yarp/os/Property.h>

#include <iostream>
#include <cstdlib>

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
	//bool proceed = false;

    ConstString actElevation = options.findGroup("SETUP").check("elevation",Value(1),"what did the user select?").asString();
	if ( actElevation.length()<1 ) stopConfig("actElevation");//, proceed);
    FLAGIFY(flags,actElevation);

    ConstString actStartHomePos = options.findGroup("SETUP").check("startHomePos",Value(1),"what did the user select?").asString();
    if (actStartHomePos.length()<1 ) stopConfig("actStartHomePos");//, proceed);
	FLAGIFY(flags, actStartHomePos);

    ConstString actLegs = options.findGroup("PARTS").check("legs",Value(1),"what did the user select?").asString();
	if (actLegs.length()<1 ) stopConfig("actLegs");//, proceed);
	FLAGIFY(flags,actLegs);

    ConstString actTorso = options.findGroup("PARTS").check("torso",Value(1),"what did the user select?").asString();
	if (actTorso.length()<1 ) stopConfig("actTorso");//, proceed);
    FLAGIFY(flags,actTorso);

    ConstString actLArm = options.findGroup("PARTS").check("left_arm",Value(1),"what did the user select?").asString();
	if (actLArm.length()<1 ) stopConfig("actLArm");//, proceed);
    FLAGIFY(flags,actLArm);

    ConstString actRArm = options.findGroup("PARTS").check("right_arm",Value(1),"what did the user select?").asString();
	if (actRArm.length()<1 ) stopConfig("actRArm");//, proceed);
    FLAGIFY(flags,actRArm);

    ConstString actLHand = options.findGroup("PARTS").check("left_hand",Value(1),"what did the user select?").asString();
    if (actLHand.length()<1 ) stopConfig("actLHand");//, proceed);
	FLAGIFY(flags,actLHand);
    
	ConstString actRHand = options.findGroup("PARTS").check("right_hand",Value(1),"what did the user select?").asString();
	if (actRHand.length()<1 ) stopConfig("actRHand");//, proceed);
    FLAGIFY(flags,actRHand);

    ConstString actHead = options.findGroup("PARTS").check("head",Value(1),"what did the user select?").asString();
	if (actHead.length()<1 ) stopConfig("actHead");//, proceed);
    FLAGIFY(flags,actHead);

    ConstString actfixedHip = options.findGroup("PARTS").check("fixed_hip",Value(1),"what did the user select?").asString();
	if (actfixedHip.length()<1 ) stopConfig("actfixedHip");//, proceed);
    FLAGIFY(flags,actfixedHip);

    ConstString actVision = options.findGroup("VISION").check("cam",Value(1),"What did the user select?").asString();
	if (actVision.length()<1 ) stopConfig("actVision");//, proceed);
    FLAGIFY(flags,actVision);

    ConstString actPressure = options.findGroup("SENSORS").check("pressure",Value(1),"What did the user select?").asString();
	if (actPressure.length()<1 ) stopConfig("actPressure");//, proceed);
    FLAGIFY(flags,actPressure);

    ConstString actWorld = options.findGroup("RENDER").check("objects",Value(1),"What did the user select?").asString();
	if (actWorld.length()<1 ) stopConfig("actWorld");//, proceed);
    FLAGIFY(flags,actWorld);

    ConstString actScreen = options.findGroup("RENDER").check("screen",Value(1),"What did the user select?").asString();
	if (actScreen.length()<1 ) stopConfig("actScreen");//, proceed);
    FLAGIFY(flags,actScreen);

    ConstString actHeadCover = options.findGroup("RENDER").check("head_cover",Value(1),"What did the user select?").asString();
	if (actHeadCover.length()<1 ) stopConfig("actHeadCover");//, proceed);
    FLAGIFY(flags,actHeadCover);
    
    ConstString actLegsCovers = options.findGroup("RENDER").check("legs_covers",Value(1),"What did the user select?").asString();
	if (actLegsCovers.length()<1 ) stopConfig("actLegsCovers");//, proceed);
    FLAGIFY(flags,actLegsCovers);
    
    ConstString actLeftArmCovers = options.findGroup("RENDER").check("left_arm_covers",Value(1),"What did the user select?").asString();
	if (actLeftArmCovers.length()<1 ) stopConfig("actLeftArmCovers");//, proceed);
    FLAGIFY(flags,actLeftArmCovers);

    ConstString actRightArmCovers = options.findGroup("RENDER").check("right_arm_covers",Value(1),"What did the user select?").asString();
	if (actRightArmCovers.length()<1 ) stopConfig("actRightArmCovers");//, proceed);
    FLAGIFY(flags,actRightArmCovers);

    ConstString actTorsoCovers = options.findGroup("RENDER").check("torso_covers",Value(1),"What did the user select?").asString();
	if ( actTorsoCovers.length()<1 ) stopConfig("actTorsoCovers");//, proceed);
    FLAGIFY(flags,actTorsoCovers);

	//if (proceed)
		flags.valid = true;
	//else
		//flags.valid = false;

        cout << "The iCub simulator will start with the following configuration: " << endl << endl <<
        "Elevation : " << actElevation << endl <<
        "startHomePos : " << actStartHomePos << endl <<
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
        "Head Cover : " << actHeadCover << endl <<
        "Legs Cover : " << actLegsCovers << endl <<
        "Left arm Covers : " << actLeftArmCovers << endl <<
        "Right arm Covers : " << actRightArmCovers << endl <<
        "Torso Cover : " << actTorsoCovers << endl <<
        "Screen : " << actScreen << endl << endl;
		
}


//bool RobotConfig::stopConfig(ConstString error, bool proceed){
void RobotConfig::stopConfig( ConstString error ){

	cout << "\n\n\nThere seems to be a conflict with the " << error << " configuration" << endl;
	cout << "Check the iCub_parts_activation.ini under $CIUB_ROOT/app/simConfig " << endl;
	cout << "If it still does not work (update svn) and re-install the app folder\n\n" << endl;
	cout << "The iCub simulator will not start..\n\n\n" << endl;
	std::exit(1); // we can do this as no port has been created yet
	//return false;
}