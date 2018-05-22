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
#include <yarp/os/LogStream.h>
#include <iostream>
#include <cstdlib>
#include <string>

using namespace yarp::os;
using namespace std;

static void set_flag(bool& flag,const string& str) {
    flag = (str=="on" || str=="1" || str=="ON" || str=="true" || str=="TRUE");
}
#define FLAGIFY(flags,name) set_flag(flags.name,name)

void RobotConfig::setFlags() {
    string parts = getFinder().findFile("parts");
    if (parts == ""){
         parts = getFinder().findFile("general");
    }
  
    Property options;
    options.fromConfigFile(parts.c_str());

    RobotFlags& flags = getFlags();
    //bool proceed = false;

    string actElevation = options.findGroup("SETUP").check("elevation",Value(1),"what did the user select?").asString();
    if ( actElevation.length()<1 ) stopConfig("actElevation");//, proceed);
    FLAGIFY(flags,actElevation);

    string actStartHomePos = options.findGroup("SETUP").check("startHomePos",Value(1),"what did the user select?").asString();
    if (actStartHomePos.length()<1 ) stopConfig("actStartHomePos");//, proceed);
    FLAGIFY(flags, actStartHomePos);

    string actLegs = options.findGroup("PARTS").check("legs",Value(1),"what did the user select?").asString();
    if (actLegs.length()<1 ) stopConfig("actLegs");//, proceed);
    FLAGIFY(flags,actLegs);

    string actTorso = options.findGroup("PARTS").check("torso",Value(1),"what did the user select?").asString();
    if (actTorso.length()<1 ) stopConfig("actTorso");//, proceed);
    FLAGIFY(flags,actTorso);

    string actLArm = options.findGroup("PARTS").check("left_arm",Value(1),"what did the user select?").asString();
    if (actLArm.length()<1 ) stopConfig("actLArm");//, proceed);
    FLAGIFY(flags,actLArm);

    string actRArm = options.findGroup("PARTS").check("right_arm",Value(1),"what did the user select?").asString();
    if (actRArm.length()<1 ) stopConfig("actRArm");//, proceed);
    FLAGIFY(flags,actRArm);

    string actLHand = options.findGroup("PARTS").check("left_hand",Value(1),"what did the user select?").asString();
    if (actLHand.length()<1 ) stopConfig("actLHand");//, proceed);
    FLAGIFY(flags,actLHand);
    
    string actRHand = options.findGroup("PARTS").check("right_hand",Value(1),"what did the user select?").asString();
    if (actRHand.length()<1 ) stopConfig("actRHand");//, proceed);
    FLAGIFY(flags,actRHand);

    string actHead = options.findGroup("PARTS").check("head",Value(1),"what did the user select?").asString();
    if (actHead.length()<1 ) stopConfig("actHead");//, proceed);
    FLAGIFY(flags,actHead);

    string actfixedHip = options.findGroup("PARTS").check("fixed_hip",Value(1),"what did the user select?").asString();
    if (actfixedHip.length()<1 ) stopConfig("actfixedHip");//, proceed);
    FLAGIFY(flags,actfixedHip);
    
    string actSelfCol = options.findGroup("COLLISIONS").check("self_collisions",Value(1),"what did the user select?").asString();
    if (actSelfCol.length()<1 ) stopConfig("actSelfCol");//, proceed);
    FLAGIFY(flags,actSelfCol);

    string actCoversCol = options.findGroup("COLLISIONS").check("covers_collisions",Value(1),"what did the user select?").asString();
    if (actCoversCol.length()<1 ) stopConfig("actCoversCol");//, proceed);
    FLAGIFY(flags,actCoversCol);
    
    string actVision = options.findGroup("VISION").check("cam",Value(1),"What did the user select?").asString();
    if (actVision.length()<1 ) stopConfig("actVision");//, proceed);
    FLAGIFY(flags,actVision);

    string actPressure = options.findGroup("SENSORS").check("pressure",Value(1),"What did the user select?").asString();
    if (actPressure.length()<1 ) stopConfig("actPressure");//, proceed);
    FLAGIFY(flags,actPressure);
    
    string actSkinEmul = options.findGroup("SENSORS").check("whole_body_skin_emul",Value(1),"What did the user select?").asString();
    if (actSkinEmul.length()<1 ) stopConfig("actSkinEmul");//, proceed);
    FLAGIFY(flags,actSkinEmul);

    string actWorld = options.findGroup("RENDER").check("objects",Value(1),"What did the user select?").asString();
    if (actWorld.length()<1 ) stopConfig("actWorld");//, proceed);
    FLAGIFY(flags,actWorld);

    string actScreen = options.findGroup("RENDER").check("screen",Value(1),"What did the user select?").asString();
    if (actScreen.length()<1 ) stopConfig("actScreen");//, proceed);
    FLAGIFY(flags,actScreen);

    string actHeadCover = options.findGroup("RENDER").check("head_cover",Value(1),"What did the user select?").asString();
    if (actHeadCover.length()<1 ) stopConfig("actHeadCover");//, proceed);
    FLAGIFY(flags,actHeadCover);
    
    string actLegsCovers = options.findGroup("RENDER").check("legs_covers",Value(1),"What did the user select?").asString();
    if (actLegsCovers.length()<1 ) stopConfig("actLegsCovers");//, proceed);
    FLAGIFY(flags,actLegsCovers);
    
    string actLeftArmCovers = options.findGroup("RENDER").check("left_arm_covers",Value(1),"What did the user select?").asString();
    if (actLeftArmCovers.length()<1 ) stopConfig("actLeftArmCovers");//, proceed);
    FLAGIFY(flags,actLeftArmCovers);

    string actRightArmCovers = options.findGroup("RENDER").check("right_arm_covers",Value(1),"What did the user select?").asString();
    if (actRightArmCovers.length()<1 ) stopConfig("actRightArmCovers");//, proceed);
    FLAGIFY(flags,actRightArmCovers);

    string actTorsoCovers = options.findGroup("RENDER").check("torso_covers",Value(1),"What did the user select?").asString();
    if ( actTorsoCovers.length()<1 ) stopConfig("actTorsoCovers");//, proceed);
    FLAGIFY(flags,actTorsoCovers);

    //if (proceed)
        flags.valid = true;
    //else
        //flags.valid = false;

        yInfo() << "The iCub simulator will start with the following configuration: \n\n" <<
        "Elevation : " << actElevation << "\n" <<
        "startHomePos : " << actStartHomePos << "\n" <<
        "Legs : " << actLegs << "\n" <<
        "Torso : " << actTorso << "\n" <<
        "Left arm : " << actLArm << "\n" <<
        "Left hand : " << actLHand << "\n" <<
        "Right arm : " << actRArm << "\n" <<
        "Right hand : " << actRHand << "\n" <<
        "Head : " << actHead << "\n" <<
        "Fixed Hip : " << actfixedHip << "\n" << "\n" <<
        "Self-collisions : " << actSelfCol << "\n" <<
        "Collisions for covers : " << actCoversCol << "\n" << "\n" <<
        "Pressure sensors: " << actPressure << "\n" <<
        "Whole body skin emulation: " << actSkinEmul << "\n" <<
        "Cameras :" << actVision << "\n"  <<
        "Objects : " << actWorld << "\n" <<
        "Head Cover : " << actHeadCover << "\n" <<
        "Legs Cover : " << actLegsCovers << "\n" <<
        "Left arm Covers : " << actLeftArmCovers << "\n" <<
        "Right arm Covers : " << actRightArmCovers << "\n" <<
        "Torso Cover : " << actTorsoCovers << "\n" <<
        "Screen : " << actScreen << "\n" << "\n";
        
}


//bool RobotConfig::stopConfig(string error, bool proceed){
void RobotConfig::stopConfig( string error ){

    yError() << "\n\n\nThere seems to be a conflict with the " << error << " configuration file";
    yError() << "Check the iCub_parts_activation.ini under $ICUB_ROOT/app/simConfig ";
    yError() << "If it still does not work (update svn) and re-install the app folder\n\n";
    yError() << "The iCub simulator will not start..\n\n\n";
    std::exit(1); // we can do this as no port has been created yet
    //return false;
}

