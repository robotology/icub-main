// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff, Paul Fitzpatrick
* email:   vadim.tikhanoff@iit.it, paulfitz@alum.mit.edu
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


#ifndef ICUBSIMULATION_ROBOTCONFIG_INC
#define ICUBSIMULATION_ROBOTCONFIG_INC

#include <yarp/os/ResourceFinder.h>

class RobotFlags {
public:
    bool valid;
    bool actElevation, actLegs, actTorso, actLArm, actRArm, actLHand, actRHand, actHead, actfixedHip, actVision, actCover, actWorld, actPressure, actScreen;
    
    RobotFlags() {
        valid = false;
    }
};

class RobotConfig {
public:
    virtual yarp::os::ConstString getModuleName() = 0;
    virtual yarp::os::ResourceFinder& getFinder() = 0;
    virtual RobotFlags& getFlags() = 0;

    void setFlags();
};

#endif
