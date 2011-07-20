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


#ifndef ICUBSIMULATION_ROBOTSTREAMER_INC
#define ICUBSIMULATION_ROBOTSTREAMER_INC

#include <yarp/os/Bottle.h>

class RobotStreamer {
public:
    virtual void sendVision() = 0;
    virtual void sendTouchLeft(yarp::os::Bottle& report) = 0;
    virtual void sendTouchRight(yarp::os::Bottle& report) = 0;
    virtual bool shouldSendTouchLeft() = 0;
    virtual bool shouldSendTouchRight() = 0;
    virtual void sendInertial(yarp::os::Bottle& report) = 0;
    virtual bool shouldSendInertial() = 0;
    virtual void checkTorques() = 0;
};

#endif
