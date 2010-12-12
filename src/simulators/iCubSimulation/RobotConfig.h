// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 Vadim Tikhanoff, Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */


#ifndef ICUBSIMULATION_ROBOTCONFIG_INC
#define ICUBSIMULATION_ROBOTCONFIG_INC

#include <yarp/os/ResourceFinder.h>

class RobotFlags {
public:
    bool valid;
    bool actElevation, actLegs, actTorso, actLArm, actRArm, actLHand, actRHand, actHead, actfixedHip, actVision, actCover, actWorld, actPressure;
    
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
