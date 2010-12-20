// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 Vadim Tikhanoff, Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
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
};

#endif
