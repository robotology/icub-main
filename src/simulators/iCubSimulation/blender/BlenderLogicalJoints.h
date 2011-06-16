// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2011 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUBSIMULATION_BLENDERLOGICALJOINTS_INC
#define ICUBSIMULATION_BLENDERLOGICALJOINTS_INC

#include "LogicalJoints.h"

#include <yarp/dev/DeviceDriver.h>
class BlenderLogicalJoints : public LogicalJoint, public LogicalJoints {
public:
    virtual LogicalJoint& control(int part, int axis) {
        return *this;
    }
    
    virtual double getAngle() { return 0; }
    
    virtual double getVelocity() { return 0; }
    
    virtual void setControlParameters(double vel, double acc) {}
    
    virtual void setPosition(double target) {}
    
    virtual void setVelocity(double target) {}

    virtual bool isValid() {
        return true;
    }
};

#endif
