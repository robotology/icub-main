// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2011 Paul Fitzpatrick
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/

#ifndef ICUBSIMULATION_LOGICALJOINTS_INC
#define ICUBSIMULATION_LOGICALJOINTS_INC

#include "LogicalJoint.h"

#include <yarp/dev/DeviceDriver.h>

/**
 *
 * Route control for the robots controlled joints to their implementation.
 *
 */
class LogicalJoints : public yarp::dev::DeviceDriver {
public:
    virtual ~LogicalJoints() {}

    /**
     *
     * Access the control for a logical joint, based on part and axis 
     * number.
     *
     */
    virtual LogicalJoint& control(int part, int axis) = 0;
};


#endif
