// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef ICUBSIMULATION_ICUBLOGICALJOINTS_INC
#define ICUBSIMULATION_ICUBLOGICALJOINTS_INC

#include "LogicalJoints.h"
#include "OdeLogicalJoint.h"
#include "RobotConfig.h"

#include <yarp/dev/DeviceDriver.h>

#define MAX_PART 10
#define MAX_AXIS 40

#define PART_ARM_LEFT 1
#define PART_ARM_RIGHT 2
#define PART_HEAD 3
#define PART_HEAD_RAW 9
#define PART_LEG_LEFT 4
#define PART_LEG_RIGHT 5
#define PART_TORSO 6

/**
 *
 * Route control for the iCub's logical joints to their ODE implementation.
 *
 */
class iCubLogicalJoints : public LogicalJoints {
public:
    /**
     *
     * Constructor.
     *
     */
    iCubLogicalJoints(RobotConfig& config);

    /**
     *
     * Access the ODE control for a logical joint, based on part and axis 
     * number.
     *
     */
    virtual LogicalJoint& control(int part, int axis);
private:
    OdeLogicalJoint simControl[MAX_PART][MAX_AXIS];
    
    OdeLogicalJoint& getController(int part, int axis) {
        return simControl[part][axis];
    }
};

#endif
