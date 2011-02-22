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

#ifndef ICUBSIMULATION_ODELOGICALJOINT_INC
#define ICUBSIMULATION_ODELOGICALJOINT_INC

#include <ode/ode.h>

#include <string>

#include "pidfilter.h"

#include "LogicalJoint.h"

/**
 *
 * Convenience class for mapping from physical ODE joints in the model to
 * control joints in the ICUB specification.
 *
 */
class OdeLogicalJoint : public LogicalJoint {
public:
    /**
     * Constructor.
     */
    OdeLogicalJoint();

    /**
     * Destructor.
     */
    virtual ~OdeLogicalJoint();

    /**
     * Create an array of nested control units.  The array will be 
     * destroyed when this object is destroyed.
     */
    OdeLogicalJoint *nest(int len);

    /**
     * Access a nested control unit.
     */
    OdeLogicalJoint *at(int index);

    /**
     * Initialize a regular control unit.
     */
    void init(const char *unit, const char *type, int index, int sign);

    /**
     * Initialize a differential pair of control units.
     */
    void init(OdeLogicalJoint& left, OdeLogicalJoint& right, OdeLogicalJoint& peer, int sgn);

    /**
     * Get the angle of an associated joint, in ICUB units and sign.
     */
    double getAngle() {
        return getAngleRaw()*sign;
    }

    /**
     * Get the angular velocity of an associated joint, in ICUB units and sign.
     */
    double getVelocity() {
        return getVelocityRaw()*sign;
    }

    /**
     * Get the angle of an associated joint in unconverted units and sign.
     */
    double getAngleRaw();

    /**
     * Get the velocity of an associated joint in unconverted units and sign.
     */
    double getVelocityRaw();

    /**
     * Get the current target velocity.
     */
    double getSpeedSetpoint() {
        return speedSetpoint;
    }

    /**
     * Set velocity and acceleration control parameters.
     */
    void setControlParameters(double vel, double acc);

    /**
     * Drive towards an angle setpoint (in ICUB units/sign).
     */
    void setPosition(double target);

    /**
     * Set velocity of joint (in ICUB units/sign).
     */
    void setVelocity(double target);

    /**
     * Set raw velocity of joint (in ODE units/sign).
     */
    void setVelocityRaw(double target);

    virtual bool isValid() {
        return (number != -1) || (verge != 0);
    }

private:
    int number;
	std::string unit;
    dJointID *joint;
    dReal *speed;
    double speedSetpoint;
    bool hinged;
    int universal;
    int sign;
    OdeLogicalJoint *sub;
    int subLength;
    bool active;
    double vel;
    double acc;
    PidFilter filter;

    OdeLogicalJoint *left, *right, *peer;
    int verge;
};


#endif
