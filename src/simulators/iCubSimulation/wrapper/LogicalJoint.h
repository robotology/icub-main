// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef ICUBSIMULATION_LOGICALJOINT_INC
#define ICUBSIMULATION_LOGICALJOINT_INC

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick
* email:  paulfitz@alum.mit.edu
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

/////////////////////////////////////////////////////////////////////
// establish a mapping from the model to the external axes of control

/**
 *
 * Abstract class for mapping from "physical" (simulated) joints in the model 
 * to control joints in the ICUB specification.
 *
 */
class LogicalJoint {
public:
    /**
     * Destructor.
     */
    virtual ~LogicalJoint() {}

    /**
     * Get the angle of an associated joint, in ICUB units and sign.
     */
    virtual double getAngle() = 0;

    /**
     * Get the angular velocity of an associated joint, in ICUB units and sign.
     */
    virtual double getVelocity() = 0;

    /**
     * Set velocity and acceleration control parameters.
     */
    virtual void setControlParameters(double vel, double acc) = 0;

    /**
     * Drive towards an angle setpoint (in ICUB units/sign).
     */
    virtual void setPosition(double target) = 0;

    /**
     * Set velocity of joint (in ICUB units/sign).
     */
    virtual void setVelocity(double target) = 0;

    virtual bool isValid() = 0;
};


#endif
