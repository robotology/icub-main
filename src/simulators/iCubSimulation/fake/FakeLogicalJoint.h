// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick
* email:    paulfitz@alum.mit.edu
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

#ifndef ICUBSIMULATION_FAKELOGICALJOINT_INC
#define ICUBSIMULATION_FAKELOGICALJOINT_INC

#include "LogicalJoint.h"

#include <yarp/os/Time.h>

class FakeLogicalJoint : public LogicalJoint {
private:
    double angle;
    double vel;
    double last;
    double torque;
public:
    FakeLogicalJoint() { 
        last = yarp::os::Time::now(); 
        angle = vel = torque = 0;
    }

    virtual double getAngle() { 
        update();
        return angle; 
    }
    
    virtual double getVelocity() { 
        update();
        return vel; 
    }
    
    virtual void setControlParameters(double vel, double acc) {
        this->vel = vel;
    }
    
    virtual void setPosition(double target) {
        update();
        vel = (target-angle)*5;
    }
    
    virtual void setVelocity(double target) {
        update();
        vel = target;
    }

    virtual bool isValid() {
        return true;
    }

    
    virtual double getTorque(){
        return true;
    }

    void update() {
        double now = yarp::os::Time::now();
        angle += vel*(now-last);
        last = now;
    }

};


#endif
