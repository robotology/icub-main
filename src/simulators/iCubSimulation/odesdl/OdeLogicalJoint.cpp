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

#include "OdeLogicalJoint.h"
#include "OdeInit.h"

#include <stdio.h>
#include <stdlib.h>

using namespace std;

OdeLogicalJoint::OdeLogicalJoint() : filter(6,0.3,0.0,100) {
    sub = NULL;
    joint = NULL;
    speed = NULL;
    active = false;
    left = NULL;
    right = NULL;
    verge = 0;
    speedSetpoint = 0;
    number = -1;
    vel = 1;
    acc = 1;
}



OdeLogicalJoint::~OdeLogicalJoint() {
    if (sub!=NULL) {
        delete[] sub;
        sub = NULL;
    }
}

OdeLogicalJoint *OdeLogicalJoint::nest(int len) {
    if (sub!=NULL) {
        delete[] sub;
        sub = NULL;
    }
    sub = new OdeLogicalJoint[len];
    subLength = len;
    return sub;
}

OdeLogicalJoint *OdeLogicalJoint::at(int index) {
    if (sub!=NULL) {
        return sub+index;
    }
    return NULL;
}

void OdeLogicalJoint::init(OdeLogicalJoint& left,
                      OdeLogicalJoint& right,
                      OdeLogicalJoint& peer,
                      int sgn) {
    printf("Vergence motor remap %d\n", sgn);
    active = true;
    verge = sgn;
    sign = 1;
    this->left = &left;
    this->right = &right;
    this->peer = &peer;
}


void OdeLogicalJoint::init(const char *unit,
                      const char *type,
                      int index,
                      int sign) {
    OdeInit& odeinit = OdeInit::get();
    active = true;
    number = index;
    this->sign = sign;
    this->unit = unit;
    universal = 0;
    string sunit = unit;
    string stype = type;
    printf("Motor %s %s %d %d\n", unit, type, index, sign);
    if (stype=="hinge") {
        hinged = true;
        universal = 0;
    } else if (stype=="universalAngle1") {
        hinged = false;
        universal = 1;
    } else if (stype=="universalAngle2") {
        hinged = false;
        universal = 2;
    } else {
        printf("Unknown axis type %s\n", type);
        exit(1);
    }
    if (sunit=="leftarm") {
        joint = &(odeinit._iCub->LAjoints[index]);
        if (hinged) {
            speed = &(odeinit._iCub->la_speed[index]);
        } else if (universal==2) {
            speed = &(odeinit._iCub->la_speed1[index]);
        } else {
            speed = &(odeinit._iCub->la_speed[index]);
        }
    } else if (sunit=="rightarm") {
        joint = &(odeinit._iCub->RAjoints[index]);
        if (hinged) {
            speed = &(odeinit._iCub->ra_speed[index]);
        } else if (universal==2) {
            speed = &(odeinit._iCub->ra_speed1[index]);
        } else {
            speed = &(odeinit._iCub->ra_speed[index]);
        }
    } 
    else if (sunit=="head") {
        joint = &(odeinit._iCub->Hjoints[index]);
        speed = &(odeinit._iCub->h_speed[index]);
    } 
    else if (sunit=="leftleg") {
        joint = &(odeinit._iCub->LLegjoints[index]);
        if (hinged) {
            speed = &(odeinit._iCub->LLeg_speed[index]);	
        }
    }
    else if (sunit=="rightleg") {
        joint = &(odeinit._iCub->RLegjoints[index]);
        if (hinged) {
            speed = &(odeinit._iCub->RLeg_speed[index]);
        }
    }
    else if (sunit=="torso") {
        joint = &(odeinit._iCub->Torsojoints[index]);
        if (hinged) {
            speed = &(odeinit._iCub->Torso_speed[index]);
        }
    }
    else {
        printf("Unknown body unit %s\n", unit);
        exit(1);
    }

}

double OdeLogicalJoint::getTorque(int axis) {
    OdeInit& odeinit = OdeInit::get();
    double result = odeinit._iCub->torqueData[axis];
    return result;
}

double OdeLogicalJoint::getAngleRaw() {
    if (verge==0) {
        if (hinged) {
            return dJointGetHingeAngle(*joint);
        } else if (universal == 1) {
            return dJointGetUniversalAngle1(*joint);
        } else if (universal == 2) {
            return dJointGetUniversalAngle2(*joint);
        } 
        return 0;
    } 
    double result = (left->getAngleRaw() + 
                     verge*right->getAngleRaw());
    return result;
}


double OdeLogicalJoint::getVelocityRaw() {
    if (verge==0) {
        if (joint == NULL){
            return 0;
        }
        if (hinged) {
            return dJointGetHingeAngleRate(*joint);
        } else if (universal == 1) {
            return dJointGetUniversalAngle1Rate(*joint);
        } else if (universal == 2) {
            return dJointGetUniversalAngle2Rate(*joint);
        } 
        return 0;
    } 
    double result = (left->getVelocityRaw() + 
                     verge*right->getVelocityRaw());
    return result;
}


void OdeLogicalJoint::setControlParameters(double vel, double acc) {
    this->vel = vel;
    this->acc = acc;
    if (sub!=NULL) {
        for (int i=0; i<subLength; i++) {
            sub[i].setControlParameters(vel,acc);
        }
    }
    if (verge==1) {
        left->setControlParameters(vel,acc);
        right->setControlParameters(vel,acc);
    }
}


void OdeLogicalJoint::setPosition(double target) {
    double error = target - getAngleRaw()*sign;
    double ctrl = filter.pid(error);
    setVelocityRaw(ctrl*sign*vel);
    if (sub!=NULL) {
        for (int i=0; i<subLength; i++) {
            sub[i].setPosition(target);
        }
    }
}

void OdeLogicalJoint::setVelocity(double target) {
    if (sub!=NULL) {
        for (int i=0; i<subLength; i++) {
            sub[i].setVelocity(target);
        }
    }
    setVelocityRaw(sign*target);
}


void OdeLogicalJoint::setVelocityRaw(double target) {
    speedSetpoint = target;
    if (verge==0) {
        if (speed != NULL){
            (*speed) = speedSetpoint;
        }
    } else {
        if (fabs(target)>0.1) {
            //printf("verger %d velocity %g\n", verge, target);
        }
        double altSpeed = peer->getSpeedSetpoint();
        if (verge==1) {
            left->setVelocityRaw(speedSetpoint+altSpeed);
            right->setVelocityRaw(speedSetpoint-altSpeed);
        }
    }
}
