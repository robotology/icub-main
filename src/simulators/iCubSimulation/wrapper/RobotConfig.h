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


#ifndef ICUBSIMULATION_ROBOTCONFIG_INC
#define ICUBSIMULATION_ROBOTCONFIG_INC

#include <string>
#include <yarp/os/ResourceFinder.h>

class RobotFlags {
public:
    bool valid;
    bool actElevation, actStartHomePos , actLegs, actTorso, actLArm, actRArm, actLHand, actRHand, actHead, actfixedHip, actVision, actHeadCover, actWorld, actPressure, actScreen, actLegsCovers, actLeftArmCovers, actRightArmCovers, actTorsoCovers, actSelfCol, actCoversCol,	actSkinEmul;
    RobotFlags() {
        valid = false;
    }
};

class OdeParams
{
public:
    double fudgeFactor;
    double stopCFM;
    double jointCFM;
    double worldCFM; 
    int    worldTimestep;
    double stopERP;   
    double worldERP;
    double maxContactCorrectingVel;
    double contactFrictionCoefficient;
    double contactSurfaceLayer;
    double motorMaxTorque;
    double motorDryFriction;
    double jointStopBouncyness;
};

class RobotConfig {
public:
    virtual std::string getModuleName() = 0;
    virtual int getVerbosity() = 0;
    virtual yarp::os::ResourceFinder& getFinder() = 0;
    virtual RobotFlags& getFlags() = 0;

    void setFlags();
    //bool stopConfig(std::string error, bool proceed);
    void stopConfig(std::string error);

    virtual int getWorldTimestep() = 0;
    virtual double getWorldCFM() = 0;
    virtual double getWorldERP() = 0;

    virtual double getFudgeFactor() = 0;
    virtual double getStopCFM() = 0;
    virtual double getJointCFM() = 0;
    virtual double getStopERP() = 0;
    
    virtual double getMaxContactCorrectingVel() = 0;
    virtual double getContactFrictionCoefficient() = 0;
    virtual double getContactSurfaceLayer() = 0;

    virtual double getMotorMaxTorque() = 0;
    virtual double getMotorDryFriction() = 0;
    virtual double getJointStopBouncyness() = 0;

    virtual OdeParams getOdeParameters() = 0;
};

#endif
