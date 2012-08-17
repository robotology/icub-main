// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick, Vadim Tikhanoff
* email:   paulfitz@alum.mit.edu, vadim.tikhanoff@iit.it
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

/**
 * \file SimConfig.h
 * \brief Header for the automatic configuration of the iCub Simulator
 * \author  Paul Fitzpatrick and Vadim Tikhanoff
 * \date 2008
 * \note CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 **/

#ifndef SIMCONFIG_INC
#define SIMCONFIG_INC

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>
#include <stdio.h>
#include <string>
#include <fstream>
#include <sstream>

#include "RobotConfig.h"

using namespace std;
using namespace yarp::os;

class SimConfig : public yarp::os::ResourceFinder, public RobotConfig {
    
public:

    SimConfig(){
        odeParamRead = false;
    }

    // can't actually configure from command line yet, since
    // some config files get loaded before main() - this needs
    // to be fixed.
    std::string configure(int argc, char *argv[], std::string & moduleName);
    
    //yarp::os::ConstString find(const char *fileName);
    //yarp::os::ConstString findPath(const char *key);
    //bool isActive();

    //void deleteFinder();

    // RobotConfig interface

    virtual yarp::os::ConstString getModuleName() {
        return moduleName.c_str();
    }

    virtual yarp::os::ResourceFinder& getFinder() {
        return *this;
    }

    RobotFlags& getFlags() {
        return flags;
    }

    virtual double getWorldCFM() {
        readOdeParams();
        return worldCFM;
    }
    virtual double getWorldERP() {
        readOdeParams();
        return worldERP;
    }
    virtual double getFudgeFactor(){
        readOdeParams();
        return fudgeFactor;
    }
    virtual double getStopCFM(){
        readOdeParams();
        return stopCFM;
    }
    virtual double getJointCFM(){
        readOdeParams();
        return jointCFM;
    }
    virtual double getStopERP(){
        readOdeParams();
        return stopERP;
    }
    virtual double getMaxContactCorrectingVel(){
        readOdeParams();
        return maxContactCorrectingVel;
    }
    virtual double getContactSurfaceLayer(){
        readOdeParams();
        return contactSurfaceLayer;
    }
    virtual double getMotorMaxTorque(){
        readOdeParams();
        return motorMaxTorque;
    }
    virtual double getMotorDryFriction(){
        readOdeParams();
        return motorDryFriction;
    }
    virtual double getJointStopBouncyness(){
        readOdeParams();
        return jointStopBouncyness;
    }
    virtual int getWorldTimestep(){
        readOdeParams();
        return worldTimestep;
    }
   

private:
    std::string moduleName;    
    RobotFlags flags;

    bool odeParamRead;
    double fudgeFactor;
    double stopCFM;
    double jointCFM;
    double worldCFM; 
    int    worldTimestep;
    double stopERP;   
    double worldERP;
    double maxContactCorrectingVel;
    double contactSurfaceLayer;
    double motorMaxTorque;
    double motorDryFriction;
    double jointStopBouncyness;

    
    void readOdeParams(){
        if(odeParamRead==true)
            return;

        Property bParams;
        bParams.fromConfigFile(findFile(find("ode").asString().c_str()).c_str());
        Bottle &bParamWorld     = bParams.findGroup("WORLD");
        Bottle &bParamContacts  = bParams.findGroup("CONTACTS");
        Bottle &bParamJoints    = bParams.findGroup("JOINTS");

        worldTimestep   = bParamWorld.check("timestep", Value(10)).asInt();
        worldCFM        = bParamWorld.check("worldCFM", Value(0.00001)).asDouble();
        worldERP        = bParamWorld.check("worldERP", Value(0.2)).asDouble();
        
        maxContactCorrectingVel = bParamContacts.check("maxContactCorrectingVel", Value(1e6)).asDouble();
        contactSurfaceLayer     = bParamContacts.check("contactSurfaceLayer", Value(0.0)).asDouble();

        fudgeFactor         = bParamJoints.check("fudgeFactor", Value(0.02)).asDouble();
        jointCFM            = bParamJoints.check("jointCFM", Value(1e-5)).asDouble();
        stopCFM             = bParamJoints.check("stopCFM", Value(1e-5)).asDouble();
        stopERP             = bParamJoints.check("stopERP", Value(0.2)).asDouble();
        motorMaxTorque      = bParamJoints.check("motorMaxTorque", Value(1e3)).asDouble();
        motorDryFriction    = bParamJoints.check("motorDryFriction", Value(0.1)).asDouble();
        jointStopBouncyness = bParamJoints.check("jointStopBouncyness", Value(0.1)).asDouble();

        odeParamRead = true;
    }
};


#endif

