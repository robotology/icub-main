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
    std::string configure(int argc, char *argv[], std::string &moduleName, int &verbosity);
    
    //std::stringfind(const char *fileName);
    //std::string findPath(const char *key);
    //bool isActive();

    //void deleteFinder();

    // RobotConfig interface

    virtual std::string getModuleName() {
        return moduleName.c_str();
    }
    
    virtual int getVerbosity() {
        return verbosity;
    }

    virtual yarp::os::ResourceFinder& getFinder() {
        return *this;
    }

    RobotFlags& getFlags() {
        return flags;
    }

    virtual double getWorldCFM() {
        readOdeParams();
        return p.worldCFM;
    }
    virtual double getWorldERP() {
        readOdeParams();
        return p.worldERP;
    }
    virtual double getFudgeFactor(){
        readOdeParams();
        return p.fudgeFactor;
    }
    virtual double getStopCFM(){
        readOdeParams();
        return p.stopCFM;
    }
    virtual double getJointCFM(){
        readOdeParams();
        return p.jointCFM;
    }
    virtual double getStopERP(){
        readOdeParams();
        return p.stopERP;
    }
    virtual double getMaxContactCorrectingVel(){
        readOdeParams();
        return p.maxContactCorrectingVel;
    }
    virtual double getContactFrictionCoefficient(){
        readOdeParams();
        return p.contactFrictionCoefficient;
    }
    virtual double getContactSurfaceLayer(){
        readOdeParams();
        return p.contactSurfaceLayer;
    }
    virtual double getMotorMaxTorque(){
        readOdeParams();
        return p.motorMaxTorque;
    }
    virtual double getMotorDryFriction(){
        readOdeParams();
        return p.motorDryFriction;
    }
    virtual double getJointStopBouncyness(){
        readOdeParams();
        return p.jointStopBouncyness;
    }
    virtual int getWorldTimestep(){
        readOdeParams();
        return p.worldTimestep;
    }
    virtual OdeParams getOdeParameters(){
        readOdeParams();
        return p;
    }
   

private:
    std::string moduleName;    
    int verbosity;
    RobotFlags flags;

    bool odeParamRead;
    OdeParams p;

    
    void readOdeParams(){
        if(odeParamRead==true)
            return;

        Property bParams;
        bParams.fromConfigFile(findFile(find("ode").asString().c_str()).c_str());
        Bottle &bParamWorld     = bParams.findGroup("WORLD");
        Bottle &bParamContacts  = bParams.findGroup("CONTACTS");
        Bottle &bParamJoints    = bParams.findGroup("JOINTS");

        p.worldTimestep   = bParamWorld.check("timestep", Value(10)).asInt();
        p.worldCFM        = bParamWorld.check("worldCFM", Value(0.00001)).asDouble();
        p.worldERP        = bParamWorld.check("worldERP", Value(0.2)).asDouble();
        
        p.maxContactCorrectingVel = bParamContacts.check("maxContactCorrectingVel", Value(1e6)).asDouble();
        p.contactFrictionCoefficient = bParamContacts.check("contactFrictionCoefficient",Value(1.0)).asDouble();
        p.contactSurfaceLayer     = bParamContacts.check("contactSurfaceLayer", Value(0.0)).asDouble();

        p.fudgeFactor         = bParamJoints.check("fudgeFactor", Value(0.02)).asDouble();
        p.jointCFM            = bParamJoints.check("jointCFM", Value(1e-5)).asDouble();
        p.stopCFM             = bParamJoints.check("stopCFM", Value(1e-5)).asDouble();
        p.stopERP             = bParamJoints.check("stopERP", Value(0.2)).asDouble();
        p.motorMaxTorque      = bParamJoints.check("motorMaxTorque", Value(1e3)).asDouble();
        p.motorDryFriction    = bParamJoints.check("motorDryFriction", Value(0.1)).asDouble();
        p.jointStopBouncyness = bParamJoints.check("jointStopBouncyness", Value(0.1)).asDouble();

        odeParamRead = true;
    }
};


#endif

