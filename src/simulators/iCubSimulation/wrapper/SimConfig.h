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
#include <stdio.h>
#include <string>

#include "RobotConfig.h"

class SimConfig : public yarp::os::ResourceFinder, public RobotConfig {
    
public:

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

private:
    std::string moduleName;    
    RobotFlags flags;
};


#endif

