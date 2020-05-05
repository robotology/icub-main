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

/**
 * \file SimConfig.cpp
 * \brief This deals with the automatic configuration of the iCub Simulator
 * \author  Vadim Tikhanoff, Paul Fitzpatrick
 * \date 2008
 * \note Release under GNU GPL v2.0
 **/

#include "SimConfig.h"
#include <stdio.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace std;

/*
static ResourceFinder *_the_finder = NULL;

void SimConfig::deleteFinder(){
    yDebug("deleting finder\n");
    delete _the_finder;
}
*/

static string configureFinder(int argc, char *argv[], 
                              string &moduleName, int & verbosity,
                              ResourceFinder& finder) {
    //string moduleName;
    finder.setDefaultConfigFile("simulator.ini");
    finder.setDefaultContext("simConfig");
    finder.configure(argc, argv); 


    if ( finder.check( "name" ) ) {
        moduleName = finder.find( "name" ).asString();
        moduleName = "/" + moduleName;
        yInfo() << "NEW MODULE NAME " << moduleName;
    }
    else {    
        moduleName ="/icubSim";
        yInfo()  << "default module name: " << moduleName;
    }
    if ( finder.check( "verbosity" ) ) {
        verbosity = finder.find( "verbosity" ).asInt();
        yInfo()  << "custom verbosity level: " << verbosity;
    }
    else {    
        verbosity = 0;
        yInfo() << "default verbosity level: " << verbosity;
    }
    return moduleName;
}

/*
string SimConfig::find(const char *fileName) {
    yDebug("SimConfig::find\n");
    // yDebug("SimConfig: asked to find %s\n", fileName);
    string location = findFile(fileName);
    if (location!="") {
        //yDebug("Found config: %s\n", location.c_str());
    }
    return location;
}

string SimConfig::findPath(const char *key) {
    yDebug("SimConfig::findPath\n");
   // yDebug("SimConfig: asked to find %s\n", fileName);
    return ResourceFinder::findPath(key);//find(key);
  // return getFinder().find(key);
}
*/

string SimConfig::configure(int argc, char *argv[], string &moduleName, int &verbosity)
{
    yDebug("SimConfig::configure\n");
    //string moduleName;
    ::configureFinder( argc, argv, moduleName, verbosity, *this );
    this->moduleName = moduleName;
    this->verbosity = verbosity;
    return moduleName;
}


