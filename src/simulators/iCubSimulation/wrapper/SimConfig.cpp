// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2008 Vadim Tikhanoff, Paul Fitzpatrick
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
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

#include <iostream>

using namespace yarp::os;
using namespace std;

/*
static ResourceFinder *_the_finder = NULL;

void SimConfig::deleteFinder(){
    printf("deleting finder\n");
    delete _the_finder;
}
*/

static string configureFinder(int argc, char *argv[], 
                              string &moduleName,
                              ResourceFinder& finder) {
    //string moduleName;
    finder.setVerbose();
    finder.setDefaultConfigFile("simulator.ini");
    finder.setDefaultContext("simConfig");
    finder.configure("ICUB_ROOT", argc, argv); 
    if ( finder.check( "name" ) ) {
        moduleName = finder.find( "name" ).asString();
        moduleName = "/" + moduleName;
        cout << "NEW MODULE NAME " << moduleName << endl;
    }
    else {    
        moduleName ="/icubSim";
        cout << "OLD MODULE NAME " << moduleName << endl;
    }
    return moduleName;
}

/*
ConstString SimConfig::find(const char *fileName) {
    printf("SimConfig::find\n");
    // printf("SimConfig: asked to find %s\n", fileName);
    ConstString location = findFile(fileName);
    if (location!="") {
        //printf("Found config: %s\n", location.c_str());
    }
    return location;
}

ConstString SimConfig::findPath(const char *key) {
    printf("SimConfig::findPath\n");
   // printf("SimConfig: asked to find %s\n", fileName);
	return ResourceFinder::findPath(key);//find(key);
  // return getFinder().find(key);
}
*/

string SimConfig::configure(int argc, char *argv[], string &moduleName )
{
    printf("SimConfig::configure\n");
    //string moduleName;
    ::configureFinder( argc, argv, moduleName, *this );
    this->moduleName = moduleName;
    return moduleName;
}


