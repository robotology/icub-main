// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/**
 * \file SimConfig.cpp
 * \brief This deals with the automatic configuration of the iCub Simulator
 * \author  Paul Fitzpatrick Vadim Tikhanoff
 * \date 2008
 * \note Release under GNU GPL v2.0
 **/
#include "SimConfig.h"
// std
#include <stdio.h>
#include <iostream>

using namespace yarp::os;
using namespace std;

#ifndef EXPERIMENTAL_CONFIG_METHOD

ConstString SimConfig::find(const char *fileName) {
    return fileName;
}

bool SimConfig::isActive() { return false; }

#else // EXPERIMENTAL_CONFIG_METHOD

// HERE we use capability-style configuration search

#include <yarp/os/ResourceFinder.h>
// std
#include <stdio.h>
#include <iostream>

static ResourceFinder *_the_finder = NULL;

void SimConfig::deleteFinder(){
    cout << "deleting finder " << endl;
    delete _the_finder;
}

static string configureFinder(int argc, char *argv[], string &moduleName)
{
    //string moduleName;
     if (_the_finder==NULL) {
        _the_finder = new ResourceFinder;
        if (_the_finder!=NULL) {
            ResourceFinder& finder = *_the_finder;
            finder.setVerbose();
            finder.setDefaultConfigFile("simulator.ini");
            finder.setDefaultContext("simConfig");
            finder.configure("ICUB_ROOT", argc, argv); 
            if ( finder.check( "moduleName" ) ) {
                moduleName = finder.find( "moduleName" ).asString();
                moduleName = "/" + moduleName;
                cout << "NEW MODULE NAME " << moduleName << endl;
            }
            else {    
                moduleName ="/icubSim";
                cout << "OLD MODULE NAME " << moduleName << endl;
            }
        }
    }
    return moduleName;
}

static ResourceFinder& getFinder() {   
    if (_the_finder==NULL) {
        _the_finder = new ResourceFinder;
        if (_the_finder!=NULL) {
            ResourceFinder& finder = *_the_finder;
            finder.setVerbose();
            finder.setDefaultConfigFile("simulator.ini");
            finder.setDefaultContext("simConfig");
            finder.configure("ICUB_ROOT", 0, NULL); 

        }
    }
    if (_the_finder==NULL) {
        printf("Could not create ResourceFinder, no memory\n");
       // exit(1);
    }

    return *_the_finder;
}

ConstString SimConfig::find(const char *fileName) {
   // printf("SimConfig: asked to find %s\n", fileName);
    ConstString location = getFinder().findFile(fileName);
    if (location!="") {
     //printf("Found config: %s\n", location.c_str());
    }
    return location;
}

ConstString SimConfig::findPath(const char *key) {
   // printf("SimConfig: asked to find %s\n", fileName);
	return getFinder().findPath(key);//find(key);
  // return getFinder().find(key);
}
string SimConfig::configure(int argc, char *argv[], string &moduleName )
{
    //string moduleName;
    configureFinder( argc, argv, moduleName );
    return moduleName;
}

bool SimConfig::isActive() { return true; }

#endif // EXPERIMENTAL_CONFIG_METHOD
