// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// added test for ResouceFinder
// -- Lorenzo Natale

// Ideally, header files should be prefixed by "iCub/" to
// avoid conflicts with other projects.
#include <iCub/DiabolicalPlan.h>

// This comes from the sharksWithLasers library
#include <iCub/Sharks.h>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/ConstString.h>

#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include <iostream>

using namespace yarp::os;
using namespace std;

int main(int argc, char *argv[]) {
 
    yarp::os::ResourceFinder rf;

    rf.setDefaultContext("takeOverWithSharks");
    rf.setDefaultConfigFile("takeOver.ini");

    rf.configure("ICUB_ROOT", argc, argv);

    yarp::os::Value &v=rf.find("robot");
    rf.setVerbose();

    //rf.setDefault

    ConstString name=rf.find("robot").asString();
    ConstString weapon=rf.findFile("weapon");
    ConstString resource=rf.findFile("resource");
   
    if (name=="")
    {
        cout<<"Sorry can't find robot name"<<endl;
        return -1;
    }
    if (weapon=="")
    {
        cout<<"Sorry can't find weapon file"<<endl;
        return -1;
    }
    if (resource=="")
    {
        cout<<"Sorry can't find resource file"<<endl;
        return -1;
    }

    DiabolicalPlan db;

    bool done = false;
    do {
        done = db.plan();
    } while (!done);

    printf("I have a plan.\n");
    printf("And so... IT BEGINS!\n");
    Sharks sharks;
    sharks.enableLasers();

    return 0;
}

