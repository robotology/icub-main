// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \file SimConfig.h
 * \brief Header for the automatic configuration of the iCub Simulator
 * \author  Paul Fitzpatrick Vadim Tikhanoff
 * \date 2008
 * \note Release under GNU GPL v2.0
 **/

#ifndef SIMCONFIG_INC
#define SIMCONFIG_INC

#include <yarp/os/ConstString.h>
#include <yarp/os/Value.h>
// std
#include <stdio.h>
#include <iostream>

#include <string>

class SimConfig {
    
public:

    // can't actually configure from command line yet, since
    // some config files get loaded before main() - this needs
    // to be fixed.
    std::string configure(int argc, char *argv[], std::string & moduleName);
    
    yarp::os::ConstString find(const char *fileName);
	yarp::os::ConstString findPath(const char *key);
    bool isActive();
    static std::string moduleName;
    void deleteFinder();
    
};


#endif

