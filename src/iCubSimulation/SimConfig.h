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

class SimConfig {
public:
    // can't actually configure from command line yet, since
    // some config files get loaded before main() - this needs
    // to be fixed.
    void configure(int argc, char *argv[]);
    
    yarp::os::ConstString find(const char *fileName);
    bool isActive();
    
};


#endif

