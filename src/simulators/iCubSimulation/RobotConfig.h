// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 Vadim Tikhanoff, Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */


#ifndef ICUBSIMULATION_ROBOTCONFIG_INC
#define ICUBSIMULATION_ROBOTCONFIG_INC

#include <yarp/os/ResourceFinder.h>

class RobotConfig {
public:
    virtual yarp::os::ConstString getModuleName() = 0;
    virtual yarp::os::ResourceFinder& getFinder() = 0;
};

#endif
