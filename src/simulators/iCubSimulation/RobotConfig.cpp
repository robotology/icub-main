// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "RobotConfig.h"

#include <yarp/os/Property.h>

void RobotConfig::setFlags() {
    yarp::os::ConstString general = getFinder().findFile("general");
  
    yarp::os::Property options;
	options.fromConfigFile(general.c_str());

    // not finished...
}
