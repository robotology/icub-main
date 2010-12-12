// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "RobotConfig.h"

void RobotConfig::setFlags() {
    ConstString general = getFinder().findFile("general");
  
    Property options;
	options.fromConfigFile(general.c_str());

    RobotFlags& flags = config.getFlags();

}
