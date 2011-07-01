// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick
* email:   paulfitz@alum.mit.edu
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

#ifndef ICUBSIMULATION_WORLDMANAGER_INC
#define ICUBSIMULATION_WORLDMANAGER_INC

#include <yarp/os/Bottle.h>

#include "WorldOp.h"

class WorldManager {
public:
    virtual ~WorldManager() {}

    virtual bool respond(const yarp::os::Bottle &command, 
                         yarp::os::Bottle &reply);

    virtual void clear() {
    }

    virtual void apply(const WorldOp& op, WorldResult& result) = 0;

};

#endif
