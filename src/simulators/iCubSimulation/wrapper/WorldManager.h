// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2011 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
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
