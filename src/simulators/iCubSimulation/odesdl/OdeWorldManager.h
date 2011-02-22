// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007, 2010 Vadim Tikhanoff, Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef ICUBSIMULATION_ODEWORLDMANAGER_INC
#define ICUBSIMULATION_ODEWORLDMANAGER_INC

#include "WorldManager.h"

class OdeWorldManager : public WorldManager {
public:

    OdeWorldManager() {
        num = 0;
        a = b = c = 0;
    }

#ifdef OLD_RESPONDER
    // will need to move logic from respond to apply asap
    virtual bool respond(const yarp::os::Bottle &command, 
                         yarp::os::Bottle &reply);
#endif

    virtual void clear();

    // will only be used when respond() is removed.
    virtual void apply(const WorldOp& op, WorldResult& result);

private:
    int num;
    int a, b, c;
};

#endif
