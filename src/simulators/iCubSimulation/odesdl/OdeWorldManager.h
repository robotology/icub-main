// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef ICUBSIMULATION_ODEWORLDMANAGER_INC
#define ICUBSIMULATION_ODEWORLDMANAGER_INC

#include "WorldManager.h"

class OdeWorldManager : public WorldManager {
public:

    OdeWorldManager() {
        num = 0;
        a = b = c = 0;
    }

    // will need to move logic from respond to apply asap
    virtual bool respond(const yarp::os::Bottle &command, 
                         yarp::os::Bottle &reply);

    virtual void clear();

    virtual void apply(const WorldOp& op, WorldResult& result) {}

private:
    int num;
    int a, b, c;
};

#endif
