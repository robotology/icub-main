// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef ICUBSIMULATION_WORLDMANAGER_INC
#define ICUBSIMULATION_WORLDMANAGER_INC

#include <yarp/os/Bottle.h>

class WorldManager {
public:
    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);

    void clear();
};

#endif
