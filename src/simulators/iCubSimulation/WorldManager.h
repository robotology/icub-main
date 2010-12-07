// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef ICUBSIMULATION_WORLDMANAGER_INC
#define ICUBSIMULATION_WORLDMANAGER_INC

#include <yarp/os/Bottle.h>

class WorldManager {
public:
    WorldManager() {
        num = 0;
        a = b = c = 0;
    }

    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);

    void clear();

private:
    int num;
    int a, b, c;
    //static int a = 0, b = 0, c = 0;
    //static int num=0;// number of objects in simulation

    
};

#endif
