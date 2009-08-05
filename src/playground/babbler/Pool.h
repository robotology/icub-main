// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef BABBLER_POOL
#define BABBLER_POOL

#include <yarp/os/Random.h>

#include "Evolver.h"

#define POOL_SIZE 100

class Pool {
private:
    Evolver unit[POOL_SIZE];
    int active;
    Range *range;
public:
    Pool() {
        active = 0;
        range = NULL;
    }

    void setRange(Range *range) {
        this->range = range;
        for (int i=0; i<POOL_SIZE; i++) {
            unit[i].setRange(range);
        }
    }

    void hold(int sz) {
        for (int i=0; i<POOL_SIZE; i++) {
            unit[i].hold(sz);
        }
    }
 
    void reward(double val=0.1) {
        //val = 1;
        for (int i=0; i<POOL_SIZE*val; i++) {
            int alt = yarp::os::Random::uniform(0,POOL_SIZE-1);
            if (alt!=active) {
                unit[alt].copy(unit[active]);
                //printf("Copy %d to %d\n", active, alt);
                unit[alt].perturb();
            }
        }
    }

    void next() {
        int alt = yarp::os::Random::uniform(0,POOL_SIZE-1);
        if (yarp::os::Random::uniform()<0.1) {
            unit[alt].zap();
        } else {
            unit[alt].perturb();
        }
        active = alt;
        //printf("Viewing %d\n", active);
    }

    double get(int index) {
        return unit[active].get(index);
    }
};


#endif


