// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef BABBLER_EVOLVER
#define BABBLER_EVOLVER

#include <yarp/sig/Vector.h>
#include <yarp/os/Random.h>

#include "Range.h"

class Evolver {
private:
    yarp::sig::Vector data;
    Range *range;
public:
    Evolver() {
        range = NULL;
    }
    void setRange(Range *range) {
        this->range = range;
    }
    void hold(int sz) {
        if (sz>data.size()) {
            data = yarp::sig::Vector(sz);
            zap();
        }
    }

    double get(int index) {
        hold(index+1);
        return data[index];
    }

    void copy(const Evolver& e) {
        data = e.data;
        range = e.range;
    }

    void perturb() {
        for (int i=0; i<data.size(); i++) {
            double lo = -5;
            double hi = +5;
            if (range!=NULL) {
                lo = range->getMin(i);
                hi = range->getMax(i);
                hi -= lo;
                hi /= 24;
                lo = -hi;
            }
            double v = yarp::os::Random::uniform()*(hi-lo)+lo;
            data[i] += v;
        }
    }
  
    void zap() {
        for (int i=0; i<data.size(); i++) {
            double lo = -70;
            double hi = +70;
            if (range!=NULL) {
                lo = range->getMin(i);
                hi = range->getMax(i);
            }
            double v = yarp::os::Random::uniform()*(hi-lo)+lo;
            data[i] = v;
        }
    }
};

#endif

