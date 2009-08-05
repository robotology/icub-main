// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ICUB_FRAMERATE__
#define __ICUB_FRAMERATE__

 // std
#include <stdio.h>

#include <yarp/os/all.h>


using namespace yarp;
using namespace yarp::os;

namespace iCub {
    namespace contrib {
        class Framerate;
    }
}

/**
 *
 * Helper class for Framerate measurements
 *
 */
class iCub::contrib::Framerate : public Module {

private:

    double *_vctDiffs;
    int _counter;
    int _periods;
    double _startTime;
    float _framerate;
	
	void init_internal(int averageOverNumPeriods);

public:

    Framerate();
    Framerate(int averageOverNumPeriods);
    ~Framerate();

    void addStartTime(double time); // time in seconds
    void addEndTime(double time); // time in seconds
    float getFramerate(); // returns current (average) framerate
    bool hasNewFramerate(); // returns true if number of cycles for average is reached
	void init(int averageOverNumPeriods);
};


#endif
