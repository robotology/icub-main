// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2010 Paul Fitzpatrick
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/

#ifndef ICUBSIMULATION_FAKESIMULATION_INC
#define ICUBSIMULATION_FAKESIMULATION_INC

#include "Simulation.h"

#include <yarp/os/Time.h>

class FakeSimulation : public Simulation {
private:
    RobotStreamer *streamer;
    int at;
public:
    virtual void init(RobotStreamer *streamer, RobotConfig *config) {
        this->streamer = streamer;
        at = 0;
    }

	virtual void drawView(bool left, bool right, bool wide) {}

    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& img);

    virtual void clearBuffer() {
    }

	virtual void simLoop(int h,int w);

    virtual bool checkSync(bool reset) {
        return true;
    }
};

#endif
