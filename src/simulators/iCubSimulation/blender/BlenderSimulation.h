// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) Paul Fitzpatrick
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/

#ifndef ICUBSIMULATION_BLENDERSIMULATION_INC
#define ICUBSIMULATION_BLENDERSIMULATION_INC

#include "Simulation.h"

#include <yarp/os/Time.h>

class BlenderSimulation : public Simulation {
public:
    virtual void init(RobotStreamer *streamer, RobotConfig *config) {}

	virtual void drawView(bool left, bool right, bool wide) {}

    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& img) {
        img.resize(320,240);
        img.zero();
        return true;
    }

    virtual void clearBuffer() {
    }

	virtual void simLoop(int h,int w) {
        int n = 60;
        for (int i=0; i<n; i++) {
            printf("Fake simulation cycle %d of %d\n", i+1, n);
            yarp::os::Time::delay(1);
        }
    }

    virtual bool checkSync(bool reset) {
        return true;
    }
};

#endif
