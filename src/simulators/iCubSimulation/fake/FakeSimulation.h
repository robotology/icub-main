// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick
* email:    paulfitz@alum.mit.edu
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
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

    virtual bool getTrqData(yarp::os::Bottle &data);

    virtual void clearBuffer() {
    }

    virtual void simLoop(int h,int w);

    virtual bool checkSync(bool reset) {
        return true;
    }
};

#endif
