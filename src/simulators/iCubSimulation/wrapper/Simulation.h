// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff, Paul Fitzpatrick
* email:   vadim.tikhanoff@iit.it, paulfitz@alum.mit.edu
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

#ifndef ICUBSIMULATION_SIMULATION_INC
#define ICUBSIMULATION_SIMULATION_INC

#include "RobotStreamer.h"
#include "RobotConfig.h"
#include <yarp/sig/Image.h>

class Simulation {
public:
    /**
     *
     * Initialization.  Passed a streamer object, which during simulation
     * will be called back to transport vision, touch, and inertial
     * information.
     *
     */
    virtual void init(RobotStreamer *streamer, RobotConfig *config) = 0;

    /**
     *
     * Destructor.
     *
     */
    virtual ~Simulation() {}

    /**
     *
     * Render the requested view.  
     *
     */
    virtual void drawView(bool left, bool right, bool wide) = 0;


    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& img) = 0;

    /**
     *
     * Signal that we're done with a view.
     *
     */
    virtual void clearBuffer() = 0;

    /**
     *
     * Run the simulation.  This will not return until the simulation
     * is terminated.  This method creates a window for the simulation,
     * and will process keyboard and mouse events related to that
     * window.
     *
     */
    virtual void simLoop(int h,int w) = 0;

    virtual bool checkSync(bool reset = false) = 0;

    virtual bool getTrqData(yarp::os::Bottle &data) = 0;
};

#endif
