// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick, Martin Peniak
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
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
};

#endif
