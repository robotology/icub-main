// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick, Vadim Tikhanoff, Martin Peniak
* email:    paulfitz@alum.mit.edu, vadim.tikhanoff@iit.it, martin.peniak@plymouth.ac.uk
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
/**
 * \file iCub_Sim.h
 * \brief This class controls the simulation speed using dWorldstep for "exact" calculations, the collisions between objects/spaces and the rendering functions. It also deals with separating the physics calsulations from the rendering 
 * \author Vadim Tikhanoff, Paul Fitzpatrick
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/
//#pragma once
#ifndef ICUB_SIMH
#define ICUB_SIMH

#include <yarp/os/Os.h>
#include <yarp/os/Semaphore.h>

#include "SDL_thread.h"
#include "SDL.h"
#include "SDL_timer.h"
#include "SDL_opengl.h"
#include "rendering.h"
#include <ode/ode.h>
#include <assert.h>
#include "iCub.h"
#include <stdio.h>
#include "world.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <yarp/os/Time.h>
#include "pidfilter.h"
#include <time.h>
#include <signal.h>

#include "VideoTexture.h"
#include "RobotStreamer.h"
#include "RobotConfig.h"
#include "Simulation.h"

extern Semaphore ODE_access;

/**
 *
 * Main simulation driver, using SDL and ODE.
 *
 */
class OdeSdlSimulation : public Simulation {
public:	
    /**
     *
     * Constructor.  Be sure to also call init().
     *
     */
    OdeSdlSimulation();

    void init(RobotStreamer *streamer, RobotConfig *config);

    /**
     *
     * Destructor.
     *
     */
    ~OdeSdlSimulation();

    /**
     *
     * Render the requested view.  
     *
     */
	void drawView(bool left, bool right, bool wide);

    /**
     *
     * Signal that we're done with a view.
     *
     */
    void clearBuffer();

    /**
     *
     * Run the simulation.  This will not return until the simulation
     * is terminated.  This method creates a window for the simulation,
     * and will process keyboard and mouse events related to that
     * window.
     *
     */
    void simLoop(int h,int w);

    bool checkSync(bool reset = false);

    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& target);

    virtual bool getTrqData(Bottle data);

private:
    static void draw();

    static void printStats();

    static void handle_key_down(SDL_keysym* keysym);

    static void handle_mouse_motion(SDL_MouseMotionEvent* mousemotion);

    static void process_events(void);

    static void nearCallback (void *data, dGeomID o1, dGeomID o2);

    // returns true if the body with the bodyID is a touch-sensitive body, returns false otherwise.
    static bool isBodyTouchSensitive (dBodyID bodyID);

    static void inspectBodyTouch_continuousValued(Bottle& report);

    static void inspectBodyTouch_icubSensors(Bottle& reportLeft, Bottle& reportRight, bool boolean);

    static void getAngles(const dReal *m, float& z, float& y, float& x);

    static void initViewpoint();

    static void mouseMovement(float x, float y);

    static void draw_screen();

    static void retreiveInertialData(Bottle& inertialReport);

    static Uint32 ODE_process(Uint32 interval, void *param);

    //static int thread_func(void *unused);
    static int thread_ode(void *unused);

    static void sighandler(int sig);

    //////////////////////////////
    //////////////////////////////
    //////////////////////////////

private:
    yarp::os::Bottle sceneSize;
};

#endif

