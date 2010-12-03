// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick, MArtin Peniak
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
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

#include "SDL_thread.h"
#include "SDL.h"
#include "SDL_timer.h"
#include "SDL_opengl.h"
#include "rendering.h"
#include <ode/ode.h>
#include <assert.h>
#include "OdeInit.h"
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

extern Semaphore ODE_access;

/**
 *
 * Main simulation driver, using SDL and ODE.
 *
 */
class Simulation {
public:	
    /**
     *
     * Constructor.  Passed a streamer object, which during simulation
     * will be called back to transport vision, touch, and inertial
     * information.
     *
     */
    Simulation(RobotStreamer *streamer, RobotConfig *config);

    /**
     *
     * Destructor.
     *
     */
    ~Simulation();

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
	static void simLoop(int h,int w);

private:
	static void draw();

	static void setJointTorques();

	static void setJointSpeed();

	static void printStats();

	static void handle_key_down(SDL_keysym* keysym);

	static void handle_mouse_motion(SDL_MouseMotionEvent* mousemotion);

	static void process_events(void);

    static void nearCallback (void *data, dGeomID o1, dGeomID o2);

    // returns true if the body with the bodyID is a touch-sensitive body, returns false otherwise.
    static bool isBodyTouchSensitive (dBodyID bodyID);

    static void inspectBodyTouch_continuousValued(Bottle& report);

	static void inspectBodyTouch(Bottle& report);

	static void getAngles(const dReal *m, float& z, float& y, float& x);

	static void initViewpoint();

	static void mouseMovement(float x, float y);

	static void draw_screen();

    static void retreiveInertialData(Bottle& inertialReport);

	static Uint32 ODE_process(Uint32 interval, void *param);

	static int thread_func(void *unused);

	static void sighandler(int sig);

    //////////////////////////////
    //////////////////////////////
    //////////////////////////////

};

#endif

