// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Vadim Tikhanoff
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
/**
 * \file OdeInit.h
 * \brief This file is responsible for the initialisation of the world parameters that are controlled by ODE. Some extra parameters are/can be added here for simulation stability
 * \author Vadim Tikhanoff
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/

#ifndef ICUBSIMULATION_ODEINIT_INC
#define ICUBSIMULATION_ODEINIT_INC

#include "iCub.h"
#include "world.h"
#include <time.h>
#include <yarp/os/Semaphore.h>
//#include <vector>

#include "RobotConfig.h"

using namespace std;

/**
 *
 * ODE state information.
 *
 */
class OdeInit {
public:
	double SimTime;
	dWorldID world;
	dSpaceID space;
	dJointGroupID contactgroup;
	dGeomID ground;
	//dJointFeedback *feedback;
	//dJointFeedback *feedback1;
	//dJointFeedback *feedback_mat;
	yarp::os::Semaphore mutex;
	yarp::os::Semaphore mutexTexture;
	ICubSim *_iCub;
	worldSim *_wrld;
	bool stop;
    bool sync;
    string name;

    void setName( string module ){
        name = module;
    }
    string getName(){
        return name;
    }

    ~OdeInit();

    static void init(RobotConfig *config);

    static OdeInit& get();

    static void destroy();

private:
    OdeInit(RobotConfig *config);

    static OdeInit *_odeinit;

    RobotConfig *robot_config;
};

// hack, for compatibility with existing code
#define odeinit (OdeInit::get())

#endif
