// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff
* email:   vadim.tikhanoff@iit.it
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
 * \file OdeInit.h
 * \brief This file is responsible for the initialisation of the world parameters that are controlled by ODE. Some extra parameters are/can be added here for simulation stability
 * \author Vadim Tikhanoff
 * \date 2007
 * \note Released under GNU GPL v2.0
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

    static OdeInit& init(RobotConfig *config);

    static OdeInit& get();

    static void destroy();

private:
    OdeInit(RobotConfig *config);

    static OdeInit *_odeinit;

    RobotConfig *robot_config;
};

#endif
