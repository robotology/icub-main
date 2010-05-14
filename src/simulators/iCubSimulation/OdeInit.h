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
#include "iCub.h"
#include "world.h"
#include <time.h>
#include <yarp/os/Semaphore.h>
//#include <vector>

using namespace std;

class OdeInit{
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


	OdeInit():mutex(1)
	{	
			
			//create the world parameters
			world = dWorldCreate();
			space = dHashSpaceCreate (0);
			contactgroup = dJointGroupCreate (0);

			dWorldSetGravity (world,0,-9.8,0);
			ground = dCreatePlane (space,0, 1, 0, 0);
	//		feedback = new dJointFeedback;
	//		feedback1 = new dJointFeedback;
	//		feedback_mat = new dJointFeedback;
			_iCub = new ICubSim(world, space, 0,0,0);
			_wrld = new worldSim(world, space, 0,0,0);	
	}

    ~OdeInit()
    {
        delete _wrld;    
        delete _iCub;
        
        dGeomDestroy(ground);
        dJointGroupDestroy(contactgroup);
        dSpaceDestroy(space);
        dWorldDestroy(world);
 		
    }
};
