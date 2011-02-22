// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Vadim Tikhanoff
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "OdeInit.h"

OdeInit *OdeInit::_odeinit = NULL;

OdeInit::OdeInit(RobotConfig *config) : mutex(1), robot_config(config) {
    //create the world parameters
    world = dWorldCreate();
    space = dHashSpaceCreate (0);
    contactgroup = dJointGroupCreate (0);
    
    dWorldSetGravity (world,0,-9.8,0);
    ground = dCreatePlane (space,0, 1, 0, 0);
	//		feedback = new dJointFeedback;
	//		feedback1 = new dJointFeedback;
	//		feedback_mat = new dJointFeedback;
    _iCub = new ICubSim(world, space, 0,0,0, *robot_config);
    _wrld = new worldSim(world, space, 0,0,0, *robot_config);	
    
    _wrld->OBJNUM = 0;
	_wrld->waitOBJ = 0;
	_wrld->S_OBJNUM = 0;
    
  	_wrld->SPHNUM = 0;
	_wrld->waitSPH = 0;
	_wrld->S_SPHNUM = 0;
		
	_wrld->cylOBJNUM = 0;
	_wrld->waitOBJ1 = 0;
	_wrld->S_cylOBJNUM = 0;
	
	_wrld->waitMOD = 0;
	_wrld->MODEL_NUM = 0;

	_wrld->s_waitMOD = 0;
	_wrld->s_MODEL_NUM = 0;
}

OdeInit::~OdeInit() {
    delete _wrld;    
    delete _iCub;
    
    dGeomDestroy(ground);
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
}

OdeInit& OdeInit::init(RobotConfig *config) {
    if (_odeinit==NULL) {
        _odeinit=new OdeInit(config);
    }
    return *_odeinit;
}

OdeInit& OdeInit::get() {
    return *_odeinit;
}

void OdeInit::destroy() {
  if (_odeinit!=NULL) {
    delete _odeinit;
    _odeinit = NULL;
  }
}

