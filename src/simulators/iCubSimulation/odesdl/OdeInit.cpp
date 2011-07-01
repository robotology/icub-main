// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author:  Vadim Tikhanoff
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

#include "OdeInit.h"

OdeInit *OdeInit::_odeinit = NULL;

OdeInit::OdeInit(RobotConfig *config) : mutex(1), robot_config(config) {
    //create the world parameters
    world = dWorldCreate();
    space = dHashSpaceCreate (0);
    contactgroup = dJointGroupCreate (0);
    
    dWorldSetGravity (world,0,-9.8,0);
    ground = dCreatePlane (space,0, 1, 0, 0);
    //feedback = new dJointFeedback;
    //feedback1 = new dJointFeedback;
    //feedback_mat = new dJointFeedback;
    _iCub = new ICubSim(world, space, 0,0,0, *robot_config);
    _wrld = new worldSim(world, space, 0,0,0, *robot_config);
    _controls = new iCubSimulationControl*[MAX_PART];
    for (int i=0; i<MAX_PART; i++) 
    { 
        _controls[i] = NULL; 
    } // initialize at NULL
    
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
    delete[] _controls;
    
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
void OdeInit::setSimulationControl(iCubSimulationControl *control, int part){
    if (_controls != NULL) {
        _controls[part] = control;
    }
}

void OdeInit::sendHomePos()
{
    double refs[16] = {0,0,0,0,0,0,0,0,0,0,0,10*M_PI/180,10*M_PI/180,10*M_PI/180,10*M_PI/180,10*M_PI/180};
    if (_wrld->actWorld == "on")
    {
       refs[0] = -0*M_PI/180;
       refs[1] = 80*M_PI/180;
       refs[3] = 50*M_PI/180;
       refs[8] = 20*M_PI/180; 
       refs[9] = 20*M_PI/180; 
       refs[10] = 20*M_PI/180;
    }else
    {   
       refs[0] = -25*M_PI/180;
       refs[1] = 20*M_PI/180;
       refs[3] = 50*M_PI/180;
       refs[8] = 20*M_PI/180; 
       refs[9] = 20*M_PI/180; 
       refs[10] = 20*M_PI/180;
    }

    if (_iCub->actLArm == "on" || _iCub->actLHand == "on")
        _controls[1]->positionMoveRaw(refs);
    if (_iCub->actRArm == "on" || _iCub->actRHand == "on")
        _controls[2]->positionMoveRaw(refs);
}

void OdeInit::removeSimulationControl(int part){
    if (_controls != NULL) {
        _controls[part] = NULL;
    }
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

