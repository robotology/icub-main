// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Hornstein, Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __HEADMOVERMANUALMODULE__
#define __HEADMOVERMANUALMODULE__

 // std
#include <stdio.h>
#include <string>
#include <iostream>

// yarp
#include <yarp/os/all.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

// iCub
#include <simio.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

namespace iCub {
    namespace contrib {
        class HeadMoverManualModule;
    }
}

using namespace iCub::contrib;


/**
 *
 * HeadMoverManual Module class
 *
 * \see icub_headmovermanual
 *
 */
class iCub::contrib::HeadMoverManualModule : public Module {

private:

	// key codes (ascii)
	int key_neck_til_d;
	int key_neck_til_u;
	int key_neck_swi_r;
	int key_neck_swi_l;
	int key_neck_pan_r;
	int key_neck_pan_l;
	int key_eyes_til_u;
	int key_eyes_til_d;
	int key_eyes_ax4_r;
	int key_eyes_ax4_l;
	int key_eyes_ax5_r;
	int key_eyes_ax5_l;
	int key_quit;

	// configuration
	double step_neck;
	double step_eyes;
	double refVelocityNeck;
	double refAccelerationNeck;
	double refVelocityEyes;
	double refAccelerationEyes;

	// controlboard
    PolyDriver			                _dd;
    IEncoders			                *_ienc;
    IPositionControl                    *_ipos;
	IAmplifierControl					*_iamp;
	IPidControl							*_ipid;
    double                              *_encoders;
    int                                 _numAxes;

public:

    HeadMoverManualModule();
    virtual ~HeadMoverManualModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();

};


#endif
