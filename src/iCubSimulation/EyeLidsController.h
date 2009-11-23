#ifndef __EYELIDS_CONTROLLER__
#define __EYELIDS_CONTROLLER__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
 * Copyright (C) 2008 Martin Peniak Vadim Tikhanoff
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
/**
 * \file EyeLidsController.h
 * \brief header for EyeLidsController
 * \author Martin Peniak, Vadim Tikhanoff
 * \date 2008
 * \note Release under GNU GPL v2.0
 **/
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <iostream>

class EyeLids
{
public:
    EyeLids();
    ~EyeLids();
	
    yarp::os::BufferedPort<yarp::os::Bottle> port;
public:
	float eyeLidsRotation;
	bool OpenPort();
    
    void ClosePort();
	void checkPort();
};

#endif

