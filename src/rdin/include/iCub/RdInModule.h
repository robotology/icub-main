// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __RDINMODULE__
#define __RDINMODULE__

 // std
#include <iostream>
#include <string>

// yarp
#include <yarp/os/all.h>


namespace iCub {
    namespace contrib {
        class RdInModule;
    }
}


/**
 *
 * RdIn Module class\n
 *
 * \see icub_rdin
 *
 */
class iCub::contrib::RdInModule : public yarp::os::Module {

private:

	yarp::os::BufferedPort<yarp::os::Bottle>    _prtOut;

public:

    RdInModule();
    ~RdInModule();
    
	virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
	virtual double getPeriod();
	virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
};


#endif
