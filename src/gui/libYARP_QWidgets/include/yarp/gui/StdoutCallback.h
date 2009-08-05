// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_STDOUTCALLBACK__
#define __ICUB_STDOUTCALLBACK__

// std
#include <iostream>
#include <string>
#include <vector>

// yarp
#include <yarp/os/all.h>

namespace yarp {
    namespace gui {
        class StdoutCallback;
    }
}

class yarp::gui::StdoutCallback : public yarp::os::TypedReaderCallback<yarp::os::Bottle> {

public:
	StdoutCallback();
	virtual ~StdoutCallback();

	virtual void onRead (yarp::os::Bottle &bot);
	virtual void onRead (yarp::os::Bottle &bot, const yarp::os::TypedReader<yarp::os::Bottle> &reader);

	virtual std::string getReadings();

protected:
	yarp::os::Semaphore _mutex;
	std::vector<std::string> _vecStdout;

	StdoutCallback(StdoutCallback const & toCopy){};

};
#endif 

