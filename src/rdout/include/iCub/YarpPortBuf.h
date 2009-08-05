// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __YARPPORTBUF__
#define __YARPPORTBUF__

// yarp
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/all.h>

// iCub
#include <iCub/StringBuf.h>

namespace iCub {
    namespace contrib {
        class YarpPortBuf;
    }
}

/**
 *
 * A C++ streambuf which can be used to redirect output written
 * to cout to a yarp port.\n
 * Usage: \n
 * YaprPortBuf myYarpBuf;\n
 * myYarpBuf.open(getName("cout");\n
 * std::cout.rdbuf(&myYarpBuf);\n
 *
 */

class iCub::contrib::YarpPortBuf : iCub::contrib::StringBuf {

public:

    YarpPortBuf();
    virtual ~YarpPortBuf();

	virtual bool open(std::string portName);
    virtual bool close();

protected:

	virtual void writeString(const std::string &str);
	yarp::os::BufferedPort<yarp::os::Bottle> _prtOut;
};
#endif 

