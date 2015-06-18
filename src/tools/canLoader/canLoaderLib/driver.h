// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo, Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef DRIVER_H
#define DRIVER_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Searchable.h>

#define MAX_READ_MSG 64

class iDriver
{
public:
    virtual ~iDriver(){}
    virtual int init(yarp::os::Searchable &config)=0;
    virtual int uninit()=0;
    virtual int receive_message(yarp::dev::CanBuffer &messages, int howMany = MAX_READ_MSG, double TIMEOUT = 1)=0;
    virtual int send_message(yarp::dev::CanBuffer &message, int n)=0;

    virtual yarp::dev::CanBuffer createBuffer(int m)=0;
    virtual void destroyBuffer(yarp::dev::CanBuffer &buff)=0;
};

class cDriver : public iDriver
{
private:
    yarp::dev::PolyDriver dd;
    yarp::dev::ICanBus *iCanBus;
    yarp::dev::ICanBufferFactory *iFactory;
public:
    cDriver();
    ~cDriver(){}
    int init(yarp::os::Searchable &config);
    int uninit();
    int receive_message(yarp::dev::CanBuffer &messages, int howMany = MAX_READ_MSG, double TIMEOUT = 1);
    int send_message(yarp::dev::CanBuffer &message, int n);

    yarp::dev::CanBuffer createBuffer(int m);
    void destroyBuffer(yarp::dev::CanBuffer &buff);
};

#endif
