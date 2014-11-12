// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2013 iCub Facility, Istituto Italiano di Tecnologia
// Authors: Marco Randazzo and Lorenzo Natale <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __CANBUSINERTIALMTB_H__
#define __CANBUSINERTIALMTB_H__

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;

class CanBusInertialMTB : public RateThread, public yarp::dev::IAnalogSensor, public DeviceDriver 
{
protected:
    PolyDriver         driver;
    ICanBus            *pCanBus;
    ICanBufferFactory  *pCanBufferFactory;
    CanBuffer          inBuffer;
    CanBuffer          outBuffer;
   
    yarp::os::Semaphore mutex;

    unsigned int       channelsNum;
    unsigned short     boardId;
    short              status;
    double             timeStamp;
    
    yarp::sig::Vector  data;
    yarp::sig::Vector  privateData;
    
    bool              initted;
    int               count;
public:
    CanBusInertialMTB(int period=20) : RateThread(period),mutex(1), initted(false), count(0)
    {}
    

    ~CanBusInertialMTB()
    {
    }

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
   
    
    //IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    int calibrateSensor();
    virtual int calibrateChannel(int ch, double v);

    virtual int calibrateSensor(const yarp::sig::Vector& v);
    virtual int calibrateChannel(int ch);

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();
};

#endif
