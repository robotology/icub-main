// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2010 RobotCub Consortium
// Authors: Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __SKINPROTOTYPE_H__
#define __SKINPROTOTYPE_H__

#include <mutex>
#include <string>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;

class SkinPrototype : public PeriodicThread, public yarp::dev::IAnalogSensor, public DeviceDriver 
{
protected:
	PolyDriver driver;
    ICanBus *pCanBus;
    ICanBufferFactory *pCanBufferFactory;
    CanBuffer inBuffer;
    CanBuffer outBuffer;
   
    std::mutex mtx;

    yarp::sig::VectorOf<int> cardId;
    int sensorsNum;

    yarp::sig::Vector data;

public:
    SkinPrototype(int period=20) : PeriodicThread((double)period/1000.0),mutex(1)
    {}
    

    ~SkinPrototype()
    {
    }

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
   
    
    //IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
	virtual int calibrateSensor();
    virtual int calibrateChannel(int ch, double v);

    virtual int calibrateSensor(const yarp::sig::Vector& v);
    virtual int calibrateChannel(int ch);

	virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

   
};

#endif
