// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2010 RobotCub Consortium
// Authors: Alberto Cardellino
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __SKIN_MESH_THREAD_H__
#define __SKIN_MESH_THREAD_H__

//#include <stdio.h>
#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/sig/Vector.h>


#include <yarp/os/Log.h>
#include <yarp/os/impl/Logger.h>

#include <ethManager.h>
//#include "../ethManager/ethManager.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::os::impl;

#define SIZE_INFO			128

class EmbObjSkin : 	public RateThread,
					public yarp::dev::IAnalogSensor,
					public DeviceDriver
{
protected:
	PolyDriver driver;
    PolyDriver resource;
    ethResources *res;

	// can stuff... to be removed
    ICanBus *pCanBus;
    ICanBufferFactory *pCanBufferFactory;
    CanBuffer inBuffer;
    CanBuffer outBuffer;
   
    yarp::os::Semaphore mutex;

    yarp::sig::VectorOf<int> cardId;
    int sensorsNum;

    yarp::sig::Vector data;

public:
    EmbObjSkin(int period=20) : RateThread(period),mutex(1)
    {}
    

    ~EmbObjSkin()
    {
    }

    char					info[SIZE_INFO];

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
