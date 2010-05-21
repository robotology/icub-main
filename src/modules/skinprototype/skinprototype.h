// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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

using namespace yarp::os;
using namespace yarp::dev;

class SkinPrototype : public RateThread, public yarp::dev::IAnalogSensor, public DeviceDriver 
{
protected:
	PolyDriver driver;
    ICanBus *pCanBus;
    ICanBufferFactory *pCanBufferFactory;
    CanBuffer inBuffer;
    CanBuffer outBuffer;
   
    yarp::os::Semaphore mutex;

    int cardId;
    int sensorsNum;

    yarp::sig::Vector data;

public:
    SkinPrototype(int period=20) : RateThread(period),mutex(1)
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
    virtual bool calibrate(int ch, double v);

	virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

   
};

#endif
