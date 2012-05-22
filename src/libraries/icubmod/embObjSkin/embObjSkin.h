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

// Temporary inclusion because we encapsulate can messages inside a eth frame... for now.
//  In the future this behaviour will dropped in favor of a more flexible and HW independent use of NVs

#include <yarp/dev/CanBusInterface.h>
//#include "/usr/local/src/robot/iCub/pc104/device-drivers/cfw002/src/LinuxDriver/API/libcfw002.h"
//#include "/usr/local/src/robot/iCub/main/src/libraries/icubmod/cfw2Can/Cfw2Can.h"
//#include "/usr/local/src/robot/iCub/pc104/device-drivers/cfw002/src/LinuxDriver/API/cfw002_api.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::os::impl;

#define SIZE_INFO			128

#if 1
namespace yarp{
    namespace dev{
        class Cfw2CanMessage;
    }
}

typedef struct cfw002_rtx_payload CFWCAN_MSG;

class yarp::dev::Cfw2CanMessage : public yarp::dev::CanMessage
{
 public:
    CFWCAN_MSG *msg;

 public:
    Cfw2CanMessage();

     ~Cfw2CanMessage();

    CanMessage &operator=(const CanMessage &l);

    unsigned int getId() const;

     unsigned char getLen() const;

     void setLen(unsigned char len);

     void setId(unsigned int id);

     const unsigned char *getData() const;

     unsigned char *getData();

     unsigned char *getPointer();

     const unsigned char *getPointer() const;

     void setBuffer(unsigned char *b);
};
#endif


class EmbObjSkin : 	public RateThread,
					public yarp::dev::IAnalogSensor,
					public DeviceDriver
					//public ImplementCanBufferFactory<Cfw2CanMessage, CFWCAN_MSG>
//					public ICanBus
{
protected:
//	PolyDriver driver;
    PolyDriver resource;
    ethResources *res;

	// can stuff... to be removed
    ICanBus *pCanBus;
//    ICanBufferFactory *pCanBufferFactory;
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
   
    CanBuffer createBuffer(int elem);
    void destroyBuffer(CanBuffer &buffer);
    
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
