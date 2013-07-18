// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2013 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __CANBUSANALOGSENSOR_H__
#define __CANBUSANALOGSENSOR_H__

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;

class CanBusAnalogSensor : public RateThread, public yarp::dev::IAnalogSensor, public DeviceDriver 
{
    enum AnalogDataFormat
    {
        ANALOG_FORMAT_ERR    = 0,
        ANALOG_FORMAT_8_BIT  = 8,
        ANALOG_FORMAT_16_BIT = 16,
    };

    enum SensorStatus
    {
        ANALOG_IDLE=0,
        ANALOG_OK=1,
        ANALOG_NOT_RESPONDING=-1,
        ANALOG_SATURATION=-2,
        ANALOG_ERROR=-3,
    };

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
    AnalogDataFormat   dataFormat;
    yarp::sig::Vector  data;
    yarp::sig::Vector  scaleFactor;
    bool               useCalibration;

public:
    CanBusAnalogSensor(int period=20) : RateThread(period),mutex(1)
    {}
    

    ~CanBusAnalogSensor()
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

    //internal methods
    private:
    bool decode8 (const unsigned char *msg, int msg_id, double *data);
    bool decode16(const unsigned char *msg, int msg_id, double *data);
    bool sensor_start (yarp::os::Searchable& config);
    bool sensor_stop  ();
    bool readFullScaleAnalog(int ch);
};


#endif
