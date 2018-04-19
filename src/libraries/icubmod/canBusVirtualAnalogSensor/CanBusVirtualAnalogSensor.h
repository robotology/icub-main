// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2013 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __CANBUS_VIRTUAL_ANALOG_SENSOR_H__
#define __CANBUS_VIRTUAL_ANALOG_SENSOR_H__

//#include <stdio.h>
#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVirtualAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/sig/Vector.h>
#include <string>

using namespace yarp::os;
using namespace yarp::dev;

class CanBusVirtualAnalogSensor : public RateThread, public yarp::dev::IVirtualAnalogSensor, public DeviceDriver 
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
    unsigned short     canId;
    //std::string        deviceName;
    yarp::dev::VAS_status   status;
    double             timeStamp;
    AnalogDataFormat   dataFormat;
    yarp::sig::Vector  data;
    yarp::sig::Vector  scaleFactor;
    bool               useCalibration;

public:
    CanBusVirtualAnalogSensor(int period=20) : RateThread(period),mutex(1)
    {}
    

    ~CanBusVirtualAnalogSensor()
    {
    }

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
   
    
    //IVirtualAnalogSensor interface
    virtual yarp::dev::VAS_status getVirtualAnalogSensorStatus (int ch);
    virtual int getVirtualAnalogSensorChannels();
   
    virtual bool updateVirtualAnalogSensorMeasure(int ch, double &measure);
    virtual bool updateVirtualAnalogSensorMeasure(yarp::sig::Vector &data);

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


///------------------------------------------------------------------------



///--------------------------------------------------------
/*

class TBR_AnalogSensor: public yarp::dev::IAnalogSensor,
                    public yarp::dev::DeviceDriver
{
public:
    enum AnalogDataFormat
    {
        ANALOG_FORMAT_8,
        ANALOG_FORMAT_16,
    };

    enum SensorStatus
    {
        ANALOG_IDLE=0,
        ANALOG_OK=1,
        ANALOG_NOT_RESPONDING=-1,
        ANALOG_SATURATION=-2,
        ANALOG_ERROR=-3,
    };

private:
    // debug messages
    unsigned int counterSat;
    unsigned int counterError;
    unsigned int counterTimeout;
    int rate;

    ////////////////////
    //AnalogData *data;
    short status;
    double timeStamp;
    double* scaleFactor;
    yarp::os::Semaphore mutex;
    AnalogDataFormat dataFormat;
    yarp::os::Bottle initMsg;
    yarp::os::Bottle speedMsg;
    yarp::os::Bottle closeMsg;
    std::string deviceIdentifier;
    short boardId;
    short useCalibration;
    bool  isVirtualSensor; //RANDAZ

    bool decode8(const unsigned char *msg, int id, double *data);
    bool decode16(const unsigned char *msg, int id, double *data);

public:
    TBR_CanBackDoor* backDoor; //RANDAZ

    TBR_AnalogSensor();
    ~TBR_AnalogSensor();
    bool handleAnalog(void *);

    void resetCounters()
    {
        counterSat=0;
        counterError=0;
        counterTimeout=0;
    }

    void getCounters(unsigned int &sat, unsigned int &err, unsigned int &to)
    {
        sat=counterSat;
        err=counterError;
        to=counterTimeout;
    }

    void setDeviceId(std::string id)
    {
        deviceIdentifier=id;
    }

    std::string getDeviceId()
    {
        return deviceIdentifier;
    }

    short getId()
    { return boardId;}

    short getStatus()
    { return status;}

    bool isOpen()
    {
        if (data)
            return true;
        else
            return false;
    }

    short getUseCalibration()
        {return useCalibration;}
    double* getScaleFactor()
        {return scaleFactor;}
    double getScaleFactor(int chan)
        {
            if (chan>=0 && chan<data->size())
                return scaleFactor[chan];
            else
                return 0;
        }

    bool open(int channels, AnalogDataFormat f, short bId, short useCalib, bool isVirtualSensor);

    /////////////////////////////////
};
*/

#endif
