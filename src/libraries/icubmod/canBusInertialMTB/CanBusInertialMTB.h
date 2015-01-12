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


/**
*  @ingroup icub_hardware_modules
*  \defgroup canbusinertialmtb canbusinertialmtb
*
*
* Driver for CAN communication with inertial sensors (accelerometers, gyroscopes)
*        mounted on MTB boards.
*
* The minimum MTB firmware version supported by this device is 2.10.17 .
*
* Parameters accepted in the config argument of the open method:
* | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | canbusDevice   | string | -     | - | Yes | Yarp device name of CAN Bus wrapper | - |
* | physDevice     | string | -     | - | Yes | Yarp device name for the low level CAN device driver | - |
* | canDeviceNum   | int    | -     | - | Yes | ID of the CAN Bus line | - |
* | canAddress     | int    | -     | - | Yes | CAN Bus Address for the sensor board | - |
* | period         | int    | milliseconds | 10 | No | Period of the thread reading messages from the CAN bus | - |
* | sensorType     | string |       | - | Yes | Type of sensor to read from MTB.  | Possible values: acc for the internal LIS331DLH accelerometer of the MTB,
                                                                                                     extAccAndGyro for the external LIS331DLH accelerometer and L3G4200D gyroscope. |
* | sensorPeriod   | int    | milliseconds | 5 | No | Every sensorPeriod milliseconds the MTB publishes the sensor measurements on the CAN Bus | Possible values: from 1 to 255 |
*/

class CanBusInertialMTB : public RateThread, public yarp::dev::IAnalogSensor, public DeviceDriver
{
private:
    /**
     * Validate the input configuration object
     */
    bool validateConf(yarp::os::Searchable& config);

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
    double             accTimeStamp;
    double             gyroTimeStamp;
    unsigned char      enabledSensors;
    unsigned char      sensorPeriod;
    bool               enabledGyro;

    yarp::sig::Vector  data;
    yarp::sig::Vector  privateData;

    bool              initted;
    int               count;
public:
    CanBusInertialMTB(int period=20) : RateThread(period),
                                       mutex(1),
                                       initted(false),
                                       count(0)
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
