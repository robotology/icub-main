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

#include <vector>

using namespace yarp::os;
using namespace yarp::dev;




struct MTBInertialBoardInfo
{
        unsigned short     boardId;
        double             accTimeStamp;
        double             gyroTimeStamp;
        unsigned char      enabledSensors;
        unsigned char      sensorPeriod;
        bool               enabledGyro;
        unsigned int       nrOfChannels; ///< 6 if extAccAndGyro, 3 if acc
        unsigned int       vectorOffset; ///< offset of the board data in the output vector
};

/**
*  @ingroup icub_hardware_modules
*  @brief `canbusinertialmtb` : driver for CAN communication with inertial sensors (accelerometers, gyroscopes) mounted on MTB boards.
*
* The minimum MTB firmware version supported by this device is 2.10.17 .
*
* It is possible to read from multiple MTB. This readings will be combined in a single vector.
*
* | YARP device name |
* |:-----------------:|
* | `canbusinertialmtb` |
*
* Parameters accepted in the config argument of the open method:
* | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | canbusDevice   | string | -     | - | Yes | Yarp device name of CAN Bus wrapper | - |
* | physDevice     | string | -     | - | Yes | Yarp device name for the low level CAN device driver | - |
* | canDeviceNum   | int    | -     | - | Yes | ID of the CAN Bus line | - |
* | canAddress     | vector of int    | -     | - | Yes | Vector of the CAN Bus Addresses for the sensor boards | - |
* | period         | int    | milliseconds | 10 | No | Period of the thread reading messages from the CAN bus | - |
* | sensorType     | int |       | - | Yes | Type of sensor to read from MTBs.  | Possible values: acc for the internal LIS331DLH accelerometer of the MTB, extAccAndGyro for the external LIS331DLH accelerometer and L3G4200D gyroscope. |
* | sensorPeriod   | int    | milliseconds | 5 | No | Every sensorPeriod milliseconds the MTB publishes the sensor measurements on the CAN Bus | Possible values: from 1 to 255 |
*/
class CanBusInertialMTB : public RateThread, public yarp::dev::IAnalogSensor, public DeviceDriver
{
private:
    /**
     * Validate the input configuration object
     */
    bool validateConf(yarp::os::Searchable& config,
                      std::vector<int> & canAddresses);

    void setPrivateBoardStatus(int boardIndex, short status);
    void setPrivateBoardAccStatus(int boardIndex, short status);
    void setPrivateBoardGyroStatus(int boardIndex, short status);


protected:
    PolyDriver         driver;
    ICanBus            *pCanBus;
    ICanBufferFactory  *pCanBufferFactory;
    CanBuffer          inBuffer;
    CanBuffer          outBuffer;

    yarp::os::Semaphore mutex;

    std::vector<MTBInertialBoardInfo> boards;

    int nrOfTotalChannels;

    yarp::sig::Vector  data;
    yarp::sig::Vector  privateData;
    std::vector<short> sharedStatus;
    std::vector<short> privateStatus;
    short sharedGlobalStatus;
    short privateGlobalStatus;

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
