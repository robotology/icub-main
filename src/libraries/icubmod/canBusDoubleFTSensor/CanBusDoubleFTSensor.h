// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2013 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef CANBUSDOUBLEFTSENSOR_H
#define CANBUSDOUBLEFTSENSOR_H

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
*
*
* Driver for CAN communication with two six-axis FT sensor, mounted in parallel.
* This driver acts as a wrapper and it combines the readings of this two FT sensors,
* emulating the presence of a single six-axis FT sensor.
*
* This device is currently used in an experimental setup of the iCub v2 ankle where
* two six-axis FT sensor are mounted in the ankle.
*
* This driver permits to share the keep the same software interface, regardless of
* whether a single six-axis FT sensor or a double one is mounted in the ankle.
*
* For this double FT sensor ankle, we refer as the "first" sensor as the one on the back
*  of the ankle, while the "second" one is the one on the front of the ankle. We assume
* that the back sensor is oriented as the single sensor that we want to emulate, while
* the front sensor is rotated of 180 degrees around the z axis of the sensor
*  ( http://wiki.icub.org/wiki/FT_sensor ). We then need two geometric parameters to
* simulate the single sensor: the distance between the two sensors and the distance between
* the back sensor and the single sensor that we want to emulate.
*
* For the double sensor ankle setup this parameters extracted from the CAD are:
*   firstSecondDistance : 0.046 m
*   firstSingleDistance : 0.0101175 m
*
* Parameters accepted in the config argument of the open method:
* | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | canbusDevice   | string | -     | - | Yes | Yarp device name of CAN Bus wrapper | - |
* | physDevice     | string | -     | - | Yes | Yarp device name for the low level CAN device driver | - |
* | canDeviceNum   | int    | -     | - | Yes | ID of the CAN Bus line | - |
* | canAddressBackSensor    | int    | -     | - | Yes | CAN Bus Address for the first sensor board | - |
* | canAddressFrontSensor     | int    | -     | - | Yes | CAN Bus Address for the first sensor board | - |
* | period         | int    | ms    | - | Yes | Publication period (in ms) of the sensor reading on the Can Bus | - |
* | backFrontDistance | double | m  | - | Yes | Distance between the back and the front sensor, along the x axis of the back sensor. | - |
* | backSingleDistance | double | m | - | Yes | Distance between the back and the single emulated sensor, along the x axis of the second sensor. | - |
*
*/
class CanBusDoubleFTSensor : public RateThread, public yarp::dev::IAnalogSensor, public DeviceDriver
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

    enum SensorLocation
    {
        BACK_SENSOR=0,
        FRONT_SENSOR=1,
    };

protected:
    PolyDriver         driver;
    ICanBus            *pCanBus;
    ICanBufferFactory  *pCanBufferFactory;
    CanBuffer          inBuffer;
    CanBuffer          outBuffer;
    int                canDeviceNum;

    yarp::os::Semaphore mutex;

    unsigned int       sensorsChannelsNum;
    unsigned int       outputChannelsNum;
    unsigned short     sensorBoardId[2];
    short              status[2];
    short              overallStatus;
    double             timeStamp[2];
    AnalogDataFormat   dataFormat;


    yarp::sig::Vector  sensorData[2];
    yarp::sig::Vector  scaleFactor[2];

    yarp::sig::Vector  buffer;
    yarp::sig::Vector  outputData;

    double backSingleDistance;
    double frontSingleDistance;

public:
    CanBusDoubleFTSensor(int period=20) : RateThread(period),mutex(1),overallStatus(IAnalogSensor::AS_OK)
    {
        status[0] = IAnalogSensor::AS_OK;
        status[1] = IAnalogSensor::AS_OK;
    }


    ~CanBusDoubleFTSensor()
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
    bool decode16(const unsigned char *msg, int msg_id, int sensor_number);
    /**
     * @param sensor_number number of the sensor to open: 0 for the first sensor, 1 for the second sensor
     */
    bool sensor_start (yarp::os::Searchable& config,  int sensor_number);
    bool sensor_stop  (int sensor_number);
    /**
     * @param sensor_number number of the sensor to open: 0 for the first sensor, 1 for the second sensor
     */
    bool readFullScaleAnalog(int ch, int sensor_number);
    void combineDoubleSensorReadings();
    void combineDoubleSensorStatus();
    void openCanMask(int sensor_number);
};


#endif
