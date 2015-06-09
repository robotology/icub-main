// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2013 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __BCBBATTERY_H__
#define __BCBBATTERY_H__

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IBattery.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/SerialInterfaces.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;

/**
*  @ingroup icub_hardware_modules
*  \defgroup canbusanalogsensor canbusanalogsensor
*
*
* Driver for CAN communication with analog sensors.
*
* Parameters accepted in the config argument of the open method:
* | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | canbusDevice   | string | -     | - | Yes | Yarp device name of CAN Bus wrapper | - |
* | physDevice     | string | -     | - | Yes | Yarp device name for the low level CAN device driver | - |
* | canDeviceNum   | int    | -     | - | Yes | ID of the CAN Bus line | - |
* | canAddress     | int    | -     | - | Yes | CAN Bus Address for the sensor board | - |
* | format         | int    | bits  | - | Yes | Format (i.e. number of bits) of analog data transmitted on the CAN bus (16 for STRAIN board, 8 for MAIS board) | - |
* | period         | int    | ms    | - | Yes | Publication period (in ms) of the sensor reading on the Can Bus | - |
* | channels       | int    | -     | - | Yes | Number of output channels of the sensor (6 for STRAIN board, 16 for MAIS board) | - |
* | useCalibration | int    | -     | - | No  | If useCalibration is present and set to 1 output the calibrated readings, otherwise output the raw values | - |
* | diagnostic  | int    | -     | - | No  | If diagnostic is present and set to 1 properly return the state of the sensor, otherwise always return IAnalogSensor::AS_OK | - |
*/
class BcbBattery : public RateThread, public yarp::dev::IBattery, public DeviceDriver
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
    yarp::os::Semaphore mutex;

    unsigned int       channelsNum;
    unsigned short     boardId;
    short              status;
    double             timeStamp;
    AnalogDataFormat   dataFormat;
    yarp::sig::Vector  data;
    yarp::sig::Vector  scaleFactor;
    unsigned short     useCalibration;
    bool               diagnostic;
    double             battery_charge;
    double             battery_voltage;
    double             battery_current;
    std::string        battery_info;
    unsigned char      backpack_status;

    bool logEnable;
    bool verboseEnable;
    bool screenEnable;
    bool debugEnable;
    bool shutdownEnable;

    FILE                *logFile;
    ResourceFinder      rf;
    PolyDriver          driver;
    ISerialDevice       *pSerial;
    char                serial_buff[255];
    char                log_buffer[255];
    std::string         remoteName;
    std::string         localName;

public:
    BcbBattery(int period = 20) : RateThread(period), mutex(1)
    {}


    ~BcbBattery()
    {
    }

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    virtual bool getBatteryVoltage (double &voltage);
    virtual bool getBatteryCurrent (double &current);
    virtual bool getBatteryCharge  (double &charge);
    virtual bool getBatteryStatus  (int &status);
    virtual bool getBatteryInfo    (yarp::os::ConstString &info);

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    void check_battery_status();
    void notify_message(std::string msg);
    void stop_robot(std::string quit_port);
};


#endif
