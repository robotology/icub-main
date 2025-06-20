// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2015 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __BMSBATTERY_H__
#define __BMSBATTERY_H__

#include <mutex>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IBattery.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ISerialDevice.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

// This is to provide compatibility with both YARP 3.11 and 3.12
#include <iCub/YarpDevReturnValueCompat.h>

using namespace yarp::os;
using namespace yarp::dev;

class BmsBattery : public PeriodicThread, public yarp::dev::IBattery, public DeviceDriver
{
protected:
    std::mutex mtx;

    unsigned short     batteryId;
    short              status;
    double             timeStamp;
    yarp::sig::Vector  data;
    double             battery_charge;
    double             battery_voltage;
    double             battery_current;
    double             battery_temperature;
    std::string        battery_info;
    unsigned char      backpack_status;

    bool verboseEnable;
    bool screenEnable;
    bool debugEnable;

    ResourceFinder      rf;
    PolyDriver          driver;
    ISerialDevice       *pSerial;
    char                serial_buff[255];
    std::string         remoteName;
    std::string         localName;

public:
    BmsBattery(int period = 20) : PeriodicThread((double)period/1000.0)
    {}


    ~BmsBattery()
    {
    }

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    virtual YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryVoltage     (double &voltage);
    virtual YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryCurrent     (double &current);
    virtual YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryCharge      (double &charge);
    virtual YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryStatus      (Battery_status &status);
    virtual YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryInfo        (std::string &info);
    virtual YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryTemperature (double &temperature);

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

private:
    virtual bool verify_checksum(int& raw_battery_current, int&  raw_battery_voltage, int&  raw_battery_charge, int&  raw_battery_checksum);
};


#endif
