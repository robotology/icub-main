// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2015 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __BCBBATTERY_H__
#define __BCBBATTERY_H__

#include <mutex>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IBattery.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/SerialInterfaces.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;

class BcbBattery : public PeriodicThread, public yarp::dev::IBattery, public DeviceDriver
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
    bool silenceSyncWarnings;

    ResourceFinder      rf;
    PolyDriver          driver;
    ISerialDevice       *pSerial;
    char                serial_buff[255];
    std::string         remoteName;
    std::string         localName;

public:
    BcbBattery(int period = 20) : PeriodicThread((double)period/1000.0)
    {}

    ~BcbBattery()
    {
    }

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    virtual bool getBatteryVoltage     (double &voltage);
    virtual bool getBatteryCurrent     (double &current);
    virtual bool getBatteryCharge      (double &charge);
    virtual bool getBatteryStatus      (Battery_status &status);
    virtual bool getBatteryInfo        (std::string &info);
    virtual bool getBatteryTemperature (double &temperature);

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();
};


#endif
