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
#include <yarp/dev/ISerialDevice.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <regex>

using namespace yarp::os;
using namespace yarp::dev;

class batteryReaderThread : public PeriodicThread
{
    public:
    //debug
    static const int   debugTextBufferSize = 10000;
    char               debugTextBuffer[debugTextBufferSize];

    //configuration options
    bool               verboseEnable = false;
    bool               screenEnable = true;

    //the buffer
    double             timeStamp;
    static const int   buff_len = 10000;
    unsigned char      tmp_buff[buff_len];
    static const int   packet_len =10;
    unsigned char      packet[packet_len];
    std::regex         r_exp;

    ISerialDevice*     iSerial = nullptr;
    std::mutex         datamut;
    double             battery_charge = 0;
    double             battery_voltage = 0;
    double             battery_current = 0;
    std::string        battery_info = "icub battery system v1.0";
    int                backpack_status = 0;
    IBattery::Battery_status     battery_status = IBattery::Battery_status::BATTERY_OK_STANDBY;    

    batteryReaderThread (ISerialDevice *_iSerial, double period) :
    PeriodicThread((double)period),
    iSerial(_iSerial)
    {
       // the following regedit expressions means: search for an occurrence of the pattern:
       // \0........\r\n which is not followed by any other occurrence of the same pattern.
       // See for example: https://docs.pexip.com/admin/regex_reference.htm
       //
       //                    01234567  8  9*****  01234567  8  9
       std::string c_exp ("\\0.......\\r\\n(?!.*\\0.......\\r\\n)");
       r_exp=c_exp;
    }

    void startTransmission();
    void stopTransmission();
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;
};

class BcbBattery: public yarp::dev::IBattery, public DeviceDriver
{
protected:
    batteryReaderThread* batteryReader =nullptr;

    ResourceFinder      rf;
    PolyDriver          driver;
    ISerialDevice       *iSerial = nullptr;

public:
    BcbBattery()  {}
    ~BcbBattery()  {}

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    virtual bool getBatteryVoltage     (double &voltage) override;
    virtual bool getBatteryCurrent     (double &current) override;
    virtual bool getBatteryCharge      (double &charge) override;
    virtual bool getBatteryStatus      (Battery_status &status) override;
    virtual bool getBatteryInfo        (std::string &info) override;
    virtual bool getBatteryTemperature (double &temperature) override;
};


#endif
