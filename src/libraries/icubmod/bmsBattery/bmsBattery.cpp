// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2015 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <bmsBattery.h>

#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <stdlib.h>

using namespace std;
#define DEBUG_TEST 1

bool BmsBattery::open(yarp::os::Searchable& config)
{
    bool correct=true;

    //debug
    yDebug("%s\n", config.toString().c_str());

    Bottle& group_general = config.findGroup("GENERAL");
    Bottle& group_serial  = config.findGroup("SERIAL_PORT");

    if (group_general.isNull())
    {
        yError() << "Insufficient parameters to BmsBattery, section GENERAL missing";
        return false;
    }

    if (group_serial.isNull())
    {
        yError() << "Insufficient parameters to BmsBattery, section SERIAL_PORT missing";
        return false;
    }

    int period=config.find("thread_period").asInt();
    setPeriod((double)period/1000.0);

    Property prop;
    std::string ps = group_serial.toString();
    prop.fromString(ps);
    prop.put("device", "serialport");

    //open the driver
    driver.open(prop);
    if (!driver.isValid())
    {
        yError() << "Error opening PolyDriver check parameters";
#ifndef DEBUG_TEST
        return false;
#endif
    }

    //open the serial interface
    driver.view(pSerial);
    if (!pSerial)
    {
        yError("Error opening serial driver. Device not available");
#ifndef DEBUG_TEST
        return false;
#endif
    }

    // Other options
    this->verboseEnable = group_general.check("verbose", Value(0), "enable/disable the verbose mode").asBool();
    this->screenEnable = group_general.check("screen", Value(0), "enable/disable the screen output").asBool();
    this->debugEnable = group_general.check("debug", Value(0), "enable/disable the debug mode").asBool();

    PeriodicThread::start();
    return true;
}

bool BmsBattery::close()
{
    //stop the thread
    PeriodicThread::stop();

    //stop the driver
    driver.close();

    return true;
}

bool BmsBattery::threadInit()
{
    battery_info = "icub battery system v1.0";
    battery_voltage     = 0.0;
    battery_current     = 0.0;
    battery_charge      = 0.0;
    battery_temperature = 0.0;
    timeStamp = yarp::os::Time::now();

    return true;
}

bool BmsBattery::verify_checksum(int& raw_battery_current, int&  raw_battery_voltage, int&  raw_battery_charge, int&  raw_battery_checksum)
{
    if (raw_battery_checksum == raw_battery_current + raw_battery_voltage + raw_battery_charge)
        return true;
    return false;
}

void BmsBattery::run()
{
    double timeNow=yarp::os::Time::now();
    lock_guard<mutex> lck(mtx);

    //if 100ms have passed since the last received message
    if (timeStamp+0.1<timeNow)
    {
        //status=IBattery::BATTERY_TIMEOUT;
    }

    //read battery data.
    //if nothing is received, rec=0, the while exits immediately. The string will be not valid, so the parser will skip it and it will leave unchanged the battery status (voltage/current/charge)
    //if a text line is received, then try to receive more text to empty the buffer. If nothing else is received, serial_buff will be left unchanged from the previous value. The loop will exit and the sting will be parsed.
    serial_buff[0] = 0;
    int rec = 0;
    do
    {
        rec = pSerial->receiveLine(serial_buff, 250);
        if (debugEnable) yDebug("%d <%s> ", rec, serial_buff);
    } while
        (rec>0);

    int len = strlen(serial_buff);
    bool reading_ok = false;
    if (len>0)
    {
        int pars = 0;
        int raw_battery_current = 0;
        int raw_battery_voltage = 0;
        int raw_battery_charge = 0;
        int raw_battery_checksum = 0;
        pars = sscanf(serial_buff, "%*s %d %*s %d %*s %d %*s %d", &raw_battery_current, &raw_battery_voltage, &raw_battery_charge, &raw_battery_checksum);

        if (pars == 4)
        {
            if (verify_checksum(raw_battery_current, raw_battery_voltage, raw_battery_charge, raw_battery_checksum))
            {
                time_t rawtime;
                struct tm * timeinfo;
                time(&rawtime);
                timeinfo = localtime(&rawtime);
                //battery_data.timestamp = asctime(timeinfo);
                battery_voltage = double(battery_voltage) / 1024 * 66;
                battery_current = (double(battery_current) - 512) / 128 * 20; //+- 60 is the maximum current that the sensor can read. 128+512 is the value of the AD
                //when the current is 20A.
                battery_charge = double(battery_charge) / 100; // the value coming from the BCS board goes from 0 to 100%
                reading_ok = true;
            }
            else
            {
                yError("checksum error while reading battery data\n");
            }
        }
        else
        {
            yError("error reading battery data: %d\n", pars);
        }
    }

    // print data to screen
    if (screenEnable)
    {
        char buff[1024];
        sprintf(buff, "battery status: %+6.1fA   % 6.1fV   charge:% 6.1f%%", battery_current, battery_voltage, battery_charge);
        yDebug("BmsBattery::run() log_buffer is: %s", buff);
    }
}

bool BmsBattery::getBatteryVoltage(double &voltage)
{
    lock_guard<mutex> lck(mtx);
    voltage = battery_voltage;
    return true;
}

bool BmsBattery::getBatteryCurrent(double &current)
{
    lock_guard<mutex> lck(mtx);
    current = battery_current;
    return true;
}

bool BmsBattery::getBatteryCharge(double &charge)
{
    lock_guard<mutex> lck(mtx);
    charge = battery_charge;
    return true;
}

bool BmsBattery::getBatteryStatus(Battery_status &status)
{
    //yError("Not yet implemented");
    return false;
}

bool BmsBattery::getBatteryTemperature(double &temperature)
{
    //yError("Not yet implemented");
    return false;
}

bool BmsBattery::getBatteryInfo(string &info)
{
    lock_guard<mutex> lck(mtx);
    info = battery_info;
    return true;
}

void BmsBattery::threadRelease()
{
    yTrace("BmsBattery Thread released\n");
}
