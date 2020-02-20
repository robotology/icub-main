// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2015 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <bcbBattery.h>

#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <stdlib.h>

using namespace std;
//#define DEBUG_TEST 1

bool BcbBattery::open(yarp::os::Searchable& config)
{
    bool correct=true;

    //debug
    yDebug("%s\n", config.toString().c_str());

    Bottle& group_general = config.findGroup("GENERAL");
    Bottle& group_serial  = config.findGroup("SERIAL_PORT");

    if (group_general.isNull())
    {
        yError() << "Insufficient parameters to BcbBattery, section GENERAL missing";
        return false;
    }

    if (group_serial.isNull())
    {
        yError() << "Insufficient parameters to BcbBattery, section SERIAL_PORT missing";
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

bool BcbBattery::close()
{
    //stop the thread
    PeriodicThread::stop();

    //stop the driver
    driver.close();

    return true;
}

bool BcbBattery::threadInit()
{
    battery_info = "icub battery system v1.0";
    battery_voltage     = 0.0;
    battery_current     = 0.0;
    battery_charge      = 0.0;
    battery_temperature = 0.0;
    timeStamp = yarp::os::Time::now();

    //start the transmission
    char c = 0x01;
    if (pSerial)
    {
        bool ret = pSerial->send(&c, 1);
        if (ret == false) { yError("BcbBattery problems starting the transmission"); return false; }
    }
    else
    {
        yError("BcbBattery pSerial == NULL");
        return false;
    }


    return true;
}

void BcbBattery::run()
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

    if (pSerial)
    {
        //empty the buffer
        char c = 0;
        int r = 0;
        do
        {
           r = pSerial->receiveChar(c);
        } while (r != 0);

    search_0:
        pSerial->receiveChar(serial_buff[8]); if (serial_buff[8] != '\r') { yWarning("BcbBattery sync error r"); goto search_0; }
        pSerial->receiveChar(serial_buff[9]); if (serial_buff[9] != '\n') { yWarning("BcbBattery sync error n"); goto search_0; }
        pSerial->receiveChar(serial_buff[0]); if (serial_buff[0] != '\0') {goto search_0; }
        pSerial->receiveChar(serial_buff[1]); //voltage
        pSerial->receiveChar(serial_buff[2]); //voltage
        pSerial->receiveChar(serial_buff[3]); //current
        pSerial->receiveChar(serial_buff[4]); //current
        pSerial->receiveChar(serial_buff[5]); //charge
        pSerial->receiveChar(serial_buff[6]); //charge
        pSerial->receiveChar(serial_buff[7]); //status

        serial_buff[10] = 0;
    }
    else
    {
        yError("BcbBattery pSerial == NULL");
    }

#if DEBUG_TEST
    battery_voltage     = 40.0;
    battery_current = 5.0;
    battery_charge = 72.0;
    battery_temperature = 35.0;
#endif

    if (verboseEnable) yDebug("BcbBattery::run() serial_buffer is: %s", serial_buff);
    battery_voltage = (unsigned char) serial_buff[1] * 256 + (unsigned char) serial_buff[2];
    battery_current = (unsigned char) serial_buff[3] * 256 + (unsigned char) serial_buff[4];
    battery_charge =  (unsigned char) serial_buff[5] * 256 + (unsigned char) serial_buff[6];
    backpack_status = (unsigned char) serial_buff[7];

    //add checksum verification
    //...

    // print data to screen
    if (screenEnable)
    {
        time_t rawtime;
        struct tm * timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char* battery_timestamp = asctime(timeinfo);
        char buff[1024];
        sprintf(buff, "battery status: %+6.1fA   % 6.1fV   charge:% 6.1f%%    time: %s", battery_current, battery_voltage, battery_charge, battery_timestamp);
        yDebug("BcbBattery::run() log_buffer is: %s", buff);
    }
}

bool BcbBattery::getBatteryVoltage(double &voltage)
{
    lock_guard<mutex> lck(mtx);
    voltage = battery_voltage;
    return true;
}

bool BcbBattery::getBatteryCurrent(double &current)
{
    lock_guard<mutex> lck(mtx);
    current = battery_current;
    return true;
}

bool BcbBattery::getBatteryCharge(double &charge)
{
    lock_guard<mutex> lck(mtx);
    charge = battery_charge;
    return true;
}

bool BcbBattery::getBatteryStatus(Battery_status &status)
{
    //yError("Not yet implemented");
    return false;
}

bool BcbBattery::getBatteryTemperature(double &temperature)
{
    //yError("Not yet implemented");
    return false;
}

bool BcbBattery::getBatteryInfo(string &info)
{
    lock_guard<mutex> lck(mtx);
    info = battery_info;
    return true;
}

void BcbBattery::threadRelease()
{
    yTrace("BcbBattery Thread released\n");
}
