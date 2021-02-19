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

    int period=group_general.find("thread_period").asInt();
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
    this->silenceSyncWarnings = group_general.check("silence_sync_warnings", Value(0), "enable/disable the print of warnings in case of sync errors.").asBool();

    PeriodicThread::start();
    return true;
}

bool BcbBattery::close()
{
    lock_guard<mutex> lck(mtx);

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

        //empty the buffer
        char c = 0;
        int r = 0;
        do
        {
            r = pSerial->receiveChar(c);
        } while (r != 0);
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
    serial_buff[0] = 0;

    int output[10] = {0};
    bool syncError = false;

    if (pSerial)
    {
        size_t i = 0;
        bool sync0 = false;
        bool syncr = false;
        bool syncn = false;
        do
        {
            output[0] = pSerial->receiveChar(serial_buff[0]);
            sync0 = output[0] && (serial_buff[0] == '\0');
            ++i;
        } while (!(sync0) && (i < 20));

        if (sync0)
        {
            output[1] = pSerial->receiveChar(serial_buff[1]); //voltage
            output[2] = pSerial->receiveChar(serial_buff[2]); //voltage
            output[3] = pSerial->receiveChar(serial_buff[3]); //current
            output[4] = pSerial->receiveChar(serial_buff[4]); //current
            output[5] = pSerial->receiveChar(serial_buff[5]); //charge
            output[6] = pSerial->receiveChar(serial_buff[6]); //charge
            output[7] = pSerial->receiveChar(serial_buff[7]); //status
        }

        i = 0;
        do
        {
            output[8] = pSerial->receiveChar(serial_buff[8]);
            syncr = output[8] && (serial_buff[8] == '\r');
            if (syncr)
            {
                output[9] = pSerial->receiveChar(serial_buff[9]);
                syncn = output[9] && (serial_buff[9] == '\n');

                if (!syncn)
                {
                    if (!syncError && !silenceSyncWarnings)
                    {
                        yDebug() << "Sync error (n)";
                    }
                    syncError = true;
                }
            }
            else
            {
                if (!syncError && !silenceSyncWarnings)
                {
                    yDebug() << "Sync error (r)";
                }
                syncError = true;
            }
            ++i;
        } while (!(syncn) && (i < 20));

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
#else

    if (verboseEnable)
    {
        char hexBuffer[31];
        char decBuffer[41];
        for (size_t i = 0; i < 10; ++i)
        {
            if (output[i])
            {
                sprintf(hexBuffer + 3*i,"%02X ", (unsigned int)(serial_buff[i] & 0xFF));
                sprintf(decBuffer + 4*i,"%03u ", (unsigned int)(serial_buff[i] & 0xFF));
            }
            else
            {
                sprintf(hexBuffer + 3*i,"-- ");
                sprintf(decBuffer + 4*i,"--- ");
            }
        }

        yDebug("BcbBattery::run() serial_buffer is: (hex) %s, (dec) %s", hexBuffer, decBuffer);
    }

    if (output[1] && output[2] && !syncError)
    {
        battery_voltage = ((unsigned char) serial_buff[1] * 256 + (unsigned char) serial_buff[2])/1000.0;
    }

    if (output[3] && output[4] && !syncError)
    {
        battery_current = ((unsigned char) serial_buff[3] * 256 + (unsigned char) serial_buff[4])/1000.0;
    }

    if (output[5] && output[6] && !syncError)
    {
        battery_charge =  (unsigned char) serial_buff[5] * 256 + (unsigned char) serial_buff[6];
    }

    if (output[7] && !syncError)
    {
        backpack_status = (unsigned char) serial_buff[7];
    }

#endif

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
    status=Battery_status::BATTERY_OK_IN_USE;
    return true;
}

bool BcbBattery::getBatteryTemperature(double &)
{
    //yError("Not yet implemented");
    return true;
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
