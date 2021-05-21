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
#include <cmath>

using namespace std;

//#define DEBUG_TEST 1

#ifdef DEBUG_TEST
int test_buffer(unsigned char* tmp_buff, int buff_len)
{
    //          012345678901234567890123456789
    //          567890123456789012345678901234
    string s = "MMN...AABBCCD...EEFFGGH...IILL";
    strcpy((char*)(tmp_buff), s.c_str());
    (tmp_buff)[03] = '\r';  (tmp_buff)[04] = '\n';  (tmp_buff)[05] = '\0';
    (tmp_buff)[13] = '\r';  (tmp_buff)[14] = '\n';  (tmp_buff)[15] = '\0';
    (tmp_buff)[23] = '\r';  (tmp_buff)[24] = '\n';  (tmp_buff)[25] = '\0';
    return s.size();
}
#endif

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

    //create the thread
    batteryReader = new batteryReaderThread(pSerial, (double)period / 1000.0);
    // Other options
    batteryReader->verboseEnable = group_general.check("verbose", Value(0), "enable/disable the verbose mode").asBool();
    batteryReader->screenEnable = group_general.check("screen", Value(0), "enable/disable the screen output").asBool();
    batteryReader->silenceSyncWarnings = group_general.check("silence_sync_warnings", Value(0), "enable/disable the print of warnings in case of sync errors.").asBool();

    //start the thread
    batteryReader->start();
    return true;
}

bool BcbBattery::close()
{
    //stop the thread
    batteryReader->stop();

    //stop the driver
    driver.close();

    return true;
}

bool batteryReaderThread::threadInit()
{
    timeStamp = yarp::os::Time::now();

    if (pSerial)
    {
        yInfo("BcbBattery starting transmission");
        startTransmission();
        yInfo("BcbBattery started successfully");
    }
    else
    {
#ifndef DEBUG_TEST
        yError("BcbBattery pSerial == NULL");
        return false;
#endif
    }

    return true;
}

void batteryReaderThread::run()
{
    double timeNow=yarp::os::Time::now();
    int recb = 0;

    //read battery data.
#ifndef DEBUG_TEST
    if (pSerial)
    {
        recb = pSerial->receiveBytes(tmp_buff, buff_len);
    }
    else
    {
        yError("BcbBattery pSerial == NULL");
    }
#else
    recb = test_buffer((unsigned char*)(tmp_buff), buff_len);
#endif

    if (verboseEnable)
    {
        printf("internal buffer:");
        for (size_t i = 0; i < recb; i++)
            printf("%02X ", (unsigned int)(tmp_buff[i] & 0xFF));
        printf("\n");
    }

    //parse battery data
    std::smatch match;
    string stringbuf((char*)tmp_buff, recb);
    bool b = std::regex_search(stringbuf, match, r_exp);
    if (b)
    {
        //get the data
        for (size_t i=0; i< packet_len; i++)
        {
            packet[i] = match.str().c_str()[i];
        }

        //add checksum verification.
        //...

        if (verboseEnable)
        {
            printf("found:");
            for (size_t i = 0; i < packet_len; i++)
                printf("%02X ", (unsigned int)(packet[i] & 0xFF));
            printf("\n");
        }

        battery_voltage = ((unsigned char)packet[1] * 256 + (unsigned char)packet[2]) / 1000.0;
        battery_current = ((unsigned char)packet[3] * 256 + (unsigned char)packet[4]) / 1000.0;
        battery_charge = ((unsigned char)packet[5] * 256 + (unsigned char)packet[6]);
        backpack_status = (unsigned char)packet[7];
        battery_status = IBattery::Battery_status::BATTERY_OK_IN_USE;
    }
    else
    {
        //do nothing
    }

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

    //flush the buffer
    pSerial->flush();
}

bool BcbBattery::getBatteryVoltage(double &voltage)
{
    voltage = batteryReader->battery_voltage;
    return true;
}

bool BcbBattery::getBatteryCurrent(double &current)
{
    current = batteryReader->battery_current;
    return true;
}

bool BcbBattery::getBatteryCharge(double &charge)
{
    charge = batteryReader->battery_charge;
    return true;
}

bool BcbBattery::getBatteryStatus(Battery_status &status)
{
    status= batteryReader->battery_status;
    return true;
}

bool BcbBattery::getBatteryTemperature(double &temperature)
{
    //yError("Not yet implemented");
    temperature = std::nan("");
    return false;
}

bool BcbBattery::getBatteryInfo(string &info)
{
    info = batteryReader->battery_info;
    return true;
}

void batteryReaderThread::threadRelease()
{
    stopTransmission();
}

void batteryReaderThread::startTransmission()
{
    //start the transmission
    char cmd = 0x01;
    bool ret = pSerial->send(&cmd, 1);
    if (ret == false)
    {
        yError("BcbBattery problems starting the transmission");
        return;
    }

    //empty the buffer
    pSerial->flush();
}

void batteryReaderThread::stopTransmission()
{
    if (pSerial)
    {
        char c = 0x00;
        bool ret = pSerial->send(&c, 1);
        if (ret == false) { yError("BcbBattery problems while stopping the transmission"); }
    }
}
