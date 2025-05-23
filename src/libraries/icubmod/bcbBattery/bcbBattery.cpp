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
    return (int)(s.size());
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

    int period=group_general.find("thread_period").asInt32();

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
    driver.view(iSerial);
    if (!iSerial)
    {
        yError("Error opening serial driver. Device not available");
#ifndef DEBUG_TEST
        return false;
#endif
    }

    //create the thread
    batteryReader = new batteryReaderThread(iSerial, (double)period / 1000.0);
    // Other options
    batteryReader->verboseEnable = group_general.check("verbose", Value(0), "enable/disable the verbose mode").asBool();
    batteryReader->screenEnable = group_general.check("screen", Value(0), "enable/disable the screen output").asBool();

    //start the thread
    batteryReader->start();
    return true;
}

bool BcbBattery::close()
{
    //stop the thread
    if (batteryReader)
    {
        batteryReader->stop();
        delete batteryReader;
    }

    //stop the driver
    driver.close();

    return true;
}

bool batteryReaderThread::threadInit()
{
    timeStamp = yarp::os::Time::now();

    if (iSerial)
    {
        yInfo("BcbBattery starting transmission");
        startTransmission();
        yInfo("BcbBattery started successfully");
    }
    else
    {
#ifndef DEBUG_TEST
        yError("BcbBattery iSerial == NULL");
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
    if (iSerial)
    {
        recb = iSerial->receiveBytes(tmp_buff, buff_len);
    }
    else
    {
        yError("BcbBattery iSerial == NULL");
    }
#else
    recb = test_buffer((unsigned char*)(tmp_buff), buff_len);
#endif

    if (verboseEnable)
    {
        snprintf(debugTextBuffer, debugTextBufferSize, "Internal buffer: ");
        for (size_t i = 0; i < recb; i++)
            snprintf(debugTextBuffer+strlen(debugTextBuffer), debugTextBufferSize-strlen(debugTextBuffer), "%02X ", (unsigned int)(tmp_buff[i] & 0xFF));
        yDebug() << debugTextBuffer;
    }

    //parse battery data
    std::cmatch cmatch;
    bool b = std::regex_search((const char*)tmp_buff, (const char*)tmp_buff+recb, cmatch, r_exp);

    if (b)
    {
        //get the data
        for (size_t i=0; i< packet_len; i++)
        {
            packet[i] = cmatch.str().c_str()[i];
        }

        //add checksum verification.
        //...

        if (verboseEnable)
        {
            snprintf(debugTextBuffer, debugTextBufferSize, "Found: ");
            snprintf(debugTextBuffer + strlen(debugTextBuffer), debugTextBufferSize - strlen(debugTextBuffer), "<%02X> ", (unsigned int)(packet[0] & 0xFF));
            snprintf(debugTextBuffer + strlen(debugTextBuffer), debugTextBufferSize - strlen(debugTextBuffer), "(%02X %02X) ", (unsigned int)(packet[1] & 0xFF), (unsigned int)(packet[2] & 0xFF));
            snprintf(debugTextBuffer + strlen(debugTextBuffer), debugTextBufferSize - strlen(debugTextBuffer), "(%02X %02X) ", (unsigned int)(packet[3] & 0xFF), (unsigned int)(packet[4] & 0xFF));
            snprintf(debugTextBuffer + strlen(debugTextBuffer), debugTextBufferSize - strlen(debugTextBuffer), "(%02X %02X) ", (unsigned int)(packet[5] & 0xFF), (unsigned int)(packet[6] & 0xFF));
            snprintf(debugTextBuffer + strlen(debugTextBuffer), debugTextBufferSize - strlen(debugTextBuffer), "(%02X) ", (unsigned int)(packet[7] & 0xFF));
            snprintf(debugTextBuffer + strlen(debugTextBuffer), debugTextBufferSize - strlen(debugTextBuffer), "<%02X %02X>", (unsigned int)(packet[8] & 0xFF), (unsigned int)(packet[9] & 0xFF));
            yDebug() << debugTextBuffer;
        }

        //parse values
        datamut.lock();
        battery_voltage  = ((unsigned int)(packet[1])) << 8;
        battery_voltage += (unsigned char)(packet[2]);
        battery_voltage /= 1000.0;
        battery_current = ((unsigned int)(packet[3])) << 8;
        battery_current += (unsigned char)(packet[4]);
        battery_current /= 1000.0;
        battery_charge  = ((unsigned int)(packet[5])) << 8;
        battery_charge += (unsigned char)(packet[6]);
        backpack_status = (unsigned int)(packet[7]);
        battery_status = IBattery::Battery_status::BATTERY_OK_IN_USE;
        datamut.unlock();
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
        snprintf(buff, 1024, "%6.1fV %+6.1fA, charge:%6.1f%%,   time: %s", battery_voltage, battery_current, battery_charge, battery_timestamp);
        yDebug("BcbBattery::run() log_buffer is: %s", buff);
    }

    //flush the buffer
#ifndef DEBUG_TEST
    if (iSerial)
    {
        iSerial->flush();
    }
#endif
}

YARP_DEV_RETURN_VALUE_TYPE_CH312 BcbBattery::getBatteryVoltage(double &voltage)
{
    if (!batteryReader) return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH312;
    std::lock_guard<std::mutex> lg(batteryReader->datamut);
    voltage = batteryReader->battery_voltage;
    return YARP_DEV_RETURN_VALUE_OK_CH312;
}

YARP_DEV_RETURN_VALUE_TYPE_CH312 BcbBattery::getBatteryCurrent(double &current)
{
    if (!batteryReader) return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH312;
    std::lock_guard<std::mutex> lg(batteryReader->datamut);
    current = batteryReader->battery_current;
    return YARP_DEV_RETURN_VALUE_OK_CH312;
}

YARP_DEV_RETURN_VALUE_TYPE_CH312 BcbBattery::getBatteryCharge(double &charge)
{
    if (!batteryReader) return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH312;
    std::lock_guard<std::mutex> lg(batteryReader->datamut);
    charge = batteryReader->battery_charge;
    return YARP_DEV_RETURN_VALUE_OK_CH312;
}

YARP_DEV_RETURN_VALUE_TYPE_CH312 BcbBattery::getBatteryStatus(Battery_status &status)
{
    if (!batteryReader) return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH312;
    std::lock_guard<std::mutex> lg(batteryReader->datamut);
    status= batteryReader->battery_status;
    return YARP_DEV_RETURN_VALUE_OK_CH312;
}

YARP_DEV_RETURN_VALUE_TYPE_CH312 BcbBattery::getBatteryTemperature(double &temperature)
{
    //yError("Not yet implemented");
    temperature = std::nan("");
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH312;
}

YARP_DEV_RETURN_VALUE_TYPE_CH312 BcbBattery::getBatteryInfo(string &info)
{
    if (!batteryReader) return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH312;
    std::lock_guard<std::mutex> lg(batteryReader->datamut);
    info = batteryReader->battery_info;
    return YARP_DEV_RETURN_VALUE_OK_CH312;
}

void batteryReaderThread::threadRelease()
{
    stopTransmission();
}

void batteryReaderThread::startTransmission()
{
    if (!iSerial) return;

    //start the transmission
    char cmd = 0x01;
    bool ret = iSerial->send(&cmd, 1);
    if (ret == false)
    {
        yError("BcbBattery problems starting the transmission");
        return;
    }

    //empty the buffer
    iSerial->flush();
}

void batteryReaderThread::stopTransmission()
{
    if (!iSerial) return;

    char c = 0x00;
    bool ret = iSerial->send(&c, 1);
    if (ret == false) { yError("BcbBattery problems while stopping the transmission"); }
}
