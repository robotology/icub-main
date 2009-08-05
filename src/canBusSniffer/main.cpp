// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
@ingroup icub_module
\defgroup icub_canbussniffer canBusSniffer

Connect to the canbus, read all messages and dump to file.

\author Lorenzo Natale

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/canBusSniffer/main.cpp.
**/

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/RateThread.h>

#include <iostream>
#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

const int SNIFFER_THREAD_RATE=50;
const int CAN_DRIVER_BUFFER_SIZE=2047;
const int localBufferSize=512;

class SnifferThread: public RateThread
{
    PolyDriver driver;
    ICanBus *iCanBus;
    ICanBufferFactory *iBufferFactory;
    CanBuffer readBuffer;

public:
    SnifferThread(int r=SNIFFER_THREAD_RATE): RateThread(r)
    {}

    bool threadInit()
    {
        Property prop;
        prop.put("device", "ecan");

        prop.put("CanTxTimeout", 500);
        prop.put("CanRxTimeout", 500);
        prop.put("CanDeviceNum", 0);
        prop.put("CanMyAddress", 0);

        prop.put("CanTxQueueSize", CAN_DRIVER_BUFFER_SIZE);
        prop.put("CanRxQueueSize", CAN_DRIVER_BUFFER_SIZE);

        driver.open(prop);

        if (!driver.isValid())
        {
            fprintf(stderr, "Error opening PolyDriver check parameters\n");
            return false;
        }

        driver.view(iCanBus);
        driver.view(iBufferFactory);

        iCanBus->canSetBaudRate(0); //default 1MB/s

        readBuffer=iBufferFactory->createBuffer(localBufferSize);
        return true;
    }

    void run()
    {
        unsigned int messages=localBufferSize;
        unsigned int readMessages=0;
        bool res=iCanBus->canRead(readBuffer, messages, &readMessages);

        fprintf(stderr, "Read %u messages\n", readMessages);
    }

    void threadRelease()
    {
        iBufferFactory->destroyBuffer(readBuffer);
        driver.close();
    }
};

int main(int argc, char *argv[]) 
{
    SnifferThread thread;

    thread.start();

    std::string input;
    bool done=false;
    while(!done)
    {
        std::cin>>input;
        if (input=="quit")
            done=true;
    }

    thread.stop();
}