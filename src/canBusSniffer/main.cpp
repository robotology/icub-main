// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
@ingroup icub_tools
\defgroup icub_canbussniffer canBusSniffer

A canbus sniffer.

\section intro_sec Description
Connect to a canbus network and dump all messages to file.

\section lib_sec Libraries

- YARP libraries. 
- A device implementing ICanBus interface (e.g. ecan or pcan).

\section parameters_sec Parameters
--device device_name: name of the device (e.g. ecan/pcan...)

\section tested_os_sec Tested OS
Linux and Windows.

\author Lorenzo Natale and Alberto Parmiggiani

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
    std::string devname;
public:
    SnifferThread(std::string dname, int r=SNIFFER_THREAD_RATE): RateThread(r)
    {
        devname=dname;   
    }

    bool threadInit()
    {
        Property prop;
        prop.put("device", devname.c_str());

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
    if (argc!=3)
    {
        std::cout<<"Usage: --device device_name {ecan|pcan}\n";
        return -1;
    }

    std::string p1=std::string(argv[1]);
    std::string p2=std::string(argv[2]);
    if (p1!="--device")
    {
        std::cout<<"Usage: --device device_name {ecan|pcan}\n";
        return -1;
    }
  
    SnifferThread thread(p2);

    if (!thread.start())
    {
        std::cerr<<"Error thread did not start, quitting\n";
        return -1;
    }

    std::string input;
    bool done=false;
    while(!done)
    {
        std::cin>>input;
        if (input=="quit")
            done=true;
    }

    thread.stop();
    return 0;
}