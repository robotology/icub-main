// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup shcan shcan
 *
 * Implements ICanBus interface for multiple access from a single access can driver (for example cfw2can).
 * It wraps the low level device driver (physdevice in the configuration file) in a higher level, multiple
 * access virtual device driver.
 *
 * Copyright (C) 2012 RobotCub Consortium.
 *
 * Author: Alessandro Scalzo
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/modules/shcan/SharedCanBus.h
 *
 */

#ifndef __SHARED_CAN_BUS_H__
#define __SHARED_CAN_BUS_H__

#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "canControlConstants.h"

#define UNREQ 0
#define REQST 1

namespace yarp{
    namespace dev{
        class CanBusAccessPoint;
    }
}

class SharedCanBus;

class yarp::dev::CanBusAccessPoint : 
    public ICanBus, 
    public ICanBufferFactory,
    public DeviceDriver
{
public:
    CanBusAccessPoint() : waitReadMutex(0),synchroMutex(1)
    {
        mSharedPhysDevice=NULL;

        reqIds=new char[0x800];

        for (int i=0; i<0x800; ++i) reqIds[i]=UNREQ;

        waitingOnRead=false;

        mBufferSize=0;

        nRecv=0;
    }

    ~CanBusAccessPoint()
    {
        destroyBuffer(readBuffer);

        delete [] reqIds;
    }

    bool hasId(unsigned int id)
    {
        return reqIds[id]==REQST;
    }

    bool pushReadMsg(CanMessage& msg)
    {
        synchroMutex.wait();

        if (nRecv>=mBufferSize)
        {
            synchroMutex.post();
            fprintf(stderr, "[ERROR] recv buffer overrun (%4d > %4d) \n", nRecv, mBufferSize);
            return false;
        }

        readBuffer[nRecv++]=msg;
        
        if (waitingOnRead)
        {
            waitingOnRead=false;
            waitReadMutex.post();
        }
        
        synchroMutex.post();
        return true;
    }

    ////////////
    // ICanBus
    virtual bool canGetBaudRate(unsigned int *rate);
    virtual bool canSetBaudRate(unsigned int rate)
    {
        fprintf(stderr, "[WARNING] set baud rate not allowed from CanBusAccessPoint implementation\n");
        return false;
    }

    virtual bool canIdAdd(unsigned int id);
    virtual bool canIdDelete(unsigned int id);

    virtual bool canRead(CanBuffer &msgs, unsigned int size, unsigned int *nmsg, bool wait=false)
    {
        synchroMutex.wait();

        if (wait && !nRecv)
        {
            waitingOnRead=true;
            synchroMutex.post();
            waitReadMutex.wait();
            synchroMutex.wait();
        }

        if (nRecv)
        {
            for (unsigned int i=0; i<nRecv && i<size; ++i)
            {
                msgs[i]=readBuffer[i];
            }

            *nmsg=nRecv;
            nRecv=0;
            synchroMutex.post();
            return true;
        }
        else
        {
            *nmsg=0;
            synchroMutex.post();
            return true;
        }
    }

    virtual bool canWrite(const CanBuffer &msgs, unsigned int size, unsigned int *sent, bool wait=false);
    // ICanBus
    ////////////
    
    //////////////////////
    // ICanBufferFactory
    virtual CanBuffer createBuffer(int nmessage);
    virtual void destroyBuffer(CanBuffer &msgs);
    // ICanBufferFactory
    //////////////////////

    /////////////////
    // DeviceDriver
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
    // DeviceDriver
    /////////////////

protected:
    yarp::os::Semaphore waitReadMutex;
    yarp::os::Semaphore synchroMutex;
    
    bool waitingOnRead;

    unsigned int nRecv;
    CanBuffer readBuffer;
    
    char *reqIds; //[0x800];

    unsigned int mBufferSize;

    SharedCanBus* mSharedPhysDevice;
};

#endif

