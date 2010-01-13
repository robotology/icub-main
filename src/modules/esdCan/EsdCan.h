// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup ecan ecan
 *
 * Implements <a href="http://eris.liralab.it/yarpdoc/d3/d5b/classyarp_1_1dev_1_1ICanBus.html"> 
 * ICanBus interface <\a> for a esd can bus board.
 * This is the ecan device.
 *
 * Copyright (C) 2008 RobotCub Consortium.
 *
 * Author: Lorenzo Natale
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/modules/esdCan/EsdCan.h
 *
 */

//
// $Id: EsdCan.h,v 1.4 2008/06/25 22:33:53 nat Exp $
//
//

#ifndef __ESDCANH__
#define __ESDCANH__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "ntcan.h"
#include "memory.h"

#ifdef WIN32
#ifndef NTCAN_HANDLE
#define NTCAN_HANDLE HANDLE
#endif
#endif

namespace yarp{
    namespace dev{
        class EsdCan;
        class EsdCanMessage;
    }
}

class yarp::dev::EsdCanMessage:public yarp::dev::CanMessage
{
public:
    CMSG *msg;

public:
    EsdCanMessage()
    {
        msg=0;
    }

    virtual ~EsdCanMessage()
    {
    }

    virtual CanMessage &operator=(const CanMessage &l)
    {
        const EsdCanMessage &tmp=dynamic_cast<const EsdCanMessage &>(l);
        memcpy(msg, tmp.msg, sizeof(CMSG));
        return *this;
    }

    virtual unsigned int getId() const
    { return msg->id;}

    virtual unsigned char getLen() const
    { return msg->len;}

    virtual void setLen(unsigned char len)
    { msg->len=len;}

    virtual void setId(unsigned int id)
    { msg->id=id;}

    virtual const unsigned char *getData() const
    { return msg->data; }

    virtual unsigned char *getData()
    { return msg->data; }

    virtual unsigned char *getPointer()
    { return (unsigned char *) msg; }

    virtual const unsigned char *getPointer() const
    { return (const unsigned char *) msg; }

    virtual void setBuffer(unsigned char *b)
    { 
        if (b!=0)
            msg=(CMSG *)(b);
    }
};

class yarp::dev::EsdCan: public ImplementCanBufferFactory<EsdCanMessage, CMSG>,
    public ICanBus, 
    public DeviceDriver
{
private:
    NTCAN_HANDLE *handle;
public:
    EsdCan();
    ~EsdCan();

    /* ICanBus */
    virtual bool canSetBaudRate(unsigned int rate);
    virtual bool canGetBaudRate(unsigned int *rate);
    virtual bool canIdAdd(unsigned int id);
    virtual bool canIdDelete(unsigned int id);

    virtual bool canRead(CanBuffer &msgs, 
        unsigned int size, 
        unsigned int *read,
        bool wait=false);

    virtual bool canWrite(const CanBuffer &msgs,
        unsigned int size,
        unsigned int *sent,
        bool wait=false);

    /*Device Driver*/
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();

};

#endif
